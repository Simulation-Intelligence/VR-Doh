import argparse
import os
import numpy as np
import taichi as ti
from math import pi
from mpm_util import *

parser = argparse.ArgumentParser()
parser.add_argument("--arch", type=str, default='vulkan')
parser.add_argument("--cgraph", action='store_true', default=False)
args = parser.parse_args()

def T(a):
    phi, theta = np.radians(28), np.radians(32)

    a = a - 0.5
    x, y, z = a[:, 0], a[:, 1], a[:, 2]
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    x, z = x * cp + z * sp, z * cp - x * sp
    u, v = x, y * ct + z * st
    return np.array([u, v]).swapaxes(0, 1) + 0.5

def get_save_dir(name, arch):
    curr_dir = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(curr_dir, f"{name}_{arch}")

def compile_mpm3D(arch, save_compute_graph, run=False):
    ti.init(arch, vk_api_version="1.0", debug=False)  

    if ti.lang.impl.current_cfg().arch != arch:
        return
    
    dim, n_grid, steps, dt, cube_size, particle_per_grid = 3, 64, 25, 1e-4, 0.2, 16
    n_particles  = int((((n_grid * cube_size) ** dim) * particle_per_grid))
    print("Number of particles: ", n_particles)
    dx = 1 / n_grid
    p_rho = 1000
    _p_vol = dx** 3  
    _p_mass = _p_vol * p_rho / particle_per_grid
    allowed_cfl = 0.5
    v_allowed = dx * allowed_cfl / dt
    gx = 0
    gy = -9.8
    gz = 0
    k = 0.5
    friction_k = 0.4
    damping = 1
    bound = 3
    E = 10000  # Young's modulus for snow
    _SigY = 1000
    nu = 0.45  # Poisson's ratio
    mu_0, lambda_0 = E / (2 * (1 + nu)), E * nu / ((1 + nu) * (1 - 2 * nu))  # Lame parameters
    v = 0.1
    _max_clamp = -0.1
    _min_clamp = -0.1
    friction_angle = 30.0
    sin_phi = ti.sin(friction_angle / 180 * 3.141592653)
    _alpha = ti.sqrt(2 / 3) * 2 * sin_phi / (3 - sin_phi)

    neighbour = (3,) * dim

    use_sticky_cond = 1

    @ti.kernel
    def substep_reset_grid(grid_v: ti.types.ndarray(ndim=3), 
                           grid_m: ti.types.ndarray(ndim=3),
                           marching_m: ti.types.ndarray(ndim=4),
                           min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for I in ti.grouped(grid_m):
            # pos = I * dx + dx * 0.5
            # if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                grid_v[I] = [0, 0, 0] # Reset velocity
                grid_m[I] = 0         # Reset mass
        for I in ti.grouped(marching_m):
            marching_m[I] = 0

    @ti.kernel        
    def set_zero_1dim3(x: ti.types.ndarray(ndim=1)):
        for p in x:
            x[p] = [0, 0, 0]

    @ti.kernel        
    def set_zero_1dim1(x: ti.types.ndarray(ndim=1)):
        for p in x:
            x[p] = 0

    @ti.kernel
    def substep_p2g_multi(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), C: ti.types.ndarray(ndim=1), 
                    dg: ti.types.ndarray(ndim=1), grid_v: ti.types.ndarray(ndim=3), grid_m: ti.types.ndarray(ndim=3),
                    E: ti.types.ndarray(ndim=1), nu: ti.types.ndarray(ndim=1), material: ti.types.ndarray(ndim=1),
                    p_vol: ti.types.ndarray(ndim=1), p_mass: ti.types.ndarray(ndim=1), dx: ti.f32, dt: ti.f32, use_unified_material: ti.i32,
                    min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for p in x:
            if(x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):
                E_p , nu_p , material_p ,p_vol_p, p_mass_p = 0.0, 0.0, 0, 0.0, 0.0
                if(use_unified_material == 1):
                    E_p, nu_p, material_p, p_vol_p, p_mass_p = E[0], nu[0], material[0], p_vol[0], p_mass[0]
                else: 
                    E_p , nu_p , material_p ,p_vol_p, p_mass_p = E[p], nu[p], material[p], p_vol[p], p_mass[p]
                Xp = x[p] / dx
                base = int(Xp - 0.5)
                fx = Xp - base
                w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1) ** 2, 0.5 * (fx - 0.5) ** 2]

                dg[p] = (ti.Matrix.identity(float, dim) + dt * C[p]) @ dg[p]

                mu = E_p / (2 * (1 + nu_p))
                la = E[p] * nu_p / ((1 + nu_p) * (1 - 2 * nu_p))
                stress = ti.Matrix([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
                plasticity_type = (material_p >> 16) & 0xFFFF
                # Calculate stress based on material type
                if material_p & 0xFFFF == 1:  # kirchhoff
                    U, sig, V = ti.svd(dg[p])
                    J_new = sig[0, 0] * sig[1, 1] * sig[2, 2]
                    if(plasticity_type==2):
                        h = ti.exp(50 * (1.0 - J_new))
                        mu ,la = mu * h, la * h
                    stress = 2 * mu * (dg[p] - U @ V.transpose()) @ dg[p].transpose() + \
                             ti.Matrix.identity(float, dim) * la * J_new * (J_new - 1)
                    stress = (-dt * p_vol_p * 4) * stress / dx**2
                else :  # neohookean
                    J = dg[p].determinant()
                    cauchy = mu * (dg[p] @ dg[p].transpose()) + ti.Matrix.identity(float, dim) * (la * ti.log(J) - mu)
                    stress = -(dt * p_vol_p * 4 / dx**2) * cauchy

                affine = stress + p_mass_p * C[p]

                for offset in ti.static(ti.grouped(ti.ndrange(*neighbour))):
                    dpos = (offset - fx) * dx
                    weight = 1.0
                    for i in ti.static(range(dim)):
                        weight *= w[offset[i]][i]
                    grid_v[base + offset] += weight * (p_mass_p * v[p] + affine @ dpos)
                    grid_m[base + offset] += weight * p_mass_p

    @ti.kernel
    def substep_p2marching(x: ti.types.ndarray(ndim=1),
                    point_color: ti.types.ndarray(ndim=1), marching_m: ti.types.ndarray(ndim=4),
                    p_mass: ti.types.ndarray(ndim=1), use_unified_material: ti.i32,
                    min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for p in x:
            dx_marching=1/marching_m.shape[1]
            if(x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):
                p_mass_p=0.0
                if(use_unified_material == 1):
                    p_mass_p=p_mass[0]
                else: 
                    p_mass_p=p_mass[p]
                color=point_color[p]
                Xp_marching = x[p] / dx_marching
                base_marching = int(Xp_marching - 0.5)
                fx_marching = Xp_marching - base_marching
                w_marching = [0.5 * (1.5 - fx_marching) ** 2, 0.75 - (fx_marching - 1) ** 2, 0.5 * (fx_marching - 0.5) ** 2]
                for offset in ti.static(ti.grouped(ti.ndrange(*neighbour))):
                    weight = 1.0
                    for i in ti.static(range(dim)):
                        weight *= w_marching[offset[i]][i]
                    marching_m[color,base_marching + offset] += weight * p_mass_p
    
    @ti.kernel
    def substep_apply_plasticity(dg: ti.types.ndarray(ndim=1), x: ti.types.ndarray(ndim=1), 
                                 E: ti.types.ndarray(ndim=1), nu: ti.types.ndarray(ndim=1), 
                                 material: ti.types.ndarray(ndim=1), SigY: ti.types.ndarray(ndim=1), 
                                 alpha: ti.types.ndarray(ndim=1), min_clamp: ti.types.ndarray(ndim=1), max_clamp: ti.types.ndarray(ndim=1),use_unified_material: ti.i32,
                                 min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        ratio = float(dg.shape[0]) / E.shape[0]
        for p in dg:
            p_ratio=int(p/ratio)
            # Check if the particle is within the specified simulation boundaries
            if(x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):

                E_p, nu_p, material_p, SigY_p, alpha_p, min_clamp_p, max_clamp_p = 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0
                if use_unified_material == 0:
                    E_p, nu_p, material_p, SigY_p, alpha_p, min_clamp_p, max_clamp_p = E[p_ratio], nu[p_ratio], material[p_ratio], SigY[p_ratio], alpha[p_ratio], min_clamp[p_ratio], max_clamp[p_ratio]
                else:
                    E_p, nu_p, material_p, SigY_p, alpha_p, min_clamp_p, max_clamp_p = E[0], nu[0], material[0], SigY[0], alpha[0], min_clamp[0], max_clamp[0]


                U, sig, V = ti.svd(dg[p])
                sig_vec = ti.Vector([sig[i, i] for i in range(dim)])
                
                # Common computations
                epsilon = ti.log(ti.abs(sig_vec))
                trace_epsilon = epsilon.sum()
                epsilon_hat = epsilon - trace_epsilon / 3 * ti.Vector([1.0, 1.0, 1.0])
                epsilon_hat_squared_norm = epsilon_hat.norm_sqr()
                epsilon_hat_norm = ti.sqrt(epsilon_hat_squared_norm)
                mu = E_p / (2 * (1 + nu_p))
                la= E_p * nu_p / ((1 + nu_p) * (1 - 2 * nu_p))
                delta_gamma = 0.0

                # Apply plasticity based on material type
                plasticity_type = (material_p >> 16) & 0xFFFF
                if plasticity_type == 2:  # Clamp plasticity type
                    for i in ti.static(range(dim)):
                        sig[i, i] = min(max(sig[i, i], 1 - min_clamp_p), 1 + max_clamp_p)
                    dg[p] = U @ sig @ V.transpose()
                else:
                    if plasticity_type == 1:  # Von_Mises plasticity type
                        delta_gamma = epsilon_hat_norm - SigY_p / (2 * mu)

                    elif plasticity_type == 3:  # Drucker_Prager plasticity type
                        if trace_epsilon <= 0:
                            delta_gamma = epsilon_hat_norm + (3 * la + 2 * mu) / (2 * mu) * trace_epsilon * alpha_p
                        else:
                            delta_gamma = epsilon_hat_norm

                    # Apply plasticity
                    Z = ti.Matrix.identity(float, 3)
                    if delta_gamma <= 0:
                        for i in range(dim):
                            Z[i, i] = sig_vec[i]
                    else:
                        H = epsilon - (delta_gamma / epsilon_hat_norm) * epsilon_hat
                        Em = ti.exp(H)
                        Z = ti.Matrix([[Em[0], 0, 0], [0, Em[1], 0], [0, 0, Em[2]]])
                    # Reconstruct the deformation gradient with the corrected singular values
                    dg[p] = U @ Z @ V.transpose()
    
    @ti.kernel
    def substep_calculate_signed_distance_field(obstacle_pos: ti.types.ndarray(ndim=1),
                                                sdf: ti.types.ndarray(ndim=3), obstacle_normals: ti.types.ndarray(ndim=3),
                                                obstacle_radius: ti.types.ndarray(ndim=1), 
                                                dx: ti.f32, dt: ti.f32, min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for I in ti.grouped(sdf):
            pos = I * dx + dx * 0.5
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                min_dist = float('inf')
                norm = ti.Vector([0.0, 0.0, 0.0])
                for j in obstacle_pos:
                    # Calculate the distance from the current grid cell to the sphere obstacle
                    dist = (pos - obstacle_pos[j]).norm() - obstacle_radius[j]
                    if dist < min_dist:
                        min_dist = dist
                        norm = (pos-obstacle_pos[j]).normalized()
                # Update the signed distance field and the obstacle normals
                sdf[I] = min_dist
                obstacle_normals[I] = norm
    
    @ti.kernel
    def substep_update_grid_v(grid_v: ti.types.ndarray(ndim=3),
                              sdf: ti.types.ndarray(ndim=3),
                              obstacle_normals: ti.types.ndarray(ndim=3),
                              obstacle_velocities: ti.types.ndarray(ndim=3),
                              gx: float, gy: float, gz: float, k: float, damping: float, friction_k: float,
                              v_allowed: ti.f32, dt: ti.f32, n_grid: ti.i32, dx: ti.f32, bound: ti.i32, use_sticky_cond: ti.i32,use_grid_force: ti.i32,use_standard_mpm_boundary: ti.i32,
                              min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z:ti.f32):
        for I in ti.grouped(grid_v):
            pos = I * dx + dx * 0.5     
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                # # Normalize the velocity if the grid cell has mass
                # if grid_m[I] > 0:
                #     grid_v[I] /= grid_m[I]
                # Apply gravitational force
                gravity = ti.Vector([gx, gy, gz])
                grid_v[I] += dt * gravity
                # Apply damping to reduce the velocity over time
                grid_v[I] *= ti.exp(-damping * dt)
                if sdf[I] < 0:
                    d = -sdf[I] # Calculate penetration depth
                    rel_v = grid_v[I] - obstacle_velocities[I] # Calculate grid relative velocity with respect to the obstacle
                    normal_v_norm = rel_v.dot(obstacle_normals[I])
                    normal_v = normal_v_norm * obstacle_normals[I] # Calculate the normal component of the relative velocity
                    if use_standard_mpm_boundary and normal_v_norm <= 0:
                        grid_v[I] = obstacle_velocities[I] # Set velocity to obstacle velocity
                    if use_grid_force:
                        pressure_force = obstacle_normals[I] * d / dt * k - normal_v # Calculate the velocity correction due to collision
                        tangent_direction = (rel_v - normal_v).normalized() # Determine the tangential direction of the relative velocity
                        friction_force = friction_k * pressure_force # Calculate the frictional force
                        grid_v[I] += pressure_force - friction_force * tangent_direction # Apply both the collision correction and the frictional force to the velocity
                # Enforce boundary conditions by setting velocity to zero if it points outside the grid at the boundaries
                cond = (I < bound) & (grid_v[I] < 0) | (I > n_grid - bound) & (grid_v[I] > 0)
                if (use_sticky_cond):
                    if cond[0] or cond[1] or cond[2]:
                        grid_v[I] = ti.Vector([0, 0, 0]) # Sticky boundary condition
                else:
                    grid_v[I] = ti.select(cond, 0, grid_v[I]) # Set normal velocity to 0
                # Sticky boundary condition by setting tangential velocity to zero if it points outside the grid at the boundaries
                # Limit the velocity w.r.t. CFL condition
                grid_v[I] = min(max(grid_v[I], -v_allowed), v_allowed)
    
    @ti.kernel
    def substep_apply_force_field(grid_v: ti.types.ndarray(ndim=3),
                              grid_m: ti.types.ndarray(ndim=3),
                              center_x: ti.f32, center_y: ti.f32, center_z: ti.f32, radius: ti.f32, force_x: ti.f32, force_y: ti.f32, force_z: ti.f32, 
                              min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        dx = 1 / grid_v.shape[0]
        for I in ti.grouped(grid_m):
            pos = I * dx + dx * 0.5
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                # Normalize the velocity if the grid cell has mass
                if grid_m[I] > 0:
                    grid_v[I] /= grid_m[I]
                # Apply force field to the grid cell if it is within the specified sphere
                if (pos - ti.Vector([center_x, center_y, center_z])).norm() < radius:
                    grid_v[I] = ti.Vector([force_x, force_y, force_z]) 
    
    @ti.kernel
    def substep_apply_force_field_two_hands(grid_v: ti.types.ndarray(ndim=3),
                                        grid_m: ti.types.ndarray(ndim=3),
                                        center_x1: ti.f32, center_y1: ti.f32, center_z1: ti.f32, radius1: ti.f32, force_x1: ti.f32, force_y1: ti.f32, force_z1: ti.f32,
                                        center_x2: ti.f32, center_y2: ti.f32, center_z2: ti.f32, radius2: ti.f32, force_x2: ti.f32, force_y2: ti.f32, force_z2: ti.f32,
                                        min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z:ti.f32):
        dx = 1 / grid_v.shape[0]
        for I in ti.grouped(grid_m):
            pos = I * dx + dx * 0.5
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                if grid_m[I] > 0:
                    grid_v[I] /= grid_m[I]
                if (pos - ti.Vector([center_x1, center_y1, center_z1])).norm() < radius1:
                    # grid_v[I] += ti.Vector([force_x1, force_y1, force_z1]) * dt
                    grid_v[I] = ti.Vector([force_x1, force_y1, force_z1])
                if (pos - ti.Vector([center_x2, center_y2, center_z2])).norm() < radius2:
                    # grid_v[I] += ti.Vector([force_x2, force_y2, force_z2]) * dt
                    grid_v[I] = ti.Vector([force_x2, force_y2, force_z2])
    
    @ti.kernel
    def substep_apply_rotate_force_field_two_hands(grid_v: ti.types.ndarray(ndim=3),
                                        grid_m: ti.types.ndarray(ndim=3),
                                        center_x1: ti.f32, center_y1: ti.f32, center_z1: ti.f32, radius1: ti.f32, axis_x1: ti.f32, axis_y1: ti.f32, axis_z1: ti.f32,
                                        center_x2: ti.f32, center_y2: ti.f32, center_z2: ti.f32, radius2: ti.f32, axis_x2: ti.f32, axis_y2: ti.f32, axis_z2: ti.f32,
                                        min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z:ti.f32):
        dx = 1 / grid_v.shape[0]
        for I in ti.grouped(grid_m):
            pos = I * dx + dx * 0.5
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                if (pos - ti.Vector([center_x1, center_y1, center_z1])).norm() < radius1:
                    force_1 = ti.Vector([axis_x1, axis_y1, axis_z1]).cross(pos - ti.Vector([center_x1, center_y1, center_z1]))
                    grid_v[I] += force_1
                if (pos - ti.Vector([center_x2, center_y2, center_z2])).norm() < radius2:
                    force_2 = ti.Vector([axis_x2, axis_y2, axis_z2]).cross(pos - ti.Vector([center_x2, center_y2, center_z2]))
                    grid_v[I] += force_2
    
    @ti.kernel
    def substep_update_grid_v_lerp(grid_v: ti.types.ndarray(ndim=3),
                              sdf: ti.types.ndarray(ndim=3),
                              obstacle_normals: ti.types.ndarray(ndim=3),
                              obstacle_velocities: ti.types.ndarray(ndim=3),
                              sdf_last: ti.types.ndarray(ndim=3),
                              obstacle_normals_last: ti.types.ndarray(ndim=3),
                              obstacle_velocities_last: ti.types.ndarray(ndim=3),
                              ratio: ti.f32,
                              gx: ti.f32, gy: ti.f32, gz: ti.f32, k: ti.f32, damping: ti.f32, friction_k: ti.f32,
                              v_allowed: ti.f32, dt: ti.f32, n_grid: ti.i32, dx: ti.f32, bound: ti.i32, use_sticky_cond: ti.i32,use_grid_force: ti.i32,use_standard_mpm_boundary: ti.i32, use_lerp: ti.i32,
                              min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z:ti.f32):
        for I in ti.grouped(grid_v):
            pos = I * dx + dx * 0.5
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                sdf_lerp = 0.0 
                obstacle_velocities_lerp = ti.Vector([0.0, 0.0, 0.0])
                obstacle_normals_lerp = ti.Vector([0.0, 0.0, 0.0])
                if use_lerp:
                    sdf_lerp = sdf_last[I] + (sdf[I] - sdf_last[I]) * ratio
                    obstacle_normals_lerp = obstacle_normals_last[I] + (obstacle_normals[I] - obstacle_normals_last[I]) * ratio
                    obstacle_velocities_lerp = obstacle_velocities_last[I] + (obstacle_velocities[I] - obstacle_velocities_last[I]) * ratio
                else:
                    sdf_lerp = sdf[I]
                    obstacle_normals_lerp = obstacle_normals[I]
                    obstacle_velocities_lerp = obstacle_velocities[I]
                # Apply gravitational force
                gravity = ti.Vector([gx, gy, gz]) 
                grid_v[I] += dt * gravity
                # Apply damping to reduce the velocity over time
                grid_v[I] *= ti.exp(-damping * dt)
                if sdf_lerp < 0:
                    d = -sdf_lerp # Calculate penetration depth
                    rel_v = grid_v[I] - obstacle_velocities_lerp # Calculate relative velocity with respect to the obstacle
                    normal_v_norm = rel_v.dot(obstacle_normals_lerp)
                    normal_v = normal_v_norm * obstacle_normals_lerp # Calculate the normal component of the relative velocity
                    if use_standard_mpm_boundary and normal_v_norm <= 0:
                        grid_v[I] = obstacle_velocities_lerp
                    pressure_force = obstacle_normals_lerp * d / dt * k - normal_v # Calculate the velocity correction due to collision
                    tangent_direction = (rel_v - normal_v).normalized() # Determine the tangential direction of the relative velocity
                    friction_force = friction_k * pressure_force # Calculate the frictional force
                    if use_grid_force:
                        grid_v[I] += pressure_force - friction_force * tangent_direction # Apply both the collision correction and the frictional force to the velocity
                # Enforce boundary conditions by setting velocity to zero if it points outside the grid at the boundaries
                cond = (I < bound) & (grid_v[I] < 0) | (I > n_grid - bound) & (grid_v[I] > 0)
                if (use_sticky_cond):
                    if cond[0] or cond[1] or cond[2]:
                        grid_v[I] = ti.Vector([0, 0, 0]) # Sticky boundary condition
                else:
                    grid_v[I] = ti.select(cond, 0, grid_v[I]) # Set normal velocity to 0
                # Sticky boundary condition by setting tangential velocity to zero if it points outside the grid at the boundaries
                # Limit the velocity w.r.t. CFL condition
                grid_v[I] = min(max(grid_v[I], -v_allowed), v_allowed)
                
    @ti.kernel
    def substep_g2p(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), C: ti.types.ndarray(ndim=1),grid_v: ti.types.ndarray(ndim=3),
                    dx:ti.f32,dt:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in x:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                Xp = x[p] / dx
                base = int(Xp - 0.5)
                fx = Xp - base
                w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1) ** 2, 0.5 * (fx - 0.5) ** 2]
                new_v = ti.zero(v[p])
                new_C = ti.zero(C[p])
                for offset in ti.static(ti.grouped(ti.ndrange(*neighbour))):
                    dpos = (offset - fx) * dx
                    weight = 1.0
                    for i in ti.static(range(dim)):
                        weight *= w[offset[i]][i]
                    g_v = grid_v[base + offset]
                    new_v += weight * g_v
                    new_C += 4 * weight * g_v.outer_product(dpos) / dx**2
                v[p] = new_v
                x[p] += dt * v[p]
                C[p] = new_C

    @ti.kernel
    def substep_update_dg(x: ti.types.ndarray(ndim=1),C: ti.types.ndarray(ndim=1),dg: ti.types.ndarray(ndim=1),dt:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in x:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                dg[p] = (ti.Matrix.identity(float, dim) + dt * C[p]) @ dg[p]

    @ti.kernel
    def substep_adjust_particle_hash(x: ti.types.ndarray(ndim=1), 
                                v: ti.types.ndarray(ndim=1),
                                hash_table: ti.types.ndarray(ndim=4),
                                segments_count_per_cell: ti.types.ndarray(ndim=3),
                                skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                skeleton_velocities: ti.types.ndarray(ndim=2),
                                skeleton_segments: ti.types.ndarray(ndim=2),
                                min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        # Calculate the grid cell size
        dx = 1 / segments_count_per_cell.shape[0]
        # Iterate over all particles
        for p in x:
            # Check if the current position is within the specified bounding box
            if(x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):
                Xp = x[p] / dx # Convert the particle's position to grid coordinates
                base = int(Xp - 0.5) # Calculate the base grid cell index
                min_dist = ti.Vector([float('inf'), float('inf'), float('inf')])
                min_seg_idx = -1
                min_r = 0.0
                # Iterate over all skeleton segments in the base grid cell
                for i in range(segments_count_per_cell[base]):
                    seg_idx = hash_table[base, i]
                    if seg_idx != -1:
                        start = skeleton_segments[seg_idx, 0]
                        end = skeleton_segments[seg_idx, 1]
                        # Calculate the distance from the particle to the skeleton segment
                        result = calculate_point_segment_distance(x[p], start, end)
                        dist = result.distance
                        r = result.b
                        # Check if the particle is within the capsule radius of the segment and closer than previous segments
                        if dist.norm() < skeleton_capsule_radius[seg_idx] and dist.norm() < min_dist.norm():
                            min_seg_idx = seg_idx
                            min_dist = dist # Update the minimum distance
                            min_r = r
                if min_seg_idx != -1:
                    # Adjust the particle's position to be outside the capsule radius
                    x[p] = x[p] + min_dist.normalized() * (skeleton_capsule_radius[min_seg_idx] - min_dist.norm())
                    # Update the particle's velocity to match the segment's velocity (with interpolation)
                    v[p] = skeleton_velocities[min_seg_idx, 0] * (1 - min_r) + skeleton_velocities[min_seg_idx, 1] * min_r
    
    @ti.kernel
    def substep_adjust_particle_mat(x: ti.types.ndarray(ndim=1),
                                    v: ti.types.ndarray(ndim=1),
                                    mat_primitives: ti.types.ndarray(ndim=2),
                                    mat_primitives_radius: ti.types.ndarray(ndim=2),
                                    mat_velocities: ti.types.ndarray(ndim=2),
                                    min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for p in x:
            if (x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):
                min_dist = float('inf')
                norm = ti.Vector([0.0, 0.0, 0.0])
                min_alpha = 0.0
                min_beta = 0.0
                min_primitive_idx = -1
                for i in range(mat_primitives.shape[0]):
                    dist = float('inf')
                    dist_normal = ti.Vector([0.0, 0.0, 0.0])
                    alpha, beta = 0.0, 0.0
                    if mat_primitives_radius[i, 2] == 0.0:
                        dist, dist_normal, alpha, beta = compute_point_cone_distance(mat_primitives[i, 0], mat_primitives_radius[i, 0],
                                                     mat_primitives[i, 1], mat_primitives_radius[i, 1],
                                                     x[p], 0.0,
                                                     x[p], 0.0)
                    else:
                        dist, dist_normal, alpha, beta = compute_point_slab_distance(mat_primitives[i, 0], mat_primitives_radius[i, 0],
                                                     mat_primitives[i, 1], mat_primitives_radius[i, 1],
                                                     mat_primitives[i, 2], mat_primitives_radius[i, 2],
                                                     x[p], 0.0)
                    # dist < 0 means the particle is inside the primitive
                    if dist < 0.0 and dist < min_dist:
                        min_dist = dist
                        norm = dist_normal
                        min_alpha = alpha
                        min_beta = beta
                        min_primitive_idx = i
                if min_primitive_idx != -1:
                    # Adjust the particle's position to be just outside the primitive
                    x[p] = x[p] + norm * min_dist * (-1)
                    # Update the particle's velocity to match the primitive velocity
                    if (mat_primitives_radius[min_primitive_idx, 2] == 0.0):
                        v[p] = mat_velocities[min_primitive_idx, 0] * min_alpha + mat_velocities[min_primitive_idx, 1] * (1 - min_alpha)
                    else:
                        v[p] = mat_velocities[min_primitive_idx, 0] * min_alpha + mat_velocities[min_primitive_idx, 1] * min_beta + mat_velocities[min_primitive_idx, 2] * (1 - min_alpha - min_beta) 
    
    @ti.kernel
    def substep_adjust_particle(x: ti.types.ndarray(ndim=1), 
                                v: ti.types.ndarray(ndim=1),
                                skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                skeleton_velocities: ti.types.ndarray(ndim=2),
                                skeleton_segments: ti.types.ndarray(ndim=2),
                                min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        # Iterate over all particles
        for p in x:
            # Check if the current position is within the specified bounding box
            if(x[p][0] > min_x and x[p][0] < max_x and x[p][1] > min_y and x[p][1] < max_y and x[p][2] > min_z and x[p][2] < max_z):
                min_dist = ti.Vector([float('inf'), float('inf'), float('inf')])
                min_r = 0.0
                min_seg_idx = -1
                # Iterate over all skeleton segments
                for i in range(skeleton_capsule_radius.shape[0]):
                    start = skeleton_segments[i, 0]
                    end = skeleton_segments[i, 1]
                    # Calculate the distance from the particle to the skeleton segment
                    result = calculate_point_segment_distance(x[p], start, end)
                    dist = result.distance
                    r = result.b
                    # Check if the particle is within the capsule radius of the segment and closer than previous segments
                    if dist.norm() < skeleton_capsule_radius[i] and dist.norm() < min_dist.norm():
                        min_seg_idx = i
                        min_r = r
                        min_dist = dist # Update the minimum distance
                if min_seg_idx != -1:
                    # Adjust the particle's position to be just outside the capsule radius
                    x[p] = x[p] + min_dist.normalized() * (skeleton_capsule_radius[min_seg_idx] - min_dist.norm())

                    #vectorized version keep in bound
                    x[p] = ti.Vector([min(max(x[p][i], 0.03), 0.97) for i in range(3)])

                    # Update the particle's velocity to match the segment's velocity (with interpolation)
                    v[p] = skeleton_velocities[min_seg_idx, 0] * (1 - min_r) + skeleton_velocities[min_seg_idx, 1] * min_r

    @ti.kernel
    def init_particles(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), dg: ti.types.ndarray(ndim=1), cube_size:ti.f32):
        # Init a cube
        for i in range(x.shape[0]):
            x[i] = [ti.random() * cube_size + (0.5-cube_size/2), ti.random() * cube_size+ (0.5-cube_size/2), ti.random() * cube_size+(0.5-cube_size/2)]
            dg[i] = ti.Matrix.identity(float, dim)
    
    @ti.kernel
    def init_sphere(x: ti.types.ndarray(ndim=1), dg: ti.types.ndarray(ndim=1), cube_size: ti.f32):
        # Init a sphere
        for i in range(x.shape[0]):
            dg[i] = ti.Matrix.identity(float, dim)
            while True:
                # Generate a random position within the unit cube for each dimension
                rand_pos = ti.Vector([ti.random() * 2 - 1 for _ in range(dim)])
                # Check if the random position is inside the unit sphere
                if rand_pos.norm() <= 1.0:
                    # Normalize the position, scale it by cube_size and translate
                    x[i] = rand_pos * cube_size + 0.5
                    dg[i] = ti.Matrix.identity(float, dim)
                    break
    
    @ti.kernel
    def init_cylinder(x: ti.types.ndarray(ndim=1), dg: ti.types.ndarray(ndim=1), cylinder_length: ti.f32, cylinder_radius: ti.f32):
        # Init a cylinder
        for i in range(x.shape[0]):
            while True:
                rand_y = (ti.random() * 2 - 1) * cylinder_radius
                rand_z = (ti.random() * 2 - 1) * cylinder_radius
                if rand_y ** 2 + rand_z ** 2 <= cylinder_radius ** 2:
                    x[i] = ti.Vector([0.5 + (ti.random() - 0.5) * cylinder_length, rand_y + 0.5, rand_z + 0.5])
                    dg[i] = ti.Matrix.identity(float, dim)
                    break
    
    @ti.kernel
    def init_torus(x: ti.types.ndarray(ndim=1), dg: ti.types.ndarray(ndim=1), torus_radius: ti.f32, torus_tube_radius: ti.f32):
        # Init a torus
        for i in range(x.shape[0]):
            rand_theta = ti.random() * 2 * pi
            rand_phi = ti.random() * 2 * pi
            rand_r = torus_tube_radius * (ti.random() * ti.cos(rand_phi)) + torus_radius  # Add random radial component
            x[i] = ti.Vector([rand_r * ti.cos(rand_theta) + 0.5, rand_r * ti.sin(rand_theta) + 0.5, torus_tube_radius * ti.sin(rand_phi) + 0.5])
            dg[i] = ti.Matrix.identity(float, dim)

    @ti.kernel
    def init_dg(dg: ti.types.ndarray(ndim=1)):
        for i in range(dg.shape[0]):
            dg[i] = ti.Matrix.identity(float, dim)
    
    @ti.kernel
    def substep_get_max_speed(v: ti.types.ndarray(ndim=1), x: ti.types.ndarray(ndim=1), max_speed: ti.types.ndarray(ndim=1),
                              min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for I in ti.grouped(v):
            if(x[I][0] > min_x and x[I][0] < max_x and x[I][1] > min_y and x[I][1] < max_y and x[I][2] > min_z and x[I][2] < max_z):
                max_speed[0] = ti.atomic_max(max_speed[0], v[I].norm())
    
    @ti.kernel
    def normalize_m(marching_m: ti.types.ndarray(ndim=4), max_m: ti.f32):
        for I in ti.grouped(marching_m):
            marching_m[I] /= max_m
    
    @ti.kernel
    def substep_fix_object(grid_v: ti.types.ndarray(ndim=3),
                           fix_center_x: ti.f32, fix_center_y: ti.f32, fix_center_z: ti.f32, 
                           fix_range: ti.f32):
        # Fix the object in place by setting the velocity to zero within the specified sphere range
        dx = 1 / grid_v.shape[0]
        for I in ti.grouped(grid_v):
            pos = I * dx + dx * 0.5
            dist_to_center = (pos - ti.Vector([fix_center_x, fix_center_y, fix_center_z])).norm()
            if dist_to_center < fix_range:
                grid_v[I] = ti.Vector([0.0, 0.0, 0.0])
    
    @ti.kernel
    def substep_calculate_mat_sdf(mat_primitives: ti.types.ndarray(ndim=2),
                                  mat_primitives_radius: ti.types.ndarray(ndim=2),
                                  mat_velocities: ti.types.ndarray(ndim=2),
                                  mat_sdf: ti.types.ndarray(ndim=3),
                                  obstacle_normals: ti.types.ndarray(ndim=3),
                                  obstacle_velocities: ti.types.ndarray(ndim=3),
                                  dx:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for I in ti.grouped(mat_sdf):
            pos = I * dx + dx * 0.5
            obstacle_velocities[I] = ti.Vector([0.0, 0.0, 0.0])
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                min_dist = float('inf')
                norm = ti.Vector([0.0, 0.0, 0.0])
                min_alpha = 0.0
                min_beta = 0.0
                for i in range(mat_primitives.shape[0]):
                    dist = float('inf')
                    dist_normal = ti.Vector([0.0, 0.0, 0.0])
                    alpha, beta = 0.0, 0.0
                    # Determine cone and slab by the radius of the third sphere
                    if mat_primitives_radius[i, 2] == 0.0:
                        # Compute distance from grid node to a Medial Cone
                        dist, dist_normal, alpha, beta = compute_point_cone_distance(mat_primitives[i, 0], mat_primitives_radius[i, 0],
                                                     mat_primitives[i, 1], mat_primitives_radius[i, 1],
                                                     pos, 0.0,
                                                     pos, 0.0)
                    else:
                        # Compute distance from grid node to a Medial Slab
                        dist, dist_normal, alpha, beta = compute_point_slab_distance(mat_primitives[i, 0], mat_primitives_radius[i, 0],
                                                     mat_primitives[i, 1], mat_primitives_radius[i, 1],
                                                     mat_primitives[i, 2], mat_primitives_radius[i, 2],
                                                     pos, 0.0)
                    if dist < min_dist:
                        min_dist = dist
                        norm = dist_normal
                        min_alpha = alpha
                        min_beta = beta
                        if mat_primitives_radius[i, 2] == 0.0:
                            # Cone velocity
                            obstacle_velocities[I] = mat_velocities[i, 0] * min_alpha + mat_velocities[i, 1] * (1 - min_alpha)
                        else:
                            # Slab velocity
                            obstacle_velocities[I] = mat_velocities[i, 0] * min_alpha + mat_velocities[i, 1] * min_beta + mat_velocities[i, 2] * (1 - min_alpha - min_beta) 
                mat_sdf[I] = min_dist
                obstacle_normals[I] = norm
    
    @ti.kernel
    def substep_calculate_hand_sdf(skeleton_segments: ti.types.ndarray(ndim=2),
                                   skeleton_velocities: ti.types.ndarray(ndim=2),
                                   hand_sdf: ti.types.ndarray(ndim=3),
                                   obstacle_normals: ti.types.ndarray(ndim=3),
                                   obstacle_velocities: ti.types.ndarray(ndim=3),
                                   skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                   dx:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for I in ti.grouped(hand_sdf):
            pos = I * dx + dx * 0.5
            obstacle_velocities[I] = ti.Vector([0.0, 0.0, 0.0])
            hand_sdf[I] = float('inf')
            obstacle_normals[I] = ti.Vector([0.0, 0.0, 0.0])
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                min_dist = float('inf')
                norm = ti.Vector([0.0, 0.0, 0.0])
                for i in range(skeleton_segments.shape[0]):
                    start = skeleton_segments[i, 0]
                    end = skeleton_segments[i, 1]
                    result = calculate_point_segment_distance(pos, start, end)
                    dist = result.distance # Vector from the segment to the grid point
                    r = result.b           # Parameter indicating the relative position along the segment
                    distance = dist.norm() - skeleton_capsule_radius[i] # Calculate the signed distance considering the capsule radius
                    if distance < min_dist:
                        min_dist = distance
                        norm = dist.normalized() # Normalize the distance vector to get the normal
                        # Linearly interpolate the velocity based on the relative position 'r'
                        obstacle_velocities[I] = skeleton_velocities[i, 0] * (1 - r) + skeleton_velocities[i, 1] * r
                hand_sdf[I] = min_dist
                obstacle_normals[I] = norm
    
    @ti.kernel
    def substep_calculate_hand_hash(skeleton_segments: ti.types.ndarray(ndim=2),
                                    skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                    n_grid: ti.i32,
                                    hash_table: ti.types.ndarray(ndim=4),
                                    segments_count_per_cell: ti.types.ndarray(ndim=3)):
        # Reset the segments count and hash table
        for I in ti.grouped(segments_count_per_cell):
            segments_count_per_cell[I] = 0
            for l in range(hash_table.shape[3]):
                hash_table[I, l] = -1    
        # Calculate the hash table
        for i in range(skeleton_segments.shape[0]):
            seg_start = skeleton_segments[i, 0]
            seg_end = skeleton_segments[i, 1]
            # Compute the bounding box for the segment considering the capsule radius
            min_bb = ti.min(seg_start, seg_end) - skeleton_capsule_radius[i]
            max_bb = ti.max(seg_start, seg_end) + skeleton_capsule_radius[i]
            # Convert the bounding box coordinates to grid cell indices
            min_cell = get_hash(min_bb, n_grid)
            max_cell = get_hash(max_bb, n_grid)
            for I in ti.grouped(ti.ndrange((min_cell[0], max_cell[0] + 1), 
                                           (min_cell[1], max_cell[1] + 1), 
                                           (min_cell[2], max_cell[2] + 1))):
                # Check if the current cell is within the valid grid range
                if (0 <= I < n_grid).all():
                    idx = ti.atomic_add(segments_count_per_cell[I], 1)
                    # Store the segment index in the hash table
                    hash_table[I, idx] = i
    
    @ti.kernel
    def substep_calculate_hand_sdf_hash(skeleton_segments: ti.types.ndarray(ndim=2),
                                        skeleton_velocities: ti.types.ndarray(ndim=2),
                                        hand_sdf: ti.types.ndarray(ndim=3),
                                        obstacle_normals: ti.types.ndarray(ndim=3),
                                        obstacle_velocities: ti.types.ndarray(ndim=3),
                                        skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                        dx: ti.f32,
                                        hash_table: ti.types.ndarray(ndim=4),
                                        segments_count_per_cell: ti.types.ndarray(ndim=3),
                                        min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        # calculate hand sdf for each grid cell
        for I in ti.grouped(hand_sdf):
            pos = I * dx + dx * 0.5
            obstacle_velocities[I] = ti.Vector([0.0, 0.0, 0.0])
            hand_sdf[I] = float('inf')
            obstacle_normals[I] = ti.Vector([0.0, 0.0, 0.0])
            # Check if the current position is within the specified bounding box
            if pos[0] > min_x and pos[0] < max_x and pos[1] > min_y and pos[1] < max_y and pos[2] > min_z and pos[2] < max_z:
                min_dist = float('inf')
                norm = ti.Vector([0.0, 0.0, 0.0])
                for i in range(segments_count_per_cell[I]):
                    # Get the index of the segment from the hash table
                    segment_idx = hash_table[I, i]
                    if segment_idx != -1:
                        seg_start = skeleton_segments[segment_idx, 0]
                        seg_end = skeleton_segments[segment_idx, 1]
                        result = calculate_point_segment_distance(pos, seg_start, seg_end)
                        dist = result.distance # Vector from the segment to the grid point
                        r = result.b           # Parameter indicating the relative position along the segment
                        distance = dist.norm() - skeleton_capsule_radius[segment_idx] # Calculate the signed distance considering the capsule radius
                        if distance < min_dist:
                            min_dist = distance
                            norm = dist.normalized() # Normalize the distance vector to get the normal
                             # Linearly interpolate the velocity based on the relative position r
                            obstacle_velocities[I] = skeleton_velocities[segment_idx, 0] * (1-r) + skeleton_velocities[segment_idx, 1] * r
                hand_sdf[I] = min_dist
                obstacle_normals[I] = norm

    
    # region - Gaussian Kernel Functions
    @ti.kernel
    def init_sample_gaussian_data(x_gaussian: ti.types.ndarray(ndim=1), x: ti.types.ndarray(ndim=1)):
        n_gaussian = x_gaussian.shape[0]
        n_x = x.shape[0]
        ratio = float(n_gaussian) / n_x  # 确保 ratio 是 float 类型
        for i in x:
            index = int(i * ratio)  # 将 i * ratio 转换为整数
            x[i] = x_gaussian[index]

    @ti.kernel
    def init_gaussian_data(init_rotation:ti.types.ndarray(ndim=1),
                           init_scale:ti.types.ndarray(ndim=1),
                           other_data:ti.types.ndarray(ndim=1)):
        for i in other_data:
            init_rotation[i] = DecodeRotation(DecodePacked_10_10_10_2(ti.bit_cast(other_data[i][0], ti.u32)))
            init_scale[i] = ti.Vector([other_data[i][1], other_data[i][2], other_data[i][3]])
    
    @ti.kernel
    def substep_update_gaussian_data(init_rotation: ti.types.ndarray(ndim=1), init_scale: ti.types.ndarray(ndim=1),
                                     dg: ti.types.ndarray(ndim=1),
                                     other_data: ti.types.ndarray(ndim=1), init_sh:ti.types.ndarray(ndim=1), sh:ti.types.ndarray(ndim=1),
                                     x:ti.types.ndarray(ndim=1),
                                     min_x: ti.f32, max_x: ti.f32, min_y: ti.f32, max_y: ti.f32, min_z: ti.f32, max_z: ti.f32):
        for i in dg:
            if(x[i][0] > min_x and x[i][0] < max_x and x[i][1] > min_y and x[i][1] < max_y and x[i][2] > min_z and x[i][2] < max_z):
                dg_matrix = dg[i]
                # SVD 分解 dg_matrix = U * Sigma * V^T
                U, sig, V = ti.svd(dg_matrix)
                R = U @ V.transpose()  # 旋转矩阵
                S = V @ sig @ V.transpose()  # 尺度矩阵
                
                # 将旋转矩阵 R 转换为四元数 rot
                rot = rotation_matrix_to_quaternion(R)
    
                # 用 R 作用在 init_rotation 上
                q_init = init_rotation[i]
                rot = quaternion_multiply(rot, q_init)
    
                # 用 S 作用在 init_scale 上
                scale = S @ init_scale[i]
    
                # 编码 rot 并填入 other_data
                packed_rot = PackSmallest3Rotation(rot)
                encoded_rot = EncodeQuatToNorm10(packed_rot)
                # other_data containts scale and rotation
                other_data[i][0] = ti.bit_cast(encoded_rot, ti.f32)
                other_data[i][1]  = scale[0]
                other_data[i][2]  = scale[1]
                other_data[i][3]  = scale[2]

                # Rotate spherical harmonics
                sh[i] = RotateSH(R,init_sh[i])

    @ti.kernel
    def scale_to_unit_cube(x: ti.types.ndarray(ndim=1), other_data: ti.types.ndarray(ndim=1), eps:ti.f32):
        # Calculate the bounding box of all particls
        min_val = ti.Vector([float('inf'), float('inf'), float('inf')])
        max_val = ti.Vector([float('-inf'), float('-inf'), float('-inf')])
        for j in range(1):  
            for i in x:
                # Update min_val for the bounding box
                if x[i][0] < min_val[0]:
                    min_val[0] = x[i][0]
                if x[i][1] < min_val[1]:
                    min_val[1] = x[i][1]
                if x[i][2] < min_val[2]:
                    min_val[2] = x[i][2]
                # Update max_val for the bounding box
                if x[i][0] > max_val[0]:
                    max_val[0] = x[i][0]
                if x[i][1] > max_val[1]:
                    max_val[1] = x[i][1]
                if x[i][2] > max_val[2]:
                    max_val[2] = x[i][2]
        center = (min_val + max_val) / 2.0
        size = max_val - min_val
        # Calculate the scaling factor based on the largest dimension
        scaleFactor = (1.0 - 2 * eps) / ti.max(size[0], size[1], size[2])
        # Define the center of the unit cube
        new_center = ti.Vector([0.5, 0.5, 0.5])
        for i in x:
            # Scale and translate
            x[i] = (x[i] - center) * scaleFactor + new_center
            # Scale the corresponding values in other_data
            other_data[i][1] = scaleFactor * other_data[i][1]
            other_data[i][2] = scaleFactor * other_data[i][2]
            other_data[i][3] = scaleFactor * other_data[i][3]
    
    @ti.kernel
    def recenter_to_unit_cube(x_gaussian: ti.types.ndarray(ndim=1), x: ti.types.ndarray(ndim=1), other_data: ti.types.ndarray(ndim=1), eps:ti.f32):
        # Calculate the bounding box of all particls
        min_val = ti.Vector([float('inf'), float('inf'), float('inf')])
        max_val = ti.Vector([float('-inf'), float('-inf'), float('-inf')])
        for j in range(1):  
            for i in x_gaussian:
                # Update min_val for the bounding box
                if x_gaussian[i][0] < min_val[0]:
                    min_val[0] = x_gaussian[i][0]
                if x_gaussian[i][1] < min_val[1]:
                    min_val[1] = x_gaussian[i][1]
                if x_gaussian[i][2] < min_val[2]:
                    min_val[2] = x_gaussian[i][2]
                # Update max_val for the bounding box
                if x_gaussian[i][0] > max_val[0]:
                    max_val[0] = x_gaussian[i][0]
                if x_gaussian[i][1] > max_val[1]:
                    max_val[1] = x_gaussian[i][1]
                if x_gaussian[i][2] > max_val[2]:
                    max_val[2] = x_gaussian[i][2]
        center = (min_val + max_val) / 2.0
        size = max_val - min_val
        # Calculate the scaling factor based on the largest dimension
        scaleFactor = (1.0 - 2 * eps) / ti.max(size[0], size[1], size[2])
        # Define the center of the unit cube
        new_center = ti.Vector([0.5, 0.5, 0.5])
        for i in x:
            # Scale and translate
            x[i] = (x[i] - center) * scaleFactor + new_center
            # x[i] = x[i] - center + new_center
            # Scale the corresponding values in other_data
            other_data[i][1] = scaleFactor * other_data[i][1]
            other_data[i][2] = scaleFactor * other_data[i][2]
            other_data[i][3] = scaleFactor * other_data[i][3]
        for i in x_gaussian:
            # Scale and translate
            x_gaussian[i] = (x_gaussian[i] - center) * scaleFactor + new_center
            # x_gaussian[i] = x_gaussian[i] - center + new_center
            # Scale the corresponding values in other_data
            other_data[i][1] = scaleFactor * other_data[i][1]
            other_data[i][2] = scaleFactor * other_data[i][2]
            other_data[i][3] = scaleFactor * other_data[i][3]
    
    @ti.kernel
    def copy_array_1dim1(src: ti.types.ndarray(ndim=1), dst:ti.types.ndarray(ndim=1)):
        for I in ti.grouped(src):
            dst[I] = src[I]
    
    @ti.kernel
    def copy_array_1dim3(src: ti.types.ndarray(ndim=1), dst:ti.types.ndarray(ndim=1)):
        for I in ti.grouped(src):
            dst[I] = src[I]
    
    @ti.kernel
    def copy_array_1dim1I(src: ti.types.ndarray(ndim=1), dst:ti.types.ndarray(ndim=1)):
        for I in ti.grouped(src):
            dst[I] = src[I]

    @ti.kernel
    def copy_array_3dim1(src: ti.types.ndarray(ndim=3), dst:ti.types.ndarray(ndim=3)):
        for I in ti.grouped(src):
            dst[I] = src[I]
    
    @ti.kernel
    def copy_array_3dim3(src: ti.types.ndarray(ndim=3), dst:ti.types.ndarray(ndim=3)):
        for I in ti.grouped(src):
            dst[I] = src[I]
    
    @ti.kernel
    def transform_and_merge(x1: ti.types.ndarray(ndim=1), 
                            x2: ti.types.ndarray(ndim=1), 
                            x3: ti.types.ndarray(ndim=1), 
                            mat2: ti.types.ndarray(ndim=2), 
                            mat3: ti.types.ndarray(ndim=2)):
        n2 = x2.shape[0]
        n3 = x3.shape[0]

        for i in range(n2):
            # 将 x2 的顶点转换到世界坐标系并写入 x1
            x1[i] = multiply_point(mat2, x2[i])

        for i in range(n3):
            # 将 x3 的顶点转换到世界坐标系并写入 x1
            x1[i + n2] = multiply_point(mat3, x3[i])
    # endregion

    @ti.kernel
    def substep_squeeze_particles_circle(x: ti.types.ndarray(ndim=1), p_mass: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), target_p_mass: ti.f32,
                                         center_x: ti.f32, center_y: ti.f32, center_z: ti.f32, 
                                         velocity_x: ti.f32, velocity_y: ti.f32, velocity_z: ti.f32, 
                                         radius: ti.f32, dt: ti.f32,
                                         starting_index: ti.i32, ending_index: ti.i32):
        # 如果center在边界内才进行操作
        if center_x > 0.03 and center_x < 0.97 and center_y > 0.03 and center_y < 0.97 and center_z > 0.03 and center_z < 0.97:
            # 计算圆柱高度（速度大小乘以dt）
            velocity_norm = ti.Vector([velocity_x, velocity_y, velocity_z])
            speed = velocity_norm.norm()
            cylinder_height = speed * dt
            if speed > 0:
                velocity_norm = velocity_norm / speed
            else:
                velocity_norm = ti.Vector([0.0, 0.0, 1.0])  # 默认方向
            
            # 构建正交基底
            # 选择一个与 velocity_norm 不平行的向量
            arbitrary = ti.Vector([1.0, 0.0, 0.0])
            if ti.abs(velocity_norm.dot(arbitrary)) > 0.99:
                arbitrary = ti.Vector([0.0, 1.0, 0.0])
            x_axis = velocity_norm.cross(arbitrary).normalized()
            y_axis = velocity_norm.cross(x_axis).normalized()

            for p in range(starting_index, ending_index):
                p_mass[p] = target_p_mass
                x_0 ,y_0, z_0 = 0.0, 0.0, 0.0
                # 粒子位置随机均匀分部在圆柱内,圆柱的底面与velocity方向垂直，圆柱的高度为速度大小乘以dt
                while True:
                    x_0 = ti.random() * 2 - 1
                    y_0 = ti.random() * 2 - 1
                    z_0 = ti.random()
                    if x_0 * x_0 + y_0 * y_0 <= 1:
                        break
                # 将生成的点转到速度方向
                x[p] = (x_0 * x_axis + y_0 * y_axis) * radius + ti.Vector([center_x, center_y, center_z]) + velocity_norm * z_0 * cylinder_height 
                # 限制在边界内
                x[p] = ti.Vector([min(max(x[p][i], 0.03), 0.97) for i in range(3)])
                v[p] = velocity_norm * speed

    @ti.kernel
    def substep_squeeze_particles_square(x: ti.types.ndarray(ndim=1), p_mass: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), target_p_mass: ti.f32,
                                         center_x: ti.f32, center_y: ti.f32, center_z: ti.f32, 
                                         velocity_x: ti.f32, velocity_y: ti.f32, velocity_z: ti.f32, 
                                         radius: ti.f32, dt: ti.f32,
                                         starting_index: ti.i32, ending_index: ti.i32):
        # 如果center在边界内才进行操作
        if center_x > 0.03 and center_x < 0.97 and center_y > 0.03 and center_y < 0.97 and center_z > 0.03 and center_z < 0.97:
            # 计算圆柱高度（速度大小乘以dt）
            velocity_norm = ti.Vector([velocity_x, velocity_y, velocity_z])
            speed = velocity_norm.norm()
            cylinder_height = speed * dt
            if speed > 0:
                velocity_norm = velocity_norm / speed
            else:
                velocity_norm = ti.Vector([0.0, 0.0, 1.0])  # 默认方向
    
            # 构建正交基底
            # 选择一个与 velocity_norm 不平行的向量
            arbitrary = ti.Vector([1.0, 0.0, 0.0])
            if ti.abs(velocity_norm.dot(arbitrary)) > 0.99:
                arbitrary = ti.Vector([0.0, 1.0, 0.0])
            x_axis = velocity_norm.cross(arbitrary).normalized()
            y_axis = velocity_norm.cross(x_axis).normalized()
    
            for p in range(starting_index, ending_index):
                p_mass[p] = target_p_mass
                x_0 ,y_0, z_0 = 0.0, 0.0, 0.0
                # 粒子位置随机均匀分部在方形内，高度为速度大小乘以dt
                x_0 = ti.random() * 2 - 1
                y_0 = ti.random() * 2 - 1
                z_0 = ti.random()
                # 将生成的点转到速度方向
                x[p] = (x_0 * x_axis + y_0 * y_axis) * radius + ti.Vector([center_x, center_y, center_z]) + velocity_norm * z_0 * cylinder_height 
                # 限制在边界内
                x[p] = ti.Vector([min(max(x[p][i], 0.03), 0.97) for i in range(3)])
                v[p] = velocity_norm * speed                
    
    @ti.kernel
    def substep_squeeze_particles_star(x: ti.types.ndarray(ndim=1), p_mass: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), target_p_mass: ti.f32,
                                       center_x: ti.f32, center_y: ti.f32, center_z: ti.f32,
                                       velocity_x: ti.f32, velocity_y: ti.f32, velocity_z: ti.f32,
                                       radius: ti.f32, dt: ti.f32,
                                       starting_index: ti.i32, ending_index: ti.i32):
        #如果center在边界内才进行操作
        if center_x > 0.03 and center_x < 0.97 and center_y > 0.03 and center_y < 0.97 and center_z > 0.03 and center_z < 0.97:
            # 计算圆柱高度（速度大小乘以dt）
            velocity_norm = ti.Vector([velocity_x, velocity_y, velocity_z])
            speed = velocity_norm.norm()
            cylinder_height = speed * dt
            if speed > 0:
                velocity_norm = velocity_norm / speed
            else:
                velocity_norm = ti.Vector([0.0, 0.0, 1.0])  # 默认方向
    
            # 构建正交基底
            # 选择一个与 velocity_norm 不平行的向量
            arbitrary = ti.Vector([1.0, 0.0, 0.0])
            if ti.abs(velocity_norm.dot(arbitrary)) > 0.99:
                arbitrary = ti.Vector([0.0, 1.0, 0.0])
            x_axis = velocity_norm.cross(arbitrary).normalized()
            y_axis = velocity_norm.cross(x_axis).normalized()
    
            for p in range(starting_index, ending_index):
                p_mass[p] = target_p_mass
                x_0 ,y_0, z_0 = 0.0, 0.0, 0.0
                # 粒子位置随机均匀分部在圆柱内,圆柱的底面与velocity方向垂直，圆柱的高度为速度大小乘以dt
                while True:
                    x_0 = ti.random() * 2 - 1
                    y_0 = ti.random() * 2 - 1
                    z_0 = ti.random()
                    theta = ti.atan2(y_0, x_0)
                    # theta取与最近的坐标轴的夹角
                    theta %= pi /2
                    if theta > pi / 4:
                        theta = pi / 2 - theta
                    r = ti.sqrt(x_0 * x_0 + y_0 * y_0)
                    max_r=ti.sin(pi/12)/ti.sin(theta+pi/12)
                    if r <= max_r:
                        break
                # 将生成的点转到速度方向
                x[p] = (x_0 * x_axis + y_0 * y_axis) * radius + ti.Vector([center_x, center_y, center_z]) + velocity_norm * z_0 * cylinder_height 
                # 限制在边界内
                x[p] = ti.Vector([min(max(x[p][i], 0.03), 0.97) for i in range(3)])
                v[p] = velocity_norm * speed        

    # Medial Axis Transform (MAT) for Shape Appreximation
    mat_sdf = ti.ndarray(ti.f32, shape=(n_grid, n_grid, n_grid))
    mat_primitives = ti.Vector.ndarray(3, ti.f32, shape=(60, 3))
    mat_primitives_radius = ti.ndarray(ti.f32, shape=(60, 3))
    mat_velocities = ti.Vector.ndarray(3, ti.f32, shape=(60, 3))
    
    # Hash table
    hand_sdf = ti.ndarray(ti.f32, shape=(n_grid, n_grid, n_grid))
    obstacle_normals = ti.Vector.ndarray(3, ti.f32, shape=(n_grid, n_grid, n_grid))
    skeleton_segments = ti.Vector.ndarray(3, ti.f32, shape=(24, 2))
    skeleton_velocities = ti.Vector.ndarray(3, ti.f32, shape=(24, 2))
    skeleton_capsule_radius = ti.ndarray(ti.f32, shape=(24))
    
    hash_table = ti.ndarray(ti.i32, shape=(n_grid, n_grid, n_grid, skeleton_segments.shape[0]))
    segments_count_per_cell = ti.ndarray(ti.i32, shape=(n_grid, n_grid, n_grid))
    
    x = ti.Vector.ndarray(3, ti.f32, shape=(n_particles))
    v = ti.Vector.ndarray(3, ti.f32, shape=(n_particles))
    C = ti.Matrix.ndarray(3, 3, ti.f32, shape=(n_particles))
    dg = ti.Matrix.ndarray(3, 3, ti.f32, shape=(n_particles))
    grid_v = ti.Vector.ndarray(3, ti.f32, shape=(n_grid, n_grid, n_grid))
    grid_m = ti.ndarray(ti.f32, shape=(n_grid, n_grid, n_grid))
    sdf = ti.ndarray(ti.f32, shape=(n_grid, n_grid, n_grid))
    obstacle_velocities = ti.Vector.ndarray(3, ti.f32, shape=(n_grid, n_grid, n_grid))

    # block_size = 8
    # root= ti.root.pointer(ti.ijk, (n_grid // block_size, n_grid // block_size, n_grid // block_size))
    # root.dense(ti.ijk, (block_size, block_size, block_size)).place(grid_v, grid_m, sdf, obstacle_velocities,hand_sdf, obstacle_normals)
    
    obstacle_pos = ti.Vector.ndarray(3, ti.f32, shape=(1))
    obstacle_pos[0] = ti.Vector([0.25, 0.25, 0.25])
    obstacle_radius = ti.ndarray(ti.f32, shape=(1))
    max_speed = ti.ndarray(ti.f32, shape=(1))
    obstacle_radius[0] = 0
    init_scale = ti.Vector.ndarray(3, ti.f32, shape=(n_particles))
    init_rotation = ti.Vector.ndarray(4, ti.f32, shape=(n_particles))
    other_data = ti.Vector.ndarray(4,ti.f32, shape=(n_particles))
    init_sh = ti.Matrix.ndarray(16,3,ti.f32, shape=(n_particles))
    sh = ti.Matrix.ndarray(16,3,ti.f32, shape=(n_particles))

    # Material
    E = ti.ndarray(ti.f32, shape=(n_particles))
    nu = ti.ndarray(ti.f32, shape=(n_particles))
    material = ti.ndarray(ti.i32, shape=(n_particles))

    min_clamp = ti.ndarray(ti.f32, shape=(n_particles))
    max_clamp = ti.ndarray(ti.f32, shape=(n_particles))
    SigY = ti.ndarray(ti.f32, shape=(n_particles))
    alpha = ti.ndarray(ti.f32, shape=(n_particles))

    p_mass = ti.ndarray(ti.f32, shape=(n_particles))
    p_vol = ti.ndarray(ti.f32, shape=(n_particles))

    max_m=_p_mass*particle_per_grid

    point_color = ti.Vector.ndarray(1, ti.i32, shape=(n_particles))
    marching_m = ti.ndarray(ti.f32, shape=(3,n_grid, n_grid, n_grid))

    # Boundary
    min_x = 0.1
    max_x = 0.9
    min_y = 0.1
    max_y = 0.9
    min_z = 0.1
    max_z = 0.9
    
    # Transform
    mat2 = ti.ndarray(ti.f32, shape=(4, 4))
    mat3 = ti.ndarray(ti.f32, shape=(4, 4))
    
    def substep():
        substep_reset_grid(grid_v, grid_m,marching_m, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_squeeze_particles_circle(x, p_mass, v,_p_mass, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1,dt, 0, 1)
        substep_squeeze_particles_square(x, p_mass, v,_p_mass, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1,dt, 0, 1)
        substep_squeeze_particles_star(x, p_mass, v,_p_mass, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1,dt, 0, 1)
        substep_p2g_multi(x, v, C,  dg, grid_v, grid_m, E, nu, material, p_vol, p_mass, dx, dt, True,
                           min_x, max_x, min_y, max_y, min_z, max_z)
        substep_p2marching(x, point_color, marching_m, p_mass,True,
                            min_x, max_x, min_y, max_y, min_z, max_z)
        substep_calculate_signed_distance_field(obstacle_pos,sdf,obstacle_velocities,obstacle_radius,dx,dt,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_calculate_hand_sdf(skeleton_segments, skeleton_velocities, hand_sdf, obstacle_normals, obstacle_velocities, skeleton_capsule_radius, dx, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_calculate_hand_hash(skeleton_segments, skeleton_capsule_radius, n_grid, hash_table, segments_count_per_cell)
        substep_calculate_hand_sdf_hash(skeleton_segments, skeleton_velocities, hand_sdf, obstacle_normals, obstacle_velocities, skeleton_capsule_radius, dx, hash_table, segments_count_per_cell, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_apply_force_field(grid_v,grid_m,0.5,0.5,0.5,0.2,0,0,0,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_apply_force_field_two_hands(grid_v,grid_m,0.5,0.5,0.5,0.2,0,0,0,0.5,0.5,0.5,0.2,0,0,0,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_apply_rotate_force_field_two_hands(grid_v,grid_m,0.5,0.5,0.5,0.2,0,0,0,0.5,0.5,0.5,0.2,0,0,0,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_update_grid_v(grid_v, hand_sdf, obstacle_normals, obstacle_velocities, gx, gy, gz, k, damping, friction_k, v_allowed, dt, n_grid, dx, bound, use_sticky_cond,True,False,
                               min_x, max_x, min_y, max_y, min_z, max_z)
        substep_update_grid_v_lerp(grid_v, hand_sdf, obstacle_normals, obstacle_velocities,hand_sdf, obstacle_normals, obstacle_velocities,0.5, gx, gy, gz, k, damping, friction_k, v_allowed, dt, n_grid, dx, bound, use_sticky_cond, True,False,True,
                                   min_x, max_x, min_y, max_y, min_z, max_z)
        substep_fix_object(grid_v, fix_center_x=0.5, fix_center_y=0.5, fix_center_z=0.5, fix_range=1)
        substep_g2p(x, v, C,  grid_v, dx, dt, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_update_dg(x, C,  dg,  dt, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_adjust_particle_hash(x, v, hash_table, segments_count_per_cell, skeleton_capsule_radius, skeleton_velocities, skeleton_segments, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_adjust_particle(x, v, skeleton_capsule_radius, skeleton_velocities, skeleton_segments, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_apply_plasticity(dg, x,E,nu, material,SigY,alpha,min_clamp,max_clamp,True,
                                  min_x, max_x, min_y, max_y, min_z, max_z)
        substep_get_max_speed(v,x, max_speed, min_x, max_x, min_y, max_y, min_z, max_z)
        copy_array_1dim1(E, E)
        copy_array_1dim3(v, v)
        copy_array_1dim1I(material, material)
        copy_array_3dim1(sdf, sdf)
        copy_array_3dim3(obstacle_normals, obstacle_normals)
        normalize_m(marching_m, max_m)
        substep_calculate_mat_sdf(mat_primitives, mat_primitives_radius, mat_velocities, mat_sdf, obstacle_normals, obstacle_velocities, dx, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_adjust_particle_mat(x, v, mat_primitives, mat_primitives_radius, mat_velocities, min_x, max_x, min_y, max_y, min_z, max_z)

    def run_aot():
        mod = ti.aot.Module(arch)
        mod.add_kernel(substep_reset_grid, template_args={'grid_v': grid_v, 'grid_m': grid_m, 'marching_m': marching_m})
        mod.add_kernel(substep_g2p, template_args={'x': x, 'v': v, 'C': C, 'grid_v': grid_v})
        mod.add_kernel(substep_update_dg, template_args={'x': x, 'C': C, 'dg': dg})
        mod.add_kernel(init_particles, template_args={'x': x, 'v': v, 'dg': dg})
        mod.add_kernel(init_sphere, template_args={'x': x, 'dg': dg})
        mod.add_kernel(init_cylinder, template_args={'x': x, 'dg': dg})
        mod.add_kernel(init_torus, template_args={'x': x, 'dg': dg})
        mod.add_kernel(substep_calculate_signed_distance_field, template_args={'obstacle_pos': obstacle_pos, 'sdf': sdf, 'obstacle_normals': obstacle_normals, 'obstacle_radius': obstacle_radius})
        mod.add_kernel(substep_apply_force_field, template_args={'grid_v': grid_v, 'grid_m': grid_m})
        mod.add_kernel(substep_apply_force_field_two_hands, template_args={'grid_v': grid_v, 'grid_m': grid_m})
        mod.add_kernel(substep_apply_rotate_force_field_two_hands, template_args={'grid_v': grid_v, 'grid_m': grid_m})
        mod.add_kernel(substep_update_grid_v, template_args={'grid_v': grid_v,  'sdf': sdf, 'obstacle_normals': obstacle_normals, 'obstacle_velocities': obstacle_velocities})
        mod.add_kernel(substep_update_grid_v_lerp, template_args={'grid_v': grid_v,  'sdf': sdf, 'obstacle_normals': obstacle_normals, 'obstacle_velocities': obstacle_velocities,'sdf_last': sdf, 'obstacle_normals_last': obstacle_normals, 'obstacle_velocities_last': obstacle_velocities})
        mod.add_kernel(substep_get_max_speed, template_args={'v': v, 'x': x,'max_speed': max_speed})
        mod.add_kernel(init_dg, template_args={'dg': dg})
        mod.add_kernel(substep_p2g_multi, template_args={'x': x, 'v': v, 'C': C,  'dg': dg, 'grid_v': grid_v, 'grid_m': grid_m,'E':E,'nu':nu,'material':material,'p_vol':p_vol,'p_mass':p_mass})
        mod.add_kernel(substep_p2marching, template_args={'x': x, 'point_color': point_color, 'marching_m': marching_m, 'p_mass': p_mass})
        mod.add_kernel(substep_apply_plasticity, template_args={'dg': dg,'x': x,'material':material,"E":E,"nu":nu,"SigY":SigY,"alpha":alpha,"min_clamp":min_clamp,"max_clamp":max_clamp})
        
        # hand sdf functions
        mod.add_kernel(substep_calculate_mat_sdf, template_args={'mat_primitives': mat_primitives, 'mat_primitives_radius': mat_primitives_radius, 'mat_velocities': mat_velocities, 'mat_sdf': mat_sdf, 'obstacle_normals': obstacle_normals, 'obstacle_velocities': obstacle_velocities})
        mod.add_kernel(substep_calculate_hand_sdf, template_args={'skeleton_segments': skeleton_segments, 'skeleton_velocities': skeleton_velocities, 'hand_sdf': hand_sdf, 'obstacle_normals': obstacle_normals, 'obstacle_velocities': obstacle_velocities, 'skeleton_capsule_radius': skeleton_capsule_radius})
        mod.add_kernel(substep_calculate_hand_sdf_hash, template_args={'skeleton_segments': skeleton_segments, 'skeleton_velocities': skeleton_velocities, 'hand_sdf': hand_sdf, 'obstacle_normals': obstacle_normals, 'obstacle_velocities': obstacle_velocities, 'skeleton_capsule_radius': skeleton_capsule_radius, 'hash_table': hash_table, 'segments_count_per_cell': segments_count_per_cell, 'hash_table': hash_table, 'segments_count_per_cell': segments_count_per_cell})
        mod.add_kernel(substep_calculate_hand_hash, template_args={'skeleton_segments': skeleton_segments, 'skeleton_capsule_radius': skeleton_capsule_radius, 'hash_table': hash_table, 'segments_count_per_cell': segments_count_per_cell})
        mod.add_kernel(substep_adjust_particle_mat, template_args={'x': x, 'v': v, 'mat_primitives': mat_primitives, 'mat_primitives_radius': mat_primitives_radius, 'mat_velocities': mat_velocities})
        mod.add_kernel(substep_adjust_particle_hash, template_args={'x': x, 'v': v, 'hash_table': hash_table, 'segments_count_per_cell': segments_count_per_cell, 'skeleton_capsule_radius': skeleton_capsule_radius, 'skeleton_velocities': skeleton_velocities, 'skeleton_segments': skeleton_segments})
        mod.add_kernel(substep_adjust_particle, template_args={'x': x, 'v': v, 'skeleton_capsule_radius': skeleton_capsule_radius, 'skeleton_velocities': skeleton_velocities, 'skeleton_segments': skeleton_segments})

        # Gaussian kernel functions
        mod.add_kernel(init_gaussian_data, template_args={'init_rotation': init_rotation, 'init_scale': init_scale, 'other_data': other_data})
        mod.add_kernel(substep_update_gaussian_data, template_args={'init_rotation': init_rotation, 'init_scale': init_scale, 'dg': dg, 'other_data': other_data, 'init_sh': init_sh, 'sh': sh, 'x': x})
        mod.add_kernel(scale_to_unit_cube, template_args={'x': x, 'other_data': other_data})
        mod.add_kernel(recenter_to_unit_cube, template_args={'x_gaussian': x, 'x': x, 'other_data': other_data})

        mod.add_kernel(normalize_m, template_args={'marching_m': marching_m})
        mod.add_kernel(transform_and_merge, template_args={'x1': x, 'x2': x, 'x3': x, 'mat2': mat2, 'mat3': mat3})
        mod.add_kernel(substep_fix_object, template_args={'grid_v': grid_v})
        mod.add_kernel(copy_array_1dim1, template_args={'src': E, 'dst': E})
        mod.add_kernel(copy_array_1dim3, template_args={'src': x, 'dst': x}) 
        mod.add_kernel(copy_array_1dim1I,template_args={'src': material, 'dst': material})
        mod.add_kernel(copy_array_3dim1, template_args={'src': sdf, 'dst': sdf})
        mod.add_kernel(copy_array_3dim3, template_args={'src': obstacle_normals, 'dst': obstacle_normals})
        mod.add_kernel(set_zero_1dim1, template_args={'x': p_mass})
        mod.add_kernel(set_zero_1dim3, template_args={'x': x})
        mod.add_kernel(init_sample_gaussian_data, template_args={'x_gaussian': x, 'x': x})

        mod.add_kernel(substep_squeeze_particles_circle, template_args={'x': x, 'p_mass': p_mass, 'v': v})
        mod.add_kernel(substep_squeeze_particles_square, template_args={'x': x, 'p_mass': p_mass, 'v': v})
        mod.add_kernel(substep_squeeze_particles_star, template_args={'x': x, 'p_mass': p_mass, 'v': v})
        
        mod.archive("Assets/Resources/TaichiModules/mpm3DGaussian_part_mat.kernel.tcm")
        print("AOT done")
    
    if run:
        # filepath = "./scripts/data/insect.ma"
        # vcount, fcount, ecount, verts, radii, faces, edges = load_mat_file(filepath)
        # mat_primitives_list, mat_primitives_radius_list = generate_medial_primitives(filepath, vcount, fcount, ecount, verts, radii, faces, edges)
        # for i in range(60):
        #     for j in range(3):
        #         mat_primitives[i, j] = ti.Vector(mat_primitives_list[i][j])
        #         mat_primitives_radius[i, j] = mat_primitives_radius_list[i][j]
        # substep_calculate_mat_sdf(mat_primitives, mat_primitives_radius, mat_velocities, mat_sdf, obstacle_normals, obstacle_velocities, dx, min_x, max_x, min_y, max_y, min_z, max_z)
        
        gui = ti.GUI('MPM3D', res=(800, 800))
        init_particles(x, v, dg, cube_size)
        init_sphere(x, dg, cube_size)
        init_cylinder(x, dg, 1, 0.05)
        init_torus(x, dg, 0.3, 0.05)
        init_sample_gaussian_data(x, x)
        scale_to_unit_cube(x,  other_data, 0.1)
        init_dg(dg)
        init_gaussian_data(init_rotation, init_scale, other_data)
        while gui.running and not gui.get_event(gui.ESCAPE):
            for i in range(50):
                substep()
            substep_update_gaussian_data(init_rotation, init_scale, dg, other_data, init_sh, sh, x, min_x, max_x, min_y, max_y, min_z, max_z)
            gui.circles(T(x.to_numpy()), radius=1.5, color=0x66CCFF)
            gui.show()
    run_aot()
    
if __name__ == "__main__":
    compile_for_cgraph = args.cgraph
    
    if args.arch == "vulkan":
        compile_mpm3D(arch=ti.vulkan, save_compute_graph=compile_for_cgraph, run=True)
    elif args.arch == "cuda":
        compile_mpm3D(arch=ti.cuda, save_compute_graph=compile_for_cgraph, run=True)
    elif args.arch == "x64":
        compile_mpm3D(arch=ti.x64, save_compute_graph=compile_for_cgraph, run=True)
    else:
        assert False
