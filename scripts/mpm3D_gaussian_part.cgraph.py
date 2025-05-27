import argparse
import os
import numpy as np
import taichi as ti
import json

import timeit

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
    ti.init(arch, vk_api_version="1.0",debug=False)  

    if ti.lang.impl.current_cfg().arch != arch:
        return

    dim, n_grid, steps, dt,cube_size ,particle_per_grid= 3, 64, 25, 1e-4,0.2,16
    n_particles  = int((((n_grid*cube_size)**dim) *particle_per_grid))
    print("Number of particles: ", n_particles)
    dx = 1/n_grid
    
    p_rho = 1000
    p_vol = dx** 3  
    p_mass = p_vol * p_rho/ particle_per_grid
    allowed_cfl = 0.5
    v_allowed = dx * allowed_cfl / dt
    gx=0
    gy=-9.8
    gz=0
    k=0.5
    friction_k=0.4
    damping=1
    bound = 3
    E = 10000  # Young's modulus for snow
    SigY=1000
    nu = 0.45  # Poisson's ratio
    mu_0, lambda_0 = E / (2 * (1 + nu)), E * nu / ((1 + nu) * (1 - 2 * nu))  # Lame parameters
    friction_angle = 30.0
    sin_phi = ti.sin(friction_angle / 180 * 3.141592653)
    alpha = ti.sqrt(2 / 3) * 2 * sin_phi / (3 - sin_phi)

    neighbour = (3,) * dim

    @ti.kernel 
    def substep_reset_grid(grid_v: ti.types.ndarray(ndim=3), grid_m: ti.types.ndarray(ndim=3)):
        for I in ti.grouped(grid_m):
            grid_v[I] = [0, 0, 0]
            grid_m[I] = 0
            
    @ti.kernel
    def substep_neohookean_p2g(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), C: ti.types.ndarray(ndim=1), 
                               dg: ti.types.ndarray(ndim=1), grid_v: ti.types.ndarray(ndim=3), grid_m: ti.types.ndarray(ndim=3),
                               mu:ti.f32,la:ti.f32,p_vol:ti.f32,p_mass:ti.f32,dx:ti.f32,dt:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in x:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                Xp = x[p] / dx
                base = int(Xp - 0.5)
                fx = Xp - base
                w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1) ** 2, 0.5 * (fx - 0.5) ** 2]

                dg[p] = (ti.Matrix.identity(float, dim) + dt * C[p]) @ dg[p]
                J=dg[p].determinant()
                cauchy=mu*(dg[p]@dg[p].transpose())+ti.Matrix.identity(float, dim)*(la*ti.log(J)-mu)
                stress=-(dt * p_vol * 4 /dx**2) * cauchy
                affine = stress + p_mass * C[p]

                for offset in ti.static(ti.grouped(ti.ndrange(*neighbour))):
                    dpos = (offset - fx) * dx
                    weight = 1.0
                    for i in ti.static(range(dim)):
                        weight *= w[offset[i]][i]
                    grid_v[base + offset] += weight * (p_mass * v[p] + affine @ dpos)
                    grid_m[base + offset] += weight * p_mass
        
    @ti.kernel
    def substep_kirchhoff_p2g(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), C: ti.types.ndarray(ndim=1),
                               dg: ti.types.ndarray(ndim=1), grid_v: ti.types.ndarray(ndim=3), grid_m: ti.types.ndarray(ndim=3),
                               mu:ti.f32,la:ti.f32,p_vol:ti.f32,p_mass:ti.f32,dx:ti.f32,dt:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in x:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                Xp = x[p] / dx
                base = int(Xp - 0.5)
                fx = Xp - base
                w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1) ** 2, 0.5 * (fx - 0.5) ** 2]

                dg[p] = (ti.Matrix.identity(float, dim) + dt * C[p]) @ dg[p]
                U, sig, V = ti.svd(dg[p])
                J_new = sig[0, 0] * sig[1, 1] * sig[2, 2]
                stress = 2 * mu * (dg[p] - U @ V.transpose()) @ dg[p].transpose() + ti.Matrix.identity(
                    float, dim) * la * J_new * (J_new - 1)
                stress = (-dt * p_vol * 4) * stress / dx**2
                affine = stress + p_mass * C[p]

                for offset in ti.static(ti.grouped(ti.ndrange(*neighbour))):
                    dpos = (offset - fx) * dx
                    weight = 1.0
                    for i in ti.static(range(dim)):
                        weight *= w[offset[i]][i]
                    grid_v[base + offset] += weight * (p_mass * v[p] + affine @ dpos)
                    grid_m[base + offset] += weight * p_mass
    
    @ti.kernel
    def substep_calculate_signed_distance_field(obstacle_pos: ti.types.ndarray(ndim=1),
                                                sdf: ti.types.ndarray(ndim=3), obstacle_normals: ti.types.ndarray(ndim=3),
                                                obstacle_radius:ti.types.ndarray(ndim=1),dx:ti.f32,dt:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for I in ti.grouped(sdf):
            pos = I * dx + dx * 0.5
            if pos[0]>min_x and pos[0]<max_x and pos[1]>min_y and pos[1]<max_y and pos[2]>min_z and pos[2]<max_z:
                min_dist = float('inf')
                norm= ti.Vector([0.0, 0.0, 0.0])
                for j in obstacle_pos:
                    dist = (pos - obstacle_pos[j]).norm() - obstacle_radius[j]
                    if dist < min_dist:
                        min_dist = dist
                        norm= (pos-obstacle_pos[j]).normalized()

                sdf[I] = min_dist
                obstacle_normals[I] = norm

    @ti.kernel
    def substep_update_grid_v(grid_v: ti.types.ndarray(ndim=3), grid_m: ti.types.ndarray(ndim=3),
                              sdf: ti.types.ndarray(ndim=3),
                              obstacle_normals:ti.types.ndarray(ndim=3),
                              obstacle_velocities:ti.types.ndarray(ndim=3),
                              gx:float,gy:float,gz:float,k:float,damping:float,friction_k:float,
                              v_allowed:ti.f32,dt:ti.f32,n_grid:ti.i32,dx:ti.f32,bound:ti.i32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for I in ti.grouped(grid_m):
            pos=I*dx+dx*0.5
            if pos[0]>min_x and pos[0]<max_x and pos[1]>min_y and pos[1]<max_y and pos[2]>min_z and pos[2]<max_z:
                if grid_m[I] > 0:
                    grid_v[I] /= grid_m[I]
                gravity = ti.Vector([gx,gy,gz])
                grid_v[I] += dt * gravity
                #damping
                grid_v[I] *= ti.exp(-damping * dt)
                if sdf[I] <= 0:
                    d=-sdf[I]
                    rel_v=grid_v[I]-obstacle_velocities[I]
                    normal_v = rel_v.dot(obstacle_normals[I]) * obstacle_normals[I]
                    delta_v =obstacle_normals[I] *d / dt * k-normal_v
                    tangent_direction = (rel_v - normal_v).normalized()
                    friction_force = friction_k * delta_v
                    grid_v[I] += delta_v-friction_force*tangent_direction

                cond = (I < bound) & (grid_v[I] < 0) | (I > n_grid - bound) & (grid_v[I] > 0)
                grid_v[I] = ti.select(cond, 0, grid_v[I])
                grid_v[I] = min(max(grid_v[I], -v_allowed), v_allowed)
                sdf[I] = 1
                obstacle_normals[I] = [0,0,0]

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
    def substep_apply_Drucker_Prager_plasticity(dg: ti.types.ndarray(ndim=1),x: ti.types.ndarray(ndim=1),lambda_0:ti.f32,mu_0:ti.f32,alpha:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in dg:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                U, sig, V = ti.svd(dg[p])

                # 将 sig 转换为向量
                sig_vec = ti.Vector([sig[i, i] for i in range(dim)])

                epsilon = ti.log(ti.abs(sig_vec))
                trace_epsilon = epsilon.sum()

                epsilon_hat = epsilon - trace_epsilon / 3 * ti.Vector([1.0, 1.0, 1.0])
                epsilon_hat_squared_norm = epsilon_hat.norm_sqr()
                epsilon_hat_norm = ti.sqrt(epsilon_hat_squared_norm)
                delta_gamma = 0.0

                if trace_epsilon <= 0:
                    delta_gamma = epsilon_hat_norm + (3 * lambda_0 + 2 * mu_0) / (2 * mu_0) * trace_epsilon * alpha
                else:
                    delta_gamma = epsilon_hat_norm

                Z = ti.Matrix.identity(float, 3)
                if delta_gamma <= 0:
                    for i in range(dim):
                        Z[i, i] = sig_vec[i]
                else:
                    H = epsilon - (delta_gamma / epsilon_hat_norm) * epsilon_hat
                    E = ti.exp(H)
                    Z = ti.Matrix([[E[0], 0, 0], [0, E[1], 0], [0, 0, E[2]]])

                dg[p] = U @ Z @ V.transpose()
    
    @ti.kernel
    def substep_apply_Von_Mises_plasticity(dg: ti.types.ndarray(ndim=1),x: ti.types.ndarray(ndim=1),mu_0:ti.f32,SigY:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in dg:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                U, sig, V = ti.svd(dg[p])

                # 将 sig 转换为向量
                sig_vec = ti.Vector([sig[i, i] for i in range(dim)])

                epsilon = ti.log(ti.abs(sig_vec))
                trace_epsilon = epsilon.sum()

                epsilon_hat = epsilon - trace_epsilon / 3 * ti.Vector([1.0, 1.0, 1.0])
                epsilon_hat_squared_norm = epsilon_hat.norm_sqr()
                epsilon_hat_norm = ti.sqrt(epsilon_hat_squared_norm)
                delta_gamma = epsilon_hat_norm - SigY / (2 * mu_0)

                Z = ti.Matrix.identity(float, 3)
                if delta_gamma <= 0:
                    for i in range(dim):
                        Z[i, i] = sig_vec[i]
                else:
                    H = epsilon - (delta_gamma / epsilon_hat_norm) * epsilon_hat
                    E = ti.exp(H)
                    Z = ti.Matrix([[E[0], 0, 0], [0, E[1], 0], [0, 0, E[2]]])

                dg[p] = U @ Z @ V.transpose()
    
    @ti.kernel
    def substep_apply_clamp_plasticity(dg: ti.types.ndarray(ndim=1),x: ti.types.ndarray(ndim=1),min_clamp:ti.f32,max_clamp:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for p in dg:
            if(x[p][0]>min_x and x[p][0]<max_x and x[p][1]>min_y and x[p][1]<max_y and x[p][2]>min_z and x[p][2]<max_z):
                U, sig, V = ti.svd(dg[p]) 
                for i in ti.static(range(dim)):
                    sig[i, i] = min(max(sig[i, i], 1-min_clamp), 1+max_clamp)
                dg[p] = U @ sig @ V.transpose()

    @ti.kernel
    def init_particles(x: ti.types.ndarray(ndim=1), v: ti.types.ndarray(ndim=1), dg: ti.types.ndarray(ndim=1),cube_size:ti.f32):
        for i in range(x.shape[0]):
            x[i] = [ti.random() * cube_size + (0.5-cube_size/2), ti.random() * cube_size+ (0.5-cube_size/2), ti.random() * cube_size+(0.5-cube_size/2)]
            dg[i] = ti.Matrix.identity(float, dim)

    @ti.kernel
    def init_dg(dg: ti.types.ndarray(ndim=1)):
        for i in range(dg.shape[0]):
            dg[i] = ti.Matrix.identity(float, dim)
    
    @ti.kernel
    def substep_get_max_speed(v: ti.types.ndarray(ndim=1),x: ti.types.ndarray(ndim=1),max_speed: ti.types.ndarray(ndim=1),min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for I in ti.grouped(v):
            if(x[I][0]>min_x and x[I][0]<max_x and x[I][1]>min_y and x[I][1]<max_y and x[I][2]>min_z and x[I][2]<max_z):
                max_speed[0] = ti.atomic_max(max_speed[0], v[I].norm())
    
    
    @ti.kernel
    def substep_calculate_hand_sdf(skeleton_segments: ti.types.ndarray(ndim=2), 
                                   skeleton_velocities: ti.types.ndarray(ndim=2),
                                   hand_sdf: ti.types.ndarray(ndim=3),
                                   obstacle_normals: ti.types.ndarray(ndim=3),
                                   obstacle_velocities: ti.types.ndarray(ndim=3),
                                   skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                   dx:ti.f32,min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        #if (skeleton_segments.shape[0] != 0):
        for I in ti.grouped(hand_sdf):
            pos = I * dx + dx * 0.5
            if pos[0]>min_x and pos[0]<max_x and pos[1]>min_y and pos[1]<max_y and pos[2]>min_z and pos[2]<max_z:
                min_dist = float('inf')
                norm= ti.Vector([0.0, 0.0, 0.0])

                for i in range(skeleton_segments.shape[0]):
                    sx, sy, sz = skeleton_segments[i, 0][0], skeleton_segments[i, 0][1], skeleton_segments[i, 0][2]
                    ex, ey, ez = skeleton_segments[i, 1][0], skeleton_segments[i, 1][1], skeleton_segments[i, 1][2]
                    result = calculate_point_segment_distance(pos[0], pos[1], pos[2], sx, sy, sz, ex, ey, ez)
                    dist = result.distance
                    r = result.b
                    distance = dist.norm() - skeleton_capsule_radius[i]
                    if distance < min_dist:
                        min_dist = distance
                        norm = dist.normalized()
                        obstacle_velocities[I] = skeleton_velocities[i,0] * (1-r) + skeleton_velocities[i,1] * r
                hand_sdf[I] = min_dist
                obstacle_normals[I] = norm
    
    @ti.kernel
    def substep_calculate_hand_sdf_hash(skeleton_segments: ti.types.ndarray(ndim=2),
                                        skeleton_velocities: ti.types.ndarray(ndim=2),
                                        hand_sdf: ti.types.ndarray(ndim=3),
                                        obstacle_normals: ti.types.ndarray(ndim=3),
                                        obstacle_velocities: ti.types.ndarray(ndim=3),
                                        skeleton_capsule_radius: ti.types.ndarray(ndim=1),
                                        dx: ti.f32,
                                        n_grid: ti.i32,
                                        hash_table: ti.types.ndarray(ndim=4),
                                        segments_count_per_cell: ti.types.ndarray(ndim=3)):
        # clear and build hash
        clear_hash_table(hash_table, segments_count_per_cell)
        insert_segments(skeleton_segments, n_grid, hash_table, segments_count_per_cell)
        # calculate hand sdf
        for I in ti.grouped(hand_sdf):
            pos = I * dx + dx * 0.5
            min_dist = float('inf')
            cell = get_hash(pos, n_grid)
            norm= ti.Vector([0.0, 0.0, 0.0])

            for offset_x, offset_y, offset_z in ti.ndrange((-2, 3), (-2, 3), (-2, 3)):
                neighbor_cell = cell + ti.Vector([offset_x, offset_y, offset_z])
                if (0 <= neighbor_cell[0] < n_grid) and (0 <= neighbor_cell[1] < n_grid) and (0 <= neighbor_cell[2] < n_grid):
                    for i in range(segments_count_per_cell[neighbor_cell[0], neighbor_cell[1], neighbor_cell[2]]):
                        segment_idx = hash_table[neighbor_cell[0], neighbor_cell[1], neighbor_cell[2], i]
                        if segment_idx != -1:
                            seg_start = skeleton_segments[segment_idx, 0]
                            seg_end = skeleton_segments[segment_idx, 1]
                            result = calculate_point_segment_distance(pos[0], pos[1], pos[2], seg_start[0], seg_start[1], seg_start[2], seg_end[0], seg_end[1], seg_end[2])
                            dist = result.distance
                            r = result.b
                            distance = dist.norm() - skeleton_capsule_radius[i]
                            if distance < min_dist:
                                min_dist = distance
                                norm = dist.normalized()
                                obstacle_velocities[I] = skeleton_velocities[i,0] * (1-r) + skeleton_velocities[i,1] * r
            hand_sdf[I] = min_dist
            obstacle_normals[I] = norm

    # region - Gaussian Kernel Functions
    @ti.kernel
    def init_gaussian_data(init_rotation:ti.types.ndarray(ndim=1),init_scale:ti.types.ndarray(ndim=1),other_data:ti.types.ndarray(ndim=1)):
        for i in other_data:
            init_rotation[i] = DecodeRotation(DecodePacked_10_10_10_2(ti.bit_cast(other_data[i][0],ti.u32)))
            init_scale[i] = ti.Vector([other_data[i][1], other_data[i][2], other_data[i][3]])


    @ti.kernel
    def substep_update_gaussian_data(init_rotation:ti.types.ndarray(ndim=1),init_scale:ti.types.ndarray(ndim=1),dg:ti.types.ndarray(ndim=1),
                                     other_data:ti.types.ndarray(ndim=1),init_sh:ti.types.ndarray(ndim=1),sh:ti.types.ndarray(ndim=1),x:ti.types.ndarray(ndim=1),
                                        min_x:ti.f32,max_x:ti.f32,min_y:ti.f32,max_y:ti.f32,min_z:ti.f32,max_z:ti.f32):
        for i in dg:
            if(x[i][0]>min_x and x[i][0]<max_x and x[i][1]>min_y and x[i][1]<max_y and x[i][2]>min_z and x[i][2]<max_z):
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
                scale =S @ init_scale[i]
    
                # 编码 rot 并填入 other_data
                packed_rot = PackSmallest3Rotation(rot)
                encoded_rot = EncodeQuatToNorm10(packed_rot)
                other_data[i][0] = ti.bit_cast(encoded_rot, ti.f32)
                other_data[i][1]  = scale[0]
                other_data[i][2]  = scale[1]
                other_data[i][3]  = scale[2]

                sh[i]=RotateSH(R,init_sh[i])

    @ti.kernel
    def scale_to_unit_cube(x: ti.types.ndarray(ndim=1),other_data: ti.types.ndarray(ndim=1),eps:ti.f32):
        #计算 bounding box
        min_val = ti.Vector([float('inf'), float('inf'), float('inf')])
        max_val = ti.Vector([float('-inf'), float('-inf'), float('-inf')])
        for j in range(1):  
            for i in x:
                if x[i][0] < min_val[0]:
                    min_val[0] = x[i][0]
                if x[i][1] < min_val[1]:
                    min_val[1] = x[i][1]
                if x[i][2] < min_val[2]:
                    min_val[2] = x[i][2]

                if x[i][0] > max_val[0]:
                    max_val[0] = x[i][0]
                if x[i][1] > max_val[1]:
                    max_val[1] = x[i][1]
                if x[i][2] > max_val[2]:
                    max_val[2] = x[i][2]


        center = (min_val + max_val) / 2.0
        size = max_val - min_val

        # 基于最大尺寸计算缩放因子
        scaleFactor = (1.0 - 2 * eps) / ti.max(size[0], size[1], size[2])

        new_center = ti.Vector([0.5, 0.5, 0.5])

        for i in x:
            # 缩放位置并将其平移到新的中心
            x[i] = (x[i] - center) * scaleFactor + new_center

           # 缩放 m_other 中的比例因子
            other_data[i][1] = scaleFactor*other_data[i][1]
            other_data[i][2] = scaleFactor*other_data[i][2]
            other_data[i][3] = scaleFactor*other_data[i][3]

    
    # hand sdf
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
    obstacle_pos = ti.Vector.ndarray(3, ti.f32, shape=(1))
    obstacle_velocities = ti.Vector.ndarray(3, ti.f32, shape=(n_grid, n_grid, n_grid))
    obstacle_pos[0] = ti.Vector([0.25, 0.25, 0.25])
    obstacle_radius = ti.ndarray(ti.f32, shape=(1))
    max_speed = ti.ndarray(ti.f32, shape=(1))
    obstacle_radius[0] = 0

    #boundary
    min_x = 0.1
    max_x = 0.9
    min_y = 0.1
    max_y = 0.9
    min_z = 0.1
    max_z = 0.9

    init_scale = ti.Vector.ndarray(3, ti.f32, shape=(n_particles))
    init_rotation = ti.Vector.ndarray(4, ti.f32, shape=(n_particles))
    other_data = ti.Vector.ndarray(4,ti.f32, shape=(n_particles))
    init_sh = ti.Matrix.ndarray(16,3,ti.f32, shape=(n_particles))
    sh = ti.Matrix.ndarray(16,3,ti.f32, shape=(n_particles))

    sym_x=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'x', ndim=1, dtype=ti.types.vector(3, ti.f32))
    sym_v=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'v', ndim=1, dtype=ti.types.vector(3, ti.f32))
    sym_C=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'C', ndim=1, dtype=ti.types.matrix(3, 3, ti.f32))
    sym_dg=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'dg', ndim=1, dtype=ti.types.matrix(3, 3, ti.f32))
    sym_grid_v=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'grid_v', ndim=3, dtype=ti.types.vector(3, ti.f32))
    sym_grid_m = ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'grid_m', ti.f32, ndim=3)
    sym_sdf=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'sdf', ndim=3, dtype=ti.f32)
    sym_obstacle_pos=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'obstacle_pos', ndim=1, dtype=ti.types.vector(3, ti.f32))
    sym_obstacle_radius=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'obstacle_radius',ti.f32, ndim=1)
    sym_obstacle_velocities=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'obstacle_velocities', ndim=3, dtype=ti.types.vector(3, ti.f32))
    sym_max_speed=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'max_speed',ti.f32, ndim=1)
    sym_init_scale=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'init_scale', ndim=1, dtype=ti.types.vector(3, ti.f32))
    sym_init_rotation=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'init_rotation', ndim=1, dtype=ti.types.vector(4, ti.f32))
    sym_other_data=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'other_data', ndim=1, dtype=ti.types.vector(4, ti.f32))
    sym_init_sh=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'init_sh', ndim=1, dtype=ti.types.matrix(16, 3, ti.f32))
    sym_sh=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'sh', ndim=1, dtype=ti.types.matrix(16, 3, ti.f32))

    sym_hand_sdf=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'hand_sdf', ti.f32, ndim=3)
    sym_obstacle_normals=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'obstacle_normals', ndim=3, dtype=ti.types.vector(3, ti.f32))
    sym_skeleton_segments=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'skeleton_segments', ndim=2, dtype=ti.types.vector(3, ti.f32))
    sym_skeleton_velocities=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'skeleton_velocities', ndim=2, dtype=ti.types.vector(3, ti.f32))
    sym_skeleton_capsule_radius=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'skeleton_capsule_radius',ti.f32, ndim=1)
    sym_hash_table=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'hash_table', ndim=4, dtype=ti.i32)
    sym_segments_count_per_cell=ti.graph.Arg(ti.graph.ArgKind.NDARRAY, 'segments_count_per_cell', ndim=3, dtype=ti.i32)

    sym_n_grid=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'n_grid', ti.i32)
    sym_n_particles=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'n_particles', ti.i32)
    sym_dt=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'dt', ti.f32)
    sym_dx=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'dx', ti.f32)
    sym_mu_0=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'mu_0', ti.f32)
    sym_lambda_0=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'lambda_0', ti.f32)
    sym_cube_size=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'cube_size', ti.f32)
    sym_eps=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'eps', ti.f32)
    sym_gx=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'gx', ti.f32)
    sym_gy=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'gy', ti.f32)
    sym_gz=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'gz', ti.f32)
    sym_k=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'k', ti.f32)
    sym_damping=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'damping', ti.f32)
    sym_friction_k=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'friction_k', ti.f32)
    sym_v_allowed=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'v_allowed', ti.f32)
    sym_bound=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'bound', ti.i32)
    sym_mu=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'mu', ti.f32)
    sym_la=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'la', ti.f32)
    sym_SigY=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'SigY', ti.f32)
    sym_alpha=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'alpha', ti.f32)
    sym_p_rho=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'p_rho', ti.f32)
    sym_p_vol=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'p_vol', ti.f32)
    sym_p_mass=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'p_mass', ti.f32)
    sym_E = ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'E', ti.f32)
    sym_nu = ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'nu', ti.f32)
    sym_min_clamp=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'min_clamp', ti.f32)
    sym_max_clamp=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'max_clamp', ti.f32)
    sym_min_x=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'min_x', ti.f32)
    sym_max_x=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'max_x', ti.f32)
    sym_min_y=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'min_y', ti.f32)
    sym_max_y=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'max_y', ti.f32)
    sym_min_z=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'min_z', ti.f32)
    sym_max_z=ti.graph.Arg(ti.graph.ArgKind.SCALAR, 'max_z', ti.f32)


    g_init_builder=ti.graph.GraphBuilder()
    g_init_builder.dispatch(init_dg,sym_dg)
    g_init_builder.dispatch(scale_to_unit_cube,sym_x,sym_other_data,sym_eps)
    g_init_builder.dispatch(init_gaussian_data,sym_init_rotation,sym_init_scale,sym_other_data)
    g_init=g_init_builder.compile()

    g_substep_builder=ti.graph.GraphBuilder()

    iters=10

    g_substep_builder.dispatch(substep_calculate_hand_sdf,sym_skeleton_segments,sym_skeleton_velocities,sym_hand_sdf,sym_obstacle_normals,sym_obstacle_velocities,sym_skeleton_capsule_radius,sym_dx,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)
    for i in range(iters):
        g_substep_builder.dispatch(substep_reset_grid,sym_grid_v,sym_grid_m)
        g_substep_builder.dispatch(substep_neohookean_p2g,sym_x,sym_v,sym_C,sym_dg,sym_grid_v,sym_grid_m,sym_mu_0,sym_lambda_0,sym_p_vol,sym_p_mass,sym_dx,sym_dt,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)
        g_substep_builder.dispatch(substep_update_grid_v,sym_grid_v,sym_grid_m,sym_hand_sdf,sym_obstacle_normals,sym_obstacle_velocities,sym_gx,sym_gy,sym_gz,sym_k,sym_damping,sym_friction_k,sym_v_allowed,sym_dt,sym_n_grid,sym_dx,sym_bound,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)
        g_substep_builder.dispatch(substep_g2p,sym_x,sym_v,sym_C,sym_grid_v,sym_dx,sym_dt,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)
        g_substep_builder.dispatch(substep_apply_clamp_plasticity,sym_dg,sym_x,sym_min_clamp,sym_max_clamp,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)

    g_substep_builder.dispatch(substep_update_gaussian_data,sym_init_rotation,sym_init_scale,sym_dg,sym_other_data,sym_init_sh,sym_sh,sym_x,sym_min_x,sym_max_x,sym_min_y,sym_max_y,sym_min_z,sym_max_z)
    g_substep=g_substep_builder.compile()



    
    def substep():
        substep_reset_grid(grid_v, grid_m)
        substep_kirchhoff_p2g(x, v, C,  dg, grid_v, grid_m, mu_0, lambda_0, p_vol, p_mass, dx, dt, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_neohookean_p2g(x, v, C,  dg, grid_v, grid_m, mu_0, lambda_0, p_vol, p_mass, dx, dt, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_calculate_signed_distance_field(obstacle_pos,sdf,obstacle_velocities,obstacle_radius,dx,dt,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_calculate_hand_sdf(skeleton_segments, skeleton_velocities, hand_sdf, obstacle_normals, obstacle_velocities, skeleton_capsule_radius, dx, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_update_grid_v(grid_v, grid_m,hand_sdf,obstacle_normals ,obstacle_velocities,gx,gy,gz,k,damping,friction_k,v_allowed,dt,n_grid,dx,bound,min_x,max_x,min_y,max_y,min_z,max_z)
        substep_g2p(x, v, C,  grid_v, dx, dt, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_apply_Von_Mises_plasticity(dg,x, mu_0, SigY, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_apply_clamp_plasticity(dg, x,0.1,0.1, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_apply_Drucker_Prager_plasticity(dg, x,lambda_0, mu_0, alpha, min_x, max_x, min_y, max_y, min_z, max_z)
        substep_get_max_speed(v,x, max_speed, min_x, max_x, min_y, max_y, min_z, max_z)
    
    def run_aot():
        mod = ti.aot.Module(arch)
        mod.add_graph('init',g_init)
        mod.add_graph('substep',g_substep)
        mod.archive("Assets/Resources/TaichiModules/mpm3DGaussian_part.cgraph.tcm")
        print("AOT done")
    params = {
    'v': v,
    'grid_m': grid_m,
    'x': x,
    'C': C,
    'grid_v': grid_v,
    'init_sh': init_sh,
    'sh': sh,
    'other_data': other_data,
    'dg': dg,
    'init_scale': init_scale,
    'init_rotation': init_rotation,
    'mu_0': mu_0,
    'lambda_0': lambda_0,
    'p_vol': p_vol,
    'p_mass': p_mass,
    'dx': dx,
    'dt': dt,
    'n_grid': n_grid,
    'gx': gx,
    'gy': gy,
    'gz': gz,
    'k': k,
    'damping': damping,
    'friction_k': friction_k,
    'v_allowed': v_allowed,
    'min_clamp': 0.1,
    'max_clamp': 0.1,
    'hand_sdf': hand_sdf,
    'skeleton_segments': skeleton_segments,
    'skeleton_velocities': skeleton_velocities,
    'skeleton_capsule_radius': skeleton_capsule_radius,
    'obstacle_normals': obstacle_normals,
    'obstacle_velocities': obstacle_velocities,
    'bound': bound,
    'min_x': min_x,
    'max_x': max_x,
    'min_y': min_y,
    'max_y': max_y,
    'min_z': min_z,
    'max_z': max_z,
    }
    if run:
        gui = ti.GUI('MPM3D', res=(800, 800))
        init_particles(x, v, dg,cube_size)
        scale_to_unit_cube(x,  other_data, 0.1)
        init_dg(dg)
        init_gaussian_data(init_rotation, init_scale, other_data)
        while gui.running and not gui.get_event(gui.ESCAPE):
            for i in range(50):
                substep()
            substep_update_gaussian_data(init_rotation, init_scale, dg, other_data, init_sh, sh, x, min_x, max_x, min_y, max_y, min_z, max_z)
            g_substep.run(params)
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
