from distutils.command.build_scripts import first_line_re
import taichi as ti
import numpy as np
import json

ti.init(arch=ti.gpu)

@ti.dataclass
class DistanceResult:
    distance: ti.types.vector(3, ti.f32)
    b: ti.f32

@ti.func
def calculate_point_segment_distance(point: ti.types.vector(3, ti.f32), start: ti.types.vector(3, ti.f32), end: ti.types.vector(3, ti.f32)) -> DistanceResult:
    v = end - start
    w = point - start
    c1 = w.dot(v)
    c2 = v.dot(v)
    b = 0.0
    distance = ti.Vector([0.0, 0.0, 0.0])
    if c1 <= 0:
        distance = (point - start)
    elif c1 >= c2:
        distance = (point - end)
        b = 1
    else:
        b = c1 / c2
        Pb = start + b * v
        distance = (point - Pb)
    return DistanceResult(distance=distance, b=b)

@ti.func
def inv_square(x):  # A Taichi function
    return 1.0 / (x * x)

@ti.kernel
def partial_sum(n: int) -> float:  # A kernel
    total = 0.0
    for i in range(1, n + 1):
        total += inv_square(n)
    return total

def initialize_medial_spheres():
    sphere_centers[0] = ti.Vector([0.0, 3.0, 0.0])
    sphere_centers[1] = ti.Vector([1.0, 3.0, 1.0])
    sphere_centers[2] = ti.Vector([2.0, 3.0, 0.0])
    sphere_centers[3] = ti.Vector([2.0, 0.0, 0.0])
    sphere_radii[0] = 0.5
    sphere_radii[1] = 0.5
    sphere_radii[2] = 0.5
    sphere_radii[3] = 0.5
    
@ti.func
def value_of_quadric_surface_2d(x, y, A, B, C, D, E, F):
    return A * x * x + B * x * y + C * y * y + D * x + E * y + F

@ti.kernel
def compute_sphere_cone_distance(sphere_centers_0: ti.types.vector(3, ti.f32), sphere_radii_0: ti.f32, 
                                sphere_centers_1: ti.types.vector(3, ti.f32), sphere_radii_1: ti.f32, 
                                sphere_centers_2: ti.types.vector(3, ti.f32), sphere_radii_2: ti.f32, 
                                sphere_centers_3: ti.types.vector(3, ti.f32), sphere_radii_3: ti.f32):
    # sphere_centers and sphere_radii should contain 4 elements
    sC1 = ti.Vector([0.0, 0.0, 0.0])
    sC2 = ti.Vector([0.0, 0.0, 0.0])
    sC3 = ti.Vector([0.0, 0.0, 0.0])
    for i in ti.static(range(3)):
        sC1[i] = sphere_centers_0[i] - sphere_centers_1[i]
        sC2[i] = sphere_centers_3[i] - sphere_centers_2[i]
        sC3[i] = sphere_centers_1[i] - sphere_centers_3[i]
    sR1 = sphere_radii_0 - sphere_radii_1
    sR2 = sphere_radii_2 - sphere_radii_3
    sR3 = sphere_radii_1 + sphere_radii_3
    
    A = sC1.dot(sC1) - sR1 * sR1
    B = 2.0 * (sC1.dot(sC2) - sR1 * sR2)
    C = sC2.dot(sC2) - sR2 * sR2
    D = 2.0 * (sC1.dot(sC3) - sR1 * sR3)
    E = 2.0 * (sC2.dot(sC3) - sR2 * sR3)
    F = sC3.dot(sC3) - sR3 * sR3
    
    delta = 4 * A * C - B * B
    print("delta: ", delta)
    temp_alpha = 0.0
    temp_beta = 0.0
    
    alpha = 0.0
    beta = 0.0
    dist_f = value_of_quadric_surface_2d(alpha, beta, A, B, C, D, E, F)
    
    # parallel cases
    for temp_alpha, temp_beta in ti.static([(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]):
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case0: ", dist_f)
    
    # temp_alpha = 0, temp_beta = -E / (2.0 * C)
    temp_alpha = 0.0
    temp_beta = -E / (2.0 * C)
    if 0.0 < temp_beta < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case1: ", dist_f)

    # temp_alpha = 1.0, temp_beta = -(B + E) / (2.0 * C)
    temp_alpha = 1.0
    temp_beta = -(B + E) / (2.0 * C)
    if 0.0 < temp_beta < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case2: ", dist_f)
    
    # temp_alpha = -D / (2.0 * A), temp_beta = 0.0
    temp_alpha = -D / (2.0 * A)
    temp_beta = 0.0
    if 0.0 < temp_alpha < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case3: ", dist_f)
    
    # temp_alpha = -(B + D) / (2.0 * A), temp_beta = 1.0
    temp_alpha = -(B + D) / (2.0 * A)
    temp_beta = 1.0
    if 0.0 < temp_alpha < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case4: ", dist_f)
    
    # temp_alpha = (B * E - 2.0 * C * D) / delta, temp_beta = (B * D - 2.0 * A * E) / delta
    if delta != 0.0:
        temp_alpha = (B * E - 2.0 * C * D) / delta
        temp_beta = (B * D - 2.0 * A * E) / delta
        print(temp_alpha, temp_beta)
        if 0.0 < temp_alpha < 1.0 and 0.0 < temp_beta < 1.0:
            temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
            if dist_f > temp_dist:
                dist_f = temp_dist
                alpha = temp_alpha
                beta = temp_beta
                print("case5: ", dist_f)

    # Compute the distance
    print("alpha: ", alpha, "beta: ", beta)
    cp = ti.Vector([0.0, 0.0, 0.0], dt=ti.f32)
    cq = ti.Vector([0.0, 0.0, 0.0], dt=ti.f32)
    for i in ti.static(range(3)):
        cp[i] = alpha * sphere_centers_0[i] + (1.0 - alpha) * sphere_centers_1[i]
        cq[i] = beta * sphere_centers_2[i] + (1.0 - beta) * sphere_centers_3[i]
    rp = alpha * sphere_radii_0 + (1.0 - alpha) * sphere_radii_1
    rq = beta * sphere_radii_2 + (1.0 - beta) * sphere_radii_3
    dir = cq - cp
    distance = dir.norm() - (rp + rq)
    normal = dir.normalized()
    print("Distance: ", distance, "Normal: ", normal)

@ti.kernel
def compute_sphere_slab_distance(sphere_centers_0: ti.types.vector(3, ti.f32), sphere_radii_0: ti.f32, 
                                sphere_centers_1: ti.types.vector(3, ti.f32), sphere_radii_1: ti.f32, 
                                sphere_centers_2: ti.types.vector(3, ti.f32), sphere_radii_2: ti.f32, 
                                sphere_centers_3: ti.types.vector(3, ti.f32), sphere_radii_3: ti.f32):
    # sphere_centers and sphere_radii should contain 4 elements
    # The first three sphers compose a slab, and the last one is a sphere
    sC1 = ti.Vector([0.0, 0.0, 0.0])
    sC2 = ti.Vector([0.0, 0.0, 0.0])
    sC3 = ti.Vector([0.0, 0.0, 0.0])
    for i in ti.static(range(3)):
        sC1[i] = sphere_centers_0[i] - sphere_centers_2[i]
        sC2[i] = sphere_centers_1[i] - sphere_centers_2[i]
        sC3[i] = sphere_centers_2[i] - sphere_centers_3[i]
    sR1 = sphere_radii_0 - sphere_radii_2
    sR2 = sphere_radii_1 - sphere_radii_2
    sR3 = sphere_radii_2 + sphere_radii_3

    A = sC1.dot(sC1) - sR1 * sR1
    B = 2.0 * (sC1.dot(sC2) - sR1 * sR2)
    C = sC2.dot(sC2) - sR2 * sR2
    D = 2.0 * (sC1.dot(sC3) - sR1 * sR3)
    E = 2.0 * (sC2.dot(sC3) - sR2 * sR3)
    F = sC3.dot(sC3) - sR3 * sR3
    
    delta = 4 * A * C - B * B
    print("delta: ", delta)
    temp_alpha = 0.0
    temp_beta = 0.0
    
    alpha = 0.0
    beta = 0.0
    dist_f = value_of_quadric_surface_2d(alpha, beta, A, B, C, D, E, F)
    
    # parallel cases
    for temp_alpha, temp_beta in ti.static([(1.0, 0.0), (0.0, 1.0)]):
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case0: ", dist_f)
    
    # temp_alpha = 0, temp_beta = -E / (2.0 * C)
    temp_alpha = 0.0
    temp_beta = -E / (2.0 * C)
    if 0.0 < temp_beta < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case1: ", dist_f)
    
    # temp_alpha = -D / (2.0 * A), temp_beta = 0.0
    temp_alpha = -D / (2.0 * A)
    temp_beta = 0.0
    if 0.0 < temp_alpha < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case3: ", dist_f)
    
    temp_alpha = 0.5 * (2.0 * C + E - B - D) / (A - B + C)
    temp_beta = 1.0 - temp_alpha
    if 0.0 < temp_alpha < 1.0:
        temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
        if dist_f > temp_dist:
            dist_f = temp_dist
            alpha = temp_alpha
            beta = temp_beta
            print("case4: ", dist_f)
    
    # can be ignored
    if delta != 0.0:
        temp_alpha = (B * E - 2.0 * C * D) / delta
        temp_beta = (B * D - 2.0 * A * E) / delta
        print(temp_alpha, temp_beta)
        if 0.0 < temp_alpha < 1.0 and 0.0 < temp_beta < 1.0 and temp_alpha + temp_beta < 1.0:
            temp_dist = value_of_quadric_surface_2d(temp_alpha, temp_beta, A, B, C, D, E, F)
            if dist_f > temp_dist:
                dist_f = temp_dist
                alpha = temp_alpha
                beta = temp_beta
                print("case5: ", dist_f)
    
    # Compute the distance
    print("alpha: ", alpha, "beta: ", beta)
    cp = ti.Vector([0.0, 0.0, 0.0], dt=ti.f32)
    cq = ti.Vector([0.0, 0.0, 0.0], dt=ti.f32)
    for i in ti.static(range(3)):
        cp[i] = alpha * sphere_centers_0[i] + beta * sphere_centers_1[i] + (1.0 - alpha - beta) * sphere_centers_2[i]
        cq[i] = sphere_centers_3[i]
    
    rp = alpha * sphere_radii_0 + beta * sphere_radii_1 + (1.0 - alpha - beta) * sphere_radii_2
    rq = sphere_radii_3
    dir = cq - cp
    distance = dir.norm() - (rp + rq)
    normal = dir.normalized()
    print("cq: ", cq, "cp: ", cp)
    print("Distance: ", distance, "Normal: ", normal)

def unique_everseen(iterable, key=None):
    "List unique elements, preserving order. Remember all elements ever seen."
    # unique_everseen('AAAABBBCCDAABBB') --> A B C D
    # unique_everseen('ABBCcAD', str.lower) --> A B C D
    seen = set()
    seen_add = seen.add
    if key is None:
        for element in filterfalse(seen.__contains__, iterable):
            seen_add(element)
            yield element
    else:
        for element in iterable:
            k = key(element)
            if k not in seen:
                seen_add(k)
                yield element

def load_mat_file(filepath):
    file = open(filepath, 'r')
    # read first line number of vertices/edges/faces
    first_line = file.readline().rstrip()
    vcount, ecount, fcount = [int(x) for x in first_line.split()]
    # No Medial Vertices
    assert vcount != 0, "No Medial Vertices!"
    # line number
    lineno = 1

    # Medial Mesh Info
    verts, radii, faces, edges = [], [], [], []

    # read vertices
    i = 0
    while i < vcount:
        line = file.readline()
        # skip empty lines or comment lines
        if line.isspace() or line[0] == '#':
            lineno += 1
            continue
        v = line.split()
        # Handle exception
        assert v[0] == 'v', "vertex line: " + str(lineno) + " should start with \'v\'!"
        x = float(v[1])
        y = float(v[2])
        z = float(v[3])
        radii.append(float(v[4]))
        verts.append((x, y, z))
        lineno += 1
        i += 1

    # read edges
    i = 0
    while i < ecount:
        line = file.readline()
        if line.isspace() or line[0] == '#':
            lineno += 1
            continue
        ef = line.split()
        # Handle exception
        assert ef[0] == 'e', "line:" + str(lineno) + " should start with \'e\'!"
        ids = list(map(int, ef[1:3]))
        edges.append(tuple(ids))
        lineno += 1
        i += 1

    # read faces
    i = 0
    while i < fcount:
        line = file.readline()
        if line.isspace() or line[0] == '#':
            lineno += 1
            continue
        ef = line.split()
        # Handle exception
        assert ef[0] == 'f', "line:" + str(lineno) + " should start with \'f\'!"
        f = tuple(list(map(int, ef[1:4])))
        faces.append(f)
        edges.append(f[:2])
        edges.append(f[1:3])
        edges.append((f[0], f[2]))
        lineno += 1
        i += 1

    unique_edges = list(unique_everseen(edges, key=frozenset))
    return vcount, fcount, ecount, verts, radii, faces, unique_edges

def convert_to_serializable(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (list, tuple)):
        return [convert_to_serializable(item) for item in obj]
    else:
        return obj

def save_mat_data(filepath, filename, mat_primitives, mat_primitives_radius):
    data = convert_to_serializable(mat_primitives)
    with open(filepath + filename + "_mat_primitives.json", 'w') as f:
        json.dump(data, f)
    data = convert_to_serializable(mat_primitives_radius)
    with open(filepath + filename + "_mat_primitives_radius.json", 'w') as f:
        json.dump(data, f)
    print("Saved done!")
    
def generate_medial_primitives(filepath, vcount, fcount, ecount, verts, radii, faces, edges):
    # Store the medial primitives into this list
    mat_primitives = []
    mat_primitives_radius = []
    # Generating medial cones, not geometrically, only for storing the data
    for e in edges:
        sphere_centers_1 = np.array(verts[e[0]])
        sphere_radii_1 = radii[e[0]]
        sphere_centers_2 = np.array(verts[e[1]])
        sphere_radii_2 = radii[e[1]]
        # create an sphere for placeholder
        sphere_centers_3 = np.array([0.0, 0.0, 0.0])
        sphere_radii_3 = 0.0
        mat_primitives.append((sphere_centers_1, sphere_centers_2, sphere_centers_3))
        mat_primitives_radius.append((sphere_radii_1, sphere_radii_2, sphere_radii_3))
    # Generating medial slabs, not geometrically, only for storing the data
    for f in faces:
        sphere_centers_1 = np.array(verts[f[0]])
        sphere_radii_1 = radii[f[0]]
        sphere_centers_2 = np.array(verts[f[1]])
        sphere_radii_2 = radii[f[1]]
        sphere_centers_3 = np.array(verts[f[2]])
        sphere_radii_3 = radii[f[2]]
        mat_primitives.append((sphere_centers_1, sphere_centers_2, sphere_centers_3))
        mat_primitives_radius.append((sphere_radii_1, sphere_radii_2, sphere_radii_3))
    return mat_primitives, mat_primitives_radius

sphere_centers = ti.Vector.ndarray(3, ti.f32, shape=(4,))
sphere_radii = ti.ndarray(ti.f32, shape=(4,))
alpha = ti.ndarray(ti.f32, shape=(1,))
beta = ti.ndarray(ti.f32, shape=(1,))

# test: distance detection between sphere to cone & slab
sphere_centers[0] = ti.Vector([0.0, 0.0, 0.0])
sphere_centers[1] = ti.Vector([2.0, 0.0, 0.0])
sphere_centers[2] = ti.Vector([2.1, 0, 0.0])
sphere_centers[3] = ti.Vector([2.1, 0, 0.0])
sphere_radii[0] = 0.5
sphere_radii[1] = 0.5
sphere_radii[2] = 0
sphere_radii[3] = 0
compute_sphere_cone_distance(sphere_centers[0], sphere_radii[0], sphere_centers[1], sphere_radii[1], sphere_centers[2], sphere_radii[2], sphere_centers[3], sphere_radii[3])
# compute_sphere_slab_distance(sphere_centers[0], sphere_radii[0], sphere_centers[1], sphere_radii[1], sphere_centers[2], sphere_radii[2], sphere_centers[3], sphere_radii[3])

# Load medial mesh data (.ma file)
# filepath = "./scripts/data/insect.ma"
# vcount, fcount, ecount, verts, radii, faces, edges = load_mat_file(filepath)
# mat_primitives, mat_primitives_radius = generate_medial_primitives(filepath, vcount, fcount, ecount, verts, radii, faces, edges)
# print(len(mat_primitives))
# print(len(mat_primitives_radius))
# print(mat_primitives)
# print(mat_primitives_radius)
# Save the medial primitives into json file
# save_mat_data("./scripts/data/", "insect", mat_primitives, mat_primitives_radius)


# pos = ti.Vector([0.0, 0.0, 0.0])
# for i in range(len(mat_primitives)):
#     # Determine cone and slab by the radius of the third sphere
#     if mat_primitives_radius[i][2] == 0.0:
#         # Compute distance from grid node to cone
#         print("dist to cone: ", i)
#         compute_sphere_cone_distance(mat_primitives[i][0], mat_primitives_radius[i][0],
#                                         mat_primitives[i][1], mat_primitives_radius[i][1],
#                                         pos, 0.0,
#                                         pos, 0.0)
#     else:
#         # Compute distance from grid node to slab
#         print("dist to slab: ", i)
#         compute_sphere_slab_distance(mat_primitives[i][0], mat_primitives_radius[i][0],
#                                         mat_primitives[i][1], mat_primitives_radius[i][1],
#                                         mat_primitives[i][2], mat_primitives_radius[i][2],
#                                         pos, 0.0)