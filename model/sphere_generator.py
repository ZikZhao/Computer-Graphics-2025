import math

def generate_triangulated_sphere(file_path, center=(1.5, -1.5, 0), radius=1.25, rings=32, sectors=32):
    """
    Generate a triangulated sphere in OBJ format.

    Arguments:
        file_path: Output file path
        center: Center of the sphere (x, y, z)
        radius: Radius of the sphere
        rings: Number of segments in the latitude direction (from north pole to south pole)
        sectors: Number of segments in the longitude direction (around the sphere)
    """
    cx, cy, cz = center
    vertices = []
    normals = []
    faces = []

    # Generate vertices and normals
    # From north pole (stack_angle=0) to south pole (stack_angle=pi)
    for r in range(rings + 1):
        stack_angle = math.pi * r / rings  # 0 to pi
        sin_stack = math.sin(stack_angle)
        cos_stack = math.cos(stack_angle)

        for s in range(sectors + 1):
            sector_angle = 2 * math.pi * s / sectors  # 0 to 2*pi

            # Calculate normal direction (pointing outward from the center)
            nx = sin_stack * math.cos(sector_angle)
            ny = cos_stack  # North pole is up (y+)
            nz = sin_stack * math.sin(sector_angle)
            
            # Calculate vertex position
            x = cx + radius * nx
            y = cy + radius * ny
            z = cz + radius * nz
            
            vertices.append((x, y, z))
            normals.append((nx, ny, nz))

    # Generate faces (triangles)
    # Use counter-clockwise order (as seen from outside) so that normals point outward
    for r in range(rings):
        for s in range(sectors):
            # Indices of the four vertices of the current quad (1-based for OBJ format)
            # p1 --- p2
            # |  \   |
            # |   \  |
            # p4 --- p3
            p1 = r * (sectors + 1) + s
            p2 = p1 + 1
            p4 = (r + 1) * (sectors + 1) + s
            p3 = p4 + 1

            # Upper triangle (p1, p2, p3) - skip degenerate triangles at the north pole
            if r != 0:
                faces.append(f"{p1}/ {p2}/ {p3}/")
            
            # Lower triangle (p1, p3, p4) - skip degenerate triangles at the south pole
            if r != rings - 1:
                faces.append(f"{p1}/ {p3}/ {p4}/")

    # Write to file
    with open(file_path, 'w') as f:
        f.write("Object # sphere\n")
        
        for i in range(len(vertices)):
            v = vertices[i]
            n = normals[i]
            f.write(f"    Vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f} / {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
            
        for face in faces:
            f.write(f"    Face {face}\n")

    print(f"Generated: {file_path}")
    print(f"Vertices: {len(vertices)}, Faces: {len(faces)}")

if __name__ == "__main__":
    generate_triangulated_sphere("sphere.txt")