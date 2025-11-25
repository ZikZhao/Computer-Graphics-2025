import math

def generate_triangulated_sphere(file_path, center=(1.5, -1.5, -1.0), radius=1.0, rings=32, sectors=32):
    """
    生成带有顶点法线 (vn) 的全三角形 (Triangulated) 光滑球体 OBJ 文件。
    修复了部分渲染器只显示一半四边形的问题。
    """
    
    cx, cy, cz = center
    vertices = []
    normals = []
    faces = []

    # 1. 生成顶点 (v) 和 法线 (vn)
    for r in range(rings + 1):
        stack_angle = math.pi * r / rings
        sin_stack = math.sin(stack_angle)
        cos_stack = math.cos(stack_angle)

        for s in range(sectors + 1):
            sector_angle = 2 * math.pi * s / sectors
            sin_sector = math.sin(sector_angle)
            cos_sector = math.cos(sector_angle)

            nx = sin_stack * cos_sector
            ny = -cos_stack 
            nz = sin_stack * sin_sector
            
            x = cx + radius * nx
            y = cy + radius * (-math.cos(stack_angle))
            z = cz + radius * nz
            
            length = math.sqrt(nx*nx + (-math.cos(stack_angle))**2 + nz*nz)
            if length > 0:
                vertices.append((x, y, z))
                # 归一化法线
                normals.append((nx/length, -math.cos(stack_angle)/length, nz/length))
            else:
                vertices.append((x, y, z))
                normals.append((0, 1, 0))

    # 2. 生成面 (f) - 强制三角化
    # 逻辑：将一个四边形 (p1-p2-p3-p4) 切割为两个三角形 (p1-p2-p3) 和 (p1-p3-p4)
    for r in range(rings):
        for s in range(sectors):
            # 获取当前格子的四个顶点索引
            # p1 -- p2
            # |      |
            # p4 -- p3
            
            p1 = r * (sectors + 1) + s + 1
            p2 = p1 + 1
            p3 = (r + 1) * (sectors + 1) + s + 1 + 1
            p4 = p3 - 1
            
            # --- 关键修改：显式写入两个三角形 ---
            
            # 三角形 1 (右上半部分)
            # 只有当不是北极的第一圈时才需要这个面（北极点是退化的三角形）
            if r != 0: 
                faces.append(f"{p1}//{p1} {p2}//{p2} {p3}//{p3}")
            
            # 三角形 2 (左下半部分)
            # 只有当不是南极的最后一圈时才需要这个面
            if r != rings - 1:
                faces.append(f"{p1}//{p1} {p3}//{p3} {p4}//{p4}")

            # 注意：如果不加 if 判断，两极会产生面积为0的退化三角形，
            # 虽然大多数软件能容忍，但加上判断代码更干净。
            # 为了最稳妥的兼容性（即使极点也生成三角形），你可以把上面的 if 去掉，
            # 直接 append 两个 face，通常不会报错。

    # 3. 写入文件
    with open(file_path, 'w') as f:
        f.write("# Generated Triangulated Sphere\n")
        f.write("mtllib material.mtl\n")
        f.write("o sphere_triangulated\n")
        f.write("usemtl GreyMetallic\n")
        f.write("s 1\n")
        
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            
        for vn in normals:
            f.write(f"vn {vn[0]:.6f} {vn[1]:.6f} {vn[2]:.6f}\n")
            
        for face in faces:
            f.write(f"f {face}\n")

    print(f"成功生成三角化球体: {file_path}")
    print(f"顶点数: {len(vertices)}, 面数: {len(faces)} (全三角形)")

if __name__ == "__main__":
    generate_triangulated_sphere("sphere.obj")