import numpy as np

def move_obj_to_target(input_file, output_file, target_pos):
    """
    读取OBJ文件，计算其中心，然后将所有顶点平移，
    使得模型的几何中心位于 target_pos。
    """
    vertices = []
    lines = []
    
    print(f"正在读取: {input_file} ...")
    
    # 1. 读取文件并解析顶点
    try:
        with open(input_file, 'r') as f:
            raw_lines = f.readlines()
    except FileNotFoundError:
        print(f"错误: 找不到文件 {input_file}")
        return

    for line in raw_lines:
        if line.startswith('v '):
            # 解析顶点坐标 (x, y, z)
            parts = line.strip().split()
            # parts[0] 是 'v', parts[1:] 是坐标
            v = [float(x) for x in parts[1:4]]
            vertices.append(v)
            lines.append({'type': 'v', 'data': v}) # 暂存顶点数据以便修改
        else:
            lines.append({'type': 'other', 'data': line}) # 其他行直接保留

    if not vertices:
        print("错误: 文件中没有找到顶点数据。")
        return

    # 2. 计算当前模型的中心 (几何中心/平均值)
    vertices_np = np.array(vertices)
    current_center = np.mean(vertices_np, axis=0)
    
    print(f"当前模型中心: {current_center}")
    print(f"目标位置: {target_pos}")
    
    # 3. 计算位移向量 (Offset)
    # 目标 = 当前 + 位移  =>  位移 = 目标 - 当前
    offset = np.array(target_pos) - current_center
    print(f"需要移动的向量: {offset}")

    # 4. 应用位移并写入新文件
    print(f"正在写入: {output_file} ...")
    
    with open(output_file, 'w') as f:
        v_idx = 0
        for line_info in lines:
            if line_info['type'] == 'v':
                # 获取原始顶点
                original_v = vertices_np[v_idx]
                # 加上位移
                new_v = original_v + offset
                # 写入新坐标，保留6位小数
                f.write(f"v {new_v[0]:.6f} {new_v[1]:.6f} {new_v[2]:.6f}\n")
                v_idx += 1
            else:
                # 非顶点行直接写入原内容
                f.write(line_info['data'])

    print("完成！")

# --- 配置部分 ---

# 你的输入文件名 (请修改这里)
input_filename = 'sphere.obj' 

# 输出文件名
output_filename = 'sphere_centered.obj'

# 之前提到的目标中心点 (x, y, z)
# 放置在蓝红柱子后面，地板上
target_position = [1.5, -1.5, -1.0]

if __name__ == "__main__":
    move_obj_to_target(input_filename, output_filename, target_position)