# import open3d as o3d

# # 加载 .ply 文件
# pcd = o3d.io.read_point_cloud('/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/train/lab2/gt/pcd/fruit.ply')  # 替换为你的 .ply 文件路径

# # 显示点云
# o3d.visualization.draw_geometries([pcd], window_name="Point Cloud Visualization")

import open3d as o3d
import numpy as np

def generate_bounding_box(gt_ply_path):
    # 加载点云
    pcd = o3d.io.read_point_cloud(gt_ply_path)
    
    # 生成轴对齐的边界框
    bbox = pcd.get_axis_aligned_bounding_box()

    # 打印边界框的最小值和最大值
    print(f"Bounding box: Min bound = {bbox.min_bound}, Max bound = {bbox.max_bound}")

    # 保存 bbox 的最小值和最大值到 .npz 文件中
    np.savez('bounding_box.npz', arr_0=np.array([bbox.min_bound, bbox.max_bound]))

    # 可视化边界框和点云
    bbox.color = (1, 0, 0)  # 将边界框设置为红色，便于区分
    o3d.visualization.draw_geometries([pcd, bbox], window_name="Point Cloud and Bounding Box")

# 示例使用
#generate_bounding_box('/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/train/lab2/gt/pcd/fruit.ply')
generate_bounding_box('/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/val/p14/gt/pcd/fruit.ply')
