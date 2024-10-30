import numpy as np
import os
import open3d as o3d

base_path = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge"

def load_and_inspect_bounding_box(npz_file_path):
    # 读取 npz 文件
    data = np.load(npz_file_path)
    keys = list(data.keys())
    print("Keys in bounding_box.npz:", keys)
    # 获取 arr_0 数组
    bounding_box = data['arr_0']
    
    # 打印形状和内容
    print("Bounding Box Shape:", bounding_box.shape)
    print("Bounding Box Content:\n", bounding_box)

# 示例用法
npz_file_path = "/media/agri/DATA/serena/TCoRe_agri/data/igg_fruit/p2/tf/bounding_box.npz"  # 替换为实际 npz 文件路径
load_and_inspect_bounding_box(npz_file_path)


# splits = ["train", "val", "test"]

# for split in splits:
#     split_path = os.path.join(base_path, split)
#     for sample_id in os.listdir(split_path):
#         sample_path = os.path.join(split_path, sample_id)
#         fruit_ply_path = os.path.join(sample_path, "laser", "fruit.ply")
#         tf_path = os.path.join(sample_path, "tf")

#         # 确保 tf 文件夹存在
#         os.makedirs(tf_path, exist_ok=True)

#         # 读取完整点云并计算 bbox
#         if os.path.exists(fruit_ply_path):
#             full_pcd = o3d.io.read_point_cloud(fruit_ply_path)
#             bbox = full_pcd.get_axis_aligned_bounding_box()

#             # 提取 bbox 的最小和最大边界
#             min_bound = bbox.min_bound
#             max_bound = bbox.max_bound
#             bounding_box = np.array([min_bound, max_bound])

#             # 保存 bounding_box 为 npz 文件
#             np.savez(os.path.join(tf_path, "bounding_box.npz"), arr_0=bounding_box)
#             print(f"Saved bounding box for {sample_id} in {split} set.")
#         else:
#             print(f"fruit.ply not found for {sample_id} in {split} set.")


npz_file_path = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/train/lab11/tf/bounding_box.npz"  # 替换为实际 npz 文件路径
load_and_inspect_bounding_box(npz_file_path)
