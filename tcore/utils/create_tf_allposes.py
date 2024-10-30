import numpy as np
import os
import open3d as o3d

def load_and_inspect_tf_allposes(npz_file_path):
    # 读取 npz 文件
    data = np.load(npz_file_path)
    keys = list(data.keys())
    print("Keys in tf_allposes.npz:", keys)
    # 获取 arr_0 数组
    tf_allposes = data['arr_0']
    
    # 打印形状和内容
    print("tf_allposes Shape:", tf_allposes.shape)
    print("tf_allposes Content:\n", tf_allposes)

# 示例用法
npz_file_path = "/media/agri/DATA/serena/TCoRe_agri/data/igg_fruit/p2/tf/tf_allposes.npz"  # 替换为实际 npz 文件路径
load_and_inspect_tf_allposes(npz_file_path)


# def load_pose_from_txt(file_path):
#     """从txt文件中读取4x4的位姿矩阵"""
#     pose = np.loadtxt(file_path)
#     if pose.shape == (4, 4):
#         return pose
#     else:
#         raise ValueError(f"Invalid pose shape in file {file_path}. Expected (4,4), got {pose.shape}")

# def create_tf_allposes_npz(poses_dir, tf_dir):
#     """读取realsense/poses文件夹中的所有位姿txt文件并保存为tf/tf_allposes.npz文件"""
#     pose_files = sorted(os.listdir(poses_dir))
#     all_poses = []

#     for pose_file in pose_files:
#         file_path = os.path.join(poses_dir, pose_file)
#         if pose_file.endswith('.txt'):
#             pose = load_pose_from_txt(file_path)
#             all_poses.append(pose)

#     # 将所有位姿堆叠成一个3D数组，并保存为npz文件
#     all_poses_array = np.stack(all_poses, axis=0)
#     os.makedirs(tf_dir, exist_ok=True)
#     output_npz_path = os.path.join(tf_dir, 'tf_allposes.npz')
#     np.savez(output_npz_path, arr_0=all_poses_array)

#     print(f"Created tf_allposes.npz at {output_npz_path} with shape {all_poses_array.shape}")

# def process_dataset_structure(root_dir):
#     """递归遍历数据集结构,在每个样本文件夹中生成tf_allposes.npz"""
#     for split in ['train', 'val', 'test']:
#         split_dir = os.path.join(root_dir, split)
#         if os.path.isdir(split_dir):
#             for sample_folder in os.listdir(split_dir):
#                 sample_path = os.path.join(split_dir, sample_folder)
#                 if os.path.isdir(sample_path):
#                     poses_dir = os.path.join(sample_path, 'realsense', 'poses')
#                     tf_dir = os.path.join(sample_path, 'tf')
#                     if os.path.isdir(poses_dir):
#                         create_tf_allposes_npz(poses_dir, tf_dir)

# # 指定数据集的根目录
# root_dir = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge"

# # 生成所有tf_allposes.npz文件
# process_dataset_structure(root_dir)
npz_file_path = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/val/p12/tf/tf_allposes.npz"  # 替换为实际 npz 文件路径
load_and_inspect_tf_allposes(npz_file_path)

npz_file_path = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/train/lab72/tf/tf_allposes.npz"  # 替换为实际 npz 文件路径
load_and_inspect_tf_allposes(npz_file_path)

