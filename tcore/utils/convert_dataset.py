import os
import shutil
import json
import random

# 数据集的根目录
root_dir = "/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge"  # 请替换为您的数据集根目录路径

# # 遍历test, train, val目录
# for split in ["test", "train", "val"]:
#     split_dir = os.path.join(root_dir, split)
    
#     # 检查split目录是否存在
#     if not os.path.exists(split_dir):
#         continue
    
#     # 遍历每个样本文件夹
#     for sample_folder in os.listdir(split_dir):
#         sample_path = os.path.join(split_dir, sample_folder)
#         if not os.path.isdir(sample_path):
#             continue
        
#         # gt目录路径
#         gt_path = os.path.join(sample_path, "gt")
#         laser_path = os.path.join(sample_path, "laser")
        
#         if os.path.exists(gt_path):
#             # 如果gt目录存在，重命名为laser
#             os.rename(gt_path, laser_path)
            
#             # 移动fruit.ply文件
#             pcd_dir = os.path.join(laser_path, "pcd")
#             if os.path.exists(pcd_dir):
#                 ply_file = os.path.join(pcd_dir, "fruit.ply")
#                 if os.path.exists(ply_file):
#                     shutil.move(ply_file, laser_path)
#                 # 删除pcd文件夹
#                 shutil.rmtree(pcd_dir)
        
#         # 删除pose文件夹
#         # pose_dir = os.path.join(sample_path, "pose")
#         # if os.path.exists(pose_dir):
#         #     shutil.rmtree(pose_dir)

# print("数据集结构调整完成")


# def rename_input_to_realsense(root_dir):
#     # 遍历 train, val, test 文件夹
#     for split in ['train', 'val', 'test']:
#         split_path = os.path.join(root_dir, split)
#         if os.path.exists(split_path):
#             # 遍历每个样本文件夹
#             for sample_folder in os.listdir(split_path):
#                 sample_path = os.path.join(split_path, sample_folder)
#                 input_path = os.path.join(sample_path, 'input')
#                 realsense_path = os.path.join(sample_path, 'realsense')
                
#                 # 如果存在 'input' 文件夹，重命名为 'realsense'
#                 if os.path.isdir(input_path):
#                     os.rename(input_path, realsense_path)
#                     print(f'Renamed: {input_path} to {realsense_path}')

# # 使用示例
# rename_input_to_realsense(root_dir)


# def create_tf_folders(root_dir):
#     # 遍历 train, val, test 文件夹
#     for split in ['train', 'val', 'test']:
#         split_path = os.path.join(root_dir, split)
#         if os.path.exists(split_path):
#             # 遍历每个样本文件夹
#             for sample_folder in os.listdir(split_path):
#                 sample_path = os.path.join(split_path, sample_folder)
#                 tf_path = os.path.join(sample_path, 'tf')
                
#                 # 如果 tf 文件夹不存在，创建它
#                 if not os.path.exists(tf_path):
#                     os.makedirs(tf_path)
#                     print(f'Created: {tf_path}')

# create_tf_folders(root_dir)


# 定义原始路径和目标路径

# target_dir = '/media/agri/DATA/serena/TCoRe_agri/data/dataset_sweet_pepper'

# # 如果目标文件夹不存在则创建
# os.makedirs(target_dir, exist_ok=True)

# # 遍历 train 和 val 文件夹
# for split in ['train', 'val']:
#     split_dir = os.path.join(root_dir, split)
#     if os.path.exists(split_dir):
#         # 遍历每个样本文件夹
#         for sample_folder in os.listdir(split_dir):
#             sample_path = os.path.join(split_dir, sample_folder)
#             target_path = os.path.join(target_dir, sample_folder)
            
#             # 确保是文件夹，然后移动
#             if os.path.isdir(sample_path):
#                 shutil.move(sample_path, target_path)
#                 print(f"Moved {sample_folder} to {target_dir}")

# print("All sample folders moved to dataset_oct.")


# # 定义数据集路径
# dataset_path = '/media/agri/DATA/serena/TCoRe_agri/data/dataset_sweet_pepper'

# # 获取所有样本文件夹名称
# all_samples = [folder for folder in os.listdir(dataset_path) if os.path.isdir(os.path.join(dataset_path, folder))]

# # 统计 lab 开头和 p 开头的样本数量
# lab_samples = [sample for sample in all_samples if sample.startswith('lab')]
# p_samples = [sample for sample in all_samples if sample.startswith('p')]

# # 输出结果
# print(f"Total samples: {len(all_samples)}")
# print(f"Samples starting with 'lab': {len(lab_samples)}")
# print(f"Samples starting with 'p': {len(p_samples)}")



# # 定义数据集目录
# dataset_dir = '/media/agri/DATA/serena/TCoRe_agri/data/dataset_sweet_pepper'

# # 获取所有样本文件夹
# all_samples = os.listdir(dataset_dir)

# # 按照开头分组
# lab_samples = [sample for sample in all_samples if sample.startswith('lab')]
# p_samples = [sample for sample in all_samples if sample.startswith('p')]

# # 设置随机种子以确保分配可重复
# random.seed(42)

# # 随机打乱 lab_samples
# random.shuffle(lab_samples)

# # 分配 lab 样本：70% train，15% val，15% test_lab
# train_samples = lab_samples[:int(0.7 * len(lab_samples))]
# val_samples = lab_samples[int(0.7 * len(lab_samples)):int(0.85 * len(lab_samples))]
# test_lab_samples = lab_samples[int(0.85 * len(lab_samples)):]

# # 随机打乱 p_samples
# random.shuffle(p_samples)

# # 分配 p 样本：50% val，50% test_p
# val_samples += p_samples[:len(p_samples) // 2]
# test_p_samples = p_samples[len(p_samples) // 2:]

# # 将结果保存到 split.json
# split_data = {
#     "train": train_samples,
#     "val": val_samples,
#     "test_lab": test_lab_samples,
#     "test_p": test_p_samples
# }

# # 保存到 split.json 文件
# with open('/media/agri/DATA/serena/TCoRe_agri/data/dataset_sweet_pepper/split.json', 'w') as f:
#     json.dump(split_data, f, indent=4)

# print("分配完成，数据已保存到 split.json 文件中！")
# print(f"训练集样本数: {len(train_samples)}")
# print(f"验证集样本数: {len(val_samples)}")
# print(f"测试集 (lab) 样本数: {len(test_lab_samples)}")
# print(f"测试集 (p) 样本数: {len(test_p_samples)}")

