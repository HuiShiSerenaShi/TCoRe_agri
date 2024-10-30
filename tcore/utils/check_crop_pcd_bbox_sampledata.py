import open3d as o3d
import numpy as np
import json

def load_intrinsics_from_json(json_file_path):
    """从json文件中加载相机内参矩阵K"""
    with open(json_file_path, 'r') as f:
        data = json.load(f)['intrinsic_matrix']
    K = np.reshape(data, (3, 3), order='F')
    return K

def load_pose_from_txt(txt_file_path):
    """从txt文件中加载4x4的位姿矩阵"""
    pose = np.loadtxt(txt_file_path)
    if pose.shape == (4, 4):
        return pose
    else:
        raise ValueError(f"Invalid pose shape: {pose.shape}, expected (4,4)")

def pcd_from_rgbd(rgb, d, pose, K):
    """ 使用RGB-D图像和内参矩阵生成部分点云 """
    rgb_frame = o3d.io.read_image(rgb)
    d_frame = np.load(d)

    extrinsic = np.eye(4)  # 初始时extrinsic为单位矩阵
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        height=d_frame.shape[0],
        width=d_frame.shape[1],
        fx=K[0, 0],
        fy=K[1, 1],
        cx=K[0, 2],
        cy=K[1, 2],
    )

    # 创建RGB-D图像
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb_frame),
        o3d.geometry.Image(d_frame),
        depth_scale=1000.0,  # 深度缩放系数，根据数据集调整
        depth_trunc=1.0,     # 深度截断范围，视实际需要调整
        convert_rgb_to_intensity=False
    )

    # 使用RGB-D图像生成点云
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic, extrinsic)

    # 应用外参变换，将点云转换到世界坐标系
    return pcd.transform(pose)

# 1. 从完整点云生成 bbox
#full_pcd = o3d.io.read_point_cloud("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab1/gt/pcd/fruit.ply")
#full_pcd = o3d.io.read_point_cloud("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab48/gt/pcd/fruit.ply")
full_pcd = o3d.io.read_point_cloud("/media/agri/DATA/serena/TCoRe_agri/data/test1/p4/laser/fruit.ply")
bbox = full_pcd.get_axis_aligned_bounding_box()
print(bbox)
# p4: min: (-0.0393575, -0.0374331, -0.064908), max: (0.0463209, 0.0449065, 0.0532716)

# 2. 从RGB-D生成部分点云
# 读取深度图、RGB图、位姿和内参矩阵
# depth = "/media/agri/DATA/serena/TCoRe_agri/data/test1/lab1/input/depth/00001.npy"
# rgb = "/media/agri/DATA/serena/TCoRe_agri/data/test1/lab1/input/color/00001.png"
# K = load_intrinsics_from_json("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab1/input/intrinsic.json")  # 使用json文件的K
# pose = load_pose_from_txt("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab1/input/poses/00001.txt")  # 使用txt文件的pose
# depth = "/media/agri/DATA/serena/TCoRe_agri/data/test1/lab48/input/depth/00001.npy"
# rgb = "/media/agri/DATA/serena/TCoRe_agri/data/test1/lab48/input/color/00001.png"
# K = load_intrinsics_from_json("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab48/input/intrinsic.json")  # 使用json文件的K
# pose = load_pose_from_txt("/media/agri/DATA/serena/TCoRe_agri/data/test1/lab48/input/poses/00001.txt")  # 使用txt文件的pose
depth = "/media/agri/DATA/serena/TCoRe_agri/data/test1/p4/realsense/depth/00001.npy"
rgb = "/media/agri/DATA/serena/TCoRe_agri/data/test1/p4/realsense/color/00001.png"
K = load_intrinsics_from_json("/media/agri/DATA/serena/TCoRe_agri/data/test1/p4/realsense/intrinsic.json")  # 使用json文件的K
pose = np.load('/media/agri/DATA/serena/TCoRe_agri/data/test1/p4/tf/tf_allposes.npz')['arr_0'][0]
print(pose)
# p4: [[ 0.59823259 -0.2702704   0.7543684  -0.38490943]
#  [-0.79552392 -0.31335974  0.51860135 -0.2692474 ]
#  [ 0.09622609 -0.91036233 -0.40246858  0.12387313]
#  [ 0.          0.          0.          1.        ]]
# 使用提供的 `pcd_from_rgbd` 方法生成部分点云
pcd = pcd_from_rgbd(rgb, depth, pose, K)

# 3. 使用 bbox 裁剪部分点云
cropped_pcd = pcd.crop(bbox)

full_pcd.paint_uniform_color([0, 0, 1])
o3d.visualization.draw_geometries([pcd,full_pcd], window_name="un-cropped Point Cloud")

# 4. 可视化完整点云
o3d.visualization.draw_geometries([full_pcd], window_name="Full Point Cloud")

# 5. 可视化裁剪后的点云和bbox
bbox.color = (1, 0, 0)  # 给bbox设置颜色，例如红色
cropped_pcd.paint_uniform_color([0, 1, 0])  # 给裁剪的点云设置颜色，例如绿色
o3d.visualization.draw_geometries([cropped_pcd, bbox], window_name="Cropped Point Cloud with Bounding Box")

# 6. 可视化完整点云、部分点云和bbox
cropped_pcd.paint_uniform_color([0, 1, 0])  # 裁剪后的部分点云颜色为绿色
full_pcd.paint_uniform_color([0, 0, 1])    # 完整点云颜色为蓝色
o3d.visualization.draw_geometries([full_pcd, cropped_pcd, bbox], window_name="Full, Cropped, and Bounding Box")
