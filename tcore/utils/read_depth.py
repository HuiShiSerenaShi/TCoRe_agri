import numpy as np
import matplotlib.pyplot as plt

# 加载 .npy 文件
#depth_image = np.load('/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/train/lab10/input/depth/00352.npy')  # 替换为你的深度图像文件路径
depth_image = np.load('/media/agri/DATA/serena/TCoRe_agri/data/shape_completion_challenge/val/p41/input/depth/00025.npy')

# 显示深度图
plt.imshow(depth_image, cmap='jet')
plt.colorbar(label='Depth (m)')  # 添加颜色条，显示深度单位
plt.title("Depth Image")
plt.show()
