#!/usr/bin/env python3

# import rospy
# import open3d as o3d
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Header
# import numpy as np
# import math
# import cv2

# def publish_downsampled_pcd(pcd_file_path, publish_topic):
#     # 初始化 ROS 节点
#     rospy.init_node('downsample_pcd_publisher', anonymous=True)
#     pub = rospy.Publisher(publish_topic, PointCloud2, queue_size=10)
#     rate = rospy.Rate(1)  # 每秒发布一次

#     # 读取 PCD 文件
#     pcd = o3d.io.read_point_cloud(pcd_file_path)
    
    
#     # # 对点云主成分分析，得到点云的主方向 点云是三维的
#     # points = np.asarray(pcd.points)
#     # center = np.mean(points, axis=0)
#     # points -= center

#     # covariance_matrix = np.dot(points.T, points) / points.shape[0]

#     # eigen_values, eigen_vectors = np.linalg.eigh(covariance_matrix)
    
#     # print('eigen_values:', eigen_values)
#     # print('eigen_vectors:', eigen_vectors)
    
#     # # 将特征向量可视化
#     # line_set = o3d.geometry.LineSet()
#     # line_set.points = o3d.utility.Vector3dVector([center, center + 3 * eigen_vectors[:, 0], center + 3 * eigen_vectors[:, 1], center + 3 * eigen_vectors[:, 2]])
#     # line_set.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
#     # line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
#     # o3d.visualization.draw_geometries([pcd, line_set])
    

    
    
#     # 使用体素降采样
#     downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.05)
#     pcd = downsampled_pcd
    
    
#     # 旋转点云
#     R = pcd.get_rotation_matrix_from_xyz((-0.5/180*math.pi, -6/180*math.pi, 0))     # bigai_15-11-12_3----(-1.5/180*math.pi, -6/180*math.pi, 0)
#     pcd.rotate(R, center=(0, 0, 0))    
    
#     # 选取特征高度的点，转成2d占据地图
#     points = np.asarray(pcd.points)
#     min_x = points[:, 0].min()
#     max_x = points[:, 0].max()
#     min_y = points[:, 1].min()
#     max_y = points[:, 1].max()
#     points = points[points[:, 2] > 0.5]
#     points = points[points[:, 2] < 1]
#     points = points * 10
#     points = points.astype(np.int32)
    
#     img = np.zeros((800, 800), dtype=np.uint8)
#     for p in points:
#         img[p[0] + 400, p[1] + 400] = 255
#     cv2.imshow('img', img)
#     cv2.waitKey(0)
    
    

    

#     # 将 Open3D 点云转换为 ROS 消息格式
#     ros_cloud = open3d_to_ros(downsampled_pcd)

#     # 持续发布消息
#     while not rospy.is_shutdown():
#         pub.publish(ros_cloud)
#         rate.sleep()

# def open3d_to_ros(open3d_cloud):
#     points = np.asarray(open3d_cloud.points)
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = "map"  # 根据需要设置frame_id
#     ros_msg = pc2.create_cloud_xyz32(header, points)
#     return ros_msg

# if __name__ == '__main__':
#     rospy.init_node('downsample_pcd_publisher', anonymous=True)
#     try:
#         # 从参数服务器中获取 PCD 文件路径和发布话题
#         pcd_file_path = rospy.get_param('~pcd_file_path')
#         publish_topic = rospy.get_param('~publish_topic')
#         publish_downsampled_pcd(pcd_file_path, publish_topic)
#     except rospy.ROSInterruptException:
#         pass





import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import math
import cv2

stamp = None
use_bag_time = None

def publish_downsampled_pcd(pcd_file_path, publish_topic, map_topic, resolution):
    # 初始化 ROS 节点
    rospy.init_node('downsample_pcd_publisher', anonymous=True)
    pub = rospy.Publisher(publish_topic, PointCloud2, queue_size=10)
    map_pub = rospy.Publisher(map_topic, OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 每秒发布一次

    # 读取 PCD 文件
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    
    # 使用体素降采样
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.05)
    pcd = downsampled_pcd
    
    # 旋转点云
    R = pcd.get_rotation_matrix_from_xyz((0.0/180*math.pi, -3.0/180*math.pi, 0))  #bigai-15-12-23---((2.0/180*math.pi, -3.0/180*math.pi, 0))     # bigai_15-11-12_3----(-1.5/180*math.pi, -6/180*math.pi, 0)     bigai_15_11-13----((-0.5/180*math.pi, -6/180*math.pi, 0))
    pcd.rotate(R, center=(0, 0, 0))    
    
    # 选取特征高度的点，转成2d占据地图
    points = np.asarray(pcd.points)
    points = points[(points[:, 2] > 0.5) & (points[:, 2] < 1)]
    points = points / resolution
    points = points.astype(np.int32)
    
    # 获取长和宽
    min_x = points[:, 0].min()
    max_x = points[:, 0].max()
    min_y = points[:, 1].min()
    max_y = points[:, 1].max()
    width = max_x - min_x + 5
    height = max_y - min_y + 5
    
    img = np.zeros((height, width), dtype=np.uint8)
    for p in points:
        img[p[1] - min_y, p[0] - min_x] += 1
    
    # 将图像转换为 OccupancyGrid 消息
    occupancy_grid = image_to_occupancy_grid(img, min_x, min_y, resolution)
    
    # 将 Open3D 点云转换为 ROS 消息格式
    ros_cloud = open3d_to_ros(downsampled_pcd)

    # 持续发布消息
    while not rospy.is_shutdown():
        pub.publish(ros_cloud)
        map_pub.publish(occupancy_grid)
        rate.sleep()

def open3d_to_ros(open3d_cloud):
    global stamp, use_bag_time
    if stamp is None or not use_bag_time:
        stamp = rospy.Time.now()
    points = np.asarray(open3d_cloud.points)
    header = Header()
    header.stamp = stamp    # rospy.Time.now()
    header.frame_id = "map"  # 根据需要设置frame_id
    ros_msg = pc2.create_cloud_xyz32(header, points)
    return ros_msg

def image_to_occupancy_grid(img, min_x, min_y, resolution):
    global stamp, use_bag_time
    if stamp is None or not use_bag_time:
        stamp = rospy.Time.now()
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.stamp = stamp    # rospy.Time.now()
    occupancy_grid.header.frame_id = "map"
    
    # 设置地图元数据     origin.position表示的意思是地图原点在地图坐标系下的位置，地图原点的意思是图片像素坐标为(0, 0)的点
    map_metadata = MapMetaData()
    map_metadata.resolution = resolution  # 每个像素代表的实际大小（米）
    map_metadata.width = img.shape[1]
    map_metadata.height = img.shape[0]
    # map_metadata.origin.position.x = 1 * map_metadata.width * map_metadata.resolution   # 根据需要设置原点位置
    # map_metadata.origin.position.y = 1 * map_metadata.height * map_metadata.resolution
    map_metadata.origin.position.x =  min_x * map_metadata.resolution
    map_metadata.origin.position.y =  min_y * map_metadata.resolution
    print('map_metadata.origin.position.x:', map_metadata.origin.position.x)
    print('map_metadata.origin.position.y:', map_metadata.origin.position.y)
    map_metadata.origin.position.z = 0
    map_metadata.origin.orientation.w = 1.0
    occupancy_grid.info = map_metadata
    
    # 将图像数据转换为 OccupancyGrid 数据
    data = []
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if img[i, j] > 0:
                data.append(100)  # 占据
            else:
                data.append(1)  # 未知
    
    occupancy_grid.data = data
    return occupancy_grid

def point_callback(data):
    global stamp, use_bag_time
    if use_bag_time:
        stamp = data.header.stamp
    else:
        stamp = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('downsample_pcd_publisher', anonymous=True)
    try:
        # 从参数服务器中获取 PCD 文件路径和发布话题
        pcd_file_path = rospy.get_param('~pcd_file_path')
        publish_topic = rospy.get_param('~publish_topic')
        map_topic = rospy.get_param('~map_topic')
        resolution = float(rospy.get_param('~resolution'))
        use_bag_time = bool(rospy.get_param('~use_bag_time'))
        # 订阅/point_clouds_ori话题，接收点云数据
        rospy.Subscriber('/point_clouds_ori', PointCloud2, point_callback)
        publish_downsampled_pcd(pcd_file_path, publish_topic, map_topic, resolution)
    except rospy.ROSInterruptException:
        pass
