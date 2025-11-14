#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap
import octomap

class OctomapGenerator(Node):
    def __init__(self):
        super().__init__('octomap_generator')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/map',
            self.pointcloud_callback,
            10)
        self.octomap_pub = self.create_publisher(Octomap, '/octomap', 10)
        
        # 创建八叉树地图，分辨率为0.05米
        self.octree = octomap.OcTree(0.05)
        
    def pointcloud_callback(self, msg):
        # 将PointCloud2转换为八叉树地图
        # 这里需要实现PointCloud2到octomap的转换
        # 实际实现会更复杂，需要处理点云数据格式
        
        # 发布八叉树地图
        octomap_msg = Octomap()
        octomap_msg.header.frame_id = "map"
        octomap_msg.binary = True
        # 将octree序列化到消息中
        # 实际实现需要将octree转换为消息格式
        
        self.octomap_pub.publish(octomap_msg)
        
def main(args=None):
    rclpy.init(args=args)
    octomap_generator = OctomapGenerator()
    rclpy.spin(octomap_generator)
    octomap_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
