#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class AbdomenScenePublisher(Node):
    def __init__(self):
        super().__init__('abdomen_scene_publisher')
        
        # 创建发布器
        self.scene_pub = self.create_publisher(
            PlanningScene, 
            '/planning_scene', 
            10
        )
        
        # 等待 MoveIt 启动
        time.sleep(2.0)
        
        # 发布腹部场景
        self. publish_abdomen()
        
    def publish_abdomen(self):
        # 创建 PlanningScene 消息
        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # 表示这是增量更新
        
        # 创建腹部碰撞对象（一个长方体平面）
        abdomen = CollisionObject()
        abdomen.id = "patient_abdomen"
        abdomen.header.frame_id = "world"
        
        # 定义形状：长方体 (Box)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.4, 0.3, 0.05]  # 长40cm x 宽30cm x 厚5cm
        
        abdomen.primitives.append(box)
        
        # 定义位置：放在机械臂前方
        pose = Pose()
        pose.position.x = 0.5   # 机械臂前方 50cm
        pose.position.y = 0.0
        pose.position.z = 0.3   # 高度 30cm（模拟病床高度）
        pose.orientation.w = 1.0
        
        abdomen.primitive_poses.append(pose)
        abdomen.operation = CollisionObject. ADD
        
        # 添加到场景
        scene_msg. world. collision_objects.append(abdomen)
        
        # 发布场景
        self.scene_pub.publish(scene_msg)
        self.get_logger().info('✅ 腹部场景已添加到 MoveIt Planning Scene')
        
        # 可视化导管口（用一个小球表示）
        catheter = CollisionObject()
        catheter.id = "catheter_outlet"
        catheter.header. frame_id = "world"
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 半径 2cm
        
        catheter.primitives.append(sphere)
        
        catheter_pose = Pose()
        catheter_pose.position.x = 0.4  # 靠近腹部边缘
        catheter_pose.position.y = 0.0
        catheter_pose. position.z = 0.33  # 略高于腹部表面
        catheter_pose. orientation.w = 1.0
        
        catheter.primitive_poses.append(catheter_pose)
        catheter.operation = CollisionObject.ADD
        
        scene_msg2 = PlanningScene()
        scene_msg2.is_diff = True
        scene_msg2.world.collision_objects. append(catheter)
        
        time.sleep(0.5)
        self.scene_pub.publish(scene_msg2)
        self.get_logger().info('✅ 导管口标记已添加')

def main():
    rclpy.init()
    node = AbdomenScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()