#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from geometry_msgs.msg import Twist

class RobotCommander:
    
    # init
    def __init__(self, position: Twist) -> None:
        rclpy.init()
        self.navigator = BasicNavigator()
        initial_pose = self.create_pose_stamped(position.linear.x, position.linear.y, position.angular.z)
        self.navigator.waitUntilNav2Active()
        
    def __del__(self):

        rclpy.shutdown()

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose
    
    def direct_pathing(self, pose):
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(feedback)
    
    def waypoint_pathing(self, waypoints):
        
        for i in range(waypoints):
            self.navigator.followWaypoints(waypoints)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # print(feedback)

        #--- Get the result ---
        print(self.navigator.getResult())

def main():
    
    initial_position = Twist()
    initial_position.linear.x = 0.0
    initial_position.linear.y = 0.0
    initial_position.angular.z = 0.0
    
    commander = RobotCommander(initial_position)
    

    
    commander.__del__()

if __name__ == '__main__':
    main()