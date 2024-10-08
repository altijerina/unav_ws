#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    # --- init
    rclpy.init()
    nav = BasicNavigator()
    
    #--- Set Initial Pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w   
    #nav.setInitialPose(initial_pose)
    
    #--- Wait for Nav2
    nav.waitUntilNav2Active()
    
    #--- Send Nav2 Goal
    # PI == 3.14 = 180
    # PI/2 == 1.57 == 90
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.5
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w      
    nav.goToPose(goal_pose)
    
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        #print(feedback)
        
    print(nav.getResult())
    
    
    # --- Shutdown
    
    
    rclpy.shutdown()

if __name__=='__main__':
    main()