#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def main():
    # --- init
    rclpy.init()
    init_pose = True
    nav = BasicNavigator()
    
    #--- Set Initial Pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    
    #--- Wait for Nav2
    nav.waitUntilNav2Active()
    
    #--- Send Nav2 Goal
    # PI == 3.14 = 180
    # PI/2 == 1.57 == 90
    waypoints = []
    waypoints.append(create_pose_stamped(nav, 2.0, -2.0, 1.57))
    waypoints.append(create_pose_stamped(nav, 4.0, 0.8, 0.0))
    waypoints.append(create_pose_stamped(nav, 8.0, 1.0, -1.57))
    waypoints.append(create_pose_stamped(nav, 8.0, -0.5, 1.57))
    waypoints.append(create_pose_stamped(nav, 5.0, 5.0, 3.14))
    waypoints.append(create_pose_stamped(nav, 3.0, 4.0, 1.57))
    waypoints.append(create_pose_stamped(nav, 4.0, 5.0, 0.0))
    waypoints.append(create_pose_stamped(nav, 5.0, 3.0, -1.57))
    waypoints.append(create_pose_stamped(nav, 4.0, 0.8, 3.14))
    waypoints.append(create_pose_stamped(nav, -4.0, 3.5, -1.57))
    waypoints.append(create_pose_stamped(nav, -4.0, 0.0, 1.57))
    waypoints.append(create_pose_stamped(nav, -4.0, 3.5, 1.57))
    waypoints.append(create_pose_stamped(nav, 0.0, 4.5, 3.14))
    waypoints.append(create_pose_stamped(nav, 3.0, 0.5, 0))
    
    #--- Go to One Pose
    # nav.goToPose(goal_pose1)    
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     #print(feedback)
        
    #--- Follow Waypoints
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        #print(feedback) 
    
        
    print(nav.getResult())
    
    
    # --- Shutdown
    
    
    rclpy.shutdown()

if __name__=='__main__':
    main()