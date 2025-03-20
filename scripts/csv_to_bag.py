#!/usr/bin/env python3
import rospy
import pandas as pd
import rosbag
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Header

def publish_ground_truth(csv_file, bag_file):
    rospy.init_node('csv_to_bag_node')
    pub = rospy.Publisher('/ground_truth', PoseStamped, queue_size=10)

    bag = rosbag.Bag(bag_file, 'w')

    rate = rospy.Rate(10)  
    data = pd.read_csv(csv_file)

    for index, row in data.iterrows():
        pose_msg = PoseStamped()
        
        pose_msg.header.stamp = rospy.Time(int(row['#sec']), int(row['nsec']))        
        pose_msg.header.frame_id = "map"  
        pose_msg.pose.position.x = row['x']
        pose_msg.pose.position.y = row['y']
        pose_msg.pose.position.z = row['z']
        pose_msg.pose.orientation.x = row['qx']
        pose_msg.pose.orientation.y = row['qy']
        pose_msg.pose.orientation.z = row['qz']
        pose_msg.pose.orientation.w = row['qw']
        pub.publish(pose_msg)
        bag.write('/ground_truth', pose_msg)

        rate.sleep()

    bag.close()

if __name__ == '__main__':
    try:
        csv_file = '/root/bagfiles/gt.csv' 
        bag_file = '/root/bagfiles/gt.bag'  
        publish_ground_truth(csv_file, bag_file)
    except rospy.ROSInterruptException:
        pass
