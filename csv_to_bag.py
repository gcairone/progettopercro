#!/usr/bin/env python3
import rospy
import pandas as pd
import rosbag
from geometry_msgs.msg import PoseStamped  # Scegli il tipo di messaggio che preferisci
from std_msgs.msg import Header

def publish_ground_truth(csv_file, bag_file):
    # Inizializza il nodo ROS
    rospy.init_node('csv_to_bag_node')
    pub = rospy.Publisher('/ground_truth', PoseStamped, queue_size=10)

    # Inizia la registrazione in bag
    bag = rosbag.Bag(bag_file, 'w')

    rate = rospy.Rate(10)  # Frequenza di pubblicazione (puoi modificarla se necessario)

    # Leggi i dati dal file CSV
    data = pd.read_csv(csv_file)

    for index, row in data.iterrows():
        pose_msg = PoseStamped()
        
        # Imposta il timestamp usando i valori sec e nsec dal CSV
        pose_msg.header.stamp = rospy.Time(int(row['#sec']), int(row['nsec']))        
        pose_msg.header.frame_id = "map"  # Cambia se necessario

        # Imposta la posizione
        pose_msg.pose.position.x = row['x']
        pose_msg.pose.position.y = row['y']
        pose_msg.pose.position.z = row['z']
        
        # Imposta l'orientamento
        pose_msg.pose.orientation.x = row['qx']
        pose_msg.pose.orientation.y = row['qy']
        pose_msg.pose.orientation.z = row['qz']
        pose_msg.pose.orientation.w = row['qw']

        # Pubblica il messaggio
        pub.publish(pose_msg)

        # Scrivi nel bag
        bag.write('/ground_truth', pose_msg)

        rate.sleep()

    bag.close()

if __name__ == '__main__':
    try:
        csv_file = '/root/bagfiles/gt.csv'  # Specifica il percorso del tuo file CSV
        bag_file = '/root/bagfiles/gt.bag'  # Specifica dove vuoi salvare il bag
        publish_ground_truth(csv_file, bag_file)
    except rospy.ROSInterruptException:
        pass
