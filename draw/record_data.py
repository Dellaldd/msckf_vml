#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import euler_from_quaternion
import time


class Logger:
    def __init__(self):
        self.f = open("draw/data/test.txt", 'w')
        self.msckf_pos = [str(0),str(0),str(0)]
        self.gt_pos = [str(1),str(0),str(0)]
        self.msckf_vel = [str(0),str(0),str(0)]
        self.gt_vel = [str(0),str(0),str(0)]
        self.msckf_att = [str(0),str(0),str(0)]
        self.gt_att = [str(0),str(0),str(0)]
        self.msckf_bodyrate = [str(0),str(0),str(0)]
        self.gt_bodyrate = [str(0),str(0),str(0)]
        self.count = 0
        
        self.start_time = 0 
        self.cur_time = 0

        rospy.Subscriber("/msckf/odom", Odometry,self.msckfCb)
        rospy.Subscriber("/msckf/ground_truth_path",Path,self.gtCb)
        
        
    def write_data(self):
        if self.count != 0:
            self.cur_time = time.clock()
            self.f.write(str(self.cur_time-self.start_time))
            self.f.write(',')
            self.f.write(','.join(self.msckf_pos))
            self.f.write(',')
            self.f.write(','.join(self.gt_pos))
            self.f.write(',')
            self.f.write(','.join(self.msckf_vel))
            self.f.write(',')
            self.f.write(','.join(self.gt_vel))
            self.f.write(',')
            self.f.write(','.join(self.msckf_att))
            self.f.write(',')
            self.f.write(','.join(self.gt_att))
            self.f.write(',')
            self.f.write(','.join(self.msckf_bodyrate))
            self.f.write(',')
            self.f.write(','.join(self.gt_bodyrate))
            self.f.write('\r\n')
        self.count += 1

    def write_title(self):
        self.f.write("time,x,y,z,target_x,target_y,target_z,vel_x,vel_y,vel_z,target_vel_x,target_vel_y,target_vel_z,att_x,att_y,att_z,target_att_x,target_att_y,target_att_z,rate_x,rate_y,rate_z,target_rate_x,target_rate_y,target_rate_z")
        self.f.write('\r\n')

    def msckfCb(self,msg):
        self.msckf_pos = [str(msg.pose.pose.position.x), str(msg.pose.pose.position.y), str(msg.pose.pose.position.z)]
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.msckf_att = euler_from_quaternion(quaternion)
        self.msckf_att = list(map(str,self.msckf_att))

    def gtCb(self,msg):
        gt_pose = msg.poses.pop()
        self.gt_pos = [str(gt_pose.pose.position.x), str(gt_pose.pose.position.y), str(gt_pose.pose.position.z)]
        quaternion = [gt_pose.pose.orientation.x,gt_pose.pose.orientation.y,gt_pose.pose.orientation.z,gt_pose.pose.orientation.w]
        self.gt_att = euler_from_quaternion(quaternion)
        self.gt_att = list(map(str,self.gt_att))
        
def main():
    print("start log!")
    rospy.init_node('logger_node', anonymous=True)
    logger = Logger()
    logger.write_title()
    rate = rospy.Rate(29)
    logger.start_time = time.clock()

    while not rospy.is_shutdown():
        logger.write_data()
        rate.sleep()
    logger.f.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass