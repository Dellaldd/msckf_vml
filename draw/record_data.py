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
        self.f = open("/home/nesc/ldd/msckf_vml/src/msckf_mono/draw/data/vml_1.txt", 'w')
        self.msckf_pos = [str(0),str(0),str(0)]
        self.vml_pos = [str(0),str(0),str(0)]
        self.gt_pos = [str(1),str(0),str(0)]
        
        self.msckf_att = [str(0),str(0),str(0)]
        self.gt_att = [str(0),str(0),str(0)]
        self.vml_att = [str(0),str(0),str(0)]
        
        self.gt_time = 0
        self.msckf_time = 0
        self.vml_time = 0
        
        rospy.Subscriber("/msckf/odom", Odometry,self.msckfCb)
        rospy.Subscriber("/msckf_vml/odom_vml", Odometry,self.vmlCb)
        rospy.Subscriber("/msckf/ground_truth_path",Path,self.gtCb)
        
        self.count = 0
        
    def write_data(self):
        if self.count != 0:
            self.f.write(str(self.gt_time))#0
            self.f.write(',')
            self.f.write(','.join(self.gt_pos))#1,2,3
            self.f.write(',')
            self.f.write(','.join(self.gt_att))#4,5,6
            self.f.write(',')
            self.f.write(str(self.msckf_time))#7
            self.f.write(',')
            self.f.write(','.join(self.msckf_pos))#8,9,10
            self.f.write(',')
            self.f.write(','.join(self.msckf_att))#11,12,13
            self.f.write(',')
            self.f.write(str(self.vml_time))#14
            self.f.write(',')
            self.f.write(','.join(self.vml_pos))#15,16,17
            self.f.write(',')
            self.f.write(','.join(self.vml_att))#18,19,20
            self.f.write('\r\n')
        self.count += 1

    def write_title(self):
        self.f.write("time,x,y,z,target_x,target_y,target_z,vel_x,vel_y,vel_z,target_vel_x,target_vel_y,target_vel_z,att_x,att_y,att_z,target_att_x,target_att_y,target_att_z,rate_x,rate_y,rate_z,target_rate_x,target_rate_y,target_rate_z")
        self.f.write('\r\n')

    def msckfCb(self,msg):
        self.msckf_time = msg.header.stamp.to_sec()
        self.msckf_pos = [str(msg.pose.pose.position.x), str(msg.pose.pose.position.y), str(msg.pose.pose.position.z)]
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.msckf_att = euler_from_quaternion(quaternion)
        self.msckf_att = list(map(str,self.msckf_att))
        
    def vmlCb(self,msg):
        self.vml_time = msg.header.stamp.to_sec()
        self.vml_pos = [str(msg.pose.pose.position.x), str(msg.pose.pose.position.y), str(msg.pose.pose.position.z)]
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.vml_att = euler_from_quaternion(quaternion)
        self.vml_att = list(map(str,self.vml_att))

    def gtCb(self,msg):
        gt_pose = msg.poses.pop()
        self.gt_time = gt_pose.header.stamp.to_sec()
        self.gt_pos = [str(gt_pose.pose.position.x), str(gt_pose.pose.position.y), str(gt_pose.pose.position.z)]
        quaternion = [gt_pose.pose.orientation.x,gt_pose.pose.orientation.y,gt_pose.pose.orientation.z,gt_pose.pose.orientation.w]
        self.gt_att = euler_from_quaternion(quaternion)
        self.gt_att = list(map(str,self.gt_att))
        
def main():
    print("start log!")
    rospy.init_node('logger_node', anonymous=True)
    logger = Logger()
    logger.write_title()
    rate = rospy.Rate(20)
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