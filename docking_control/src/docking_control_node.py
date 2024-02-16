#!/usr/bin/env python3

import rospy
from docking_control.srv import DockingControll
from std_msgs.msg import Bool
import time


class DockingService:
    def __init__(self):
        self.publisher_low_power_status = rospy.Publisher('battery_charging_mode', Bool, queue_size=10)
        self.battery_low_power_status = True
    
    def start_charging(self):
        self.battery_low_power_status = True
        for _ in range(1,10):
            self.publisher_low_power_status.publish(True)

    def waiting_for_charging(self):
        rospy.loginfo("Waiting for publisher to be available...")
        rospy.wait_for_message('battery_low_power_status', Bool)
        while self.battery_low_power_status:
            time.sleep(1)
        if self.battery_low_power_status == False:
            return True
        else:
            return False

    def flag_check(self, msg):
        self.battery_low_power_status = msg.data
        
    
    def callback(self, msg):

        # code for aligning the robot to the docking station, if the alignment is correct execute the nex line

        #start charging the battery, 
        self.start_charging()
        #wait to complete charging
        self.waiting_for_charging()

        #when fully charged return to clinet with flag 1 - for success, flag 0 - for fail
        if self.self.battery_low_power_status == True:
            self.publisher_low_power_status.publish(False)
            return 1
        else:
            return 0
        

    def run(self):
        rospy.Subscriber('battery_low_power_status', Bool, self.flag_check)
        s = rospy.Service('docking_service', DockingControll, self.callback)
        rospy.loginfo("Docking_controll_live")
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('docking_control')
    docking = DockingService()
    docking.run()
    
    
    
