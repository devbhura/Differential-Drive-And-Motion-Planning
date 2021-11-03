#!/usr/bin/env python

import rospy
import unittest
import rostest
from geometry_msgs.msg import Point
from arm_move.srv import reset, step, follow, stepResponse

class Arm(unittest.TestCase):
    def __init__(self,*args):
        super(Arm,self).__init__(*args)
        rospy.init_node("test_client")
        # self.test_step(self)
        
        
        
    def test_step_fail(self):
        
        # rospy.sleep(1)
        rospy.wait_for_service('/px100/reset')
        self.reset = rospy.ServiceProxy('/px100/reset', reset)
        resp = self.reset(clear_waypoints:=False)

        rospy.wait_for_service('/px100/step')
        self.step = rospy.ServiceProxy('/px100/step',step)
        check_point = Point()
        check_point.x = -0.1
        check_point.y = -0.1
        check_point.z = -0.1
        

        error_code = self.step(point = check_point, gripper_pose=True)
        self.assertEqual(error_code.error.val,-1)
    
    def test_step_success(self):
        
        # rospy.sleep(1)
        rospy.wait_for_service('/px100/reset')
        self.reset = rospy.ServiceProxy('/px100/reset', reset)
        resp = self.reset(clear_waypoints:=False)

        rospy.wait_for_service('/px100/step')
        self.step = rospy.ServiceProxy('/px100/step',step)
        success_point = Point()
        success_point.x = 0.1
        success_point.y = 0.1
        success_point.z = 0.1
        

        error_code = self.step(point = success_point, gripper_pose=True)
        self.assertEqual(error_code.error.val,1)




if __name__ == "__main__":
    import rostest
    rostest.rosrun('arm_move', 'arm', Arm)