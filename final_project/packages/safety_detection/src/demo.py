#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from safety_detection.srv import SetString

class Demo(DTROS):
    def __init__(self, node_name):
        super(Demo, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        rospy.wait_for_service(f'/{self.vehicle_name}/part_one')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_two')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_three')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_four')
        self.part_one_service = rospy.ServiceProxy(f'/{self.vehicle_name}/part_one', SetString)
        self.part_two_service = rospy.ServiceProxy(f'/{self.vehicle_name}/part_two', SetString)
        self.part_three_service = rospy.ServiceProxy(f'/{self.vehicle_name}/part_three', SetString)
        self.part_four_service = rospy.ServiceProxy(f'/{self.vehicle_name}/part_four', SetString)
    
    def demo(self):
        # Call the services in order
        self.part_one_service("")
        rospy.sleep(2)
        self.part_two_service("")
        rospy.sleep(2)
        self.part_three_service("")
        rospy.sleep(2)
        self.part_four_service("")
        rospy.sleep(2)

    def on_shutdown(self):
        # on shutdown
        pass

if __name__ == '__main__':
    node = Demo(node_name='demo')
    rospy.sleep(2)
    node.demo()
    rospy.spin()
