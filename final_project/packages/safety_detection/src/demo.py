#!/usr/bin/env python3
import os
import signal
import rospy
from duckietown.dtros import DTROS, NodeType
from safety_detection.srv import SetString


def on_shutdown():
    rospy.loginfo("Shutting down all nodes.")

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

        rospy.wait_for_service(f'/{self.vehicle_name}/part_one_shutdown')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_two_shutdown')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_three_shutdown')
        rospy.wait_for_service(f'/{self.vehicle_name}/part_four_shutdown')
        rospy.wait_for_service(f'/{self.vehicle_name}/camera_shutdown')
        rospy.wait_for_service(f'/{self.vehicle_name}/odometry_shutdown')
        self.shutdown_services = []
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/part_one_shutdown', SetString))
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/part_two_shutdown', SetString))
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/part_three_shutdown', SetString))
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/part_four_shutdown', SetString))
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/camera_shutdown', SetString))
        self.shutdown_services.append(rospy.ServiceProxy(f'/{self.vehicle_name}/odometry_shutdown', SetString))
        self.node_names = ["Part One", "Part Two", "Part Three", "Part Four", "Camera", "Odometry"]
    
    def shutdown_all_nodes(self):
        for node_name, shutdown_service in zip(self.node_names, self.shutdown_services):
            try:
                shutdown_service("")
            except:
                print(f"Successfully shutdown {node_name}!")
        print(f"Successfully shutdown Demo Node!")
        rospy.signal_shutdown("Shutting Down Demo!")
        
    def demo(self):
        # Call the services in order
        #self.part_one_service("")
        #rospy.sleep(2)
        #self.part_two_service("")
        #rospy.sleep(2)
        #self.part_three_service("")
        #rospy.sleep(2)
        self.part_four_service("")
        rospy.sleep(2)
        self.shutdown_all_nodes()

if __name__ == '__main__':
    node = Demo(node_name='demo')
    rospy.sleep(5)
    node.demo()
    rospy.spin()