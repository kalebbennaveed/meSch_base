import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from dasc_msgs.msg import QuadToBaseMesch, BaseToQuadMesch
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker
from collections import namedtuple

## python imports
import numpy as np

'''
Flight Preview Current Version (V1):
- Initialization: 
'''
# Define a structure to hold the data for each node
DataEntry = namedtuple('DataEntry', ['quad_name', 'cand_id', 'remaining_flight_time'])


class meSchBaseNode(Node):

    def __init__(self):
        super().__init__("meSch_base_node")

        # Declare all parameters
        self.get_logger().info("Initializing meSch base node)

        ## Variables
        self.px4_100_meSch_quad_name = None
        self.px4_101_meSch_quad_name = None
        self.px4_102_meSch_quad_name = None
        self.quad_num = 3
        self.quad_data_entries = []
        self.cand_traj_time = 8.0
        self.cand_replanning_time = 1.0
        self.req_gap = 10.0
        self.gap_flags = []

        # Pubs
        self.pub_px4_100_meSch = self.create_publisher(BaseToQuadMesch, "/px4_100/gs/meSch_base_central", 10)
        self.pub_px4_101_meSch = self.create_publisher(BaseToQuadMesch, "/px4_101/gs/meSch_base_central", 10)
        self.pub_px4_102_meSch = self.create_publisher(BaseToQuadMesch, "/px4_102/gs/meSch_base_central", 10)

        self.pub_meSch_vec = np.array([self.pub_px4_100_meSch,
                                    self.pub_px4_101_meSch,
                                    self.pub_px4_102_meSch])

        # Subs
        self.sub_px4_100_meSch = self.create_subscription(
            QuadToBaseMesch,
            '/px4_100/meSch_quad_status',
            self.px4_100_meSch_cb,
            qos_profile_sensor_data)

        self.sub_px4_101_meSch = self.create_subscription(
            QuadToBaseMesch,
            '/px4_101/meSch_quad_status',
            self.px4_101_meSch_cb,
            qos_profile_sensor_data)

        self.sub_px4_102_meSch = self.create_subscription(
            QuadToBaseMesch,
            '/px4_102/meSch_quad_status',
            self.px4_102_meSch_cb,
            qos_profile_sensor_data)

        # Create a message instance and reuse it 
        self.BaseToQuadMesch = BaseToQuadMesch()
        self.QuadToBaseMesch = QuadToBaseMesch()
        self.get_logger().info("Done initializing")   

    def px4_100_meSch_cb(self, px4_100_meSch_msg):
        self.px4_100_meSch_quad_name = px4_100_meSch_msg.quad_name
        self.px4_100_meSch_cand_id = px4_100_meSch_msg.cand_id
        self.px4_100_meSch_remaining_flight_time = px4_100_meSch_msg.remaining_flight_time
        add_quad_data(self.px4_100_meSch_quad_name, self.px4_100_meSch_cand_id, self.px4_100_meSch_remaining_flight_time)


    def px4_101_meSch_cb(self, px4_101_meSch_msg):
        self.px4_101_meSch_quad_name = px4_101_meSch_msg.quad_name
        self.px4_101_meSch_cand_id = px4_101_meSch_msg.cand_id
        self.px4_101_meSch_remaining_flight_time = px4_101_meSch_msg.remaining_flight_time
        add_quad_data(self.px4_101_meSch_quad_name, self.px4_101_meSch_cand_id, self.px4_101_meSch_remaining_flight_time)

    def px4_102_meSch_cb(self, px4_102_meSch_msg):
        self.px4_102_meSch_quad_name = px4_102_meSch_msg.quad_name
        self.px4_102_meSch_cand_id = px4_102_meSch_msg.cand_id
        self.px4_102_meSch_remaining_flight_time = px4_102_meSch_msg.remaining_flight_time
        add_quad_data(self.px4_102_meSch_quad_name, self.px4_102_meSch_cand_id, self.px4_102_meSch_remaining_flight_time)

    def add_quad_data(self, quad_name, cand_id, remaining_flight_time):

        # Add data 
        self.quad_data_entries.append(DataEntry(quad_name, cand_id, remaining_flight_time))

        if len(self.quad_data_entries) == self.quad_num:
            if (all_same_cand_ids = len(set(entry.cand_id for entry in self.quad_data_entries)) == 1):
                self.meSch_central_base()
                self.quad_data_entries = []
            else:
                ## Hopefully this never gets executed
                self.get_logger().info('CAND_IDS ARE NOT SAME')



    def meSch_central_base(self):

        ## Sort the numbers
        gap_flags = []
        sorted_quad_data = sorted(self.quad_data_entries, key=lambda x:x.remaining_flight_time)
        self.get_logger().info('Sorted quad data:')
        for entry in sorted_quad_data:
            self.get_logger().info(f'{entry.quad_name}, {entry.cand_id}, {entry.remaining_flight_time}')

        ## Compute the gap flags
        for i in range(len(sorted_quad_data)-1, 0, -1):
            gap_distance_i = sorted_quad_data[i].remaining_flight_time - (self.cand_traj_time + self.cand_replanning_time)
            gap_flag_i = gap_distance_i > ((i * self.req_gap))
            gap_flags.append(gap_flag_i)

        ## Compute the 
        gap_violation = False in gap_flags

        if gap_violation == True:
            self.get_logger().info(f'Gap flag failed; {sorted_quad_data[0].quad_name} landing')

            ## Get the name of the returning quad
            returning_quad_name = sorted_quad_data[0].quad_name
            # Iterate over the publisher array

            # Publish the response back for quads
            for i, entry in enumerate(sorted_quad_data):
                # Reset cand_id for each publish
                self.BaseToQuadMesch.cand_id = entry.cand_id
                
                # Set commit_cand_traj to False for the returning quad, True for the others
                self.BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)

                # Publish to the respective quad based on the index
                self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)

                # Log the action
                self.get_logger().info(f"Published to {quad_names[i]} with commit_cand_traj={self.BaseToQuadMesch.commit_cand_traj}")

        else:
            self.get_logger().info('All gaps met; Committing all')
            for i in range(len(self.pub_meSch_vec)):
                self.BaseToQuadMesch.cand_id = entry.cand_id
                self.BaseToQuadMesch.commit_cand_traj = True 
                self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)

def main(args = None):
    rclpy.init(args=args)
    node = meSchBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
