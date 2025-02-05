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
from dataclasses import dataclass
'''
Flight Preview Current Version (V1):
- Initialization: 

TODO
- Fix tha start timer issue 

'''
# Define a structure to hold the data for each node
DataEntry = namedtuple('DataEntry', ['quad_name', 'cand_id', 'remaining_flight_time'])

@dataclass
class QuadObj:
    quad_name: str  # Renamed from 'id' to avoid conflicts
    cand_id:int
    flight_mode: str
    remaining_flight_time: float
    mission: bool
    discharge_rate: float
    visited_times: int
    int_soc: float
    charge_start_time: float
    precomp_done:bool

class meSchBaseNode(Node):

    def __init__(self):
        super().__init__("meSch_base_node")

        # Declare all parameters
        self.get_logger().info("Initializing meSch base node")

        ## Variables
        self.quad_num = 3
        self.cand_traj_time = 10.0
        self.cand_replanning_time = 1.0
        self.req_gap = 10.0
        self.cand_id = 0
        
        px4_100_charge_pos = np.array([2.525, 2.992, 0.880])
        px4_101_charge_pos = np.array([2.525, 2.992, 0.880])
        px4_102_charge_pos = np.array([2.525, 2.992, 0.880])

        px4_100_int_soc = 80.0
        px4_101_int_soc = 80.0
        px4_102_int_soc = 80.0

        px4_100_discharge_rate = 0.667
        px4_101_discharge_rate = 0.667
        px4_102_discharge_rate = 0.667

        ## Create the quad objects
        self.quad_objects = [
            QuadObj("px4_100", 0, "Grounded", 120.0, False, px4_100_discharge_rate, 0, px4_100_int_soc, 0.0, False),
            QuadObj("px4_101", 0, "Grounded", 120.0, False, px4_101_discharge_rate, 0, px4_101_int_soc, 0.0, False),
            QuadObj("px4_102", 0, "Grounded", 120.0, False, px4_102_discharge_rate, 0, px4_102_int_soc, 0.0, False)
        ]

        ## Candidate trajectories pre-compilation (specific to Julia)
        self.px4_100_precomp_done = False
        self.px4_101_precomp_done = False
        self.px4_102_precomp_done = False

        # Pubs
        self.pub_px4_100_meSch = self.create_publisher(BaseToQuadMesch, "/px4_100/gs/meSch_base_central", 10)
        self.pub_px4_101_meSch = self.create_publisher(BaseToQuadMesch, "/px4_101/gs/meSch_base_central", 10)
        self.pub_px4_102_meSch = self.create_publisher(BaseToQuadMesch, "/px4_102/gs/meSch_base_central", 10)

        self.pub_meSch_vec = np.array([self.pub_px4_100_meSch,
                                    self.pub_px4_101_meSch,
                                    self.pub_px4_102_meSch])


        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            MeschMissionStatus,
            '/px4/gs/mesch_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )        

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
        self.px4_100_BaseToQuadMesch = BaseToQuadMesch()
        self.px4_101_BaseToQuadMesch = BaseToQuadMesch()
        self.px4_102_BaseToQuadMesch = BaseToQuadMesch()

        self.QuadToBaseMesch = QuadToBaseMesch()
        self.get_logger().info("Done initializing")   

    def mission_status_cb(self, mission_status_msg):
        # self.get_logger().info("Checking msg")
        # Mission not started yet
        if (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == False):
            self.precomp_done == (self.quad_objects[0].precomp_done && self.quad_objects[1].precomp_done && self.quad_objects[2].precomp_done)
            if self.precomp_done == False:
                self.get_logger().info("Precompilation not finished yet")
            else:
                self.get_logger().info("Precompilation done")  
        elif self.CandTrajTimer_started ==  True and self.SetpointPubTimer_started == False and self.precomp_done == True:
            ## Start the candidate trajectpry 
            # Start the timer for the candidate trajectory generation
            self.start_timers()
            self.CanTrajTimer_started = True
            self.SetpointPubTimer_started = True            

    def px4_100_meSch_cb(self, px4_100_meSch_msg):
        self.quad_objects[0].quad_id = px4_100_meSch_msg.quad_id
        self.quad_objects[0].cand_id = px4_100_meSch_msg.cand_id
        self.quad_objects[0].remaining_flight_time = px4_100_meSch_msg.remaining_flight_time
        self.quad_objects[0].precomp_done = px4_100_meSch_msg.precomp_done
        self.quad_objects[0].mission = px4_100_meSch_msg.mission
        add_quad_data(self.px4_100_meSch_quad_name, self.px4_100_meSch_cand_id, self.px4_100_meSch_remaining_flight_time)


    def px4_101_meSch_cb(self, px4_101_meSch_msg):
        self.quad_objects[1].quad_id = px4_101_meSch_msg.quad_id
        self.quad_objects[1].cand_id = px4_101_meSch_msg.cand_id
        self.quad_objects[1].remaining_flight_time = px4_101_meSch_msg.remaining_flight_time
        self.quad_objects[1].precomp_done = px4_101_meSch_msg.precomp_done
        self.quad_objects[1].mission = px4_101_meSch_msg.mission
        add_quad_data(self.px4_101_meSch_quad_name, self.px4_101_meSch_cand_id, self.px4_101_meSch_remaining_flight_time)

    def px4_102_meSch_cb(self, px4_102_meSch_msg):
        self.quad_objects[2].quad_id = px4_102_meSch_msg.quad_id
        self.quad_objects[2].cand_id = px4_102_meSch_msg.cand_id
        self.quad_objects[2].remaining_flight_time = px4_102_meSch_msg.remaining_flight_time
        self.quad_objects[2].precomp_done = px4_102_meSch_msg.precomp_done
        self.quad_objects[2].mission = px4_102_meSch_msg.mission
        add_quad_data(self.px4_102_meSch_quad_name, self.px4_102_meSch_cand_id, self.px4_102_meSch_remaining_flight_time)


    def start_timers(self):
        
        # First create the timer for the candidate trajectory
        self.px4_100_BaseToQuadMesch.sp_timer_state = True
        self.px4_100_BaseToQuadMesch.discharge_rate = self.quad_objects[0].discharge_rate

        self.px4_101_BaseToQuadMesch.sp_timer_state = True
        self.px4_101_BaseToQuadMesch.discharge_rate = self.quad_objects[1].discharge_rate

        self.px4_102_BaseToQuadMesch.sp_timer_state = True
        self.px4_102_BaseToQuadMesch.discharge_rate = self.quad_objects[2].discharge_rate
        
        # Start the committed trajectory generation callback 
        self.com_start_time_ = time.time() 
        self.ComTraj_Timer_ = self.create_timer(1 / self.ComTraj_update_rate, self.ComTraj_callback)

        # Publish to the Quad to start the setpoint node
        self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
        self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
        self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)


    def ComTraj_callback(self):
        # Generate candidate trajectories

        self.cand_id += 1
        self.BaseToQuadMesch.cand_id = self.cand_id
        self.BaseToQuadMesch.time = time.time() - self.com_start_time_

        # Track which quads are in a mission
        active_quads = [i for i, quad in enumerate(self.quad_objects) if quad.mission]

        # Publish to only those quads that are in a mission
        for i in active_quads:
            self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)

        # Wait until all active quads have updated their cand_id
        if active_quads:
            while not all(self.quad_objects[i].cand_id == self.cand_id for i in active_quads):
                self.get_logger().info("Precompilation not finished yet")
                rclpy.spin_once(self, timeout_sec=0.05)  # Check every 50 ms every cand_id updated


        # After exiting the while loop, compute gap flags for active robots
        if active_quads:
            # Sort the active quads based on remaining flight time
            gap_flags = []
            sorted_quad_data = [quad for i, quad in enumerate(self.quad_objects) if i in active_quads]
            sorted_quad_data = sorted(sorted_quad_data, key=lambda x: x.remaining_flight_time)

            self.get_logger().info('Sorted quad data:')
            for entry in sorted_quad_data:
                self.get_logger().info(f'{entry.quad_name}, {entry.cand_id}, {entry.remaining_flight_time}')

            # Compute the gap flags
            for i in range(len(sorted_quad_data)-1, 0, -1):
                gap_distance_i = sorted_quad_data[i].remaining_flight_time - (self.cand_traj_time + self.cand_replanning_time)
                gap_flag_i = gap_distance_i > ((i * self.req_gap))
                gap_flags.append(gap_flag_i)

            # Check if there is any gap violation
            gap_violation = False in gap_flags

            # Handle gap flag failure or success
            if gap_violation:
                self.get_logger().info(f'Gap flag failed; {sorted_quad_data[0].quad_name} landing')

                # Get the name of the returning quad
                returning_quad_name = sorted_quad_data[0].quad_name

                # Iterate over the publisher array to publish responses
                for i, entry in enumerate(sorted_quad_data):
                    # Reset cand_id for each publish
                    self.BaseToQuadMesch.cand_id = entry.cand_id

                    # Set commit_cand_traj to False for the returning quad, True for the others
                    self.BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)

                    # Publish to the respective quad based on the index
                    self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)

                    # Log the action
                    self.get_logger().info(f"Published to {entry.quad_name} with commit_cand_traj={self.BaseToQuadMesch.commit_cand_traj}")

            else:
                self.get_logger().info('All gaps met; Committing all')
                for i in range(len(self.pub_meSch_vec)):
                    self.BaseToQuadMesch.commit_cand_traj = True
                    self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)


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
            for i in active_quads:
                entry = self.quad_objects[i]
                self.BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)

                # Publish to the respective quad
                self.pub_meSch_vec[i].publish(self.BaseToQuadMesch)
        else:
            self.get_logger().info('All gaps met; Committing all')
            for i in range(len(self.pub_meSch_vec)):
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
