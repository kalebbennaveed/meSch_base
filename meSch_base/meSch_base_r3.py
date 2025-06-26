import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import threading

from dasc_msgs.msg import QuadToBaseMesch, BaseToQuadMesch, MeschMissionStatus
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker
from collections import namedtuple

## python imports
import numpy as np
from dataclasses import dataclass
'''
Flight Preview Current Version (V1):
- Initialization: All robots start with mission and then

TODO
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
    visited_times: int
    charge_start_time: float
    precomp_done:bool

class meSchBaseNode(Node):

    def __init__(self):
        super().__init__("meSch_base_node")

        # Declare all parameters
        self.get_logger().info("Initializing meSch base node for 3 quads")

        ## Variables
        self.quad_num = 1
        self.cand_traj_time = 14.0
        self.cand_replanning_time = 1.5
        self.req_gap = 25.0
        self.curr_cand_id = 0
        self.ComTraj_update_rate = 0.667
        self.precomp_done = False
    

        ##  Timers
        self.CandTrajTimer_started = False
        self.SetpointPubTimer_started = False
        self.evaluate_candidate_trajs = False

        ## Create the quad objects
        self.quad_objects = [
            QuadObj("px4_100", 0, "Grounded", 120.0, True, 0, 0.0, False),
            QuadObj("px4_101", 0, "Grounded", 120.0, True, 0, 0.0, False),
            QuadObj("px4_102", 0, "Grounded", 120.0, True, 0, 0.0, False),
        ]

        ## Candidate trajectories pre-compilation (specific to Julia)
        self.px4_100_precomp_done = False
        self.px4_101_precomp_done = False
        self.px4_102_precomp_done = False

        # Pubs
        self.pub_px4_100_meSch = self.create_publisher(BaseToQuadMesch, "/px4_100/gs/base_to_quad", 10)
        self.pub_px4_101_meSch = self.create_publisher(BaseToQuadMesch, "/px4_101/gs/base_to_quad", 10)
        self.pub_px4_102_meSch = self.create_publisher(BaseToQuadMesch, "/px4_102/gs/base_to_quad", 10)

        self.pub_meSch_vec = np.array([self.pub_px4_100_meSch,
                                    self.pub_px4_101_meSch,
                                    self.pub_px4_102_meSch])


        # Subs
        self.sub_px4_100_meSch = self.create_subscription(
            QuadToBaseMesch,
            "/px4_100/quad_to_base",
            self.px4_100_meSch_cb,
            qos_profile_sensor_data)

        # Subs
        self.sub_px4_101_meSch = self.create_subscription(
            QuadToBaseMesch,
            "/px4_101/quad_to_base",
            self.px4_101_meSch_cb,
            qos_profile_sensor_data)

        self.sub_px4_102_meSch = self.create_subscription(
            QuadToBaseMesch,
            "/px4_102/quad_to_base",
            self.px4_102_meSch_cb,
            qos_profile_sensor_data)


        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            MeschMissionStatus,
            '/px4/gs/mesch_mission_status',
            self.mission_status_cb,
            10
        )        


        # Create a message instance and reuse it 
        self.px4_100_BaseToQuadMesch = BaseToQuadMesch()
        self.px4_101_BaseToQuadMesch = BaseToQuadMesch()
        self.px4_102_BaseToQuadMesch = BaseToQuadMesch()
        # self.BaseToQuadMesch = BaseToQuadMesch()

        self.QuadToBaseMesch = QuadToBaseMesch()
        self.get_logger().info("Done initializing")   

    def px4_100_meSch_cb(self, px4_100_meSch_msg):
        self.quad_objects[0].quad_name = px4_100_meSch_msg.quad_name
        self.quad_objects[0].cand_id = px4_100_meSch_msg.cand_id
        self.quad_objects[0].remaining_flight_time = px4_100_meSch_msg.remaining_flight_time
        self.quad_objects[0].precomp_done = px4_100_meSch_msg.precomp_done
        self.quad_objects[0].mission = px4_100_meSch_msg.mission

        if self.evaluate_candidate_trajs:
            self.get_logger().info('Evaluation from px4_100')
            self.Evaluate_trajectories()


    def px4_101_meSch_cb(self, px4_101_meSch_msg):
        self.quad_objects[1].quad_name = px4_101_meSch_msg.quad_name
        self.quad_objects[1].cand_id = px4_101_meSch_msg.cand_id
        self.quad_objects[1].remaining_flight_time = px4_101_meSch_msg.remaining_flight_time
        self.quad_objects[1].precomp_done = px4_101_meSch_msg.precomp_done
        self.quad_objects[1].mission = px4_101_meSch_msg.mission

        if self.evaluate_candidate_trajs:
            self.get_logger().info('Evaluation from px4_101')
            self.Evaluate_trajectories()


    def px4_102_meSch_cb(self, px4_102_meSch_msg):
        self.quad_objects[2].quad_name = px4_102_meSch_msg.quad_name
        self.quad_objects[2].cand_id = px4_102_meSch_msg.cand_id
        self.quad_objects[2].remaining_flight_time = px4_102_meSch_msg.remaining_flight_time
        self.quad_objects[2].precomp_done = px4_102_meSch_msg.precomp_done
        self.quad_objects[2].mission = px4_102_meSch_msg.mission

        if self.evaluate_candidate_trajs:
            self.get_logger().info('Evaluation from px4_102')
            self.Evaluate_trajectories()

    def mission_status_cb(self, mission_status_msg):
        # self.get_logger().info("Checking msg")
        # Mission not started yet
        if (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == False) and (self.precomp_done == False):
            ## For test
            # self.get_logger().info(f"px4_100 Precompilation done: {self.quad_objects[0].precomp_done}")
            # self.get_logger().info(f"px4_101 Precompilation done: {self.quad_objects[1].precomp_done}")
            # self.get_logger().info(f"px4_102 Precompilation done: {self.quad_objects[1].precomp_done}")
            self.precomp_done = (self.quad_objects[0].precomp_done and self.quad_objects[1].precomp_done and self.quad_objects[2].precomp_done)
            if self.precomp_done == True:
                self.get_logger().info("Precompilation done") 

        # Mission started 
        elif (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False):
            if self.CandTrajTimer_started ==  False and self.SetpointPubTimer_started == False and self.precomp_done == True:
                ## Start the candidate trajectpry 
                # Start the timer for the candidate trajectory generation
                self.start_timers()
                self.CanTrajTimer_started = True
                self.SetpointPubTimer_started = True
        elif (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True):
            if self.CanTrajTimer_started == True and self.SetpointPubTimer_started == True: 
                self.stop_timers()
                self.CanTrajTimer_started = False
                self.SetpointPubTimer_started = False           


    def start_timers(self):
        
        # First create the timer for the candidate trajectory
        self.px4_100_BaseToQuadMesch.sp_timer_state = True
        self.px4_101_BaseToQuadMesch.sp_timer_state = True
        self.px4_102_BaseToQuadMesch.sp_timer_state = True
        # Start the committed trajectory generation callback 
        self.com_start_time_ = time.time() 
        self.ComTraj_Timer_ = self.create_timer(1 / self.ComTraj_update_rate, self.ComTraj_callback)

        # Publish to the Quad to start the setpoint node
        self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
        self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
        self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)


    def stop_timers(self):
        # Signal to stop the candidate trajectory timer in the other node
        self.px4_100_BaseToQuadMesch.sp_timer_state = False
        self.px4_101_BaseToQuadMesch.sp_timer_state = False
        self.px4_102_BaseToQuadMesch.sp_timer_state = False

        # Publish to the Quad to start the setpoint node
        self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
        self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
        self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)

        if self.ComTraj_Timer_ is not None:
            self.ComTraj_Timer_.destroy()
            self.ComTraj_Timer_ = None
        self.get_logger().info('Timers stopped')
        self.get_logger().info("Mission Stopped; Landing")  


    def ComTraj_callback(self):
        # Generate candidate trajectories

        self.curr_cand_id += 1
        curr_time = time.time() - self.com_start_time_
        self.px4_100_BaseToQuadMesch.cand_id = self.curr_cand_id
        self.px4_100_BaseToQuadMesch.cand_time = time.time() - self.com_start_time_
        self.get_logger().info(f'--Current Cand Time; {self.curr_cand_id}')

        # Track which quads are in a mission
        self.active_quads = [i for i, quad in enumerate(self.quad_objects) if quad.mission]
        self.get_logger().info(f'Active Quads; {self.active_quads}')

        # Publish to only those quads that are in a mission
        for i in self.active_quads:
            self.get_logger().info(f'[From base] Gen new set of candidate {i}') 
            if i == 0:
                self.px4_100_BaseToQuadMesch.cand_id = self.curr_cand_id
                self.px4_100_BaseToQuadMesch.cand_time = curr_time
                self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
                self.evaluate_candidate_trajs = True
                self.get_logger().info('PX4_100 Instructed to generate trajjectories') 
            if i == 1:
                self.px4_101_BaseToQuadMesch.cand_id = self.curr_cand_id
                self.px4_101_BaseToQuadMesch.cand_time = curr_time
                self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
                self.evaluate_candidate_trajs = True
                self.get_logger().info('PX4_101 Instructed to generate trajjectories') 
            if i == 2:
                self.px4_102_BaseToQuadMesch.cand_id = self.curr_cand_id
                self.px4_102_BaseToQuadMesch.cand_time = curr_time
                self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)
                self.evaluate_candidate_trajs = True
                self.get_logger().info('PX4_102 Instructed to generate trajjectories') 


    def Evaluate_trajectories(self):
        if (all(self.quad_objects[i].cand_id == self.curr_cand_id for i in self.active_quads)) == True:
            self.get_logger().info(f'Evaluating candidate trajectories of ID; {self.curr_cand_id}')
            self.get_logger().info(f'==Evaluation Time; {time.time() - self.com_start_time_}')

            if len(self.active_quads) > 1:
                # Check gap flags otherwise no need to check gap flag
                gap_flags = []

                sorted_quad_data = [quad for i, quad in enumerate(self.quad_objects) if i in self.active_quads]
                sorted_quad_data = sorted(sorted_quad_data, key=lambda x: x.remaining_flight_time)

                self.get_logger().info('Sorted quad data:')
                for entry in sorted_quad_data:
                    self.get_logger().info(f'{entry.quad_name}, {entry.cand_id}, {entry.remaining_flight_time}')

                # Compute the gap flags
                for i in range(len(sorted_quad_data)-1, 0, -1):
                    gap_distance_i = sorted_quad_data[i].remaining_flight_time - (self.cand_traj_time - self.cand_replanning_time)
                    gap_flag_i = gap_distance_i > ((i * self.req_gap))
                    gap_flags.append(gap_flag_i)
                    self.get_logger().info(f"i: {i}, gap_distance: {gap_distance_i:.2f}, req_gap * i: {(i * self.req_gap):.2f}, gap_flag: {gap_flag_i}")


                # Check if there is any gap violation
                gap_violation = False in gap_flags

                # Handle gap flag failure or success
                if gap_violation:
                    self.get_logger().info(f'Gap flag failed; {sorted_quad_data[0].quad_name} landing')

                    # Get the name of the returning quad
                    returning_quad_name = sorted_quad_data[0].quad_name

                    # Publish the response back for quads
                    for i in self.active_quads:
                        entry = self.quad_objects[i]
                        if i == 0:
                            self.px4_100_BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)
                            self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch) 
                        if i == 1:
                            self.px4_101_BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)
                            self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch) 
                        if i == 2:
                            self.px4_102_BaseToQuadMesch.commit_cand_traj = (entry.quad_name != returning_quad_name)
                            self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch) 
                else:
                    self.get_logger().info('All gaps met; Committing all')
                    for i in self.active_quads:
                        if i == 0:
                            self.px4_100_BaseToQuadMesch.commit_cand_traj = True
                            self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
                        elif i == 1:
                            self.px4_101_BaseToQuadMesch.commit_cand_traj = True
                            self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
                        elif i == 2:
                            self.px4_102_BaseToQuadMesch.commit_cand_traj = True
                            self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)

                self.evaluate_candidate_trajs = False

            else:
                # Publish true to gap requirement met
                for i in self.active_quads:
                    if i == 0:
                        self.px4_100_BaseToQuadMesch.commit_cand_traj = True
                        self.pub_px4_100_meSch.publish(self.px4_100_BaseToQuadMesch)
                    elif i == 1:
                        self.px4_101_BaseToQuadMesch.commit_cand_traj = True
                        self.pub_px4_101_meSch.publish(self.px4_101_BaseToQuadMesch)
                    elif i == 2:
                        self.px4_102_BaseToQuadMesch.commit_cand_traj = True
                        self.pub_px4_102_meSch.publish(self.px4_102_BaseToQuadMesch)
                    

        else:
            self.get_logger().info('All candidates not received yet')
            return 

        # After exiting the while loop, compute gap flags for active robots


def main(args = None):
    rclpy.init(args=args)
    node = meSchBaseNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # rclpy.init(args=args)
    # node = meSchBaseNode()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    # try:
    #     executor.spin()  # Multi-threaded execution for timers and subscribers
    # except KeyboardInterrupt:
    #     pass  # Gracefully handle Ctrl+C
    # finally:
    #     node.destroy_node()  # Ensure the node is properly destroyed
    #     rclpy.shutdown()  # Shutdown ROS2 cleanl

if __name__ == '__main__':
    main()
