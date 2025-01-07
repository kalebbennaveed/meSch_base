import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DIState, DIAcc, DITrajectorySimple, EwareMissionStatus, SpToCand, CandToSp
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from nav_msgs.msg import Path
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker

## python imports
import numpy as np

## Julia call
from juliacall import Main as jl

'''
Flight Preview Current Version (V1):
- The flight preview button currently verifies the ros and px4 communication stack.
- Ensures that ros communication works fine
- Expected behaviour: The quadrotor will keep on starting candidate trajectory from the same initial state

Things to test:
- Mission turn on and off tested
- Timers truned on and off tested
- Timers testing (ONG)
- Julia code to generate the nominal trajectory
'''

class EwareSetpointPubNode(Node):

    def __init__(self):
        super().__init__("eware_sp_node")

        # Declare all parameters
        self.get_logger().info("Initializing Setpoint Publisher Node")
        self.mode = "Stopped"
        self.SetpointPub_update_rate = 50.0 # Hz
        self.discharge_rate = 0.677
        self.charge_duration = 5.0 # seconds

        # Declare all variables
        self.just_charged_wiating_traj = False
        self.committed_traj_available = False
        self.state_local_available = False
        self.SetpointPub_Timer_ = None
        self.CanTrajTimer_started = False
        self.SetpointPubTimer_started = False
        self.cand_precomp_done = False # Only init here; later replaced by the sub

        self.battery_SoC_cap_ = 15.0
        self.curr_SoC_ = self.battery_SoC_cap_
        self.start_time_ = None 
        self.goal_yaw = 0.0 # in the ENU frame
        self.visits = 0
        self.charge_start_time = 0.0


        # Julia
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        jl.seval("using DifferentialEquations, LinearAlgebra, BlockDiagonals, ForwardDiff, ControlSystems, ComponentArrays, Parameters, StaticArrays, Convex, ECOS")

        # Import modules
        self.eware = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        # self.dynamics = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        
        # Pubs
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_100/fmu/in/trajectory_setpoint", 10)
        self.pub_SpToCand = self.create_publisher(SpToCand, "/px4_100/SpToCand", 10)

        # Subs
        
        # Vehicle Position sub
        self.sub_state_ = self.create_subscription( 
                VehicleLocalPosition,
                'px4_100/fmu/out/vehicle_local_position',
                self.state_callback,
                qos_profile_sensor_data)

        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            EwareMissionStatus,
            '/px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )

        # Mission Status sub
        self.sub_cand_setpoint_status_ = self.create_subscription(
            CandToSp,
            '/px4_100/CandToSp', # Right now the name is fixed; TODO: Give as an arg
            self.CanToSp_cb,
            10
        )

        # Candidate trajectory Sub
        self.sub_cand_traj = self.create_subscription(
            DITrajectorySimple,
            '/px4_100/DI_committed_traj', # Right now the name is fixed; TODO: Give as an arg
            self.cand_traj_cb,
            10
        )

        # Create a message instance and reuse it 
        self.SpToCand = SpToCand()
        self.px4_msg = TrajectorySetpoint()

        # julia params
        self.charge_pos = jl.seval("charge_pos = [2.525; 2.992; 0.880]")

        # Julia varaible storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        self.get_logger().info("Done initializing")   


    def state_callback(self, state_msg):

        if not (state_msg.xy_valid  and state_msg.z_valid  and state_msg.heading_good_for_control):
            return
        ## Vehicle local position provides 
        # self.state = np.array([
        #     state_msg.y,
        #     state_msg.x,
        #     -state_msg.z,
        #     state_msg.vy,
        #     state_msg.vx,
        #     -state_msg.vz,
        #     state_msg.ay,
        #     state_msg.ax,
        #     -state_msg.az])

        # self.state_yaw = np.pi/2 - state_msg.heading # in ENU coords
        # self.get_logger().info("Getting new state")
        self.state_ = self.jlstore("state_", [
                                    state_msg.y, 
                                    state_msg.x, 
                                    -state_msg.z,
                                    state_msg.vy,
                                    state_msg.vx,
                                    -state_msg.vz,
                                    state_msg.ay,
                                    state_msg.ax,
                                    -state_msg.az])

        self.state_yaw_ = self.jlstore("state_yaw", np.pi/2 - state_msg.heading)
        self.state_local_available = True


    def mission_status_cb(self, mission_status_msg):   
        # self.get_logger().info("----- In mission_status_cb -----") 
        if (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False) and (self.visits < 2):
            # Create a local variale which tracks if this code triggered CandTraj_Timer_ to start
            # Start timer for the candidate trajectory generation [Pub]
            if self.CanTrajTimer_started == False and self.SetpointPubTimer_started == False and self.cand_precomp_done == True:
                #### Start timer for the candidate trajectory generation [Pub]

                # Check if the first committed trajectory is aviailable:
                if self.committed_traj_available == True and self.state_local_available == True:
                    ## Start timer for the setpoint pub [local]

                    ## Do the precompilation for the julia finctions local to this node
                    self.sp_julia_precomp()
                    self.start_timers() 
                    self.CanTrajTimer_started = True
                    self.SetpointPubTimer_started = True
                else:
                    self.get_logger().info('Candidate trajectory not available yet')

        elif ((mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True)) or (self.visits == 2):
            if self.CanTrajTimer_started == True and self.SetpointPubTimer_started == True:
                self.mode = "Stopped"
                self.stop_timers()
                self.CanTrajTimer_started = False
                self.SetpointPubTimer_started == False

    def CanToSp_cb(self, cand_to_sp_msg):
        # self.get_logger().info("----- In cand_setpoint_cb -----") 
        '''
        Callback to create a link between the two nodes
        '''
        self.cand_precomp_done = cand_to_sp_msg.cand_precomp_done
        self.cand_mode = cand_to_sp_msg.cand_mode

    def cand_traj_cb(self, DITraj_msg):
        '''
        Reads the candidate trajectory from cand_traj_msg so it can be used by the Setpoint_callback()
        '''
        # self.get_logger().info("----- In cand_traj_cb -----")
        # # Initialize empty lists to store the extracted data
        di_xs = []  # To store the states
        di_us = []  # To store the accelerations
        di_ts = []     # To store the times

        # Extract times
        di_ts = DITraj_msg.times  # Directly assigned since it's already a list

        # Extract states
        for state_msg in DITraj_msg.states:
            di_xs.append(state_msg.state)

        # Extract accelerations
        for acc_msg in DITraj_msg.accelerations:
            di_us.append(acc_msg.acceleration)

        # # dt can be used to construct timestamps from the times array if needed
        # dt = DITraj_msg.dt
        
        self.committed_tks_ = self.jlstore("committed_tks_", di_ts)
        self.committed_xks_ = self.jlstore("committed_xks_", di_xs)
        self.committed_uks_ = self.jlstore("committed_uks_", di_us)
        self.committed_traj_available = True
        # self.get_logger().info("----- Committed Traj Update -----")


    def start_timers(self):
       
        # First precompile the julia
        # Incase preompiling missed
        
        # Start mission and record start time
        self.mode = "Mission"
        self.SpToCand.cand_timer_state = True
        self.SpToCand.sp_mode = self.mode
        self.SpToCand.battery_soc_state = self.curr_SoC_
        
        # Store ther start time
        self.start_time_ = time.time()
        
        # Signal to EwareCandTraj Node to start the candidate trajectory node
        self.pub_SpToCand.publish(self.SpToCand)

        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)

        self.get_logger().info('Timers Started')
        self.get_logger().info("Starting Mission")

    def stop_timers(self):

        # Signal to stop the candidate trajectory timer in the other node
        self.SpToCand.cand_timer_state = False
        self.pub_SpToCand.publish(self.SpToCand)

        if self.SetpointPub_Timer_ is not None:
            self.SetpointPub_Timer_.destroy()
            self.SetpointPub_Timer_ = None
        self.get_logger().info('Timers stopped')
        self.get_logger().info("Mission Stopped; Landing")     

    def sp_julia_precomp(self):
        self.get_logger().info("Starting setpoint node Julia utilities precomp") 
        # Dry run the interpolation
        pre_t_k = self.jlstore("pre_t_k", 0.0)
        pre_di_state_sp_k, pre_di_acc_sp_k = jl.seval("pre_di_state_sp_k, pre_di_acc_sp_k = Eware.DI_interpolate_traj(pre_t_k, committed_tks_, committed_xks_, committed_uks_)")

        # Dry run the dist_to_charge_calc
        at_charging = jl.seval("at_charging = Eware.reached_charging_station(state_, charge_pos)")
        self.get_logger().info("Finished setpoint node Julia utilities precomp") 
        return

    def Setpoint_callback(self):
        '''
        Read the Candidate trajectory to publish the new setpoint at the current time


        1. Setpoint is always published
          - Setpoint from the candidate trajectory
            - This happens when quad is in the mission or landing mode
          - Setpoint with vel and acc == 0
            - This happends when quad is in the charging mode
        '''


        if (self.visits == 2):
            self.mode = "Stopped"
            self.stop_timers()
            return

        # self.get_logger().info('Publishing the new setpoint')

    
        self.t_k = self.jlstore("t_k", time.time() - self.start_time_)


        # Interpolate to get the setpoint
        # self.get_logger().info(f"Current_time: {time.time() - self.start_time_}")
        di_state_sp_k, di_acc_sp_k = jl.seval("di_state_sp_k, di_acc_sp_k = Eware.DI_interpolate_traj(t_k, committed_tks_, committed_xks_, committed_uks_)")

        # Check if the quadrotor is landing
        if self.cand_mode == "Landing" and self.mode == "Mission":
            # Start checking if the quadrotor has reached the charging station
            at_charging = jl.seval("at_charging = Eware.reached_charging_station(state_, charge_pos)")
            if (at_charging == 1):
                self.mode = "Charging"
                # Start the charging_time
                self.charge_start_time = time.time()
                self.curr_SoC_ = self.battery_SoC_cap_ # Restore battery level 

                # Publish the mesage to the candidate trajectory node
                self.SpToCand.sp_mode = self.mode
                self.SpToCand.battery_soc_state = self.curr_SoC_
                self.pub_SpToCand.publish(self.SpToCand)
                    
                self.get_logger().info(f"Charge_start_time: {self.charge_start_time}") 
        elif (self.mode == "Charging") and (time.time() >= (self.charge_start_time + self.charge_duration)):
            self.get_logger().info("---- Charging Done ---- ")
            self.mode = "Charged"
            self.just_charged_wiating_traj = True
            # Signal to the candidate trajectory to restart the mission
            self.SpToCand.sp_mode = self.mode
            self.pub_SpToCand.publish(self.SpToCand)
            self.visits += 1

        # self.get_logger().info(f"just_charged_wiating_traj: {self.just_charged_wiating_traj}") 
        if self.just_charged_wiating_traj == True and self.cand_mode == "Mission":
            self.get_logger().info(" -------Checking if just charged ------")
            self.get_logger().info(f"SP Mode: {self.mode}") 
            self.just_charged_wiating_traj = False
            self.mode = "Mission"
            self.SpToCand.sp_mode = self.mode
            self.get_logger().info(f"SP Mode: {self.mode}") 
            self.pub_SpToCand.publish(self.SpToCand)   

        # Publish the setpoint]
        self.px4_msg.raw_mode = False;

        # Tajectorty Setpoint in in NED frame
        # Set pos
        self.px4_msg.position[0] = di_state_sp_k[1]
        self.px4_msg.position[1] = di_state_sp_k[0]
        self.px4_msg.position[2] = - di_state_sp_k[2]
    
        self.px4_msg.jerk[0] = 0
        self.px4_msg.jerk[1] = 0
        self.px4_msg.jerk[2] = 0

        self.px4_msg.yaw = np.pi/2 - self.goal_yaw
        self.px4_msg.yawspeed = 0.0

        # set acc
        # self.get_logger().info(self.mode)

        # If the quad is charging give it zero
        # if the new candidate trajectory is not published yet give it zero

        if (self.mode == "Charging" or (self.just_charged_wiating_traj == True)):
            # The second condition ensures that quad only resume when the new candidate trajecytory is available
            # self.get_logger().info("Zeros for vel and acc")
            self.px4_msg.velocity[0] = 0.0
            self.px4_msg.velocity[1] = 0.0
            self.px4_msg.velocity[2] = 0.0

            self.px4_msg.acceleration[1] = 0.0
            self.px4_msg.acceleration[0] = 0.0
            self.px4_msg.acceleration[2] = 0.0

        else:
            # set vel
            # self.get_logger().info("Following Candidate")
            self.px4_msg.velocity[0] = di_state_sp_k[4]
            self.px4_msg.velocity[1] = di_state_sp_k[3]
            self.px4_msg.velocity[2] = - di_state_sp_k[5]

            # self.get_logger().info('New acc == from cand')
            self.px4_msg.acceleration[1] = di_acc_sp_k[0]
            self.px4_msg.acceleration[0] = di_acc_sp_k[1]
            self.px4_msg.acceleration[2] = -di_acc_sp_k[2]

            # Update the battery soc
            self.update_battery_soc()
            self.SpToCand.battery_soc_state = self.curr_SoC_
            self.pub_SpToCand.publish(self.SpToCand)
        
        self.pub_px4_.publish(self.px4_msg)

        # self.get_logger().info('Out Publishing the new setpoint')

    def update_battery_soc(self):
        self.curr_SoC_ = self.curr_SoC_ - (1 / self.SetpointPub_update_rate) * self.discharge_rate


def main(args = None):
    rclpy.init(args=args)
    node = EwareSetpointPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
