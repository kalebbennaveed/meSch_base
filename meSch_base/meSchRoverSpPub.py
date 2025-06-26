import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import MeschMissionStatus
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from nav_msgs.msg import Path
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker

## python imports
import numpy as np


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

import numpy as np

def figure_eight(t, k=6.0):
    """Computes the figure-eight trajectory with velocity and acceleration.
    
    Args:
        t (float): Time parameter.
        k (float, optional): Scaling factor for speed. Default is 2.0.
    
    Returns:
        tuple: (x_des, y_des, z_des, phi_d)
            - x_des, y_des, z_des: NumPy arrays containing position, velocity, and acceleration.
            - phi_d: Desired yaw angle.
    """
    N = 1.3
    xd = N * np.sin(k * t / 10) - 0.3
    yd = N * np.sin(k * t / 20) + 0.3
    zd = 0.0

    # Velocity Desired
    xd_dot = k * N * np.cos(k * t / 10) / 10
    yd_dot = k * N * np.cos(k * t / 20) / 20
    zd_dot = 0.0

    # Acceleration Desired
    xd_ddot = -k**2 * N * np.sin(k * t / 10) / 100
    yd_ddot = -k**2 * N * np.sin(k * t / 20) / 400
    zd_ddot = 0.0

    phi_d = np.arctan2(yd_dot, xd_dot)

    pos_d = np.array([xd, yd, zd])
    vel_d = np.array([xd_dot, yd_dot, zd_dot])
    acc_d = np.array([xd_ddot, yd_ddot, zd_ddot])

    return pos_d, vel_d, acc_d, phi_d

class meSchRoverSetpointPubNode(Node):

    def __init__(self):
        super().__init__("meSch_rover_sp_node")

        # Declare all parameters
        self.get_logger().info("Initializing Rover Setpoint Publisher Node")
        self.mode = "Stopped"
        self.SetpointPub_update_rate = 50.0 # Hz


        # Declare all variables
        self.state_local_available = False
        self.SetpointPub_Timer_ = None
        self.RoverSetpointPubTimer_started = False

        self.start_time_ = None 
        self.goal_yaw = 0.0 # in the ENU frame

        
        # Pubs
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint", 10)

        # Subs
        
        # Vehicle Position sub
        self.sub_state_ = self.create_subscription( 
                VehicleLocalPosition,
                'px4_1/fmu/out/vehicle_local_position',
                self.state_callback,
                qos_profile_sensor_data)

        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            MeschMissionStatus,
            '/px4/gs/mesch_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )

        # Create a message instance and reuse it 
        self.px4_msg = TrajectorySetpoint()


        self.get_logger().info("Done initializing")   


    def state_callback(self, state_msg):

        if not (state_msg.xy_valid  and state_msg.z_valid  and state_msg.heading_good_for_control):
            return

        self.state_ = np.array([
                        state_msg.y, 
                        state_msg.x, 
                        -state_msg.z,
                        state_msg.vy,
                        state_msg.vx,
                        -state_msg.vz,
                        state_msg.ay,
                        state_msg.ax,
                        -state_msg.az])

        self.state_yaw_ = np.pi/2 - state_msg.heading
        self.state_local_available = True


    def mission_status_cb(self, mission_status_msg):   
        # self.get_logger().info("----- In mission_status_cb -----") 
        if (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False):
            # Create a local variale which tracks if this code triggered CandTraj_Timer_ to start
            # Start timer for the candidate trajectory generation [Pub]
            if self.RoverSetpointPubTimer_started == False:
                #### Start timer for the candidate trajectory generation [Pub]

                if self.state_local_available == True:
                    ## Start timer for the setpoint pub [local]
                    self.start_timers() 
                    self.RoverSetpointPubTimer_started = True
                else:
                    self.get_logger().info('Rover state not available yet')

        elif ((mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True)):
            if  self.RoverSetpointPubTimer_started == True:
                self.stop_timers()
                self.RoverSetpointPubTimer_started == False


    def start_timers(self):
       
        # Store ther start time
        self.start_time_ = time.time()
        
        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)
        self.get_logger().info('Timers Started')
        self.get_logger().info("Starting Mission")

    def stop_timers(self):
        if self.SetpointPub_Timer_ is not None:
            self.SetpointPub_Timer_.destroy()
            self.SetpointPub_Timer_ = None
        self.get_logger().info('Timers stopped')
        self.get_logger().info("Mission Stopped; Landing")     


    def Setpoint_callback(self):
        # self.get_logger().info('Publishing the new setpoint')
        self.t_k = time.time() - self.start_time_

        pos_d, vel_d, acc_d, phi_d = figure_eight(self.t_k)

        # Publish the setpoint]
        self.px4_msg.raw_mode = False;

        # Tajectorty Setpoint in in NED frame
        # Set pos
        self.px4_msg.position[0] = pos_d[1]
        self.px4_msg.position[1] = pos_d[0]
        self.px4_msg.position[2] = - pos_d[2]

        self.px4_msg.velocity[0] = vel_d[1]
        self.px4_msg.velocity[1] = vel_d[0]
        self.px4_msg.velocity[2] = - vel_d[2]
    
        self.px4_msg.acceleration[0] = acc_d[1]
        self.px4_msg.acceleration[1] = acc_d[0]
        self.px4_msg.acceleration[2] = - acc_d[2]
    
        self.px4_msg.jerk[0] = 0
        self.px4_msg.jerk[1] = 0
        self.px4_msg.jerk[2] = 0

        self.px4_msg.yaw = np.pi/2 - phi_d
        self.px4_msg.yawspeed = 0.0
        
        self.pub_px4_.publish(self.px4_msg)


def main(args = None):
    rclpy.init(args=args)
    node = meSchRoverSetpointPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()