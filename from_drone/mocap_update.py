
import rclpy
import threading
from rclpy.node import Node
import math
import numpy as np
from px4_msgs.msg import VehicleOdometry, TimesyncStatus, VehicleCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class Mocap_pose(Node):

    def __init__(self):
        super().__init__('mocap_pose')

        # Publishers
        self.create_subscription(PoseStamped,"/optitrack_pose",self.pose_update,10)
        # /drone1/path is a nav_msgs/Path (the whole planned route, published
        # once per goal) - subscribing to it as PoseStamped is a type
        # mismatch (shows up as two message types on one topic name and the
        # subscription never receives). /drone1/current_target is the single
        # active waypoint, republished continuously by formation_node.
        self.create_subscription(PoseStamped,"/drone1/current_target",self.target_pose,10)

        self.create_subscription(TimesyncStatus,"/fmu/out/timesync_status",self.time_update,10)
        self.odometry_pub = self.create_publisher(VehicleOdometry,"/fmu/in/vehicle_mocap_odometry",10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)

        self.target_setpoint = self.create_publisher(PoseStamped,'/new_target',10)
        self.out_of_bound_flag = self.create_publisher(Bool,'/boundry_check',10)
        self.connection_status = self.create_publisher(Bool, '/connection_fail_status',10)


        #self.out_of_bound= False


        self.declare_parameter("marker_offset_x",-0.01)
        self.declare_parameter("marker_offset_y",0.0)
        self.declare_parameter("marker_offset_z",-0.03)
        self.declare_parameter("marker_rotation_at_x",90.0)
        self.declare_parameter("marker_rotation_at_y",0.0)
        self.declare_parameter("marker_rotation_at_z",0.0)
        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.timestamp = 0
        self.get_logger().info("Optitrack now sending position and orientation")

        self.initial_mocap_pose=np.array([0.0,0.0,0.0])
        self.initial_mocap_orientation= np.array([0.0,0.0,0.0,0.0])
        self.curr_pose=np.array([0.0,0.0,0.0])
        self.last_pose=np.array([0.0,0.0,0.0])
        self.connection_lost=False

        self.connection_check_wait = 0


        x=self.get_parameter("marker_offset_x").value
        y=self.get_parameter("marker_offset_y").value
        z=self.get_parameter("marker_offset_z").value
        x_rotation_degrees=self.get_parameter("marker_rotation_at_x").value
        y_rotation_degrees=self.get_parameter("marker_rotation_at_y").value
        z_rotation_degrees=self.get_parameter("marker_rotation_at_z").value
        x_rotation_ratation=math.radians(x_rotation_degrees)
        y_rotation_ratation=math.radians(y_rotation_degrees)
        z_rotation_ratation=math.radians(z_rotation_degrees)

        # this matrix is use when in optitrack x is forward and z is right

        self.Rmat_drone_local_2_marker_x= np.array([[1.0, 0.0,0.0],
                                                  [0.0, math.cos(x_rotation_ratation), -math.sin(x_rotation_ratation)],
                                                  [0.0, math.sin(x_rotation_ratation),math.cos(x_rotation_ratation)]])

        self.Rmat_drone_local_2_marker_y= np.array([[math.cos(y_rotation_ratation), 0.0,math.sin(y_rotation_ratation)],
                                                    [0.0, 1.0, 0.0],
                                                    [-math.sin(y_rotation_ratation), 0.0,math.cos(y_rotation_ratation)]])

        self.Rmat_drone_local_2_marker_z= np.array([[math.cos(z_rotation_ratation), -math.sin(z_rotation_ratation),0.0],
                                                  [math.sin(z_rotation_ratation), math.cos(z_rotation_ratation), 0.0],
                                                  [0.0, 0.0,1.0]])

        # Rot_mat = Roll X Pitch X Yaw
        self.Rmat_drone_local_2_marker= self.Rmat_drone_local_2_marker_x@self.Rmat_drone_local_2_marker_y@self.Rmat_drone_local_2_marker_z


        # self.Rmat_drone_local_2_marker= np.array([[1.0, 0.0,0.0],
        #                                          [0.0, math.cos(x_rotation_ratation), -math.sin(x_rotation_ratation)],
        #                                          [0.0, math.sin(x_rotation_ratation),math.cos(x_rotation_ratation)]])
        #self.marker_offset = np.array([-0.01,0.0,-0.03]) #forward is x, right is y, and down is z

        self.marker_offset = np.array([x,y,z]) #forward is x, right is y, and down is z
        self.z = np.array([self.Rmat_drone_local_2_marker[0,2],self.Rmat_drone_local_2_marker[1,2],self.Rmat_drone_local_2_marker[2,2]])

        self.Tmat_drone_local_2_marker = np.array([[self.Rmat_drone_local_2_marker[0,0],self.Rmat_drone_local_2_marker[0,1],self.Rmat_drone_local_2_marker[0,2], self.marker_offset[0]],
                                                   [self.Rmat_drone_local_2_marker[1,0],self.Rmat_drone_local_2_marker[1,1],self.Rmat_drone_local_2_marker[1,2], self.marker_offset[1]],
                                                   [self.Rmat_drone_local_2_marker[2,0],self.Rmat_drone_local_2_marker[2,1],self.Rmat_drone_local_2_marker[2,2], self.marker_offset[2]],
                                                  [0.0,0.0,0.0,1.0]])
        
        # Drone pose in marker frame (T_A1D1)
        self.Tmat_marker_2_drone_local = np.linalg.inv(self.Tmat_drone_local_2_marker)
        self.first_pose=True

    def boundry_check(self,x,y,z):
        self.curr_pose=np.array([x,y,z])
        # self.curr_y=y
        # self.curr_z=z

        horizontal_bound = np.sqrt(x*x+y*y)
        if horizontal_bound>2.5 or z<-2.0:
            return True
        else:
            return False

    def timer_callback(self):
        connection_lost=Bool()
        if self.connection_check_wait<50:
            self.connection_check_wait += 1
            connection_lost.data=False
        else:
            if np.array_equal(self.curr_pose,self.last_pose):
                connection_lost.data=True
            else:
                self.last_pose[0] =self.curr_pose[0] 
                self.last_pose[1] =self.curr_pose[1] 
                self.last_pose[2] =self.curr_pose[2] 
                connection_lost.data=False

        self.connection_status.publish(connection_lost)

        



    def time_update(self,msg):
        self.timestamp=msg.timestamp

    def pose_update(self,msg):

        if self.first_pose:
            # finding marker position and orientation in optitrack frame
            self.initial_mocap_pose= [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
            self.initial_mocap_orientation= [msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]
            
            self.Rmat_initial_mocap=self.quat_2_rot_mat(self.initial_mocap_orientation)
            self.Tmat_marker_in_mocap_frame = np.eye(4)
            print(type(self.Rmat_initial_mocap))
            print(self.Rmat_initial_mocap)

            # Marker pose & orientation in mocap frame (T_OA1)
            y=np.array([self.Rmat_initial_mocap[0,1],self.Rmat_initial_mocap[1,1],self.Rmat_initial_mocap[2,1]])
            print(f"y: {y}, norm y: {np.linalg.norm(y)}")

            if np.dot(self.z,y)<0:
                axis_flip=np.array([[1.0,0.0,0.0],[0.0,math.cos(np.pi),-math.sin(np.pi)],[0.0,math.sin(np.pi),math.cos(np.pi)]])
                self.Rmat_initial_mocap=self.Rmat_initial_mocap@axis_flip
                print("flip needed")
            else:
                print("flip not needed")
            self.Tmat_marker_in_mocap_frame[0:3,0:3] = self.Rmat_initial_mocap
            self.Tmat_marker_in_mocap_frame[0:3,3] = self.initial_mocap_pose

            # initial drone pose in mocap frame (T_OD1)
            self.Tmat_mocap_2_drone_init = self.Tmat_marker_in_mocap_frame@self.Tmat_marker_2_drone_local

            # mocap pose in initia dron pose (T_D1O)
            self.Tmat_mocap_origin_in_drone_initi_frame = np.linalg.inv(self.Tmat_mocap_2_drone_init)
        
            self.Tmat_mocap_in_init_marker_frame = np.linalg.inv(self.Tmat_marker_in_mocap_frame)

            self.first_pose=False
        else:
            new_pose=VehicleOdometry()
            new_pose.pose_frame = VehicleOdometry.POSE_FRAME_NED
            new_pose.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            time= self.get_clock().now().nanoseconds // 1000
            new_pose.timestamp = self.timestamp
            new_pose.timestamp_sample = time
            new_pose.quality = 100

            self.out_of_bound = Bool()

            self.out_of_bound.data=self.boundry_check(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
            pose, orientation = self.pose_transform(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z)
            #new_pose.x=pose[0]
            #new_pose.y=pose[1]
            #new_pose.z=pose[2]
            new_Rmat = self.quat_2_rot_mat(orientation)

            new_z=np.array([new_Rmat[0,2],new_Rmat[1,2],new_Rmat[2,2]])

            # if np.dot(self.z,new_z)<0:
            #     axis_flip=np.array([[1.0,0.0,0.0],[0.0,math.cos(np.pi),-math.sin(np.pi)],[0.0,math.sin(np.pi),math.cos(np.pi)]])
            #     new_Rmat=new_Rmat@axis_flip
            #     orientation= self.rot_mat_2_quat(new_Rmat)
            #     print("flip needed")
            # else:
            #     pass


            new_pose.position = [pose[0],pose[1],pose[2]]


            new_pose.q[0]=orientation[0]
            new_pose.q[1]=orientation[1]
            new_pose.q[2]=orientation[2]
            new_pose.q[3]=orientation[3]
            print(f"pose of drone is {new_pose.position} "
                  f" and orientation of drone is {new_pose.q}"
                  f"qualitu is {new_pose.quality}")

            self.odometry_pub.publish(new_pose)
            self.out_of_bound_flag.publish(self.out_of_bound)

    def target_pose(self,msg):
        target_x_in_gs=msg.pose.position.x
        target_y_in_gs=msg.pose.position.y
        target_z_in_gs=msg.pose.position.z

        qx=msg.pose.orientation.x
        qy=msg.pose.orientation.y
        qz=msg.pose.orientation.z
        qw=msg.pose.orientation.w

        pose, orientation = self.pose_transform(target_x_in_gs,target_y_in_gs,target_z_in_gs,qw,qx,qy,qz)

        target_pose_in_drone = PoseStamped()
        target_pose_in_drone.pose.position.x = pose[0]
        target_pose_in_drone.pose.position.y = pose[1]
        target_pose_in_drone.pose.position.z = pose[2]

        target_pose_in_drone.pose.orientation.w = orientation[0]
        target_pose_in_drone.pose.orientation.x = orientation[1]
        target_pose_in_drone.pose.orientation.y = orientation[2]
        target_pose_in_drone.pose.orientation.z = orientation[3]

        self.target_setpoint.publish(target_pose_in_drone)


    def quat_2_rot_mat(self,q):
        w,x,y,z= q
        Rot_mat = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),2*(x*z+w*y)],
                           [2*(x*y+w*z),1-2*(x*x+z*z), 2*(y*z-w*x)],
                           [2*(x*z-w*y), 2*(y*z+w*x),1-2*(x*x+y*y)]])
        

        return Rot_mat
    
    def rot_mat_2_quat(self, R):
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = math.sqrt(trace+1)*2
            w = 0.25*s
            x = (R[2,1] - R[1,2])/s
            y = (R[0,2] - R[2,0])/s
            z = (R[1,0] - R[0,1])/s
        elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])*2
            w = (R[2,1] - R[1,2])/s
            x = 0.25 * s
            y = (R[0,1] + R[1,0])/s
            z = (R[0,2] + R [2,0])/s

        elif R[1,1]>R[2,2]:
            s = math.sqrt(1 + R[1,1] - R[0,0] - R[2,2])*2
            w = (R[0,2] - R[2,0])/s
            x = (R[0,1] + R[1,0])/s
            y = 0.25 * s
            z = (R[1,2] + R[2,1])/s
        else:
            s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2
            w = (R[1,0] - R[0,1])/s
            x = (R[0,2] + R[2,0])/s
            y = (R[1,2] + R[2,1])/s
            z = 0.25*s

        q= [w,x,y,z]
        return q
    
    def pose_transform(self,x,y,z,qw,qx,qy,qz):
        p = np.array([x,y,z])
        Q = np.array([qw,qx,qy,qz])

        self.Rmat_curr_mocap_orientation = self.quat_2_rot_mat(Q)

        # new marker pose in mocap frame (T_OA1)
        self.Tmat_curr_mocap = np.array([[self.Rmat_curr_mocap_orientation[0,0],self.Rmat_curr_mocap_orientation[0,1],self.Rmat_curr_mocap_orientation[0,2], p[0]],
                                        [self.Rmat_curr_mocap_orientation[1,0],self.Rmat_curr_mocap_orientation[1,1],self.Rmat_curr_mocap_orientation[1,2], p[1]],
                                        [self.Rmat_curr_mocap_orientation[2,0],self.Rmat_curr_mocap_orientation[2,1],self.Rmat_curr_mocap_orientation[2,2], p[2]],
                                        [0.0,0.0,0.0,1.0]])
        

        #self.Tmat_init_2_curr_mocap = self.Tmat_mocap_in_init_marker_frame@self.Tmat_curr_mocap
        #self.Tmat_drone_init_2_curr = self.Tmat_drone_local_2_marker@self.Tmat_init_2_curr_mocap@self.Tmat_marker_2_drone_local
        
        # new drone pose in mocap frame (T_OD2)
        self.Tmat_drone_curr_in_mocap = self.Tmat_curr_mocap@self.Tmat_marker_2_drone_local

        # new drone pose in initial drone frame (T_D1D2)
        self.Tmat_drone_init_2_curr = self.Tmat_mocap_origin_in_drone_initi_frame@self.Tmat_drone_curr_in_mocap
        
        pose = [self.Tmat_drone_init_2_curr[0,3],self.Tmat_drone_init_2_curr[1,3],self.Tmat_drone_init_2_curr[2,3]]

        rot_mat = np.array([[self.Tmat_drone_init_2_curr[0][0],self.Tmat_drone_init_2_curr[0][1],self.Tmat_drone_init_2_curr[0][2]],
                           [self.Tmat_drone_init_2_curr[1][0],self.Tmat_drone_init_2_curr[1][1],self.Tmat_drone_init_2_curr[1][2]],
                           [self.Tmat_drone_init_2_curr[2][0],self.Tmat_drone_init_2_curr[2][1],self.Tmat_drone_init_2_curr[2][2]]])

        
        orientation = self.rot_mat_2_quat(rot_mat)

        return pose, orientation

    def publish_vehicle_command(
            self,
            command,
            param1=0.0,
            param2=0.0

        ):

            msg = VehicleCommand()

            msg.timestamp = self.timestamp

            msg.param1 = param1
            msg.param2 = param2

            msg.command = command

            msg.target_system = 1
            msg.target_component = 1

            msg.source_system = 1
            msg.source_component = 1

            msg.from_external = True

            self.vehicle_command_pub.publish(msg)


def main():

    rclpy.init()

    node = Mocap_pose()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
