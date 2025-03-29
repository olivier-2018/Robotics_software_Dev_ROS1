#!/usr/bin/env python3

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF


def S_matrix(w):
    """ This function computes the skew-symmetric matrix S corresponding to the input vector w. The skew-symmetric matrix is used
    to represent the cross product operation in 3D space. 
    """
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S


def rotation_matrix(matrix):
    """ Extracts the rotation matrix from a 4x4 homogeneous transformation matrix.
    """
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    return R33


def translation_matrix(matrix):
    """ Extracts the translation vector from a 4x4 homogeneous transformation matrix.
    """
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    #t43 = numpy.zeros((3,1))
    #t43[:3] = R[:3, 3:]
    t43 = R[:3, 3]
    return t43

def vector_norm_threshold(V, norm_threshold):
    """ This function scales a vector V if its norm is above than the specified threshold.
    """
    vel_norm = numpy.linalg.norm(V)
    if vel_norm > norm_threshold:
        return V * (norm_threshold / vel_norm) 
    else:
        return V 

def vector_component_threshold(V, norm_threshold):
    """ This function scales a vector V if one of its component is above the specified threshold.
    """
    vel_norm = numpy.linalg.norm(V)
    for Vi in V:
        if abs(Vi) > norm_threshold:
            V = V * (norm_threshold / abs(Vi))
    return V 
    
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    """The cartesian control function computes the joint velocities
    required to move the end effector from the current pose to the desired pose.
    It uses the Jacobian of the robot to compute the joint velocities based on the
    desired end effector velocity.
    
    Input args:
    - joint_transforms: The transforms of the joints in the robot.
    - b_T_ee_current: The current transform of the end effector in the base frame.
    - b_T_ee_desired: The desired transform of the end effector in the base frame.
    - red_control: A boolean indicating whether to use redundancy control.
    - q_current: The current joint angles of the robot.
    - q0_desired: The desired joint angle for the redundant degree of freedom.
    
    Returns:
    numpy array [3]: joint velocities (dq) required to move the end effector
    from the current pose to the desired pose.
    """
    # Initialize dq
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)

    # Compute Transform from current pose to desired pose (dX)
    ee_T_b_current = numpy.linalg.inv(b_T_ee_current)
    EEcur_T_EEdes = numpy.matmul(ee_T_b_current, b_T_ee_desired)
    
    # Extract translation and rotation part of current-to-Desired Transform (EEcur_T_EEdes)
    dX_ee_trans = translation_matrix(EEcur_T_EEdes)
    angle, axis = rotation_from_matrix(rotation_matrix(EEcur_T_EEdes))
    dX_ee_rot = angle * axis
    
    # scale translational and angular velocities
    # vel_limit_trans = 0.1  # 0.1 m/s
    # vel_limit_rot = 1.0  # 1.0 rad/s
    # scaled_V_ee_trans = scale_velocity(V_ee_trans, vel_limit_trans)
    # scaled_V_ee_rot = scale_velocity(V_ee_rot, vel_limit_rot)
    
    # Compute End-Effector velocity components as simple proportional controllers
    gain_trans, gain_rot = 2, 2
    Xdot_ee_trans, Xdot_ee_rot = gain_trans * dX_ee_trans, gain_rot * dX_ee_rot
        
    # Compute End-Effector velocity
    Vee = numpy.concatenate([Xdot_ee_trans, Xdot_ee_rot])

    ##### Compute Jacobian #####
    ############################
    Z = numpy.zeros((3,3))
    J = numpy.empty((6, 0))
    
    # For each joint Transform (i.e Transform from base frame to each joint frame), compute the Jacobian
    for b_T_j in joint_transforms:
        
        # compute  Translation from Joint to EE
        j_T_b = numpy.linalg.inv(b_T_j)
        j_T_ee = numpy.matmul(j_T_b, b_T_ee_current)
        j_t_ee = translation_matrix(j_T_ee)
        
        # Compute Rotation from EE to base
        ee_T_j = numpy.linalg.inv(j_T_ee)        
        ee_R_j = rotation_matrix(ee_T_j)
        
        # Compute Sym skew matrix
        RS = numpy.matmul(-ee_R_j, S_matrix(j_t_ee))
        
        # Compute Vj in the general case
        Vj = numpy.concatenate([numpy.concatenate([ee_R_j,RS],axis=1),
                                 numpy.concatenate([Z    ,ee_R_j],axis=1)],axis=0)
        
        # Extract last column (D-H assumptions ie revolute joints around Z-axis)
        J = numpy.column_stack((J, Vj[:,5])) 
    
    # Compute Jacobian-inverse using Moore-Penrose pseudo-inverse function  (Vee = J . dq)
    Jinv_pseudo = numpy.linalg.pinv(J, rcond=0.001)  # default rcond=1e-15
    
    # Compute differential joint velocities (dq) 
    dq_raw = numpy.matmul(Jinv_pseudo, Vee)
    # rospy.loginfo('dq: %s', dq)
    
    # Set threshold to dq to avoid excessive motion close to singularities 
    angular_vel_limit = 2.0  # rad/s
    # dq = vector_norm_threshold(dq_raw, abs(angular_vel_limit)) # option1: norm threshold
    dq = vector_component_threshold(dq_raw, abs(angular_vel_limit)) # option2: component threshold
    
    # ##### Null space control #####
    # ##############################
    if red_control:
        # The 7-DOF Kuka robot is redundant which means there are joints velocities that do not affect the end-effector motion.
        # In other words, there are dq that are in the null space of the Jacobian.
        # To find the dq that result in J.dq=0 (no motion of the end-effector), we need to project the dq into the null space of the Jacobian.
        
        # Initialize dq_n with the desired first joint velocity
        # This could be done for any joint but the marker should also be adjusted
        dq_n = numpy.zeros(num_joints)
        dq_n[0] = q0_desired - q_current[0]
        
        # Compute the null space of the Jacobian
        I = numpy.identity(num_joints)
        Jinv = numpy.linalg.pinv(J, rcond=0) # not sure this is needed
        dq_nullspace = numpy.subtract(I, numpy.dot(Jinv, J)) 
        
        # Project dq_n into the null space
        dq_n = numpy.dot(dq_nullspace, dq_n) 
        
        # Update the original dq with the null space component (new set of joint velocities that keep the end-effector motion)
        dq = numpy.add(dq, dq_n)

    return dq
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

# Returns the angle-axis representation of the rotation contained in the input matrix
def rotation_from_matrix(matrix):
    """ Extracts the rotation angle and axis from a rotation matrix.
    Notes:
    The rotation matrix is assumed to be a proper rotation matrix (orthogonal with determinant 1).
    The function raises a ValueError if the input matrix is not a valid rotation matrix.

    Args:
        matrix (_type_): rotation matrix (assumed to be a 4x4 homogeneous transformation matrix)

    Returns:
        angle, axis: rotation angle in radians and the rotation axis as a 3D vector.
    """
    # Extract the roation matrix from the 4x4 homogeneous transformation matrix
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    
    # Extract the axis corresponding to the unit eigenvector of R33 (corresponding to eigenvalue of 1)
    l, W = numpy.linalg.eig(R33.T)
    # w, v = numpy.linalg.eig(R33) # eighenvalues, nomalized unit length
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):
    """ This class implements a ROS node that subscribes to joint state information
    and computes the joint velocities required to move the end effector of a robot
    to a desired pose in Cartesian space. The class uses the URDF model of the robot
    to compute the Jacobian and the inverse kinematics required for the control.
    The class also subscribes to commands for the desired end-effector pose and
    the desired joint angle for the redundant degree of freedom.
    It publishes the computed joint velocities to a ROS topic.
    """

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    # Callback functions (called when a message is received on the subscribed topic)
    # to update the class variables with the new data
    # Note: 
    # The mutex is acquired before modifying the data and released after the modification is done
    # to ensure that the data is not being modified while it is being read
    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
    
    # Timer callback function that is called periodically to compute the joint velocities
    # and publish them to the joint_velocities topic
    # Note:
    # The function also checks if the last command was received within a certain time frame
    # to determine if the robot should stop moving or continue moving
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
    
    # Callback function that is called when a new joint state message is received
    # It updates the joint transforms and the current joint angles
    # Note:
    # The function also computes the transform from the base frame to the end effector frame
    # and stores it in the class variable x_current
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    # This function computes the transformation matrix that aligns the rotation axis with the z-axis
    # It uses the axis of rotation and the angle of rotation to compute the transformation matrix
    # The function returns the transformation matrix that aligns the rotation axis with the z-axis
    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
