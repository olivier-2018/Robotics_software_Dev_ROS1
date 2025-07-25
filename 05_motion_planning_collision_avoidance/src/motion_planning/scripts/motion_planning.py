#!/usr/bin/env python3
from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys
import actionlib
import control_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

def convert_from_message(msg):
    R = tf.transformations.quaternion_matrix((msg.orientation.x,
                                              msg.orientation.y,
                                              msg.orientation.z,
                                              msg.orientation.w))
    T = tf.transformations.translation_matrix((msg.position.x, 
                                               msg.position.y, 
                                               msg.position.z))
    return numpy.dot(T,R)

def convert_from_trans_message(msg):
    R = tf.transformations.quaternion_matrix((msg.rotation.x,
                                              msg.rotation.y,
                                              msg.rotation.z,
                                              msg.rotation.w))
    T = tf.transformations.translation_matrix((msg.translation.x, 
                                               msg.translation.y, 
                                               msg.translation.z))
    return numpy.dot(T,R)
   
class MoveArm(object):

    def __init__(self):
        print ("Motion Planning Initializing...")
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Some info and conventions about the robot that we hard-code in here
        # min and max joint values are not read in Python urdf, so we must hard-code them here
        self.num_joints = 7
        self.q_min = []
        self.q_max = []
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        # How finely to sample each joint
        self.q_sample = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1]
        self.joint_names = ["lwr_arm_0_joint",
                            "lwr_arm_1_joint",
                            "lwr_arm_2_joint",
                            "lwr_arm_3_joint",
                            "lwr_arm_4_joint",
                            "lwr_arm_5_joint",
                            "lwr_arm_6_joint"]

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)

        # Subscribe to command for motion planning goal
        rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform, self.move_arm_cb)

        # Publish trajectory command
        self.pub_trajectory = rospy.Publisher("/joint_trajectory", trajectory_msgs.msg.JointTrajectory, queue_size=1)        

        # Initialize variables
        self.joint_state = sensor_msgs.msg.JointState()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print ("IK service ready")

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print ("State validity service ready")

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "lwr_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name) 
        print ("MoveIt! interface ready")

        # Options
        self.subsample_trajectory = True
        print ("Initialization done.")

    def get_joint_val(self, joint_state, name):
        if name not in joint_state.name:
            print ("ERROR: joint name not found")
            return 0
        i = joint_state.name.index(name)
        return joint_state.position[i]

    def set_joint_val(self, joint_state, q, name):
        if name not in joint_state.name:
            print ("ERROR: joint name not found")
        i = joint_state.name.index(name)
        joint_state.position[i] = q

    """ Given a complete joint_state data structure, this function finds the values for  our arm's set of joints in a particular order and returns a list q[] containing just  those values.    """
    def q_from_joint_state(self, joint_state):
        q = []
        for i in range(0,self.num_joints):
            q.append(self.get_joint_val(joint_state, self.joint_names[i]))
        return q

    """ Given a list q[] of joint values and an already populated joint_state, this function assumes that the passed in values are for a our arm's set of joints in a particular order and edits the joint_state data structure to set the values to the ones passed in.    """
    def joint_state_from_q(self, joint_state, q):
        for i in range(0,self.num_joints):
            self.set_joint_val(joint_state, q[i], self.joint_names[i])

    """ This function will perform IK for a given transform T of the end-effector. It returns a list q[] of 7 values, which are the result positions for the 7 joints of the left arm, ordered from proximal to distal. If no IK solution is found, it returns an empy list.  """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "world_link"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = self.q_from_joint_state(res.solution.joint_state)
        return q

    """ This function checks if a set of joint angles q[] creates a valid state, or one that is free of collisions. The values in q[] are assumed to be values for the joints of the left arm, ordered from proximal to distal. """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        current_joint_state = deepcopy(self.joint_state)
        current_joint_state.position = list(current_joint_state.position)
        self.joint_state_from_q(current_joint_state, q)
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = current_joint_state
        res = self.state_valid_service(req)
        return res.valid
                
    def motion_plan(self, q_start, q_goal, q_min, q_max):
        print(q_min, q_max)
        # ===========================
        print("===== Motion Planning =====")
    
        def get_random_q(qmin, qmax):
            ''' Sample a random point  in config space within limits'''
            q = [ qmin[i] + random.random() * (qmax[i]-qmin[i])  for i in range(len(qmin))]
            return numpy.array(q)

        def get_closest_node_idx (new_node, node_list):
            ''' Find index of closest node in list of nodes'''
            dist_list = [numpy.linalg.norm(new_node - node) for node in node_list]
            return dist_list.index(min(dist_list))

        def get_point_in_direction (q1, q2, ratio=0.5):
            ''' Returns a point at a certain distance ratio between points q1 and q2'''
            return q1 + ratio * (q2 - q1)

        def No_collision_on_path (q1, q2):
            ''' Disctretizes path between q1 and q2 base on q_sample and check for collision.
                Returns False if collision is detected'''    
            #print ("Def: Checking collision between",q1[:3]," and ",q2[:3])
            no_steps = int( numpy.max ( abs(q2 - q1) / self.q_sample ) )
            #print ("No step: ",no_steps)
            q_step = (q2 - q1) / float(no_steps)
            #print ("q_step: ",q_step)

            for step in range(no_steps): 
                #print("step= ",step," qcheck=",q1 + (step + 1) * q_step)
                if not self.is_state_valid(q1 + (step + 1) * q_step):
                    #print ("Collision detected")
                    return False
                
            #print ("DEf: NO collision detected")
            return True     

        # Initialization
        qstart = numpy.array(q_start)
        qgoal = numpy.array(q_goal)
        qmin = numpy.array(q_min)
        qmax = numpy.array(q_max)
        print("Start:",qstart[:4])
        print("Goal:",qgoal[:4])
        
        TREE_node = [qstart]
        TREE_parent = [-1]
        print ("TREE init: ", TREE_node[0][:4], "(parent idx:",TREE_parent[0],")")

        # Build RRT
        print ("==== Build RRT ====")
        achieved_goal = False
        while not achieved_goal:    
           
            random_q = get_random_q(qmin, qmax) 
            #print ("New random q", random_q[:4])

            iclosest = get_closest_node_idx(random_q, TREE_node)
            #print ("iclosest",iclosest)

            new_node = get_point_in_direction (TREE_node[iclosest], random_q, 0.3)
            #print ("new_node",new_node[:4])

            if No_collision_on_path(TREE_node[iclosest], new_node):
                print ("Adding node ",len(TREE_node),":", new_node[:4]," parent idx: ",iclosest)
                TREE_node.append(new_node)
                TREE_parent.append(iclosest)

                if No_collision_on_path (new_node, qgoal):
                    goal_parent = len(TREE_parent)-1
                    print ("Adding GOAL node:", qgoal[:4]," parent idx: ", goal_parent)
                    
                    TREE_node.append(qgoal)
                    TREE_parent.append(goal_parent)
                    achieved_goal = True
                    
        # Trace back RRT, create path
        print ("==== create path ====")
        Path = []
        Path.append(TREE_node[-1])
        parent_node = TREE_parent[-1]
        print ("Path init:", Path[0][:4], " parent node:", parent_node)
        
        while parent_node != -1:            
            Path.append(TREE_node[parent_node])
            print ("Adding node ",parent_node,":", Path[-1][:4], " parent node:",TREE_parent[parent_node])
            parent_node = TREE_parent[parent_node]

        # Trim PATH
        print ("==== Trim path ====")
        Fully_trimmed = False
        
        while not Fully_trimmed:
            
            path_length = len(Path)
            print("Path length", len(Path))
            
            for i in range(1, path_length-1):
                print("i=",i," Checking ",Path[i-1][:4]," and ",Path[i+1][:4])
                
                if No_collision_on_path (Path[i-1], Path[i+1]):
                    removed_node = Path.pop(i)
                    print("Removing node:",removed_node[:4])
                    #print("New Path length", len(Path))
                    break
                    
            if i == path_length - 2:
                Fully_trimmed = True

        # Convert back to List
        print("==== Trimmed Path ==== ")
        Trimmed_Path = []
        print("Length:",len(Path))
        
        for i in range(len(Path)-1, -1, -1):
            tmp = Path[i].tolist()
            Trimmed_Path.append(tmp)
            print("i=",i," Path: ", tmp)

        #print (type(Trimmed_Path))
        #print(Trimmed_Path.reverse())
        
        return Trimmed_Path
    # ===========================

    def create_trajectory(self, q_list, v_list, a_list, t):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            point.velocities = list(v_list[i])
            point.accelerations = list(a_list[i])
            point.time_from_start = rospy.Duration(t[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def create_trajectory(self, q_list):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def project_plan(self, q_start, q_goal, q_min, q_max):
        q_list = self.motion_plan(q_start, q_goal, q_min, q_max)
        joint_trajectory = self.create_trajectory(q_list)
        return joint_trajectory

    def move_arm_cb(self, msg):
        T = convert_from_trans_message(msg)
        self.mutex.acquire()
        q_start = self.q_from_joint_state(self.joint_state)
        print ("Solving IK")
        q_goal = self.IK(T)
        if len(q_goal)==0:
            print ("IK failed, aborting")
            self.mutex.release()
            return
        print ("IK solved, planning")
        trajectory = self.project_plan(numpy.array(q_start), q_goal, self.q_min, self.q_max)
        if not trajectory.points:
            print ("Motion plan failed, aborting")
        else:
            print ("Trajectory received with " + str(len(trajectory.points)) + " points")
            self.execute(trajectory)
        self.mutex.release()
        
    def joint_states_callback(self, joint_state):
        self.mutex.acquire()
        self.joint_state = joint_state
        self.mutex.release()

    def execute(self, joint_trajectory):
        self.pub_trajectory.publish(joint_trajectory)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()
