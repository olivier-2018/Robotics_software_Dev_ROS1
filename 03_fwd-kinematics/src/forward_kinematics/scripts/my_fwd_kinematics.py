#!/usr/bin/env python3

import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

""" Starting from a computed transform T, creates a message that can be
communicated over the ROS wire. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):


    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. We must use
    the information we get to compute forward kinematics.

    We will iterate through the entire chain, and publish the transform for each link we find.
    """
    def callback(self, joint_values):
        # First, we must extract information about the kinematics of the robot from its URDF.
        # We will start at the root and add links and joints to lists
        link_name = self.robot.get_root()
        link_names = []
        joints = []
        while True:
            # Find the joint connected at the end of this link, or its "child"
            # Make sure this link has a child
            if link_name not in self.robot.child_map:
                break
            # Make sure it has a single child (we don't deal with forks)
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Forked kinematic chain!");
                break
            # Get the name of the child joint, as well as the link it connects to
            (joint_name, next_link_name) = self.robot.child_map[link_name][0]
            # Get the actual joint based on its name
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                break;
            joints.append(self.robot.joint_map[joint_name])
            link_names.append(next_link_name)

            # Move to the next link
            link_name = next_link_name

        # Compute all the transforms based on the information we have
        all_transforms = self.compute_transforms(link_names, joints, joint_values)

        # Publish all the transforms
        self.pub_tf.publish(all_transforms)


    """ This is the function that performs the main forward kinematics computation. It accepts as 
    parameters all the information needed about the joints and links of the robot, as well as the current
    values of all the joints, and must compute and return the transforms from the world frame to all the 
    links, ready to be published through tf.

    Parameters are as follows:
    - link_names: a list with all the names of the robot's links, ordered from proximal to distal.
    These are also the names of the link's respective coordinate frame. In other words, the transform
    from the world to link i should be published with "world_link" as the parent frame and link_names[i] 
    as the child frame.    

    - joints: a list of all the joints of the robot, in the same order as the links listed above. Each 
    entry in this list is an object which contains the following fields:
     * joint.origin.xyz: the translation from the frame of the previous joint to this one
     * joint.origin.rpy: the rotation from the frame of the previous joint to this one, 
       in ROLL-PITCH-YAW XYZ convention
     * joint.type: either 'fixed' or 'revolute'. A fixed joint does not move; it is meant to 
       contain a static transform. 
     * joint.name: the name of the current joint in the robot description
     * joint.axis: (only if type is 'revolute') the axis of rotation of the joint

     - joint_values contains information about the current joint values in the robot. It contains
     information about *all* the joints, and the ordering can vary, so we must find the relevant value 
     for a particular joint you are considering. We can use the following fields:
      * joint_values.name: a list of the names of *all* the joints in the robot
      * joint_values.position: a list of the current values of *all* the joints in the robot, in the same 
        order as the names in the list above.
     To find the value of the joint we care about, we must find its name in the "name" list, then take
     the value found at the same index in the "position" list.

    The function must return one tf message. The "transforms" field of this message must list *all* the 
    transforms from the world coordinate frame to the links of the robot. In other words, when you are done,
    all_transforms.transforms must contain a list in which you must place all the transforms from the 
    "world_link" coordinate frame to each of the coordinate frames listed in "link_names". You can use the
    "convert_to_message" function (defined above) for a convenient way to create a tf message from a 
    transformation matrix.
    """    
    def compute_transforms(self, link_names, joints, joint_values):
        all_transforms = tf.msg.tfMessage()
        # Initialization
        T = tf.transformations.identity_matrix()        
        revolute_joint_idx = 0
        
        # For each joint, compute the transform from the parent to the child and then from the world to the child
        for joint in joints:
            # Get translation matrix from joint transform
            translation_vector = numpy.array(joint.origin.xyz)
            
            if joint.type == 'revolute':
                rot_ax = numpy.array(joint.axis)  
                rot_angl = joint_values.position[revolute_joint_idx]
                
                revolute_joint_idx += 1
            else:
                rot_ax = numpy.array([1.0, 0.0, 0.0]) # dummy axis                
                rot_angl = 0.0
                        
            # Compute Transform from parent to child
            T_joint = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(translation_vector),
                tf.transformations.quaternion_matrix(
                    tf.transformations.quaternion_about_axis(rot_angl, rot_ax)))

            # Compute the Transform from Wolrd to joint i
            new_T = numpy.matmul(T, T_joint)
            
            # Add Base-to-Joint_i transform to list of transforms
            all_transforms.transforms.append( convert_to_message(new_T, joint.child, joints[0].parent))

            # move on to the next joint
            T = new_T
            rospy.loginfo(rospy.get_caller_id() + f" Joint name: {joint.child} (type: {joint.type})\nTransform child->current:\n {T}")
            
        rospy.loginfo(rospy.get_caller_id() + f" Transform from WORLD to EE:\n {T}")
        
        return all_transforms
       
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

