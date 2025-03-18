#!/usr/bin/env python3
import rospy
import numpy
import math
import tf
import tf.transformations
import tf2_ros
import geometry_msgs.msg

# Function to get translation and rotation info from a transformation matrix
def get_T_info(T):
    euler = tf.transformations.euler_from_matrix(T)
    translation = tf.transformations.translation_from_matrix(T)
    return [f"{c:.3f}" for c in translation], [f"{a*180/math.pi:.1f}" for a in euler]

# Function to get a transformation matrix from translation and rotation info
def message_from_transform (T):
   msg = geometry_msgs.msg.Transform()
   q = tf.transformations.quaternion_from_matrix(T)
   translation = tf.transformations.translation_from_matrix(T)
   msg.translation.x = translation[0]
   msg.translation.y = translation[1]
   msg.translation.z = translation[2]
   msg.rotation.x = q[0]
   msg.rotation.y = q[1]
   msg.rotation.z = q[2]
   msg.rotation.w = q[3]
   return msg

def publish_transforms():
    ### Setting BASE to OBJECT transform
    T1 = tf.transformations.concatenate_matrices(
            tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)),
            tf.transformations.translation_matrix((0.0, 1.0, 0.0))
        )

    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    object_transform.transform = message_from_transform(T1)
    # Log translation and rotation info
    info = get_T_info(T1)
    rospy.loginfo(f"BASE-->OBJECT transform (translation): {info[0]}mm")
    rospy.loginfo(f"BASE-->OBJECT transform (rotation): {info[1]}deg")
    # Publish the transform
    br.sendTransform(object_transform)


    ### Setting BASE to ROBOT transform
    T2 = tf.transformations.concatenate_matrices(
            tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(1.5, (0,0,1))), 
            tf.transformations.translation_matrix((0.0, -1.0, 0.0))           
            )

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    robot_transform.transform = message_from_transform(T2)
    info = get_T_info(T1)
    rospy.loginfo(f"BASE-->ROBOT transform (translation): {info[0]}mm")
    rospy.loginfo(f"BASE-->ROBOT transform (rotation): {info[1]}deg")
    br.sendTransform(robot_transform)

    ### Setting CAMERA to OBJECT transform

    # step1: create origin of object frame in Camera  frame 
    Pobj_object_frame = numpy.array([0.0, 0.0, 0.0]) # origin of object frame
    # step2: Create respective transformation matrices
    T_robot_to_camera = tf.transformations.translation_matrix((0.0, 0.1, 0.1))
    T_camera_to_robot = tf.transformations.inverse_matrix(T_robot_to_camera)
    T_robot_to_base = tf.transformations.inverse_matrix(T2)
    T_base_to_object = T1
    # Step3: Create transformation matrix from Robot to Object
    T_robot_to_object = tf.transformations.concatenate_matrices(T_camera_to_robot,T_robot_to_base, T_base_to_object)
    # Evaluate Object poisition in Camera frame
    Pobj_camera_frame = numpy.matmul(T_robot_to_object, numpy.append(Pobj_object_frame,[1]))

    # calculate vector from Camera to object in camera frame (Vcamobj_cam_frame)
    Vcamobj_cam_frame = Pobj_camera_frame[0:3]

    # set camera vector to point to (look at) object in camera frame (camera x-axis)
    Vx_cam_frame = numpy.array([1.0, 0.0, 0.0]) # x-axis in robot frame

    # calculate cross product between Camera x-axis and Vcamobj_cam_frame to get the rotation axis
    rot_ax = numpy.cross (Vx_cam_frame,Vcamobj_cam_frame) 

    # calculate rotation angle between Camera x-axis and Vcamobj to align Camera to look at object
    rot_angl = math.acos( numpy.dot(Vx_cam_frame, Vcamobj_cam_frame) / 
        ( numpy.linalg.norm(Vx_cam_frame) * numpy.linalg.norm(Vcamobj_cam_frame) ) )


    # Finally rotate Camera frame to point to Object
    T3 = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix((0.0, 0.1, 0.1)),
            tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(rot_angl, rot_ax))
            )

    # Create a TransformStamped message
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    # Log translation and rotation info
    info = get_T_info(T3)
    rospy.loginfo(f"CAMERA-->OBJECT transform (translation): {info[0]}mm")
    rospy.loginfo(f"CAMERA-->OBJECT transform (rotation): {info[1]}deg")
    # Publish the transform
    camera_transform.transform = message_from_transform(T3)
    
    br.sendTransform(camera_transform)
    

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('transform_publisher')

    # Create a TransformBroadcaster object
    br = tf2_ros.TransformBroadcaster() 
    rospy.sleep(1)

    # Loop to publish the transforms
    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.5)
