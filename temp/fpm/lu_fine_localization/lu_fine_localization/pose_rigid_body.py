#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

def horn(model_mat, data_mat):
    """
    Compute rotation and translation that best maps points in 'model' to the points in 'data'. Uses Horn's closed-form
    Least-Squares solution with SVD.
    :param model_mat: 2xN/3xN matrix describing points of the model
    :param data_mat: 2xN/3xN matrix describing points of the data
    :return: An estimate of the transform from data to model (d_T_m = T_dm) as a Pose2D or Pose (3D) object.

    from SLAM project (AER1)    
    """
    # disabled type check, typing not available on Ubuntu 16.04 # type: (np.ndarray, np.ndarray) -> Union[Pose, Pose2D]

    assert isinstance(model_mat, np.ndarray)
    assert isinstance(model_mat, np.ndarray)

    # get primitive (point) dimension, i.e., 2D/3D
    dim = model_mat.shape[0]
    assert data_mat.shape == model_mat.shape
    assert (dim == 2 or dim == 3)

    # compute data means and centered versions for initial alignment
    model_mean = model_mat.mean(1)
    data_mean = data_mat.mean(1)
    model_centered = model_mat - model_mean
    data_centered = data_mat - data_mean

    # build up least squares problem and solve for rotational part: closed-form with SVD
    W = np.zeros( (dim,dim) )
    for column in range(model_mat.shape[1]):
        W += np.outer(model_centered[:,column],data_centered[:,column])
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( dim ))
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[dim-1,dim-1] = -1
    rot = U*S*Vh

    # compute translational part on means
    trans = data_mean - rot * model_mean

    if dim == 2:
        R33 = np.eye(3)
        R33[:2,:2] = rot
        R33[0,2] = trans[0, 0]
        R33[1,2] = trans[1, 0]
        return R33
        #theta = np.arctan2(rot[1, 0], rot[0, 0])
        #return create_pose2d_msg(trans[0, 0], trans[1, 0], theta)
    else:
        R44 = np.eye(4)
        R44[:3,:3] = rot
        R44[0,3] = trans[0, 0]
        R44[1,3] = trans[1, 0]
        R44[2,3] = trans[2, 0]
        return R44


def pose_rigid_body(body:list, measured_points:list):
    """
    INPUT:
        body:            list of geometry_msgs.msg.PointStamped
        measured_points: list of geometry_msgs.msg.PointStamped
    OUTPUT:
        PoseWithCovarianceStamped
    """

    if not isinstance(body, list):
        raise TypeError
    if not isinstance(measured_points, list):
        raise TypeError

    if not isinstance(body[0], PointStamped):
        raise TypeError
    if not isinstance(measured_points[0], PointStamped):
        raise TypeError

    # convert input points to array
    # print(body)
    point_number = len(body)
    rigid_body_mat = np.asmatrix(np.zeros([3, point_number]))
    measured_POINTS_mat = np.asmatrix(np.zeros([3, point_number]))

    for i in range(point_number):
        rigid_body_mat[0, i] = body[i].point.x
        rigid_body_mat[1, i] = body[i].point.y
        rigid_body_mat[2, i] = body[i].point.z
        
        # put position of marker in bim-system to matrix
        measured_POINTS_mat[0, i] = measured_points[i].point.x
        measured_POINTS_mat[1, i] = measured_points[i].point.y
        measured_POINTS_mat[2, i] = measured_points[i].point.z

    # calculation of transformation matrix of leica in BIM frame
    T = horn(rigid_body_mat, measured_POINTS_mat)
    # print(repr(T))
    # print("T = [")
    # print("%9.4f,%9.4f,%9.4f,%9.4f"%(T[0][0],T[0][1],T[0][2],T[0][3]))
    # print("%9.4f,%9.4f,%9.4f,%9.4f"%(T[1][0],T[1][1],T[1][2],T[1][3]))
    # print("%9.4f,%9.4f,%9.4f,%9.4f"%(T[2][0],T[2][1],T[2][2],T[2][3]))
    # print("%9.4f,%9.4f,%9.4f,%9.4f"%(T[3][0],T[3][1],T[3][2],T[3][3]))
    # print("]\n\n")
    
    R = Rotation.from_matrix(T[:3,:3])
    # print(R.as_euler("xyz"))

    # Rotate by 90deg around z !?!?!?!?!?!?!?!?!?!?!?!?!?!??!?!?!?!?!?!?
    # self.get_logger().warn('NOW OFF: The pose have to be rotated by 90deg !?!?!?!?!?!?!?!?!?!?!')
    # R = Rotation.from_euler('xyz', 
    #                                     [R.as_euler("xyz")[0],
    #                                         R.as_euler("xyz")[1],
    #                                         R.as_euler("xyz")[2]+90.0/180.0*3.1415], 
    #                                     degrees=False)
    # print(R.as_euler("xyz"))

    Q = R.as_quat()

    # initialize pose of leica in bim-frame
    pose_rigid_body = PoseWithCovarianceStamped()
    pose_rigid_body.header.frame_id = measured_points[0].header.frame_id
    pose_rigid_body.pose.pose.position.x = T[0, 3]
    pose_rigid_body.pose.pose.position.y = T[1, 3]
    pose_rigid_body.pose.pose.position.z = T[2, 3]
    pose_rigid_body.pose.pose.orientation.w = Q[3]
    pose_rigid_body.pose.pose.orientation.x = Q[0]
    pose_rigid_body.pose.pose.orientation.y = Q[1]
    pose_rigid_body.pose.pose.orientation.z = Q[2]

    # calculation of transition error #############################################
    ones_row = np.ones(rigid_body_mat.shape[1])
    all_leica_homogen = np.vstack([rigid_body_mat,ones_row])
    all_leica_in_bim_transform = np.dot(T,all_leica_homogen)[:3,:]
    all_leica_diff_abs = np.abs(all_leica_in_bim_transform-measured_POINTS_mat)
    mean_error_transition = np.mean(all_leica_diff_abs,axis=1)

    distance_error = np.sqrt(np.multiply(all_leica_diff_abs[0,:],all_leica_diff_abs[0,:]) + \
                    np.multiply(all_leica_diff_abs[1,:],all_leica_diff_abs[1,:]) + \
                    np.multiply(all_leica_diff_abs[2,:],all_leica_diff_abs[2,:]))
    all_BIM_mat_length = np.sqrt(np.multiply(measured_POINTS_mat[0,:],measured_POINTS_mat[0,:]) + \
                    np.multiply(measured_POINTS_mat[1,:],measured_POINTS_mat[1,:]) + \
                    np.multiply(measured_POINTS_mat[2,:],measured_POINTS_mat[2,:]))
    angle_error_rad = np.divide(distance_error,all_BIM_mat_length)

    pose_rigid_body.pose.covariance[0] = np.mean(distance_error)
    pose_rigid_body.pose.covariance[7] = np.mean(distance_error)
    pose_rigid_body.pose.covariance[14] = np.mean(distance_error)
    pose_rigid_body.pose.covariance[21] = np.mean(angle_error_rad)
    pose_rigid_body.pose.covariance[28] = np.mean(angle_error_rad)
    pose_rigid_body.pose.covariance[35] = np.mean(angle_error_rad)

    # publish the calculated pose of leica in BIM coordinate
    # print('Calculated pose published')
    return pose_rigid_body
