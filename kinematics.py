"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def FK_dh(dh_params, joint_angles, link=4):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    
    A = []
    for i in range(link+1):
        row = dh_params[i]
        A.append(get_transform_from_dh(row[0], row[1], row[2], row[3]+joint_angles[i]))

    tf = np.identity(4)
    for a in A:
        tf = np.matmul(tf, a)
    
    (x,y,z,phi) = get_pose_from_T(tf)
    phi = clamp(phi+1.5707)
    return [x,y,z,phi]




def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]])


    # rotz = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
    #         [np.sin(theta), np.cos(theta), 0, 0],
    #         [0, 0, 1, 0],
    #         [0, 0, 0, 1]])
    # transz = np.array([[1, 0, 0, 0],
    #           [0, 1, 0, 0],
    #           [0, 0, 1, d],
    #           [0, 0, 0, 1]])
    # transx = np.array([[1, 0, 0, a],
    #           [0, 1, 0, 0],
    #           [0, 0, 1, 0],
    #           [0, 0, 0, 1]])
    # rotx =   np.array([[1, 0, 0, 0],
    #           [0, np.cos(alpha), -np.sin(alpha), 0],
    #           [0, np.sin(alpha), np.cos(alpha), 0],
    #           [0, 0, 0, 1]])

    # A = np.matmul(np.matmul(np.matmul(rotz, transz), transx), rotx)

    return A

def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    T = np.array(T)
    rm = T[0:3, 0:3]
    tx = np.arctan2(rm[2,1], rm[2,2])
    ty = np.arctan2(-rm[2,0], np.sqrt(rm[2,1]**2 + rm[2,2]**2))
    tz = np.arctan2(rm[1,0], rm[0,0]) 

    return [tx, ty, tz]


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    # phi = np.arctan2(T[0,2],T[0,0])
    T = np.array(T)
    x = T[0,3]
    y = T[1,3]
    z = T[2,3]
    _,phi,_ = get_euler_angles_from_T(T)

    return [x,y,z,phi]


    


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    S=[]
    num_joints = len(joint_angles)
    for i in range(num_joints):
        S.append(to_s_matrix(s_lst[i][0:3], s_lst[i][3:]))

    S = np.asarray(S)
    T = np.identity(4)
    for i in range(num_joints):
        T = np.matmul(T, expm(S[i]*joint_angles[i]))
    T = np.matmul(T,m_mat)
    return get_pose_from_T(T)




def to_s_matrix(w,v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    # w_so3 = np.array([0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0])
    # s_se3 = np.hstack(np.vstack(w_so3,np.zeros(1,3)), np.vstack(v,0))
    S = [[0, -w[2], w[1], v[0]], [w[2], 0, -w[0], v[1]], [-w[1], w[0], 0, v[2]], [0, 0, 0, 0]]
    return S


def quaternion_to_rot_matrix(quaternion):
    qx=quaternion[0]; qy=quaternion[1]; qz=quaternion[2]; qw=quaternion[3]
    rot_matrix = [[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                  [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]]
    return np.asarray(rot_matrix)

def angle_diff(ang2, ang1):
    diff = ang1 - ang2 
    while(diff < -np.pi): diff+=2*np.pi 
    while(diff > np.pi): diff-=2*np.pi 
    return diff

def spatial_joints_elbow_down(pose):
    x,y,z = pose 
    # l1 = 0.10391; l2 = 0.20573; l3 = 0.2; 
    l1 = 0.7; l2 = 0.5; l3=0.5;
    t1 = np.arctan2(y,x)

    r2 = z - l1
    r1 = np.sqrt(x**2 + y**2)
    phi2 = np.arctan2(r2,r1)
    r3 = np.sqrt(r1**2 + r2**2)
    phi1 = np.arccos(((l3**2 - l2**2 - r3**2)/(-2*l2*r3)))
   
    # t2 = (phi2 - phi1)
    t2 = angle_diff(phi2, phi1)

    phi3 = np.arccos((r3**2 - l2**2 - l3**2)/(-2*l2*l3))

    # t3 = (np.pi - phi3)
    t3 = angle_diff(np.pi, phi3)
    # t2 = (0.23+t2)
    # t3 = (-0.3064+t3)
    return (t1,t2,t3)

def spatial_joints_elbow_up(pose):
    x,y,z = pose 
    # l1 = 0.10391; l2 = 0.20573; l3 = 0.2; t2 = 0.23; t3 = -0.3064
    l1 = 0.7; l2 = 0.5; l3=0.5;
    t1 = np.arctan2(y,x)
    # z=-z
    r1 = z - l1
    r2 = np.sqrt(x**2 + y**2)
    # phi2 = np.arctan2(r1,r2)
    r3 = np.sqrt(r1**2+r2**2)
    phi1 = np.arccos((l3**2 - r3**2 - l2**2)/(-2*l2*r3))

    # t2 =    (np.pi/2)-(phi2 - phi1) 

    # phi3 = np.arccos((r3**2 - l2**2 - l3**2)/(-2*l2*l3))

    # t3 = angle_diff(np.pi , phi3)

    phi2 = np.arctan2(r2,r1)
    t2 = angle_diff((np.pi/2), angle_diff(phi1 , phi2))

    phi3 = np.arccos((r3**2 - l2**2 - l3**2)/(-2*l2*l3))
    t3 = np.pi - phi3

    return (t1,t2,t3)


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    l1 = 0.10391; l2 = 0.20573; l3 = 0.2; l4 = 0.17415; l6 = 0.17415;
    x,y,z = pose[0]
    R = pose[1]
    # R = [[np.cos(phi), -np.sin(phi), 0],[np.sin(phi), np.cos(phi), 0],[0,0,1]]
    oc = [0,0,0] 

    oc[0] = x - l6*R[0,2]
    oc[1] = y - l6*R[1,2]
    oc[2] = z - l6*R[2,2] 

    t1 = np.arctan2(oc[1], oc[0]) 
    tx1 = clamp(np.pi + t1)

    r = oc[0]**2+oc[1]**2; s = (oc[2] - l1)**2
    t3 = np.arccos(((r+s) - l2**2-l3**2)/(2*l2*l3)); ty3 = clamp(-t3) 

    t2 = np.arctan2(np.sqrt(s), np.sqrt(r)) - np.arctan2(l3*np.sin(t3), l2*np.cos(t3))
    ty2 = np.arctan2(np.sqrt(s), np.sqrt(r)) - np.arctan2(l3*np.sin(ty3), l2*np.cos(ty3))

    R03 = [[np.cos(t1)*np.cos(t2*t3), -np.cos(t1)*np.sin(t2*t3), np.sin(t1)],
           [np.sin(t1)*np.cos(t2*t3), -np.sin(t1)*np.sin(t2*t3), -np.cos(t1)],
           [np.sin(t2*t3), np.cos(t2*t3), 0]]
    R36 = np.linalg.inv(R03)*R 


    R03x = [[np.cos(tx1)*np.cos(t2*t3), -np.cos(tx1)*np.sin(t2*t3), np.sin(tx1)],
           [np.sin(tx1)*np.cos(t2*t3), -np.sin(tx1)*np.sin(t2*t3), -np.cos(tx1)],
           [np.sin(t2*t3), np.cos(t2*t3), 0]]
    R36x = np.linalg.inv(R03x)*R 


    R03y = [[np.cos(t1)*np.cos(ty2*ty3), -np.cos(t1)*np.sin(ty2*ty3), np.sin(t1)],
           [np.sin(t1)*np.cos(ty2*ty3), -np.sin(t1)*np.sin(ty2*ty3), -np.cos(t1)],
           [np.sin(ty2*ty3), np.cos(t2*t3), 0]]
    R36y = np.linalg.inv(R03y)*R 

    R03xy = [[np.cos(tx1)*np.cos(ty2*ty3), -np.cos(tx1)*np.sin(ty2*ty3), np.sin(tx1)],
           [np.sin(tx1)*np.cos(ty2*ty3), -np.sin(tx1)*np.sin(ty2*ty3), -np.cos(tx1)],
           [np.sin(ty2*ty3), np.cos(ty2*ty3), 0]]
    R36xy = np.linalg.inv(R03xy)*R 


    t4 = np.arctan2(R36[1,2], R36[0,2])
    t5 = np.arctan2(np.sqrt(1 - R36[2,2]**2), R36[2,2])
    t6 = np.arctan2(R36[2,1], -R36[2,0])

    tx5 = np.arctan2(np.sqrt(1 - R36x[2,2]**2), R36x[2,2])
    tx6 = np.arctan2(R36x[2,1], -R36x[2,0])

    ty5 = np.arctan2(np.sqrt(1 - R36[2,2]**2), R36[2,2])
    ty6 = np.arctan2(R36[2,1], -R36[2,0])

    txy5 = np.arctan2(np.sqrt(1 - R36xy[2,2]**2), R36xy[2,2])
    txy6 = np.arctan2(R36xy[2,1], -R36xy[2,0])


    first   = [t1,t2,t3,t5,t6]
    second  = [tx1, t2, t3, tx5, tx6]
    third   = [t1, ty2, ty3, ty5, ty6]
    forth   = [tx1, ty2, ty3, txy5, txy6]

    solution = np.asarray([first,second, third, forth])

    return solution

