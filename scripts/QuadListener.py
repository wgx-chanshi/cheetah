#!/usr/bin/env python3

import rospy
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import pybullet as p
import pybullet_data
import time
import numpy as np
from quadruped_robot.srv import QuadrupedCmd, QuadrupedCmdResponse


def init_simulation():
    global quadruped, motor_id_list
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    motor_id_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10]
    all_motor_id_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10, 3, 7, 11, 15]
    p.setGravity(0, 0, -9.8)
    # p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 10., numSolverIterations=550, numSubSteps=4)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    planeId = p.loadURDF("plane.urdf")
    init_position = [0, 0, 0.5]
    quadruped = p.loadURDF("mini_cheetah/mini_cheetah.urdf", init_position, useFixedBase=False)
    num_joints = p.getNumJoints(quadruped)
    compensate = [-1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1]
    #init_pos = [-1, -0.8, 1.75, 1, -0.8, 1.75, 1, 0.8, -1.75, -1, 0.8, -1.75, 0, 0, 0, 0]
    # init_pos = [0, -0.8, 1.75, 0, -0.8, 1.75, 0, 0.8, -1.75, -0, 0.8, -1.75, 0, 0, 0, 0]
    init_pos = [-0.2, -1.1, 2.8, 0.2, -1.1, 2.8, 0.2, 1.1, -2.8, -0.2, 1.1, -2.8, 0, 0, 0, 0]
    init_force = [200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200]
    # init_force = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    init_new_pos = []
    init_new_force = []
    for j in range(16):
        init_new_pos.append(init_pos[j] * compensate[j])
        init_new_force.append(init_force[j] * compensate[j])
    # p.setJointMotorControlArray(quadruped,
    #                             jointIndices=all_motor_id_list,
    #                             controlMode=p.POSITION_CONTROL,
    #                             targetPositions=init_new_pos,
    #                             forces=init_new_force)
    for j in range(16):
        print(num_joints)
        p.changeVisualShape(quadruped, j, rgbaColor=[1, 1, 1, 1])
        force = 500
        pos = 0
        # p.setJointMotorControl2(quadruped, j, p.VELOCITY_CONTROL, force=force)
        p.setJointMotorControl2(quadruped, all_motor_id_list[j], p.POSITION_CONTROL, init_new_pos[j], force=force)
        p.enableJointForceTorqueSensor(quadruped, j, 1)






def thread_job():
    rospy.spin()


def set_pos(set_mode, position_list=[]):
    maxForces = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
    p.setJointMotorControlArray(quadruped,
                                jointIndices=motor_id_list,
                                controlMode=set_mode,
                                targetPositions=position_list,
                                forces=maxForces)


def set_force(set_mode, force_list=[]):
    pos_gain = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
    vel_gain = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
    p.setJointMotorControlArray(quadruped,
                                jointIndices=motor_id_list,
                                controlMode=set_mode,
                                forces=force_list,
                                positionGains=pos_gain,
                                velocityGains=vel_gain)


def set_vel(set_mode, vel_list=[]):
    p.setJointMotorControlArray(quadruped,
                                jointIndices=motor_id_list,
                                controlMode=set_mode,
                                targetVelocities=vel_list)


def get_pose_orn():
    pose_orn = p.getBasePositionAndOrientation(quadruped)
    return pose_orn


def get_vel():
    base_linear_angular_vel = p.getBaseVelocity(quadruped)
    return base_linear_angular_vel


def callback_state(msg):
    # global get_position=[]
    # global get_velocity=[]
    # global get_effort=[]
    print("hello world11111111111111111111111111")
    get_position = []
    get_velocity = []
    get_effort = []
    compensate = [-1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1]
    if mode == p.TORQUE_CONTROL:
        for i in range(12):
            get_effort.append(compensate[i] * msg.effort[i])
        set_force(p.TORQUE_CONTROL, get_effort)
        print(mode)
        print(msg.effort)

    if mode == p.POSITION_CONTROL:
        for i in range(12):
            get_position.append(compensate[i] * msg.position[i])
        set_pos(p.POSITION_CONTROL, get_position)
        print(msg.position)

    if mode == p.VELOCITY_CONTROL:
        for i in range(12):
            get_velocity.append(msg.velocity[i])
        set_vel(mode, get_velocity)


def callback_mode(req):
    print("hello world222222222222222222222222222222")
    global mode
    if req.cmd == 0:
        mode = p.TORQUE_CONTROL
        for j in range(16):
            force = 0
            p.setJointMotorControl2(quadruped, j, p.VELOCITY_CONTROL, force=force, positionGain=10, velocityGain=10)
            #p.changeDynamics(quadruped, j, spinningFriction=0.01, rollingFriction=0.01, jointDamping=1.0)
            p.changeDynamics(quadruped, j, jointDamping=0.5)
    elif req.cmd == 1:
        mode = p.POSITION_CONTROL
    elif req.cmd == 4:
        mode = p.VELOCITY_CONTROL
    return QuadrupedCmdResponse(0, "get the mode")



def talker():
    print("send the IMU messages")
    motor_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10, 3, 7, 11, 15]
    compensate = [-1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1]
    matrix = []
    pub1 = rospy.Publisher('/imu_body', Imu, queue_size=10)
    pub2 = rospy.Publisher('/get_js', JointState, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    freq = 1000
    rate = rospy.Rate(freq)  # hz
    imu_msg = Imu()
    joint_msg = JointState()
    joint_msg.header = Header()
    linearandangular_vel1 = get_vel()
    count = 0
    while not rospy.is_shutdown():
        quaternion = []
        poseandorn = get_pose_orn()
        linearandangular_vel2 = get_vel()
        for i in range(4):
            quaternion.append(poseandorn[1][i])
        matrix = p.getMatrixFromQuaternion(quaternion)
        imu_msg.orientation.x = poseandorn[1][0]
        imu_msg.orientation.y = poseandorn[1][1]
        imu_msg.orientation.z = poseandorn[1][2]
        imu_msg.orientation.w = poseandorn[1][3]
        # imu_msg.linear_acceleration.x = (linearandangular_vel2[0][0] - linearandangular_vel1[0][0]) / freq
        # imu_msg.linear_acceleration.y = (linearandangular_vel2[0][1] - linearandangular_vel1[0][1]) / freq
        # imu_msg.linear_acceleration.z = (linearandangular_vel2[0][2] - linearandangular_vel1[0][2]) / freq + 9.8
        acc_X = (linearandangular_vel2[0][0] - linearandangular_vel1[0][0]) / freq
        acc_Y = (linearandangular_vel2[0][1] - linearandangular_vel1[0][1]) / freq
        acc_Z = (linearandangular_vel2[0][2] - linearandangular_vel1[0][2]) / freq + 9.8
        imu_msg.linear_acceleration.x = matrix[0] * acc_X + matrix[1] * acc_Y + matrix[2] * acc_Z
        imu_msg.linear_acceleration.y = matrix[3] * acc_X + matrix[4] * acc_Y + matrix[5] * acc_Z
        imu_msg.linear_acceleration.z = matrix[6] * acc_X + matrix[7] * acc_Y + matrix[8] * acc_Z
        imu_msg.angular_velocity.x = linearandangular_vel2[1][0]
        imu_msg.angular_velocity.y = linearandangular_vel2[1][1]
        imu_msg.angular_velocity.z = linearandangular_vel2[1][2]
        linearandangular_vel1 = linearandangular_vel2
        print(imu_msg)

        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
                          "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"]
        joint_state = p.getJointStates(quadruped, motor_id_list)
        # print(joint_state)

        # for i in range(12):
        #     joint_msg.position.append(joint_state[i][0] * compensate[i])
        #     joint_msg.velocity.append(joint_state[i][1] * compensate[i])
        joint_msg.position = [-joint_state[0][0], joint_state[1][0], joint_state[2][0],
                              joint_state[3][0], joint_state[4][0], joint_state[5][0],
                              -joint_state[6][0], -joint_state[7][0], -joint_state[8][0],
                              joint_state[9][0], -joint_state[10][0], -joint_state[11][0]]
        joint_msg.velocity = [-joint_state[0][1], joint_state[1][1], joint_state[2][1],
                              joint_state[3][1], joint_state[4][1], joint_state[5][1],
                              -joint_state[6][1], -joint_state[7][1], -joint_state[8][1],
                              joint_state[9][1], -joint_state[10][1], -joint_state[11][1]]
        if count % 2 == 0:
            pub1.publish(imu_msg)
            pub2.publish(joint_msg)
        count = count + 1

        myjoint_sate = p.getJointStates(quadruped, motor_list)
        # for j in range(12):
        #     # print("get the", j, " ",  myjoint_sate[j][2][3] * compensate[j], " ", myjoint_sate[j][2][4] * compensate[j])
        #     print("get the", j, " ", myjoint_sate[j][3] * compensate[j])
        # print("get the 3", myjoint_sate[12][2][2])
        # print("get the 7", myjoint_sate[13][2][2])
        # print("get the 11", myjoint_sate[14][2][2])
        # print("get the 15", myjoint_sate[15][2][2])
        # print(myjoint_sate)
        p.stepSimulation()
        # rospy.spin()
        rate.sleep()


# def listener():
#     rospy.init_node('listener', anonymous=True)
#     print("adsssssssssssssssssssssssssss")
#     rospy.Subscriber('/set_js', JointState, callback_state)
#     s = rospy.Service('/set_jm', QuadrupedCmd, callback_mode)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()


if __name__ == '__main__':
    init_simulation()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("set_js", JointState, callback_state)
    s = rospy.Service('set_jm', QuadrupedCmd, callback_mode)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    talker()
