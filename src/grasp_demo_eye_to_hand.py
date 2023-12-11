# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2

import panda_py
from panda_py import libfranka
import time

'''
眼在手外标定结果
'''
H_cam_to_base = np.array([[ 0.05096545 , 0.98781948, -0.14702111,  0.5748499 ],
                        [ 0.99861288, -0.04845667,  0.02059779, -0.06987559],
                        [ 0.01322274, -0.14786695, -0.98891887,  0.84245153],
                        [ 0.,          0.,          0.,          1.       ]])

"""
手动校准偏移量
"""
x_offset = -0.000
y_offset = 0.0
z_offset = -0.013


''' 
相机设置
'''
pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)  # 配置depth流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)  # 配置color流
 
pipe_profile = pipeline.start(config)  # streaming流开始
 
# 创建对齐对象与color流对齐
align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐


'''
机械臂设置
'''
# 设置机械臂
hostname = "192.168.10.1" # Your Panda IP or hostname
panda = panda_py.Panda(hostname)
panda.set_default_behavior() # Panda default collision thresholds
panda.move_to_start()
joint_speed_factor = 0.05
cart_speed_factor = 0.05
stiffness=np.array([600., 600., 600., 600., 250., 150., 50.])
damping=np.array([50., 50., 50., 20., 20., 20., 10.])
dq_threshold=0.001
success_threshold=0.01

# Optionally set higher thresholds to execute fast trajectories
panda.get_robot().set_collision_behavior(
    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
    [100.0, 100.0, 100.0, 100.0, 100.0, 100.0])

gripper = libfranka.Gripper(hostname)

box_pos = np.array([0.67852055, -0.14208797,  0.43913478])

# 全局变量用于存储鼠标点击的坐标
clicked_pixels = None


''' 
获取对齐图像帧与相机参数
'''
def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧，获取颜色和深度的框架集
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
 
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
 
    #### 获取相机参数 ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
 
    #### 将images转为numpy arrays ####
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data(), dtype=np.float32)  # 深度图（默认16位）
 
    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame


''' 
获取像素点的三维坐标
'''
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate

# 将相机坐标系下的物体坐标转移至机械臂底座坐标系下
def transform_camera_to_base(camera_xyz, hand_eye_matrix):
    # Append 1 to the camera coordinates to make them homogeneous
    camera_xyz_homogeneous = np.append(camera_xyz, 1.0)
 
    # Transform camera coordinates to arm coordinates using hand-eye matrix
    base_xyz_homogeneous = np.dot(hand_eye_matrix, camera_xyz_homogeneous)
 
    # Remove the homogeneous component and return arm coordinates
    base_xyz = base_xyz_homogeneous[:3]
    return base_xyz

# 定义回调函数
def mouse_callback(event, x, y, flags, param):
    global clicked_pixels
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_pixels = (x, y)


if __name__ == "__main__":
    # 创建窗口并设置鼠标回调
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", mouse_callback)

    # 移动到初始位置
    if panda.move_to_start():
        gripper.homing()
    # time.sleep(10)
    # 末端执行器的方向
    orientation = panda.get_orientation()

    while True:
        ''' 
        获取对齐图像帧与相机参数
        '''
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  # 获取对齐图像与相机参数

        # 显示图像
        cv2.imshow("Image", img_color)

        ''' 
        获取像素点三维坐标
        '''
        if clicked_pixels is not None:
            # 在点击坐标处绘制红色圆圈
            print('clicked_coordinates: ', clicked_pixels)
            # cv2.circle(img_color, clicked_pixels, 10, (0, 0, 255), -1)
            # cv2.imshow("Image", img_color)
            # 相机坐标系下的x,y,z
            dis, camera_xyz = get_3d_camera_coordinate(clicked_pixels, aligned_depth_frame, depth_intrin)
            print('depth: %.6f m' % dis)  # 深度单位是m
            print('camera_xyz: ', camera_xyz)

            # end-effector坐标系下的x,y,z
            base_xyz = transform_camera_to_base(camera_xyz, H_cam_to_base)
            print("base_xyz: ", base_xyz)

            # 解决夹爪中心与末端的偏移
            base_xyz[0] += x_offset
            base_xyz[1] += y_offset
            base_xyz[2] += z_offset

            
            """
            使用关节空间控制机械臂移动：
                - 通过position和orientation计算逆运动学参数,及关节运动参数
                - 判断结果是否含有nan,没有则执行
                - 问题: 该方法可以控制关节的运动速度，从而保证机械臂整体运动匀速，不会超过本身速度限制
                        但计算得出的关节参数仅仅可以到达目的点,无法固定orientation来避免与桌面碰撞。
                        因此可以通过moveit加入避障功能
            """
            # q = panda_py.ik(position=base_xyz, orientation=orientation)
            # print("q: ", q)
            # if np.isnan(q).any():
            #     print(">>>> ik solve failure!!!!")
            #     clicked_pixels = None
            #     continue
            # finished = panda.move_to_joint_position(q, speed_factor=0.1)

            """
            使用笛卡尔空间控制机械臂运动:
                - 优点: 易于理解,可以通过固定orientatiion参数保证机械臂末端方向不变,
                        某种程度上可以避免与桌面碰撞
                - 问题: 速度空间在笛卡尔空间定义，容易触发机械臂关节速度限制
            """
            finished = panda.move_to_pose(position=base_xyz, orientation=orientation, 
                                          speed_factor=0.02,
                                          stiffness=np.array([600., 600., 600., 600., 250., 150., 50.]),
                                          damping=np.array([50., 50., 50., 20., 20., 20., 10.]),
                                          dq_threshold=0.001,
                                          success_threshold=0.10)
            time.sleep(1) # 观察精度
            if finished:
                print("finished: ", finished)
                if gripper.grasp(width=0.01, speed=0.2, force=20, epsilon_inner=0.04, epsilon_outer=0.04):
                    # time.sleep(1) # 观察精度
                    if panda.move_to_start():
                        gripper.move(0.07, 0.2) # 释放物体
                    # if panda.move_to_start():
                    #     if panda.move_to_pose(position=box_pos, orientation=orientation, 
                    #                       speed_factor=0.05,
                    #                       stiffness=np.array([600., 600., 600., 600., 250., 150., 50.]),
                    #                       damping=np.array([50., 50., 50., 20., 20., 20., 10.]),
                    #                       dq_threshold=0.001,
                    #                       success_threshold=0.10):
                    #         if gripper.move(0.07, 0.2):
                    #             panda.move_to_start()

                else:
                    print("抓取失败!!!")
                    time.sleep(1)
                    panda.move_to_start()
                
                # 重置点击坐标
                clicked_pixels = None
            else:
                # 移动到初始位置
                clicked_pixels = None
                panda.move_to_start()
        
        
        # else:
        #     continue

        # time.sleep(5)

        # 等待按键事件，按下 ESC 键退出循环
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    cv2.destroyAllWindows()