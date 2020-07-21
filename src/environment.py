#!/usr/bin/env python3
import os
import rospy
import numpy as np
import math
from math import pi
import random

from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel

from cv_bridge import CvBridge, CvBridgeError
from monodepth2 import *
import PIL
from matplotlib import cm
import torch

diagonal_dis = math.sqrt(2) * (3.6 + 3.8)
goal_model_dir = './goal.sdf'
encoder, depth_decoder, w, h = get_depth_model("mono+stereo_640x192")

class Env():
    def __init__(self, is_training, train_env_id, test_env_id=2, visual_obs=False, num_scan_ranges=10):
        self.train_env_id = train_env_id
        self.test_env_id = test_env_id
        self.visual_obs = visual_obs
        self.num_scan_ranges = num_scan_ranges

        self.position = Pose()
        self.goal_position = Pose()
        self.goal_position.position.x = 0.
        self.goal_position.position.y = 0.

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.goal = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.past_distance = 0.

        if self.test_env_id == 2:
            self.test_goals = [(3.,3.), (-3.,2.), (3.,-3.), (-3., -1.2)]
            self.test_goals_id = 0

        self.is_training = is_training
        if self.is_training:
            self.threshold_arrive = 0.2
        else:
            self.threshold_arrive = 0.4

    def getGoalDistace(self):
        goal_distance = math.hypot(self.goal_position.position.x - self.position.x, self.goal_position.position.y - self.position.y)
        self.past_distance = goal_distance

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        q_x, q_y, q_z, q_w = orientation.x, orientation.y, orientation.z, orientation.w
        yaw = round(math.degrees(math.atan2(2 * (q_x * q_y + q_w * q_z), 1 - 2 * (q_y * q_y + q_z * q_z))))

        if yaw >= 0:
             yaw = yaw
        else:
             yaw = yaw + 360

        rel_dis_x = round(self.goal_position.position.x - self.position.x, 1)
        rel_dis_y = round(self.goal_position.position.y - self.position.y, 1)

        # Calculate the angle between robot and target
        if rel_dis_x > 0 and rel_dis_y > 0:
            theta = math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x > 0 and rel_dis_y < 0:
            theta = 2 * math.pi + math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x < 0 and rel_dis_y < 0:
            theta = math.pi + math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x < 0 and rel_dis_y > 0:
            theta = math.pi + math.atan(rel_dis_y / rel_dis_x)
        elif rel_dis_x == 0 and rel_dis_y > 0:
            theta = 1 / 2 * math.pi
        elif rel_dis_x == 0 and rel_dis_y < 0:
            theta = 3 / 2 * math.pi
        elif rel_dis_y == 0 and rel_dis_x > 0:
            theta = 0
        else:
            theta = math.pi
        rel_theta = round(math.degrees(theta), 2)

        diff_angle = abs(rel_theta - yaw)

        if diff_angle <= 180:
            diff_angle = round(diff_angle, 2)
        else:
            diff_angle = round(360 - diff_angle, 2)

        self.rel_theta = rel_theta
        self.yaw = yaw
        self.diff_angle = diff_angle

    def getState(self, scan, image):
        scan_range = []
        yaw = self.yaw
        rel_theta = self.rel_theta
        diff_angle = self.diff_angle
        min_range = 0.13
        if self.is_training:
            min_range = 0.2
        done = False
        arrive = False

        cof = (len(scan.ranges) / (self.num_scan_ranges - 1))
        for i in range(0, self.num_scan_ranges):
            n_i = math.ceil(i*cof - 1)
            if n_i < 0:
                n_i = 0
            if cof == 1:
                n_i = i
            if scan.ranges[n_i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[n_i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[n_i])

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = math.hypot(self.goal_position.position.x - self.position.x, self.goal_position.position.y - self.position.y)
        if current_distance <= self.threshold_arrive:
            # done = True
            arrive = True

        obs = None
        if self.visual_obs:
            image = PIL.Image.fromarray(image)
            di = get_depth(image, encoder, depth_decoder, w, h)[0] # get disparity map
            data = di.squeeze(0).squeeze(0).cpu().numpy()
            rescaled = (255.0 / data.max() * (data - data.min())).astype(np.uint8)
            spec_row = rescaled[rescaled.shape[0] // 2] # choose the middle row of the map

            # sample 10 value from this row as observation
            obs, n_samp = [], 10
            cof = len(spec_row)*1.0 / (n_samp - 1)
            for i in range(0, n_samp):
                n_i = math.ceil(i*cof - 1)
                if n_i < 0:
                    n_i = 0
                if cof == 1:
                    n_i = i
                obs.append(1. / (spec_row[n_i] + 0.00001))
        else:
            obs = scan_range

        return obs, current_distance, yaw, rel_theta, diff_angle, done, arrive

    def setReward(self, done, arrive):
        current_distance = math.hypot(self.goal_position.position.x - self.position.x, self.goal_position.position.y - self.position.y)
        distance_rate = (self.past_distance - current_distance)

        reward = 500.*distance_rate
        self.past_distance = current_distance

        if done:
            reward = -100.
            self.pub_cmd_vel.publish(Twist())

        if arrive:
            reward = 120.
            self.pub_cmd_vel.publish(Twist())
            rospy.wait_for_service('/gazebo/delete_model')
            self.del_model('target')

            # Build the target
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                goal_urdf = open(goal_model_dir, "r").read()
                target = SpawnModel
                target.model_name = 'target'  # the same with sdf name
                target.model_xml = goal_urdf
                if self.is_training:
                    if self.train_env_id == 3:
                        while True:
                            x, y = random.uniform(-3.3, 3.3), random.uniform(-3.3, 3.3)
                            if 1.5 > abs(x) > 0.5 and abs(y) < 2.5:
                                continue
                            elif 2.5 > abs(y) > 2. and 0. > x > 1.5:
                                continue
                            else:
                                break
                        self.goal_position.position.x = x
                        self.goal_position.position.y = y
                    else:
                        while True:
                            x, y = random.uniform(-3.2, 3.2), random.uniform(-3.2, 3.2)
                            if abs(x) > 1. or abs(y) > 1.:
                                break
                        self.goal_position.position.x = x
                        self.goal_position.position.y = y
                else:
                    self.test_goals_id += 1
                    if self.test_goals_id >= len(self.test_goals):
                        pass
                        #print('FINISHED!!!')
                        #exit(0)
                    else:
                        self.goal_position.position.x, self.goal_position.position.y = self.test_goals[self.test_goals_id]

                self.goal(target.model_name, target.model_xml, 'namespace', self.goal_position, 'world')
            except (rospy.ServiceException) as e:
                print("/gazebo/failed to build the target")
            rospy.wait_for_service('/gazebo/unpause_physics')
            self.goal_distance = self.getGoalDistace()
            arrive = False

        return reward

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel / 4
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        image = None
        while self.visual_obs == True and image is None:
            try:
                image = rospy.wait_for_message('camera1/image_raw', Image, timeout=5)
                bridge = CvBridge()
                try:
                    # self.camera_image is an ndarray with shape (h, w, c) -> (228, 304, 3)
                    image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                except Exception as e:
                    raise e
            except:
                pass

        state, rel_dis, yaw, rel_theta, diff_angle, done, arrive = self.getState(data, image)
        if self.visual_obs:
            print(state)
            state = [i / max(state) for i in state]
        else:
            state = [i / 3.5 for i in state]

        for pa in past_action:
            state.append(pa)

        state = state + [rel_dis / diagonal_dis, yaw / 360, rel_theta / 360, diff_angle / 180]
        reward = self.setReward(done, arrive)

        return np.asarray(state), reward, done, arrive

    def reset(self):
        # Reset the env #
        rospy.wait_for_service('/gazebo/delete_model')
        self.del_model('target')

        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        # Build the targetz
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            goal_urdf = open(goal_model_dir, "r").read()
            target = SpawnModel
            target.model_name = 'target'  # the same with sdf name
            target.model_xml = goal_urdf
            if self.is_training:
                if self.train_env_id == 3:
                    while True:
                        x, y = random.uniform(-3.3, 3.3), random.uniform(-3.3, 3.3)
                        if 1.5 > abs(x) > 0.5 and abs(y) < 2.5:
                            continue
                        elif 2.5 > abs(y) > 2. and 0. > x > 1.5:
                            continue
                        else:
                            break
                    self.goal_position.position.x = x
                    self.goal_position.position.y = y
                else:
                    while True:
                        x, y = random.uniform(-3.2, 3.2), random.uniform(-3.2, 3.2)
                        if abs(x) > 1. or abs(y) > 1.:
                            break
                    self.goal_position.position.x = x
                    self.goal_position.position.y = y
            else:
                self.goal_position.position.x, self.goal_position.position.y = self.test_goals[self.test_goals_id]

            # if -0.3 < self.goal_position.position.x < 0.3 and -0.3 < self.goal_position.position.y < 0.3:
            #     self.goal_position.position.x += 1
            #     self.goal_position.position.y += 1

            self.goal(target.model_name, target.model_xml, 'namespace', self.goal_position, 'world')
        except (rospy.ServiceException) as e:
            print("/gazebo/failed to build the target")
        rospy.wait_for_service('/gazebo/unpause_physics')
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        image = None
        while self.visual_obs == True and image is None:
            try:
                image = rospy.wait_for_message('camera1/image_raw', Image, timeout=5)
                bridge = CvBridge()
                try:
                    # self.camera_image is an ndarray with shape (h, w, c) -> (228, 304, 3)
                    image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                except Exception as e:
                    raise e
            except:
                pass

        self.goal_distance = self.getGoalDistace()
        state, rel_dis, yaw, rel_theta, diff_angle, done, arrive = self.getState(data, image)
        if self.visual_obs:
            state = [i / max(state) for i in state]
        else:
            state = [i / 3.5 for i in state]

        state.append(0)
        state.append(0)

        state = state + [rel_dis / diagonal_dis, yaw / 360, rel_theta / 360, diff_angle / 180]

        return np.asarray(state)
