#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse, Empty
from raspimouse_ros_2.msg import *

# 変数 sim_act に関して（シミュ:0/実機:1）

# P_1BLK_SIM = 477 # ロボットが１ブロック移動するためのパルス数
# P_QUAD_SIM = 192 # ロボットが90度旋回するためのパルス数

DT_SIM = 10
T_MOVE = 1.8
T_ROT  = 2.04
T_INTERVAL = 3

# シミュと実機ごとの移動速度／センサ閾値設定用グローバル定数
# シミュのための定数
V_X_SIM  = 0.1
R_Z_SIM  = math.pi/4
S_TH_SIM = 1500
# 実機のための定数
V_X_ACT  = 0.05
R_Z_ACT  = -math.pi/4
S_TH_ACT = 1500
# 参照される定数（編集しない）
V_X  = 0
R_Z  = 0
S_TH = 0
DT = 0

class LeftHand():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # 光センサのメッセージオブジェクト
        self.sensor_values = LightSensorValues()
        # モータのパブリッシャー（シミュ（周波数）▶ 実機（x速度とz回転角に統一））
        # self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # グローバル変数の再定義
        global V_X, R_Z, S_TH, DT
        if sim_act == 0:
            V_X  = V_X_SIM
            R_Z  = R_Z_SIM
            S_TH = S_TH_SIM
            DT = DT_SIM
        else:
            V_X  = V_X_ACT
            R_Z  = R_Z_ACT
            S_TH = S_TH_ACT
            DT = DT_SIM

        # （シミュ）シミュレータを初期状態にする
        if sim_act == 0:
            self.modeSimReset = True
            self.ls_count = 0
            self.rs_count = 0

    # センサコールバック関数
    def sensor_callback(self, msg):
        self.sensor_values = msg

    # （実機）モーターパブリッシャ（シミュでも利用）
    def motor_cont_act(self, xv, zrot):
        d = Twist()
        d.linear.x  = xv
        d.angular.z = zrot
        self.cmd_vel.publish(d)

    # ロボット動作関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # 停止
    def move_stop(self):
        self.motor_cont_act(0, 0)

    # １ブロック前進
    def move_front_1bk(self):
        tb = rospy.get_time()
        while rospy.get_time() - tb < T_MOVE:
            self.motor_cont_act(V_X, 0)

    # 90度時計旋回
    def move_turncw_quater(self):
        tb = rospy.get_time()
        while rospy.get_time() - tb < T_ROT:
            self.motor_cont_act(0.0, -R_Z)

    # 90度反時計旋回
    def move_turnccw_quater(self):
        tb = rospy.get_time()
        while rospy.get_time() - tb < T_ROT:
            self.motor_cont_act(0.0, R_Z)

    # 環境設定のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # シミュレーション環境の初期化
    def init_simu(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException as e: print('Service call failed: %s', e)

    # ロボット初期化
    def init_robot(self):
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException as e: print('Service call failed: %s', e)
        # シャットダウンのためのフックを登録
        rospy.on_shutdown(self.move_stop)

    # 主関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def run(self):
        self.rate = rospy.Rate(DT)

        # シミュレーション環境初期化
        if sim_act == 0:
            self.init_simu()
        # ロボット初期化
        self.init_robot()

        self.move_turnccw_quater()
        self.move_stop()

        # 以下メインループ
        tb = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - tb > T_INTERVAL:
                tb = rospy.get_time()
                rostime = rospy.get_rostime()
                rospy.loginfo("TIME: %s", tb)

                # rospy.loginfo("TIME: %s, LF: %i, LS: %i, RF: %i, RS: %i",
                #     rostime, 
                #     self.sensor_values.left_forward, 
                #     self.sensor_values.left_side, 
                #     self.sensor_values.right_forward, 
                #     self.sensor_values.right_side)

                # self.move_turnccw_quater()
                # self.move_stop()

                self.move_front_1bk()
                self.move_stop()

            self.rate.sleep()
        # メインループ（ここまで）

if __name__ == '__main__':
    # ノード初期化
    rospy.init_node('LeftHand', log_level=rospy.DEBUG)
    # シミュ／実機パラメータの取得（シミュ:0/実機:1）
    sim_act = rospy.get_param("sim_act")

    # run 関数の呼び出し
    LeftHand().run()
