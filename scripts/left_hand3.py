#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse, Empty
from raspimouse_ros_2.msg import *

# 変数 sim_act に関して（シミュ:0/実機:1）

# シミュのための定数
P_1BLK_SIM = 477 # ロボットが１ブロック移動するためのパルス数
P_QUAD_SIM = 192 # ロボットが90度旋回するためのパルス数
S_TH_SIM   = 300 # 壁の有無を判断するための閾値
# 実機のための定数
S_TH_ACT = 100

# 参照される定数（編集しない）
P_1BLK = 0
P_QUAD = 0
S_TH   = 0

class LeftHand():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # 光センサのメッセージオブジェクト
        self.sensor_values = LightSensorValues()
        # モータのパブリッシャー（シミュ（周波数）:0／実機（速度）:1）
        if sim_act == 0:
            self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        else:
            self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # グローバル変数の再定義
        global P_1BLK, P_QUAD, S_TH
        if sim_act == 0:
            P_1BLK = P_1BLK_SIM
            P_QUAD = P_QUAD_SIM
            S_TH   = S_TH_SIM
        else:
            S_TH   = S_TH_ACT

        # （シミュ）シミュレータを初期状態にする
        if sim_act == 0:
            self.modeSimReset = True
            self.ls_count = 0
            self.rs_count = 0

    # センサコールバック関数
    def sensor_callback(self, msg):
        self.sensor_values = msg

    # （シミュ）モーターパブリッシャ
    def motor_cont_simu(self, left_hz, right_hz):
        d = MotorFreqs()
        d.left_hz = left_hz
        d.right_hz = right_hz
        self.motor_raw_pub.publish(d)

    # （実機）モーターパブリッシャ
    def motor_cont_act(self, xv, zrot):
        d = Twist()
        d.linear.x  = xv
        d.angular.z = zrot
        self.cmd_vel.publish(d)

    # ロボット動作関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # 停止
    def move_stop(self):
        if sim_act == 0:
            self.motor_cont_simu(0, 0)

    # １マス前進
    def move_front(self):
        if sim_act == 0:
            self.motor_cont_simu(P_1BLK, P_1BLK) 
        else:
            self.motor_cont_act(0.2, 0)

    # 右旋回
    def move_turnright(self):
        if sim_act == 0:
            self.motor_cont_simu(P_QUAD, -P_QUAD)
        else:
            self.motor_cont_act(0.0, -math.pi)

    # 環境設定のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # シミュレーション環境の初期化
    def init_simu(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException, e: print "Service call failed: %s"%e

    # ロボット初期化
    def init_robot(self):
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException, e: print "Service call failed: %s"%e
        # シャットダウンのためのフックを登録
        rospy.on_shutdown(self.move_stop)

    # 主関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def run(self):
        self.rate = rospy.Rate(20)
        # シミュレーション環境初期化
        if sim_act == 0:
            self.init_simu()
        # ロボット初期化
        self.init_robot()

        # 以下メインループ
        while not rospy.is_shutdown():
            # 計算部（センサ値▶モーター速度）
            if self.sensor_values.left_forward < S_TH or self.sensor_values.right_forward < S_TH:
                self.move_front()
            else:
                self.move_turnright()

            self.rate.sleep()
            # メインループ（ここまで）

if __name__ == '__main__':
    # ノード初期化
    rospy.init_node('LeftHand')
    # シミュ／実機パラメータの取得（シミュ:0/実機:1）
    sim_act = rospy.get_param("sim_act")

    # run 関数の呼び出し
    LeftHand().run()
