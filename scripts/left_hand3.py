#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 変数 sim_act に関して（シミュ:0/実機:1）

import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse, Empty
from raspimouse_ros_2.msg import *

P_1BLK = 477 # ロボットが１ブロック移動するためのパルス数
P_QUAD = 192 # ロボットが90度旋回するためのパルス数
S_TH   = 300 # 壁の有無を判断するための閾値

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

        # （シミュ）シミュレータを初期状態にする
        if sim_act == 0:
            self.modeSimReset = True
            self.ls_count = 0
            self.rs_count = 0

    def sensor_callback(self, msg):
        # クラス変数のメッセージオブジェクトに受信したデータをセット
        self.sensor_values = msg

    # （シミュ）モーターパブリッシャ
    def motor_cont(self, left_hz, right_hz):
        if not rospy.is_shutdown():
            d = MotorFreqs()
            d.left_hz = left_hz
            d.right_hz = right_hz
            self.motor_raw_pub.publish(d)

    # ロボット動作関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # 停止
    def move_stop(self):
        self.motor_cont(0, 0)

    # １マス前進
    def move_front(self):
        self.motor_cont(P_1BLK, P_1BLK) 

    # 右旋回
    def move_turnright(self):
        self.motor_cont(P_QUAD, -P_QUAD)

    # 環境設定のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # シミュレーション環境の初期化
    def init_simu(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException, e: print "Service call failed: %s"%e

    # ロボットの初期化
    def init_robot(self):
        rospy.wait_for_service('/motor_on')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException, e: print "Service call failed: %s"%e
        # シャットダウンのためのフックを登録
        rospy.on_shutdown(self.move_stop)

    # 主関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def run(self):
        # シミュレーション環境／ハードウェアの初期化
        if sim_act == 0:
            self.rate = rospy.Rate(1000)
            self.init_simu()
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
