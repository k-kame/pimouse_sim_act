#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
# from pimouse_ros.msg import LightSensorValues
from raspimouse_ros_2.msg import *

S_TH_ACT = 100

class WallAround():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # 光センサのメッセージオブジェクト
        self.sensor_values = LightSensorValues()
        # モータに速度を入力するためのパブリッシャ
        if sim_act == 1:
            self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def sensor_callback(self, msg):
        self.sensor_values = msg

    # （実機）モーターパブリッシャ
    def motor_cont_a(self, xv, zrot):
        d = Twist()
        d.linear.x  = xv
        d.angular.z = zrot
        self.cmd_vel.publish(d)

    # ロボット動作関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def move_front(self):
        if sim_act == 1:
            self.motor_cont_a(0.2, 0)

    def move_turnright(self):
        if sim_act == 1:
            self.motor_cont_a(0.0, -math.pi)

    # 環境設定のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def init_robot(self):
        # サービスが立ち上がるまでサービスコールをストップ
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        # シャットダウンのためのフックを登録
        rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
        # /motor_on という名前でTrigger型のサービスを定義
        rospy.ServiceProxy('/motor_on',Trigger).call()

    # 主関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def run(self):
        # シミュレーション環境／ハードウェアの初期化
        if sim_act == 1:
            self.rate = rospy.Rate(20)
            self.init_robot()

        # 以下メインループ
        while not rospy.is_shutdown():
            # 計算部（センサ値▶モーター速度）
            if self.sensor_values.left_forward < S_TH_ACT or self.sensor_values.right_forward < S_TH_ACT:
                self.move_front()
            else:
                self.move_turnright()            

            self.rate.sleep()
            # メインループ（ここまで）

if __name__ == '__main__':
    # ノード初期化
    rospy.init_node('wall_trace')
    # シミュ／実機切り替えパラメータの取得（シミュ:0/実機:1）
    sim_act = rospy.get_param("sim_act")

    # run 関数の呼び出し
    WallAround().run()
