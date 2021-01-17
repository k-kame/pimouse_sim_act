#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 変数 sim_act に関して（シミュ:0/実機:1）

import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse, Empty
# ここ分岐できるか？＜実機もこのままで行けるかも？
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
        # モータのパブリッシャー
        if sim_act == 0:
            # （シミュ）モータに周波数を入力するパブリッシャ
            self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        else:
            # （実機）モータに速度を入力するパブリッシャ
            self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        # （シミュ）実行時にシミュレータを初期状態にする
        if sim_act == 0:
            self.modeSimReset = True
            self.ls_count = 0
            self.rs_count = 0

    def sensor_callback(self, msg):
        # クラス変数のメッセージオブジェクトに受信したデータをセット
        self.sensor_values = msg

    # （シミュ）メインループで計算した車輪の周波数データを MotorFreqs()型にまとめてパブリッシュ
    def motor_cont(self, left_hz, right_hz):
        if not rospy.is_shutdown():
            d = MotorFreqs()
            # 両輪の周波数を設定
            d.left_hz = left_hz
            d.right_hz = right_hz
            # パブリッシュ
            self.motor_raw_pub.publish(d)

    # # （実機）周波数データをTwist型に変換
    # def freq2twist(self, left_hz, right_hz):
    #     RADIUS = 24         # 車輪半径
    #     TREAD  = 92         # 車輪間距離
    #     RESOLUSION = 400    # 一回転ステップ数
    #     data = Twist()
    #     vR = 2*math.pi*RADIUS*right_hz/RESOLUSION
    #     vL = 2*math.pi*RADIUS*left_hz/RESOLUSION
    #     v  = (vR+vL)/2
    #     th = math.atan((vR-vL)/TREAD)
    #     data.linear.x  = v*math.sin(th)
    #     data.linear.y  = v*math.cos(th)
    #     data.angular.z = (vR-vL)/TREAD

    # メインループの中身と動作のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    # 停止
    def move_stop(self):
        self.motor_cont(0, 0)

    # １マス前進
    def move_front(self):
        self.motor_cont(P_1BLK, P_1BLK) 

    # 右旋回
    def move_turnright(self):
        self.motor_cont(P_QUAD, -P_QUAD)

    # 主関数
    def motion(self):
        if self.sensor_values.left_forward < S_TH or self.sensor_values.right_forward < S_TH:
            self.move_front()
        else:
            self.move_turnright()

    # メインループの中身（ここまで）＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

    # （シミュ）シミュレーション環境の初期化
    def init(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException, e: print "Service call failed: %s"%e
        rospy.wait_for_service('/motor_on')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException, e: print "Service call failed: %s"%e
        
    def run(self):
        # 更新頻度の設定
        self.rate = rospy.Rate(1000)
        # シミュレーション環境／ハードウェアの初期化
        if sim_act == 0:
            self.init()
        else:
            # サービスが立ち上がるまでサービスコールをストップ
            rospy.wait_for_service('/motor_on')
            rospy.wait_for_service('/motor_off')

        # シャットダウンのためのフックを登録
        if sim_act == 0:
            rospy.on_shutdown(self.move_stop)
        else:
            rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)

        # /motor_on という名前でTrigger型のサービスを定義
        if sim_act == 1:
            rospy.ServiceProxy('/motor_on',Trigger).call()

        # 以下メインループ
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    # ノード初期化
    rospy.init_node('LeftHand')
    # パラメータの取得（シミュ:0/実機:1）
    sim_act = rospy.get_param("sim_act")
    # run 関数の呼び出し
    LeftHand().run()
