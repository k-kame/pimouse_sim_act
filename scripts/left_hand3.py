#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 変数 sim_act に関して（シミュ:0/実機:1）

import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse, Empty
# ここ分岐できるか？＜実機もこのままで行けるかも？
from raspimouse_ros_2.msg import *

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

    # （実機）周波数データをTwist型に変換
    def freq2twist(self, left_hz, right_hz):
        RADIUS = 24         # 車輪半径
        TREAD  = 92         # 車輪間距離
        RESOLUSION = 400    # 一回転ステップ数
        data = Twist()
        vR = 2*math.pi*RADIUS*right_hz/RESOLUSION
        vL = 2*math.pi*RADIUS*left_hz/RESOLUSION
        v  = (vR+vL)/2
        th = math.atan((vR-vL)/TREAD)
        data.linear.x  = v*math.sin(th)
        data.linear.y  = v*math.cos(th)
        data.angular.z = (vR-vL)/TREAD

    # メインループの中身と動作のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def turn_move(self, m):
        if m == "LEFT": self.motor_cont(-200, 200)
        if m == "RIGHT": self.motor_cont(200, -200)

    def moveFeedback(self, offset, speed, k, mode):
        # left_sideが2000より大きい時は、右回り旋回
        if self.sensor_values.left_side > 1500:
            self.turn_move("RIGHT")
            return
        
        # right_sideが2000より大きい時は、右回り旋回
        if self.sensor_values.right_side > 1500:
            self.turn_move("LEFT")
            return

        # 壁沿いを追従走行するための計算
        # (基準値 - 現在のleft_side) * ゲイン
        if mode == "LEFT":
            diff = (offset - self.sensor_values.left_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed - diff, speed + diff)
        if mode == "RIGHT":
            diff = (offset - self.sensor_values.right_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed + diff, speed - diff)

    def stopMove(self):
        # 終了時にモータを止める
        self.motor_cont(0, 0)

    def checker(self):
        # 壁無し判定
        if self.sensor_values.left_side < 100:
            print("--RS_COUNT:", self.sensor_values.left_side)
            self.rs_count += 1
        if self.sensor_values.right_side < 150:
            print("--LS_COUNT:", self.sensor_values.right_side)
            self.ls_count += 1

    def motion(self):
        # 左側に壁がある確率が高くて、目の前に壁がなさそうなとき
        if self.sensor_values.left_forward < 300 or self.sensor_values.right_forward < 300:
            print("Move: STRAIGHT")
            for time in range(12):
                self.checker()
                if self.sensor_values.left_side > self.sensor_values.right_side:
                    self.moveFeedback(500, 500, 0.2, "LEFT")
                else:
                    self.moveFeedback(500, 500, 0.2, "RIGHT")
                self.rate.sleep()
            self.stopMove()
            
            # 目の前に壁がなくて、右側に壁がない場合
            if self.sensor_values.left_forward < 300 or self.sensor_values.right_forward < 300:
                if self.rs_count > 0:
                    print("Move: MID LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
            # 直進した後に、目の前に壁があったとき
            elif self.sensor_values.left_forward > 300 and self.sensor_values.right_forward > 300:
                # 左右の壁がない場合
                if self.ls_count > 0 and self.rs_count > 0:
                    print("Move: LEFT TURN_2")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
                # 右の壁がない場合
                elif self.ls_count > 0:
                    print("Move: RIGHT TURN")
                    for time in range(10):
                        self.turn_move("RIGHT")
                        self.rate.sleep()
                    self.stopMove()
                # 左の壁がない場合
                elif self.rs_count > 0:
                    print("Move: LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()          
            self.ls_count = 0
            self.rs_count = 0
            return
        # 左右関係なく、目の前に壁があるとき
        if self.sensor_values.left_forward > 2000 and self.sensor_values.right_forward > 2000:
            print("Move: DEAD END")
            for time in range(20):
                self.turn_move("LEFT")
                self.rate.sleep()
            self.stopMove()
            self.ls_count = 0
            self.rs_count = 0
            return
        if self.sensor_values.left_side > self.sensor_values.right_side:
            self.moveFeedback(500, 500, 0.2, "LEFT")
        else:
            self.moveFeedback(500, 500, 0.2, "RIGHT")
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
        self.rate = rospy.Rate(10)
        # シミュレーション環境／ハードウェアの初期化
        if sim_act == 0:
            self.init()
        else:
            # サービスが立ち上がるまでサービスコールをストップ
            rospy.wait_for_service('/motor_on')
            rospy.wait_for_service('/motor_off')

        # シャットダウンのためのフックを登録
        if sim_act == 0:
            rospy.on_shutdown(self.stopMove)
        else:
            rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)

        # /motor_on という名前でTrigger型のサービスを定義
        if sim_act == 1:
            rospy.ServiceProxy('/motor_on',Trigger).call()

        # 以下の条件により停止（必要？）
        # while self.sensor_values.left_side == 0 and self.sensor_values.right_side == 0:
        #     self.rate.sleep()
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
