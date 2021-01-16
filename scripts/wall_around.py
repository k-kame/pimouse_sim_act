#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,copy,math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallAround():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # モータに速度を入力するためのパブリッシャ（ここが違う）
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        # 光センサのメッセージオブジェクト
        self.sensor_values = LightSensorValues()

    def sensor_callback(self,messages):
        self.sensor_values = messages

    # メインループの中身と動作のための関数　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    def wall_front(self,ls):
        return ls.left_forward > 50 or ls.right_forward > 50

    def too_right(self,ls):
        return ls.right_side > 50
     
    def too_left(self,ls):
        return ls.left_side > 50

    def run(self):
        # 更新頻度の設定
        rate = rospy.Rate(20)
        # node:motors に渡すデータ(Twist型)の定義と初期化（前進速度と旋回速度を0に設定）
        data = Twist()
        data.linear.x = 0.0
        data.angular.z = 0.0

        # この while がメインループ
        while not rospy.is_shutdown():
            # 速度 0.1 で前進
            data.linear.x = 0.1

            # node:lightsensors から得たデータ(LightSensorValues型)に基づいて行動分岐
            if self.wall_front(self.sensor_values):
                data.angular.z = - math.pi
            elif self.too_right(self.sensor_values):
                data.angular.z = math.pi
            elif self.too_left(self.sensor_values):
                data.angular.z = - math.pi
            else:
                e = 1.0 * (50 - self.sensor_values.left_side)
                data.angular.z = e * math.pi / 180.0
                
            # node:motors にデータをパブリッシュ cmd_vel
            self.cmd_vel.publish(data)
            # 次のタイミングまでスリープ
            rate.sleep()
    # メインループの中身（ここまで）＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

if __name__ == '__main__':
    # ノード初期化
    rospy.init_node('wall_trace')

    ### サービスの準備
    # サービスが立ち上がるまでサービスコールをストップ
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    # シャットダウンのためのフックを登録
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    # /motor_on という名前でTrigger型のサービスを定義
    rospy.ServiceProxy('/motor_on',Trigger).call()

    # run 関数の呼び出し
    w = WallAround()
    w.run()
