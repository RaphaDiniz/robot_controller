#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial



class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.default_command = 'S'
        self.obstacle_command = 'A'
        self.obstacle_all_command = 'W'
        self.obstacle_left_command = 'D'
        self.stop_command = 'K'
        self.linear = 0.3
        self.angular = 1
        self.ResponseDist = 0.25
        self.twist_cmd = Twist() 
        self.LaserAngle = 30  # 10~180
        self.Moving = False
        self.switch = False
        self.autonomous_mode = False
        self.running = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.before_command = ''
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.serial_port = serial.Serial(self.serial_port, 115200, timeout=1)

    def cancel(self):
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config

    def registerScan(self, scan_data):
        if self.running == True: return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        ranges = np.array(scan_data.ranges)
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        sortedIndices = np.argsort(ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        # print "scan_data:", len(sortedIndices)
        # if we already have a last scan to compare to:
        for i in sortedIndices:
            if len(np.array(scan_data.ranges)) == 270:
                # 通过清除不需要的扇区的数据来保留有效的数据
                if 10 < i < self.LaserAngle:
                    if ranges[i] < self.ResponseDist: self.Left_warning += 1
                elif (270 - self.LaserAngle) < i < 270:
                    if ranges[i] < self.ResponseDist: self.Right_warning += 1
                elif (270 <= i <= 280) or (0<= i <=10):
                    # print ("i: {},dist: {}", format(i, ranges[i]))
                    if ranges[i] < self.ResponseDist: self.front_warning += 1
        # print (self.Left_warning,self.front_warning,self.Right_warning)

    def cmd_vel_callback(self, twist_cmd):
        # Callback para receber comandos do tópico cmd_vel
        linear = twist_cmd.linear.x
        angular = twist_cmd.angular.z

        if linear > 0 and angular < 0:
            self.autonomous_mode = True
        if linear > 0 and angular > 0:
            self.autonomous_mode = False

        # Lógica para converter os comandos Twist em comandos específicos do seu robô
        if linear > 0:
            command = 'S' 
        elif linear < 0:
            command = 'W'
        elif angular > 0:
            command = 'A' 
        elif angular < 0: 
            command = 'D'
        else:
            command = 'K'  # Parar o robô

        self.send_serial_command(command)

    def robot_move(self):
        linear = self.twist_cmd.linear.x
        angular = self.twist_cmd.angular.z

        if linear > 0 and angular < 0:
            self.autonomous_mode = True
        if linear > 0 and angular > 0:
            self.autonomous_mode = False

        if self.autonomous_mode:
            if self.switch:
                if self.Moving:
                    self.send_serial_command(self.default_command)
                    self.Moving = not self.Moving
                return

            self.Moving = True

            # Lógica para controle autônomo
            if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
                self.send_serial_command(self.obstacle_all_command)
            elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
                self.send_serial_command(self.obstacle_left_command)
                if self.Left_warning > 10 and self.Right_warning <= 10:
                    self.send_serial_command(self.obstacle_command)
            elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
                self.send_serial_command(self.obstacle_command)
                if self.Left_warning <= 10 and self.Right_warning > 10:
                    self.send_serial_command(self.obstacle_left_command)
            elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:
                self.send_serial_command(self.obstacle_left_command)
            elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
                self.send_serial_command(self.obstacle_command)
            elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
                self.send_serial_command(self.obstacle_left_command)
            elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
                self.send_serial_command(self.obstacle_command)
            elif self.front_warning <= 10 and (self.Left_warning <= 10 and self.Right_warning <= 10):
                self.send_serial_command(self.default_command)
        else:
            self.autonomous_mode = False

        self.r.sleep()

    
    def send_serial_command(self, command):
        # Envia o comando para a porta serial
        try:
            if command != self.before_command:
                self.before_command = command;
                cammandEncode = command.encode('utf-8');
                self.serial_port.write(cammandEncode)
                print(cammandEncode)
                rospy.loginfo("Comando enviado para a porta serial: {}".format(cammandEncode))
        except serial.SerialException as e:
            rospy.logerr("Erro ao escrever na porta serial: {}".format(e))

    def switch_control(self, twist_cmd):
        linear = twist_cmd.linear.x
        angular = twist_cmd.angular.z

        if linear > 0 and angular < 0:
            self.autonomous_mode = True
        if linear > 0 and angular > 0:
            self.autonomous_mode = False

        if self.autonomous_mode:
            self.robot_move()
        if not self.autonomous_mode:
            self.cmd_vel_callback(twist_cmd)



if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=False)
    tracker = laserAvoid()

    # Subscreva o tópico cmd_vel com a função switch_control
    rospy.Subscriber('/cmd_vel', Twist, tracker.switch_control)

    rospy.spin()
    tracker.cancel()
