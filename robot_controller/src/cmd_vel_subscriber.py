#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
sys.path.append('/home/catkin_ws/src/robot_controller/src')
from geometry_msgs.msg import Twist
import serial

def callback(data):
    linear_cmd = data.linear.x  # Comando linear (para frente/trás)
    angular_cmd = data.angular.z  # Comando angular (rotação)

    cmd = ' '  # Comando padrão (parado)

    if linear_cmd > 0:
        cmd = 'W'  # Avançar
        rospy.logerr("W")
    elif linear_cmd < 0:
        cmd = 'S'  # Ré
        rospy.logerr("S")
    if angular_cmd > 0:
        cmd = 'A'  # Girar à esquerda
        rospy.logerr("A")
    elif angular_cmd < 0:
        cmd = 'D'  # Girar à direita
        rospy.logerr("D")

    try:
        # Abra a porta serial do ESP32 e envie o comando
        ser = serial.Serial('/dev/ttyUSB0', 115200)  # Porta serial do Raspberry Pi
        ser.write(cmd.encode())
        ser.close()
    except serial.SerialException as e:
        rospy.logerr("Erro na porta serial: %s", str(e))

def teleop_listener():
    rospy.init_node('teleop_listener', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback)  # Assine o tópico 'cmd_vel' do teleop_twist_keyboard
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop_listener()
    except rospy.ROSInterruptException:
        pass
