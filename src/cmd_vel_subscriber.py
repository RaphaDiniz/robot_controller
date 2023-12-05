#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial

class RobotController:

    def __init__(self):

        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.serial_port_name = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.serial_port = serial.Serial(self.serial_port_name, 115200, timeout=1)
        self.twist = Twist()

    def cancel(self):
        self.sub_cmd_vel.unregister()
        rospy.loginfo("Shutting down this node.")

    def cmd_vel_callback(self, twist_cmd):
        # Callback para receber comandos do tópico cmd_vel
        linear = twist_cmd.linear.x
        angular = twist_cmd.angular.z

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

    def send_serial_command(self, command):
        # Envia o comando para a porta serial
        try:
            self.serial_port.write(command.encode())
            rospy.loginfo("Comando enviado para a porta serial: {}".format(command))
        except serial.SerialException as e:
            rospy.logerr("Erro ao escrever na porta serial: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=False)
    controller = RobotController()

    # Subscreva o tópico cmd_vel com a função cmd_vel_callback
    rospy.Subscriber('/cmd_vel', Twist, controller.cmd_vel_callback)

    rospy.spin()
    controller.cancel()
