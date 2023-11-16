#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial
#Raphael Diniz relampago_marquinhos_robot

class RobotController:
    def __init__(self, serial_port):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.serial_port = serial.Serial(serial_port, 115200, timeout=1)
        self.twist = Twist()
        self.default_command = 'S'
        self.obstacle_command = 'D'
        self.obstacle_distance = 0.03  # Distância mínima para detecção de obstáculos
        self.obstacle_detected = False

    def scan_callback(self, scan_data):
        # Imprimir os valores das leituras do LIDAR
        # rospy.loginfo("Leituras do LIDAR: {}".format(scan_data.ranges))

        # Verificar se há obstáculo à frente e ajustar o movimento
        if min(scan_data.ranges[1:31]) < self.obstacle_distance:
            if not self.obstacle_detected:
                # Se um obstáculo foi detectado, envie o comando 'D' e marque o obstáculo como detectado
                self.send_serial_command(self.obstacle_command)
                rospy.loginfo("Obstáculo detectado. Comando enviado para a porta serial: {}".format(self.obstacle_command))
                self.obstacle_detected = True
        else:
            # Se não há obstáculo, envie o comando 'S' e marque o obstáculo como não detectado
            self.send_serial_command(self.default_command)
            rospy.loginfo("Nenhum obstáculo detectado. Comando enviado para a porta serial: {}".format(self.default_command))
            self.obstacle_detected = False

    def control_loop(self):
        rospy.loginfo("Entrando no loop de controle.")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Publica os comandos Twist para movimentar o robô
            self.cmd_pub.publish(self.twist)
            rate.sleep()
        rospy.loginfo("Saindo do loop de controle.")

    def send_serial_command(self, command):
        # Envia o comando para a porta serial
        try:
            self.serial_port.write(command.encode())
            rospy.loginfo("Comando enviado para a porta serial: {}".format(command))
        except serial.SerialException as e:
            rospy.logerr("Erro ao escrever na porta serial: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('robot_controller')
    serial_port = '/dev/ttyUSB1'  # Substitua pela sua porta serial correta
    controller = RobotController(serial_port)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
