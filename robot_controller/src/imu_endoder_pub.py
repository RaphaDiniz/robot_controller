#!/usr/bin/env python

import rospy
import serial

def serial_data_reader():
    rospy.init_node('robot_controller', anonymous=True)

    # Configura a porta serial
    serial_port = '/dev/ttyUSB0'  
    baud_rate = 115200
    ser = serial.Serial(serial_port, baud_rate)

    while not rospy.is_shutdown():
        try:
            # Lê os dados da porta serial
            data = ser.readline()
            
            # Publica os dados como uma mensagem ROS
            rospy.loginfo("Dados da porta serial: %s", data)
        except serial.SerialException as e:
            rospy.logerr("Erro na porta serial: %s", str(e))
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        serial_data_reader()
    except rospy.ROSInterruptException:
        pass
