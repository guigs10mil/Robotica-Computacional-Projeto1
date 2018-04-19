#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math


def leu_imu(dado):
    
    quat = dado.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))

    bateu=False
    mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. Angular: x {:.2f}, y {:.2f}, z {:.2f}\

	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}
    """.format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
    print(mensagem)

    if dado.linear_acceleration.x + 0.75 < -3:
        print("bateu mermao")
        
        vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)

        rospy.sleep(0.5)

        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)

        rospy.sleep(0.5)

    elif dado.linear_acceleration.x + 0.75 > 3:
        print("deu re no kibe") 
        
        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)

        rospy.sleep(0.5)

        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)

        rospy.sleep(0.5)
    
    else:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)


    


	


if __name__=="__main__":
    rospy.init_node("le_imu")
    recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu,queue_size=1)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
    try:
        while not rospy.is_shutdown():
            print("Main loop")
            rospy.sleep(1)


    except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")



        


