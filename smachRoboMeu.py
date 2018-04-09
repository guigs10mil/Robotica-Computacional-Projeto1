#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import cormodule

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0



tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados


def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)


class Procurando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['achou', 'procurando', 'vaibater'])
	def execute(self, userdata):
		global velocidade_saida
		if media is None or len(media) == 0:
			# print(media)
			return 'procurando'

		if False:
			return 'vaibater'

		else:
			print(media)
			return 'achou'


class Movendo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando'])
	def execute(self, userdata):
		return 'alinhando'


class Alinhando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando', 'andando', 'sumiu', 'vaibater'])
	def execute(self, userdata):
		global velocidade_saida

		if media is None or len(media) == 0:
			return 'sumiu'

		if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'alinhando'

		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			return 'alinhando'

		

		if False:
			return 'vaibater'

		else:
			return 'andando'


class Andando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando', 'andando', 'sumiu', 'vaibater'])

	def execute(self, userdata):
		global velocidade_saida
		if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'

		if media is None or len(media)==0:
			return 'sumiu'

		if False:
			return 'vaibater'

		else:
			vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'andando'


# class Sobrevivendo(smach.State):
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['tasafeprocurando', 'tasafemovendo', 'vaibater'])
# 	def execute(self, userdata):
# 		if centro not in centro:
# 			return 'alinhando'

# 		if centro caixa == centro cam:
# 			return 'andando'

# 		if caixa not in tela:
# 			return 'perdeu'

# 		if achou parede:
# 			return 'vaibater'


# main
def main():
	global velocidade_saida
	rospy.init_node('unico_estado')

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Cria uma máquina de estados
	sm_top = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm_top:
	    smach.StateMachine.add('PROCURANDO', Procurando(),
	    	transitions={'achou': 'MOVENDO','procurando':'PROCURANDO', 'vaibater': 'fim_geral'})

	    smach.StateMachine.add('MOVENDO', Movendo(),
	    	transitions={'alinhando': 'SUB'})

	    sm_sub = smach.StateMachine(outcomes=['procurando', 'vaibater'])

	    with sm_sub:
	    	smach.StateMachine.add('ALINHANDO', Alinhando(),
	    		transitions={'alinhando': 'ALINHANDO', 'andando': 'ANDANDO', 'sumiu': 'procurando', 'vaibater': 'vaibater'})

	    	smach.StateMachine.add('ANDANDO', Andando(),
	    		transitions={'alinhando': 'ALINHANDO', 'andando': 'ANDANDO', 'sumiu': 'procurando', 'vaibater': 'vaibater'})

	    smach.StateMachine.add('SUB', sm_sub, 
	    	transitions={'procurando': 'PROCURANDO', 'vaibater': 'fim_geral'})

	# Executa a máquina de estados
	outcome = sm_top.execute()

	print("Execute finished")

if __name__ == '__main__':
	print("Main")
	main()
