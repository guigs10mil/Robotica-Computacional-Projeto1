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
from sensor_msgs.msg import LaserScan
import smach
import smach_ros

import Cor_Rosa

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0



tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.25
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

scanmin = 5
distanciaMin = 0.3
sleeptime = 0.2 

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
		media, centro, area = Cor_Rosa.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

def scaneou(dado):
	global scanmin
	scanmin = 5
	
	ranges = np.array(dado.ranges).round(decimals=2)
	# ranges = ranges[140:210]  # só na frente
	for i in ranges: 
		if i < scanmin and i != 0:
			scanmin = i


class Procurando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['achou', 'procurando', 'vaibater'])
	def execute(self, userdata):
		global velocidade_saida

		if scanmin < distanciaMin:
			return 'vaibater'

		global scanmin
		if media is None or len(media) == 0 or media[0]==0:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'procurando'

		else:
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
		global scanmin

		if scanmin < distanciaMin:
			return 'vaibater'

		if media is None or len(media) == 0 or media[0]==0:
			return 'sumiu'

		if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'alinhando'

		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'alinhando'		

		else:
			return 'andando'


class Andando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando', 'andando', 'sumiu', 'vaibater'])

	def execute(self, userdata):
		global velocidade_saida
		global scanmin

		if scanmin < distanciaMin:
			return 'vaibater'

		if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'

		if media is None or len(media)==0 or media[0]==0:
			return 'sumiu'

		else:
			vel = Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'andando'


class Sobrevivendo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['sobrevivendo', 'procurando'])
	def execute(self, userdata):
		global scanmin

		if scanmin < distanciaMin:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'sobrevivendo'

		else:
			return 'procurando'


# main
def main():
	global velocidade_saida
	rospy.init_node('unico_estado')

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=40, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	# Cria uma máquina de estados
	sm_top = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm_top:
	    smach.StateMachine.add('PROCURANDO', Procurando(),
	    	transitions={'achou': 'MOVENDO','procurando':'PROCURANDO', 'vaibater': 'SOBREVIVENDO'})

	    smach.StateMachine.add('SOBREVIVENDO', Sobrevivendo(),
	    	transitions={'sobrevivendo': 'SOBREVIVENDO','procurando':'PROCURANDO'})

	    smach.StateMachine.add('MOVENDO', Movendo(),
	    	transitions={'alinhando': 'SUB'})

	    sm_sub = smach.StateMachine(outcomes=['procurando', 'vaibater'])

	    with sm_sub:
	    	smach.StateMachine.add('ALINHANDO', Alinhando(),
	    		transitions={'alinhando': 'ALINHANDO', 'andando': 'ANDANDO', 'sumiu': 'procurando', 'vaibater': 'vaibater'})

	    	smach.StateMachine.add('ANDANDO', Andando(),
	    		transitions={'alinhando': 'ALINHANDO', 'andando': 'ANDANDO', 'sumiu': 'procurando', 'vaibater': 'vaibater'})

	    smach.StateMachine.add('SUB', sm_sub, 
	    	transitions={'procurando': 'PROCURANDO', 'vaibater': 'SOBREVIVENDO'})

	# Executa a máquina de estados
	outcome = sm_top.execute()

	print("Execute finished")

if __name__ == '__main__':
	print("Main")
	main()
