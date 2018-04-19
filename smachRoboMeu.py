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
from sensor_msgs.msg import Imu
import transformations
import smach
import smach_ros

import Cor_Rosa
import Dragonite

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0

media2 = []
centro2 = []
area2 = 0.0



tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.25
turn_speed = 0.5
escape_speed = -0.2
walking_speed = -0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000
contador = 0

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

surf = cv2.xfeatures2d.SURF_create(hessianThreshold=5000)
dragonite=cv2.imread("dragonite.jpg")
kp1, des1 = surf.detectAndCompute(dragonite,None)

scanmin = 5
distanciaMin = 0.20
sleeptime = 0.3

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area
	global contador
	global media2
	global centro2
	global area2
	
	

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = Cor_Rosa.identifica_cor(cv_image)
		if contador%5==0:
			frame_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
			frame_gray=cv2.medianBlur(frame_gray,5)
			media2, centro2, area2= Dragonite.detect_features(dragonite,kp1, des1,cv_image,frame_gray)

		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
		
	contador+=1

def scaneou(dado):
	global scanmin
	scanmin = 5
	global angle_min
	angle_min = 0
	
	ranges = np.array(dado.ranges).round(decimals=2)
	# ranges = ranges[140:210]  # só na frente
	for i in range(len(ranges)-1): 
		if ranges[i] < scanmin and ranges[i] != 0:
			scanmin = ranges[i]
			angle_min =  i

def leu_imu(dado):
	global linearAcelX

	linearAcelX = dado.linear_acceleration.x + 0.75
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	

class Procurando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['achou', 'procurando', 'vaibater', 'brincando'])
	def execute(self, userdata):
		global velocidade_saida

		if scanmin < distanciaMin:
			return 'vaibater'

		if media2 is not None and len(media2) > 0 and media2[0] > 0:
			if math.fabs(media2[0]) < math.fabs(centro2[0] + tolerancia_x) and math.fabs(media2[0]) > math.fabs(centro2[0] - tolerancia_x):
				return 'brincando'

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
		global linearAcelX
		global area

		if scanmin < distanciaMin:
			return 'vaibater'

		if linearAcelX < -2:
			vel = Twist(Vector3(-escape_speed,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)

			rospy.sleep(0.5)

			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)

			rospy.sleep(1.5)

		if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'

		if media is None or len(media)==0 or media[0]==0:
			return 'sumiu'

		else:
			#print(math.sqrt(area))
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			vel = Twist(Vector3(-1.2*(1-(math.sqrt(area)/450)), 0, 0), Vector3(0, 0, 0))
			print("/////////////////?????????",-1.2*(1-(math.sqrt(area)/450)))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'andando'


class Sobrevivendo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['sobrevivendo', 'procurando'])
	def execute(self, userdata):
		global scanmin

		if scanmin < distanciaMin:
			if angle_min > 0 and angle_min <= 90:
				vel = Twist(Vector3(-escape_speed, 0, 0), Vector3(0, 0, (math.tan(math.radians(angle_min)))*turn_speed))
			elif angle_min > 90 and angle_min <= 180:
				vel = Twist(Vector3(+escape_speed, 0, 0), Vector3(0, 0, (math.tan(math.radians(angle_min)))*turn_speed))
			elif angle_min > 180 and angle_min <= 270 :
				vel = Twist(Vector3(+escape_speed, 0, 0), Vector3(0, 0, (math.tan(math.radians(angle_min)))*turn_speed))
			else:
				vel = Twist(Vector3(-escape_speed, 0, 0), Vector3(0, 0, (math.tan(math.radians(angle_min)))*turn_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(sleeptime)
			return 'sobrevivendo'

		else:
			return 'procurando'

class Brincando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['procurando'])
	def execute(self, userdata):
		vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		rospy.sleep(0.2)
		vel = Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		rospy.sleep(0.2)
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
		velocidade_saida.publish(vel)
		rospy.sleep(0.2)
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))
		velocidade_saida.publish(vel)
		rospy.sleep(0.2)
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		return 'procurando'

# main
def main():
	global velocidade_saida
	rospy.init_node('unico_estado')

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=40, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	recebe_scan1 = rospy.Subscriber("/scan", LaserScan, scaneou)

	recebe_scan2 = rospy.Subscriber("/imu", Imu, leu_imu,queue_size=1)

	# Cria uma máquina de estados
	sm_top = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm_top:
		smach.StateMachine.add('PROCURANDO', Procurando(),
			transitions={'achou': 'MOVENDO','procurando':'PROCURANDO', 'vaibater': 'SOBREVIVENDO', 'brincando': 'BRINCANDO'})

		smach.StateMachine.add('SOBREVIVENDO', Sobrevivendo(),
			transitions={'sobrevivendo': 'SOBREVIVENDO','procurando':'PROCURANDO'})

		smach.StateMachine.add('BRINCANDO', Brincando(),
			transitions={'procurando': 'PROCURANDO'})

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
