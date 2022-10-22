#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf2_ros 
import time
from sensor_msgs.msg import LaserScan
import math
print("Se cargaron las librerias")
import os

###################################  Coordenadas ##################################
def coord_pro(msg2):
    global objetivox
    global objetivoy
    objetivox=msg2.pose.position.x
    objetivoy=msg2.pose.position.y

#############################################################
def callback_scan(msg): 

    global obstacle_detected1
    global obstacle_detected2
    global obstacle_detected3
    global rango1#derecha del robot
    global rango2#izquierda del robot
    global rango3#frente del robot
    rango1=msg.ranges[90]
    rango2=msg.ranges[629]
    
    n= int((msg.angle_max-msg.angle_min)/msg.angle_increment/2)
    rango3=msg.ranges[n]
    obstacle_detected1 = rango1<2
    obstacle_detected2 = rango2<2
    obstacle_detected3 = rango3<2
    return

#####################################################################

def get_coords():
    got_transform= False
    
    while not got_transform:
        try:
            trans=tfBuffer.lookup_transform('map','base_link',rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        
############################### Funciones de movimiento   ##############################


def mov_derecha(msg_cmd_vel):
    while (not obstacle_detected1) and not rospy.is_shutdown():


        if obstacle_detected1 :
            print("un objeto se detecto, para")
            msg_cmd_vel.linear.y=0
            msg_cmd_vel.linear.x=0
            pub_cmd_vel.publish(msg_cmd_vel)
        else:
            print("no hay objetos cercanos, sigue") 
            msg_cmd_vel.linear.y=-0.1
            msg_cmd_vel.linear.x=0
            pub_cmd_vel.publish(msg_cmd_vel)
        global objetivo
    return
    
def mov_izquierda(msg_cmd_vel):
    while (not obstacle_detected2) and not rospy.is_shutdown():


        if obstacle_detected2 :
            print("un objeto se detecto, para")
            msg_cmd_vel.linear.y=0
            msg_cmd_vel.linear.x=0
            pub_cmd_vel.publish(msg_cmd_vel)
        else:
            print("no hay objetos cercanos, sigue") 
            msg_cmd_vel.linear.y=0.1
            msg_cmd_vel.linear.x=0
            pub_cmd_vel.publish(msg_cmd_vel)
        global objetivo
    return

def mov_frente(msg_cmd_vel):
    while (not obstacle_detected3) and not rospy.is_shutdown():


        if obstacle_detected3 :
            print("un objeto se detecto, para")
            msg_cmd_vel.linear.y=0
            msg_cmd_vel.linear.x=0
            pub_cmd_vel.publish(msg_cmd_vel)
        else:
            print("no hay objetos cercanos, sigue") 
            msg_cmd_vel.linear.y=0
            msg_cmd_vel.linear.x=0.1
            pub_cmd_vel.publish(msg_cmd_vel)
        global objetivo
    return

def girar(msg_cmd_vel):

    inicio_giro=rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec()-inicio_giro<5.235) and not rospy.is_shutdown(): #20.943 completo, 1/4 5.235
        msg_cmd_vel.angular.z=-0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
    return

def girar_completo(msg_cmd_vel):

    inicio_giro=rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec()-inicio_giro<20.943) and not rospy.is_shutdown(): #20.943 completo, 1/4 5.235
        msg_cmd_vel.angular.z=-0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
    return


def objetivo(goal, ox, oy):
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = ox
    goal.pose.position.y = oy
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0
    rospy.sleep(1)
    goal_publisher.publish(goal)
    

######################################## Inicio ########################################3
rospy.init_node('obtener_coordenadas')
rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
rospy.Subscriber("/meta_competencia", PoseStamped,coord_pro)
pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
loop = rospy.Rate(10)

#######################################################################
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
coords_act=get_coords()
print("Las coordenadas actuales en x son: ", coords_act.transform.translation.x) 
print("Las coordenadas actuales en y son: ", coords_act.transform.translation.y)
################################################################################# 

start_ejecution=rospy.Time.now().to_sec() #Tiempo en el que inicia el programa

#############################################################################
obstacle_detected1=0
obstacle_detected2=0
obstacle_detected3=0
########################### Mensaje tipo Twist ##########################################
msg_cmd_vel=Twist()
rospy.sleep(1)



############################### Creando publicador #######################################

goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
goal = PoseStamped()

ox=0
oy=0

#################################### Dimensiones del lugar ###################################

##Giro de 1/4
girar(msg_cmd_vel)

print(rango1)
print(rango2)
print(rango3)

#Definicion de dimensiones
alto=rango1+rango2
ancho=rango3*2

#print(alto)
#print(ancho)

################################### Puntos de escaneo ########################################
"""
Nuestro criterio se basa en que cualquier espacio esta conformado por rectangulos o cuadrados a pesar de las dimensiones
por lo que si el espacio es cuadrado podemos situar el robot en los centroides de las esquinas del cuadrado de tal manera que si
se recorren esas esquinas, como consecuencia se habra escaneado gran parte del lugar. Una vez que robot llega a la esquina deseada debe de girar 360 grados para escanear todo el espacio. Las dimensiones del lugar se obtienen a traves de los sensores lidar del robot justo antes de comenzar su recorrido. Nota los centroides se pueden definir por el criterio que se desee
"""
puntox1=ancho/2
puntoy1=alto/3

#objetivo(goal,oy,ox)
#punto 1 de escaneo
objetivo(goal,puntox1,-1*puntoy1)
rospy.sleep(4*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))
girar_completo(msg_cmd_vel)

#punto 2 de escaneo
objetivo(goal,-1*puntox1,puntoy1)
rospy.sleep(8*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))
girar_completo(msg_cmd_vel)

#punto3 de escaneo
objetivo(goal,puntox1,puntoy1)
rospy.sleep(4*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))
girar_completo(msg_cmd_vel)

objetivo(goal,0,puntoy1)
rospy.sleep(4*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))

#punto4
objetivo(goal,-1*puntox1,-1*puntoy1)
rospy.sleep(4*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))
girar_completo(msg_cmd_vel)

#punto de inicio
objetivo(goal,0,0)
rospy.sleep(4*(int(math.sqrt((ancho*ancho)+(alto*alto)))/2))
################################################################################################
print("El tiempo de ejecucion del programa es: ",rospy.Time.now().to_sec()-start_ejecution) 

os.system('rosrun map_server map_saver -f mapa51')

   



#if name == "main":
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass
