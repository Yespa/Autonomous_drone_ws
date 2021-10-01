#!/usr/bin/env python
# -*- coding: utf-8 -*-

#from os import name
import rospy


#Importaciones generales
import time
import math

#Importaciones vision artificial
import cv2
import pyrealsense2 as rs
import numpy as np


from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from std_msgs.msg import Float64MultiArray


#Importamos los servicios
from drone.srv import arm, armResponse, takeoff, takeoffResponse, rot_yaw, rot_yawResponse, vel_lin, vel_linResponse, land, landResponse

class Node_navegation_drone:

    def __init__(self):
        
        #Variables de inicializacion 
        self.longitude_now = 0
        self.latitude_now = 0
        self.altitude_now = 0
        self.heading = 0
        self.vel_lin_x = 0
        self.vel_lin_y = 0
        self.vel_lin_z = 0
        
        #Constates de control
        self.Kp = 0.77
        self.Ki = 0
        self.Kd = 0.01
        self.Vx = 0

        self.Kp_avoidVy = 0.9
        self.Ki_avoidVy = 0
        self.Kd_avoidVy = 0.08
        self.Vy = 0

        self.Kp_avoidVz = 0.4
        self.Ki_avoidVz = 0
        self.Kd_avoidVz = 0.08
        self.Vz = 0

        self.Ts = 0.5 #Tiempo de muestreo

        #Inicializacion de variables para el controlador
        self.Acum = 0
        self.Dist_old = 0

        self.Acum_avoid = 0
        self.error_old = 0
        
        self.time_old = 0     

        self.rate = rospy.Rate(10)
        
        
        #CLIENT SERVICES

        #Cliente del servicio de armado
        #Esperamos que el servicio este activo y asignamos el cliente a un objeto
        self.wait_srv_arm_drone = rospy.wait_for_service("drone/srv/arm")
        self.client_srv_arm_drone = rospy.ServiceProxy("drone/srv/arm",arm)

        #Cliente del servicio de despegue
        self.wait_srv_take_off = rospy.wait_for_service("drone/srv/take_off")
        self.client_srv_take_off = rospy.ServiceProxy("drone/srv/take_off",takeoff)

        #Cliente del servicio de rotacion
        self.wait_srv_rot_yaw = rospy.wait_for_service("drone/srv/rot_yaw")
        self.client_srv_rot_yaw= rospy.ServiceProxy("drone/srv/rot_yaw",rot_yaw)

        #Cliente para el servicio de velocidad lineal   
        self.wait_srv_vel_lin = rospy.wait_for_service("drone/srv/vel_lin")
        self.client_srv_vel_lin= rospy.ServiceProxy("drone/srv/vel_lin",vel_lin)

        #Cliente para el servicio de aterrizaje
        self.wait_srv_land = rospy.wait_for_service("drone/srv/land")
        self.client_srv_land= rospy.ServiceProxy("drone/srv/land",land) 

        #SUSCRIPTORES

        #Suscriptor de la posición actual del drone.
        self.sub_pos_gps = rospy.Subscriber('drone/pos_gps',NavSatFix,self.update_pos_gps)

        #Suscriptor del cabeceo actual del drone
        self.sub_orient_angle_z_now = rospy.Subscriber('drone/orient_angle_z_now', PointStamped, self.update_angle_z)

        #Suscriptor a distancias medidas por la depth camera
        self.sub_depth_distances = rospy.Subscriber('depth_camera/distances', Float64MultiArray, self.update_depth_distances)

        #PUBLICADORES

        #Publicador de la velocidad calculado para el dron
        self.pub_vel_nav = rospy.Publisher('Nav/vel_lin',TwistStamped,queue_size=10)

    #Metodo para publicar la velocidad del dron
    def publish_velocity(self):

        #Se solicita la información a la pixhawk
        # --> Lleno los vectores de velocidad lineales y angulares
        
        vel_drone = TwistStamped()

        vel_drone.header.stamp = rospy.Time.now()
        vel_drone.header.frame_id = "Nav/Velocidades"
        vel_drone.twist.linear.x = self.vel_lin_x
        vel_drone.twist.linear.y = self.vel_lin_y
        vel_drone.twist.linear.z = self.vel_lin_z
        vel_drone.twist.angular.x = 0
        vel_drone.twist.angular.y = 0
        vel_drone.twist.angular.z = self.heading

        self.pub_vel_nav.publish(vel_drone)

    #METODO PARA ACTUALIZAR EL VALOR DE LA COORDENADA ENVIADA DESDE EL NODO DEL DRON
    def update_pos_gps(self,msg):
        self.latitude_now = msg.latitude
        self.longitude_now = msg.longitude
        self.altitude_now = msg.altitude

    def update_angle_z(self,point_ang_z):
        self.angle_now = point_ang_z.point.z

    #METODO PARA ACTUALIZAR LAS DISTANCIAS MEDIDAS POR LA CAMARA DE PROFUNDIDAD
    def update_depth_distances(self,array_distances):

        self.d1 = array_distances.data[0]
        self.d2 = array_distances.data[1]
        self.d3 = array_distances.data[2]
        self.d4 = array_distances.data[3]
        self.d5 = array_distances.data[4]
        self.d6 = array_distances.data[5]
        self.d7 = array_distances.data[6]
        self.d8 = array_distances.data[7]
        self.d9 = array_distances.data[8]

    #METODO PARA PARA IR A LA COORDENADA INGRESADA.
    def goto(self):
        
        tol = 0

        self.latitude_destino = -35.3530008* 10000
        self.longitude_destino = 149.1650351* 10000

        self.dist_latitude = (self.latitude_destino - self.latitude_now*10000)  #Distancia variable a recorrer en latitud
        self.dist_longitude = (self.longitude_destino - self.longitude_now*10000) #Distancia variabale a recorrer en longitud
        self.dist_recorrer = math.sqrt(((self.dist_latitude)**2)+((self.dist_longitude)**2)) #Distancia más corta entre la latitud y longitud variable

        self.ang_rotacion = int(math.degrees(math.atan2(self.dist_longitude,self.dist_latitude))) #Angulo del punto de destino

        #Convertir los angulos negativos a positivos
        if self.ang_rotacion < 0:
            self.ang_rotacion = self.ang_rotacion + 360
    

        #Tolerancia entre los datos que estan entre 357 a 3 grados
        if self.ang_rotacion in range(357,361) or self.ang_rotacion in range(0,4):

            Ang_problem = [357,358,359,360,0,1,2,3]

            if self.ang_rotacion in Ang_problem:

                pos = Ang_problem.index(self.ang_rotacion)

                infpos = pos - 3

                if infpos < 0:
                    infpos = 0

                suppos = pos + 3

                if suppos > len(Ang_problem):
                    suppos = len(Ang_problem)


                for i in range(infpos,suppos):
                    if int(self.angle_now) == Ang_problem[i]:

                        tol = 1
                        break
                    else:
                        tol = 0


        if int(self.ang_rotacion) in range((int(self.angle_now) - 3),(int(self.angle_now) + 3)) or tol==1: #Condicion para enviar velocidades cuando estemos en la orientacion deseada

            if time.time() - self.time_old >= self.Ts: #Controlamos el periodo de muestreo

                Term_proporcional = self.dist_recorrer
                Term_integrativo = ((self.dist_recorrer*self.Ts)-self.Acum)
                Term_derivativo = ((self.dist_recorrer - self.Dist_old)/self.Ts)
                
                #Ecuacion de controlador PID
                self.Vx = self.Kp*Term_proporcional + self.Ki*Term_integrativo + self.Kd*Term_derivativo

                #Variables T(k-1)
                self.Acum = self.Acum + (self.dist_recorrer*self.Ts)
                self.Dist_old = self.dist_recorrer
                self.time_old = time.time()
            
            if self.Vx>3: #Evito un sobre esfuerzo
                self.vel_lin_x = 3
            elif self.Vx<0: #Evito valores negativos
                self.vel_lin_x = 0
            else: 
                self.vel_lin_x = self.Vx
            self.vel_lin_y = 0
            self.vel_lin_z = 0
            self.publish_velocity()
        
        else: 
            self.vel_lin_x = 0
            self.vel_lin_y = 0
            self.vel_lin_z = 0
            self.heading = (self.ang_rotacion)

            self.publish_velocity()


    #METODO PARA EVADIR OBSTACULOS.
    def AvoidObstacle(self):
              
        self.setpoint = (144000000)
        self.dist_med = (self.d1**2+self.d2**2+self.d3**2+self.d4**2+self.d5**2+self.d6**2+self.d7**2+self.d8**2+self.d9**2)

        self.error_avoid = self.setpoint - self.dist_med
        print(self.error_avoid)
        
        self.center = self.d2**2 + self.d5**2 + self.d7**2

        self.lateral_izquierdo = self.d1**2 + self.d4**2 + self.d6**2 + self.center

        self.lateral_derecho = self.d3**2 + self.d6**2 + self.d9**2 + self.center

        self.franja_superior = self.d1**2 + self.d2**2 + self.d3**2 + self.center

        self.franja_inferior = self.d7**2 + self.d8**2 + self.d9**2 + self.center

        
        self.Orient_Vy = self.lateral_derecho - self.lateral_izquierdo

        self.Orient_Vz = self.franja_inferior - self.franja_superior
        
        #VELOCIDAD EVASION EN Y

        if time.time() - self.time_old >= self.Ts: #Controlamos el periodo de muestreo

            #CONTROLADOR VY
            Term_proporcional = self.error_avoid
            Term_integrativo = ((self.error_avoid*self.Ts)-self.Acum_avoid)
            Term_derivativo = ((self.error_avoid - self.error_old)/self.Ts)
            
            #Ecuacion de controlador PID
            self.Vy = self.Kp_avoidVy*Term_proporcional + self.Ki_avoidVy*Term_integrativo + self.Kd_avoidVy*Term_derivativo
            
            self.Vy = self.Vy/72001130

            if self.Orient_Vy < 0:
                self.Vy = -self.Vy
                

            #Ecuacion de controlador PID
            self.Vz = self.Kp_avoidVz*Term_proporcional + self.Ki_avoidVz*Term_integrativo + self.Kd_avoidVz*Term_derivativo
            
            self.Vz = self.Vz*(0.8/72001130)

            if self.Orient_Vz < 0:
                self.Vz = -self.Vz
            
            #Variables T(k-1)
            self.Acum_avoid = self.Acum_avoid + (self.error_avoid*self.Ts)
            self.error_old = self.error_avoid

            self.time_old = time.time()

        if self.Vy>1: #Evito un sobre esfuerzo
            self.vel_lin_y = 1
        elif self.Vy<-1: #Evito valores negativos
            self.vel_lin_y = -1
        else: 
            self.vel_lin_y = self.Vy

        
        if self.Vz>0: #Evito un sobre esfuerzo
            self.vel_lin_z = 0
        elif self.Vz<-0.8: #Evito valores negativos
            self.vel_lin_z = -0.8
        else: 
            self.vel_lin_z = self.Vz

        if self.vel_lin_x > 1:
            self.vel_lin_x =1

              
        self.publish_velocity()

                                 
    #METODO PARA RESETEAR VARIABLES
    def reset(self):

        self.vel_lin_x = 0
        
        self.publish_velocity()

        #Inicializacion de variables para el controlador
        self.Acum = 0
        self.Dist_old = 0
        self.time_old = 0    

    #METODO PARA RESETEAR VARIABLES
    def reset_controlers(self):

        #Inicializacion de variables para el controlador
        self.Acum = 0
        self.Dist_old = 0

        self.Acum_avoid = 0
        self.error_old = 0

        self.time_old = 0   

def main():

    print("Nodo inicializado........")
    rospy.init_node('Navegacion')

    #Periodo de muestreo
    rospy.Rate(25)

    Navegacion = Node_navegation_drone()

    estate = "inicio"

    flag = 1

    time.sleep(2)

    print(" ")
    print("-----Maquina de estados iniciada------")

    while not rospy.is_shutdown():


        limite = 1000
        if Navegacion.d1 < limite or Navegacion.d2 < limite or Navegacion.d3 < limite or Navegacion.d4 < limite or Navegacion.d5 < limite or Navegacion.d6 < limite or Navegacion.d7 < limite or Navegacion.d8 < limite or Navegacion.d9 < limite:
            DetectObstacle = True
        else:
            DetectObstacle = False
            

        ## Finite State Machine

        if estate == "inicio":

            print("Ingrese la latitud de destino")
            Navegacion.latitude_destino = input()*10000
            print("Ingrese la longitud de destino")
            Navegacion.longitude_destino = input()*10000
            
            try:
                response_arm_dron = Navegacion.client_srv_arm_drone('Arm')
                rospy.loginfo(response_arm_dron.result)
                estate = response_arm_dron.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de armado ", e)

        elif estate == "Arm_check" :
           
            try:
                response_take_off = Navegacion.client_srv_take_off(1)
                rospy.loginfo(response_take_off.result)
                estate = response_take_off.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de despegue ", e)


        elif estate == "Alt_check":

            if flag == 1: #Flag provisional para el error del simulador"
                flag = 0
                try:
                    response_vel_lin= Navegacion.client_srv_vel_lin(0.1,0,0)
                    rospy.loginfo(response_vel_lin.result)
                except rospy.ServiceException as e:
                    print("Falla en el servicio inicio de velocidades ", e) 

            if DetectObstacle==False:
                
                Navegacion.goto()


                rospy.sleep(1)

                if Navegacion.dist_recorrer < 0.1:
                    Navegacion.reset()
                    estate = "Pos_check"

            elif DetectObstacle:
                Navegacion.reset_controlers()
                print("EVADIR EVADIR")
                Navegacion.AvoidObstacle()


            
        elif estate == "Pos_check":

            try:
                response_land = Navegacion.client_srv_land("land")
                rospy.loginfo(response_land.result)
                estate = response_land.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de aterrizaje ", e) 

        elif estate == "Land_check":
            print("Se realizará un nuevo vuelo? S o N")
            respuesta = input()

            if respuesta == "S":
                state = "inicio"
            elif respuesta == "N":
                print("FINAL")


if __name__ == "__main__":
    main()

print("Script dead")