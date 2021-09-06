#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import name
import rospy


#Importaciones generales
import time

import math


from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from std_msgs.msg import String


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
        
        #Constates de control
        self.Kp = 0.77
        self.Ki = 0
        self.Kd = 0.01
        self.Ts = 0.5 #Tiempo de muestreo
        self.Vx = 0

        #Inicializacion de variables para el controlador
        self.Acum = 0
        self.Dist_old = 0
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
        vel_drone.twist.linear.y = 0
        vel_drone.twist.linear.z = 0
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


    #METODO PARA PARA IR A LA COORDENADA INGRESADA.
    def goto(self):

        self.latitude_destino = -35.3628609* 10000
        self.longitude_destino = 149.1655353* 10000

        self.dist_latitude = (self.latitude_destino - self.latitude_now*10000)  #Distancia variable a recorrer en latitud
        self.dist_longitude = (self.longitude_destino - self.longitude_now*10000) #Distancia variabale a recorrer en longitud
        self.dist_recorrer = math.sqrt(((self.dist_latitude)**2)+((self.dist_longitude)**2)) #Distancia más corta entre la latitud y longitud variable

        self.ang_rotacion = math.degrees(math.atan2(self.dist_longitude,self.dist_latitude)) #Angulo del punto de destino

        #Convertir los angulos negativos a positivos
        if self.ang_rotacion < 0:
            self.ang_rotacion = self.ang_rotacion + 360
    
        
        if int(self.ang_rotacion) in range((int(self.angle_now) - 3),(int(self.angle_now) + 3)): #Condicion para enviar velocidades cuando estemos en la orientacion deseada

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
            
            if self.Vx>2: #Evito un sobre esfuerzo
                self.vel_lin_x = 2
            elif self.Vx<0: #Evito valores negativos
                self.vel_lin_x = 0
            else: 
                self.vel_lin_x = self.Vx
            
            self.publish_velocity()
        
        else: 
            self.vel_lin_x = 0
            self.heading = int(self.ang_rotacion)

            self.publish_velocity()

                    
                   

def main():

    print("Nodo inicializado........")
    rospy.init_node('Navegacion')

    #Periodo de muestreo
    rospy.Rate(10)

    Navegacion = Node_navegation_drone()

    estate = "inicio"

    time.sleep(5)

    print(" ")
    print("-----Maquina de estados iniciada------")

    while not rospy.is_shutdown():
        
        if estate == "inicio":

            try:
                response_arm_dron = Navegacion.client_srv_arm_drone('Arm')
                rospy.loginfo(response_arm_dron.result)
                estate = response_arm_dron.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de armado ", e)

        elif estate == "Arm_check" :
           
            try:
                response_take_off = Navegacion.client_srv_take_off(10)
                rospy.loginfo(response_take_off.result)
                estate = response_take_off.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de despegue ", e)


        elif estate == "Alt_check":
            Navegacion.goto()
            
            if Navegacion.dist_recorrer < 0.1:
                estate = "Pos_check"
            
        elif estate == "Pos_check":

            try:
                response_land = Navegacion.client_srv_land("land")
                rospy.loginfo(response_land.result)
                estate = response_land.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de aterrizaje ", e) 

        elif estate == "Land_check":
            time.sleep(5)
            print("-----HE LLEGADO------") ## POR AHORA -- Cambiar




if __name__ == "__main__":
    main()

print("Script dead")