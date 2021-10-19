#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os

#Importaciones generales
import time
import math


from tkinter import *
# Explicit imports to satisfy Flake8
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage

#Importamos los servicios
from drone.srv import arm, armResponse, takeoff, takeoffResponse, rot_yaw, rot_yawResponse, vel_lin, vel_linResponse, land, landResponse, navigation_act, navigation_actResponse

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import String

CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))
CURRENT_UPPER_FOLDER = os.path.abspath(os.path.join(CURRENT_FOLDER, os.pardir))

class GUI_node(Tk):

    def __init__(self):

        Tk.__init__(self)

        self.geometry("1208x658")
        self.configure(bg = "#131A27")
        self.title("Navigation Monitor")

        self.canvas = Canvas(
            self,
            bg = "#131A27",
            height = 658,
            width = 1208,
            bd = 0,
            highlightthickness = 0,
            relief = "ridge"
        )

        self.canvas.place(x = 0, y = 0)

        #GUI Deployment
        self.deploymentGUI()

        #Inicializacion de variables
        self.angle = 0
        self.estate_param = "none"
        self.flag = 1 
        self.longitude_now = 0
        self.latitude_now = 0
        self.altitude_now = 0
        self.heading = 0
        self.Vx = 0
        self.state_navigation = "none"
        self.state_connection = "none"

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

        #Cliente para el servicio de activacion del nodo de navegación
        self.wait_srv_navigation_act = rospy.wait_for_service("GUI/srv/activar_navigation")
        self.client_srv_navigation_act= rospy.ServiceProxy("GUI/srv/activar_navigation",navigation_act)  

        #SUSCRIPTORES

        #Suscriptor de la posición actual del drone.
        self.sub_pos_gps = rospy.Subscriber('drone/pos_gps',NavSatFix,self.update_pos_gps)

        #Subscriber at the rate sent from the navigation node
        self.sub_vel_nav = rospy.Subscriber('Nav/vel_lin',TwistStamped,self.update_velocity)

        #Suscriptor del cabeceo actual del drone
        self.sub_orient_angle_z_now = rospy.Subscriber('drone/orient_angle_z_now', PointStamped, self.update_angle_z)

        #Suscriptor del estado de navegación
        self.sub_state_navigation = rospy.Subscriber('Nav/state_navigation', String, self.update_state_navigation)

    def relative_to_assets(self,path):
        path_images = os.path.join(
                CURRENT_FOLDER, "assets", path)
        return path_images

 

    #METODO PARA ACTUALIZAR EL VALOR DE LA COORDENADA ENVIADA DESDE EL NODO DEL DRON
    def update_pos_gps(self,msg):
        self.latitude_now = msg.latitude
        self.longitude_now = msg.longitude
        self.altitude_now = msg.altitude
        
    def update_state_navigation(self,std_string):
        self.state_navigation = std_string.data
        self.state_connection = "NONE"
        #print("ACt")

    def update_angle_z(self,point_ang_z):
        self.angle_now = point_ang_z.point.z

    def update_velocity(self,Vel_nav):
        self.Vx = Vel_nav.twist.linear.x
        self.Vy = Vel_nav.twist.linear.y
        self.Vz = Vel_nav.twist.linear.z
        self.heading = Vel_nav.twist.angular.z

    def update(self):

        self.entry_3.configure(text=self.longitude_now)
        self.entry_4.configure(text=round(self.Vx,3))
        self.entry_5.configure(text=self.angle_now)
        self.entry_8.configure(text=self.latitude_now)
        self.entry_7.configure(text=self.altitude_now)
        self.entry_9.configure(text=self.state_navigation)
        #self.entry_10.configure(text=self.state_connection)

        self.after(1000,self.update)

    ##METODO ALMANCENAR LA INFO
    def run_navigation(self):

        latiude_destino = self.entry_1.get()
        longitude_destino = self.entry_2.get()
        height =self.entry_6.get()
        print("LATITUDE = ",latiude_destino)
        print("LONGITUDE = ", longitude_destino)
        print("HEIGHT = ", height)

        try:
            response_navigation_act = self.client_srv_navigation_act(1)
            rospy.loginfo(response_navigation_act.result)
            self.estate_param = response_navigation_act.result

        except rospy.ServiceException as e:
            print("Falla en el servicio de activación navegación", e)
    
    ##METODO PARA ARMAR EL DRON
    def active_arm(self):

        try:
            response_arm_dron = self.client_srv_arm_drone('Arm')
            rospy.loginfo(response_arm_dron.result)
            self.estate_param = response_arm_dron.result
        except rospy.ServiceException as e:
            print("Falla en el servicio de armado ", e)

    ##METODO PARA REALIZAR EL DESPEGUE
    def active_takeoff(self):
        
        if self.estate_param == "Arm_check":

            try:
                response_take_off = self.client_srv_take_off(2)
                rospy.loginfo(response_take_off.result)
                #estate = response_take_off.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de despegue ", e)
        
        else:
            rospy.loginfo(" -- Primero arme el dron -- ")



    ##METODO PARA REALIZAR EL ATERRIZAJE DEL DRON
    def active_land(self):

        if self.estate_param != "Arm" or self.estate_param != "Land_check":

            try:
                response_land = self.client_srv_land("land")
                rospy.loginfo(response_land.result)
                #estate = response_land.result
            except rospy.ServiceException as e:
                print("Falla en el servicio de aterrizaje ", e)

            self.flag = 1

        else:

            rospy.loginfo(" -- ACCION NO VALIDA -- ")

    ##METODO PARA ENVIAR VELOCIDAD HACIA EL FRENTE DEL DRON

    def active_velocity(self):

        if self.estate_param != "Arm" or self.estate_param != "Land_check":
            
            try:
                response_vel_lin= self.client_srv_vel_lin(2,0,0)
                rospy.loginfo(response_vel_lin.result)
            except rospy.ServiceException as e:
                print("Falla en el servicio de velocidad adelante", e)

            self.flag = 0

        else:

            rospy.loginfo(" -- ACCION NO VALIDA -- ")

    ##METODO PARA ENVIAR VELOCIDAD HACIA ARRIBA

    def active_up(self):

        if self.estate_param != "Arm" or self.estate_param != "Land_check":

            try:
                response_vel_lin= self.client_srv_vel_lin(0,0,-1)
                rospy.loginfo(response_vel_lin.result)
            except rospy.ServiceException as e:
                print("Falla en el servicio de velocidad asceso ", e)

        else:

            rospy.loginfo(" -- ACCION NO VALIDA -- ")

    ##METODO PARA ENVIAR VELOCIDAD HACIA ABAJO 

    def active_down(self):

        if self.estate_param != "Arm" or self.estate_param != "Land_check":

            try:
                response_vel_lin= self.client_srv_vel_lin(0,0,1)
                rospy.loginfo(response_vel_lin.result)
            except rospy.ServiceException as e:
                print("Falla en el servicio de velocidad descenso ", e)

        else:

            rospy.loginfo(" -- ACCION NO VALIDA -- ") 

    ##METODO PARA ENVIAR UNA ROTACION DE 45 GRADOS 

    def active_rotation(self):

        if self.estate_param != "Arm" or self.estate_param != "Land_check":

            if self.flag == 1: #Flag provisional para el error del simulador
                    self.flag = 0
                    try:
                        response_vel_lin= self.client_srv_vel_lin(0.1,0,0)
                        rospy.loginfo(response_vel_lin.result)
                    except rospy.ServiceException as e:
                        print("Falla en el servicio inicio de velocidades ", e)

            self.angle = self.angle + 45

            if self.angle >= 360:
                self.angle = 0

            try:
                response_rotation= self.client_srv_rot_yaw(self.angle)
                rospy.loginfo(response_rotation.result)
            except rospy.ServiceException as e:
                print("Falla en el servicio de rotacion ", e)

        else:

            rospy.loginfo(" -- ACCION NO VALIDA -- ")                


    def deploymentGUI(self):

        self.frame1()
        self.frame2()
        self.logo()
        self.boton_vel()
        self.boton_down()
        self.boton_up()
        self.boton_rotation()
        self.boton_land()
        self.boton_takeoff()
        self.boton_arm()
        self.boton_start()
        self.boton_connection()
        self.boton_title()
        self.enter_latitude()
        self.enter_longitude()
        self.view_longitude()
        self.view_velocity()
        self.view_orientation()
        self.enter_height()
        self.view_height()
        self.view_latitude()
        self.view_navigation_status()
        self.view_connection_status()


    def frame1(self):

        self.button_image_1 = PhotoImage(
            file=self.relative_to_assets("frame1.png"))

        self.button_1 = Button(self,
            image=self.button_image_1,
            borderwidth=0,
            background="#131A27",
            activebackground="#131A27",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_1.place(
            x=21.0,
            y=102.0,
            width=573.0,
            height=521.0
        )

    def frame2(self):
        self.button_image_2 = PhotoImage(
            file=self.relative_to_assets("frame2.png"))

        self.button_2 = Button(self,
            image=self.button_image_2,
            borderwidth=0,
            background="#131A27",
            activebackground="#131A27",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_2.place(
            x=614.0,
            y=102.0,
            width=573.0,
            height=521.0
        )

    def logo(self):
        self.button_image_3 = PhotoImage(
            file=self.relative_to_assets("logo.png"))

        self.button_3 = Button(self,
            image=self.button_image_3,
            borderwidth=0,
            background="#131A27",
            activebackground="#131A27",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_3.place(
            x=946.0,
            y=10.0,
            width=80.0,
            height=80.0
        )
    def boton_vel(self):

        self.button_image_4 = PhotoImage(
            file=self.relative_to_assets("velocity.png"))

        self.button_4 = Button(self,
            image=self.button_image_4,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command= self.active_velocity,
            relief="flat"
        )

        self.button_4.place(
            x=978.086181640625,
            y=548.2587890625,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_down(self):

        self.button_image_5 = PhotoImage(
            file=self.relative_to_assets("down.png"))

        self.button_5 = Button(self,
            image=self.button_image_5,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_down,
            relief="flat"
        )

        self.button_5.place(
            x=847.6317138671875,
            y=548.2587890625,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_up(self):

        self.button_image_6 = PhotoImage(
            file=self.relative_to_assets("up.png"))

        self.button_6 = Button(self,
            image=self.button_image_6,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_up,
            relief="flat"
        )

        self.button_6.place(
            x=717.17724609375,
            y=549.0997314453125,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_rotation(self):

        self.button_image_7 = PhotoImage(
            file=self.relative_to_assets("rotation.png"))

        self.button_7 = Button(self,
            image=self.button_image_7,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_rotation,
            relief="flat"
        )

        self.button_7.place(
            x=1046.4327392578125,
            y=452.0997314453125,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_land(self):

        self.button_image_8 = PhotoImage(
            file=self.relative_to_assets("land.png"))

        self.button_8 = Button(self,
            image=self.button_image_8,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_land,
            relief="flat"
        )

        self.button_8.place(
            x=914.3758544921875,
            y=452.38818359375,
            width=102.75331115722656,
            height=61.00269317626953
        )
    
    def boton_takeoff(self):

        self.button_image_9 = PhotoImage(
            file=self.relative_to_assets("takeoff.png"))

        self.button_9 = Button(self,
            image=self.button_image_9,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_takeoff,
            relief="flat"
        )

        self.button_9.place(
            x=783.92138671875,
            y=452.38818359375,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_arm(self):

        self.button_image_10 = PhotoImage(
            file=self.relative_to_assets("arm.png"))

        self.button_10 = Button(self,
            image=self.button_image_10,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.active_arm,
            relief="flat"
        )

        self.button_10.place(
            x=653.4669189453125,
            y=453.2291259765625,
            width=102.75331115722656,
            height=61.00269317626953
        )

    def boton_start(self):

        self.button_image_11 = PhotoImage(
            file=self.relative_to_assets("start.png"))

        self.button_11 = Button(self,
            image=self.button_image_11,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=self.run_navigation,
            relief="flat"
        )

        self.button_11.place(
            x=123.1387939453125,
            y=547.4177856445312,
            width=120.34172058105469,
            height=61.00269317626953
        )

    def boton_connection(self):

        self.button_image_12 = PhotoImage(
            file=self.relative_to_assets("logouniversity.png"))

        self.button_12 = Button(self,
            image=self.button_image_12,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_12.place(
            x=90.0147705078125,
            y=150.6253662109375,
            width=200,
            height=126
        )
    
    def boton_title(self):

        self.button_image_13 = PhotoImage(
            file=self.relative_to_assets("button_13.png"))

        self.button_13 = Button(self,
            image=self.button_image_13,
            borderwidth=0,
            background="#131A27",
            activebackground="#131A27",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_13.place(
            x=172.0,
            y=20.0,
            width=752.0,
            height=73.0
        )


    def enter_latitude(self):
        
        self.entry_image_1 = PhotoImage(
            file=self.relative_to_assets("entry_1.png"))
        
        self.entry_bg_1 = self.canvas.create_image(
            150.5,
            426.0,
            image=self.entry_image_1
        )

        self.entry_1 = Entry(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
        )

        self.entry_1.place(
            x=74.0,
            y=407.0,
            width=209.0,
            height=36.0
        )

    def enter_longitude(self):

        self.entry_image_2 = PhotoImage(
            file=self.relative_to_assets("entry_2.png"))

        self.entry_bg_2 = self.canvas.create_image(
            435.5,
            426.0,
            image=self.entry_image_2
        )

        self.entry_2 = Entry(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
        )

        self.entry_2.place(
            x=331.0,
            y=407.0,
            width=209.0,
            height=36.0
        )

    def view_longitude(self):

        self.entry_image_3 = PhotoImage(
            file=self.relative_to_assets("entry_3.png"))
            
        self.entry_bg_3 = self.canvas.create_image(
            1020.0,
            273.0,
            image=self.entry_image_3
        )

        self.entry_3 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )

        self.entry_3.place(
            x=916.0,
            y=254.0,
            width=208.0,
            height=36.0
        )

    def view_velocity(self):
    
        self.entry_image_4 = PhotoImage(
            file=self.relative_to_assets("entry_4.png"))

        self.entry_bg_4 = self.canvas.create_image(
            755.0,
            273.0,
            image=self.entry_image_4
        )
        self.entry_4 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )
        self.entry_4.place(
            x=696.0,
            y=254.0,
            width=118.0,
            height=36.0
        )

    def view_orientation(self):

        self.entry_image_5 = PhotoImage(
            file=self.relative_to_assets("entry_5.png"))

        self.entry_bg_5 = self.canvas.create_image(
            755.0,
            206.0,
            image=self.entry_image_5
        )

        self.entry_5 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )

        self.entry_5.place(
            x=696.0,
            y=187.0,
            width=118.0,
            height=36.0
        )

    def enter_height(self):

        self.entry_image_6 = PhotoImage(
            file=self.relative_to_assets("entry_6.png"))

        self.entry_bg_6 = self.canvas.create_image(
            307.0,
            494.0,
            image=self.entry_image_6
        )

        self.entry_6 = Entry(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
        )

        self.entry_6.place(
            x=246.0,
            y=475.0,
            width=122.0,
            height=36.0
        )

    def view_height(self):

        self.entry_image_7 = PhotoImage(
            file=self.relative_to_assets("entry_7.png"))

        self.entry_bg_7 = self.canvas.create_image(
            976.5,
            338.0,
            image=self.entry_image_7
        )

        self.entry_7 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )

        self.entry_7.place(
            x=916.0,
            y=319.0,
            width=121.0,
            height=36.0
        )

    def view_latitude(self):

        self.entry_image_8 = PhotoImage(
            file=self.relative_to_assets("entry_8.png"))

        self.entry_bg_8 = self.canvas.create_image(
            1020.0,
            206.0,
            image=self.entry_image_8
        )

        self.entry_8 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )

        self.entry_8.place(
            x=916.0,
            y=187.0,
            width=208.0,
            height=36.0
        )

    def view_navigation_status(self):
        
        self.entry_image_9 = PhotoImage(
            file=self.relative_to_assets("entry_9.png"))

        self.entry_bg_9 = self.canvas.create_image(
            424.0,
            576.0,
            image=self.entry_image_9
        )

        self.entry_9 = Label(self,
            bd=0,
            bg="#0888B6",
            highlightthickness=0,
            text=""
        )

        self.entry_9.place(
            x=322.0,
            y=557.0,
            width=204.0,
            height=36.0
        )

    def view_connection_status(self):

        self.button_image_14 = PhotoImage(
            file=self.relative_to_assets("center_min.png"))

        self.button_14 = Button(self,
            image=self.button_image_14,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=None,
            relief="sunken"
        )

        self.button_14.place(
            x=295.0,
            y=169,
            width=277,
            height=95
        )

   
    
    

if __name__ == "__main__":
    
    print("Nodo inicializado........")
    rospy.init_node('GUI')
    GUI = GUI_node()
    GUI.resizable(False,False)
    GUI.after(1000, GUI.update)
    GUI.mainloop()

print("Script dead")
