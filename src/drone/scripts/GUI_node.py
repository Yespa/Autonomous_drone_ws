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
from drone.srv import arm, armResponse, takeoff, takeoffResponse, rot_yaw, rot_yawResponse, vel_lin, vel_linResponse, land, landResponse

CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))
CURRENT_UPPER_FOLDER = os.path.abspath(os.path.join(CURRENT_FOLDER, os.pardir))

class GUI_node(Tk):

    def __init__(self):

        Tk.__init__(self)

        self.geometry("1208x658")
        self.configure(bg = "#131A27")
        self.title("GUI BASICA")

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



    def relative_to_assets(self,path):
        path_images = os.path.join(
                CURRENT_FOLDER, "assets", path)
        return path_images

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
            command= None,
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
            command=None,
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
            command=None,
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
            command=None,
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
            command=None,
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
            command=None,
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
            command=None,
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
            command=None,
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
            file=self.relative_to_assets("connection.png"))

        self.button_12 = Button(self,
            image=self.button_image_12,
            borderwidth=0,
            background="#0E4763",
            activebackground="#0E4763",
            highlightthickness=0,
            command=None,
            relief="flat"
        )

        self.button_12.place(
            x=112.0147705078125,
            y=180.6253662109375,
            width=120.34172058105469,
            height=77.36927032470703
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

        self.entry_3 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
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
        self.entry_4 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
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

        self.entry_5 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
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

        self.entry_7 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
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

        self.entry_8 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
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

        self.entry_9 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
        )

        self.entry_9.place(
            x=322.0,
            y=557.0,
            width=204.0,
            height=36.0
        )

    def view_connection_status(self):


        self.entry_image_10 = PhotoImage(
            file=self.relative_to_assets("entry_10.png"))

        self.entry_bg_10 = self.canvas.create_image(
            423.5,
            221.0,
            image=self.entry_image_10
        )

        self.entry_10 = Text(
            bd=0,
            bg="#0888B6",
            highlightthickness=0
        )

        self.entry_10.place(
            x=327.0,
            y=202.0,
            width=193.0,
            height=36.0
        )

if __name__ == "__main__":
    
    print("Nodo inicializado........")
    rospy.init_node('GUI')
    GUI = GUI_node()
    GUI.resizable(False,False)
    GUI.mainloop()

print("Script dead")
