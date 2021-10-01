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

CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))
CURRENT_UPPER_FOLDER = os.path.abspath(os.path.join(CURRENT_FOLDER, os.pardir))

def relative_to_assets(path):
    path_images = os.path.join(
            CURRENT_FOLDER, "assets", path)
    return path_images

window = Tk()

window.geometry("1208x658")
window.configure(bg = "#131A27")


canvas = Canvas(
    window,
    bg = "#131A27",
    height = 658,
    width = 1208,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
button_image_1 = PhotoImage(
    file=relative_to_assets("frame1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    background="#131A27",
    activebackground="#131A27",
    highlightthickness=0,
    command=None,
    relief="sunken"
)
button_1.place(
    x=21.0,
    y=102.0,
    width=573.0,
    height=521.0
)

button_image_2 = PhotoImage(
    file=relative_to_assets("frame2.png"))
button_2 = Button(
    image=button_image_2,
    borderwidth=0,
    background="#131A27",
    activebackground="#131A27",
    highlightthickness=0,
    command=None,
    relief="sunken"
)
button_2.place(
    x=614.0,
    y=102.0,
    width=573.0,
    height=521.0
)

button_image_3 = PhotoImage(
    file=relative_to_assets("logo.png"))
button_3 = Button(
    image=button_image_3,
    borderwidth=0,
    background="#131A27",
    activebackground="#131A27",
    highlightthickness=0,
    command=None,
    relief="sunken"
)
button_3.place(
    x=946.0,
    y=10.0,
    width=80.0,
    height=80.0
)


entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    150.5,
    426.0,
    image=entry_image_1
)
entry_1 = Entry(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_1.place(
    x=74.0,
    y=407.0,
    width=209.0,
    height=36.0
)

entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    435.5,
    426.0,
    image=entry_image_2
)
entry_2 = Entry(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_2.place(
    x=331.0,
    y=407.0,
    width=209.0,
    height=36.0
)

entry_image_3 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_3 = canvas.create_image(
    1020.0,
    273.0,
    image=entry_image_3
)
entry_3 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_3.place(
    x=916.0,
    y=254.0,
    width=208.0,
    height=36.0
)

entry_image_4 = PhotoImage(
    file=relative_to_assets("entry_4.png"))
entry_bg_4 = canvas.create_image(
    755.0,
    273.0,
    image=entry_image_4
)
entry_4 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_4.place(
    x=696.0,
    y=254.0,
    width=118.0,
    height=36.0
)

entry_image_5 = PhotoImage(
    file=relative_to_assets("entry_5.png"))
entry_bg_5 = canvas.create_image(
    755.0,
    206.0,
    image=entry_image_5
)
entry_5 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_5.place(
    x=696.0,
    y=187.0,
    width=118.0,
    height=36.0
)

entry_image_6 = PhotoImage(
    file=relative_to_assets("entry_6.png"))
entry_bg_6 = canvas.create_image(
    307.0,
    494.0,
    image=entry_image_6
)
entry_6 = Entry(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_6.place(
    x=246.0,
    y=475.0,
    width=122.0,
    height=36.0
)

entry_image_7 = PhotoImage(
    file=relative_to_assets("entry_7.png"))
entry_bg_7 = canvas.create_image(
    976.5,
    338.0,
    image=entry_image_7
)
entry_7 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_7.place(
    x=916.0,
    y=319.0,
    width=121.0,
    height=36.0
)

entry_image_8 = PhotoImage(
    file=relative_to_assets("entry_8.png"))
entry_bg_8 = canvas.create_image(
    1020.0,
    206.0,
    image=entry_image_8
)
entry_8 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_8.place(
    x=916.0,
    y=187.0,
    width=208.0,
    height=36.0
)

entry_image_9 = PhotoImage(
    file=relative_to_assets("entry_9.png"))
entry_bg_9 = canvas.create_image(
    424.0,
    576.0,
    image=entry_image_9
)
entry_9 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_9.place(
    x=322.0,
    y=557.0,
    width=204.0,
    height=36.0
)

entry_image_10 = PhotoImage(
    file=relative_to_assets("entry_10.png"))
entry_bg_10 = canvas.create_image(
    423.5,
    221.0,
    image=entry_image_10
)
entry_10 = Text(
    bd=0,
    bg="#0888B6",
    highlightthickness=0
)
entry_10.place(
    x=327.0,
    y=202.0,
    width=193.0,
    height=36.0
)

button_image_4 = PhotoImage(
    file=relative_to_assets("velocity.png"))
button_4 = Button(
    image=button_image_4,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command= None,
    relief="flat"
)
button_4.place(
    x=978.086181640625,
    y=548.2587890625,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_5 = PhotoImage(
    file=relative_to_assets("down.png"))
button_5 = Button(
    image=button_image_5,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_5.place(
    x=847.6317138671875,
    y=548.2587890625,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_6 = PhotoImage(
    file=relative_to_assets("up.png"))
button_6 = Button(
    image=button_image_6,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_6.place(
    x=717.17724609375,
    y=549.0997314453125,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_7 = PhotoImage(
    file=relative_to_assets("rotation.png"))
button_7 = Button(
    image=button_image_7,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_7.place(
    x=1046.4327392578125,
    y=452.0997314453125,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_8 = PhotoImage(
    file=relative_to_assets("land.png"))
button_8 = Button(
    image=button_image_8,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_8.place(
    x=914.3758544921875,
    y=452.38818359375,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_9 = PhotoImage(
    file=relative_to_assets("takeoff.png"))
button_9 = Button(
    image=button_image_9,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_9.place(
    x=783.92138671875,
    y=452.38818359375,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_10 = PhotoImage(
    file=relative_to_assets("arm.png"))
button_10 = Button(
    image=button_image_10,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_10.place(
    x=653.4669189453125,
    y=453.2291259765625,
    width=102.75331115722656,
    height=61.00269317626953
)

button_image_11 = PhotoImage(
    file=relative_to_assets("start.png"))
button_11 = Button(
    image=button_image_11,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_11.place(
    x=123.1387939453125,
    y=547.4177856445312,
    width=120.34172058105469,
    height=61.00269317626953
)


button_image_12 = PhotoImage(
    file=relative_to_assets("connection.png"))
button_12 = Button(
    image=button_image_12,
    borderwidth=0,
    background="#0E4763",
    activebackground="#0E4763",
    highlightthickness=0,
    command=None,
    relief="flat"
)
button_12.place(
    x=112.0147705078125,
    y=180.6253662109375,
    width=120.34172058105469,
    height=77.36927032470703
)

button_image_13 = PhotoImage(
    file=relative_to_assets("button_13.png"))
button_13 = Button(
    image=button_image_13,
    borderwidth=0,
    background="#131A27",
    activebackground="#131A27",
    highlightthickness=0,
    command=None,
    relief="sunken"
)
button_13.place(
    x=172.0,
    y=20.0,
    width=752.0,
    height=73.0
)



window.resizable(False, False)
window.mainloop()
