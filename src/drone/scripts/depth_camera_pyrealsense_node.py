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

from std_msgs.msg import Float64MultiArray

class Node_depth_camera_pyrealsense:

    def __init__(self):

        #PUBLICADORES

        #Publicador de las distancias medidas por la camara
        self.pub_depth_distances = rospy.Publisher('depth_camera/distances',Float64MultiArray,queue_size=10)

    #Metodo para publicar la distancias de los cuadrantes
    def publish_distances(self,d1,d2,d3,d4,d5,d6,d7,d8,d9):
        
        array_distances = Float64MultiArray()

        array_distances.data = [d1,d2,d3,d4,d5,d6,d7,d8,d9]

        self.pub_depth_distances.publish(array_distances)
                               

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # puede quitar


        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()

        depth_image = np.asanyarray(depth_frame.get_data())

        if not depth_frame:
            return False, None, None
        return True, depth_image

    def release(self):
        self.pipeline.stop()

dc = DepthCamera()

def main():

    print("Nodo inicializado........")
    rospy.init_node('depth_camera_pyrealsense')

    #Periodo de muestreo
    rate = rospy.Rate(10)

    depth_camera = Node_depth_camera_pyrealsense()


    # Initialize Camera Intel Realsense

    sum_zone1 = 0;sum_zone2 = 0;sum_zone3 = 0;sum_zone4 = 0
    sum_zone5 = 0;sum_zone6 = 0;sum_zone7 = 0;sum_zone8 = 0
    sum_zone9 = 0


    while not rospy.is_shutdown():


        ret, depth_frame= dc.get_frame()
       
        for i in iter(range(213)):
            for j in iter(range(160)):
                point = (i, j)
                sum_zone1 =  sum_zone1 + depth_frame[point[1], point[0]]

        for i in iter(range(213,426)):
            for j in iter(range(160)):
                point = (i, j)
                sum_zone2 =  sum_zone2 + depth_frame[point[1], point[0]]

        for i in iter(range(426,640)):
            for j in iter(range(160)):
                point = (i, j)
                sum_zone3 =  sum_zone3 + depth_frame[point[1], point[0]]

        for i in iter(range(213)):
            for j in iter(range(160,320)):
                point = (i, j)
                sum_zone4 =  sum_zone4 + depth_frame[point[1], point[0]]

        for i in iter(range(213,426)):
            for j in iter(range(160,320)):
                point = (i, j)
                sum_zone5 =  sum_zone5 + depth_frame[point[1], point[0]]

        for i in iter(range(426,640)):
            for j in iter(range(160,320)):
                point = (i, j)
                sum_zone6 =  sum_zone6 + depth_frame[point[1], point[0]]
            
        for i in iter(range(213)):
            for j in iter(range(320,480)):
                point = (i, j)
                sum_zone7 =  sum_zone7 + depth_frame[point[1], point[0]]

        for i in iter(range(213,426)):
            for j in iter(range(320,480)):
                point = (i, j)
                sum_zone8 =  sum_zone8 + depth_frame[point[1], point[0]]
        
        for i in iter(range(426,640)):
            for j in iter(range(320,480)):
                point = (i, j)
                sum_zone9 =  sum_zone9 + depth_frame[point[1], point[0]]
        
        
        dist_zone1 = sum_zone1/34080
        dist_zone2 = sum_zone2/34080
        dist_zone3 = sum_zone3/34080
        dist_zone4 = sum_zone4/34080
        dist_zone5 = sum_zone5/34080
        dist_zone6 = sum_zone6/34080
        dist_zone7 = sum_zone7/34080
        dist_zone8 = sum_zone8/34080
        dist_zone9 = sum_zone9/34080
     
        depth_camera.publish_distances(dist_zone1,dist_zone2,dist_zone3,dist_zone4,dist_zone5,dist_zone6,dist_zone7,dist_zone8,dist_zone9)
            
        sum_zone1 = 0
        sum_zone2 = 0
        sum_zone3 = 0
        sum_zone4 = 0
        sum_zone5 = 0
        sum_zone6 = 0
        sum_zone7 = 0
        sum_zone8 = 0
        sum_zone9 = 0
        
        rate.sleep()


if __name__ == "__main__":
    main()

print("Script dead")