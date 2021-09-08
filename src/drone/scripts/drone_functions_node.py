#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

#Libraries that allow communication with the flight control board, Pixhawk 2.4.8

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Library used to define the command messages

#General imports
import time
import socket
import math
import argparse
from rospy import service

from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from std_msgs.msg import String

#Importing services
from drone.srv import arm, armResponse, takeoff, takeoffResponse, rot_yaw, rot_yawResponse, vel_lin, vel_linResponse, land, landResponse


class Node_functions_drone:

    """
    Node that allows executing actions on the drone, such as take-off, landing, speed variations, speed monitoring, 
    position monitoring of speed, position, IMU  
    """

    def __init__(self,connection_string,baud_rate):

        self.Vx = 0
        self.Vy = 0
        self.Vz = 0
        self.heading = 0
        
        #Make the connection between the Pixhawk and the Jetson nano
        self.connection_string = connection_string
        self.baud_rate = baud_rate

        self.vehicle = connect(self.connection_string,self.baud_rate,wait_ready=True)

        print("--------------SUCCESSFUL CONNECTION------------------")

        #SERVICES

        #Service to ARM the drone
        self.srv_arm_drone = rospy.Service("drone/srv/arm",arm,self.arm_drone)

        #Service to perform the drone take-off
        self.srv_take_off = rospy.Service("drone/srv/take_off",takeoff,self.takeoff)

        #Service to perform the rotation of the drone with respect to the z-axis
        self.srv_rot_yaw = rospy.Service("drone/srv/rot_yaw",rot_yaw,self.condition_yaw)

        #Service for sending linear velocities to the drone
        self.srv_vel_lin = rospy.Service("drone/srv/vel_lin",vel_lin,self.Vel_mat_rot_Z)

        #Drone landing service
        self.srv_land = rospy.Service("drone/srv/land",land,self.aterrizaje)

        #SUBSCRIBERS

        #Subscriber at the rate sent from the navigation node
        self.sub_vel_nav = rospy.Subscriber('Nav/vel_lin',TwistStamped,self.update_velocity)

        #PUBLISHERS

        #Publisher of the current global coordinates of drone
        self.pub_pos_gps = rospy.Publisher('drone/pos_gps',NavSatFix,queue_size=10)

        #Publisher of the current linear velocity of the drone
        self.pub_vel_now = rospy.Publisher('drone/vel_now',TwistStamped,queue_size=10)

        #Publisher of the angle of orientation of the drone with respect to the north of the earth.
        self.pub_orient_angle_z_now = rospy.Publisher('drone/orient_angle_z_now', PointStamped, queue_size=10)

        #Publisher of the current drone battery status
        self.pub_battery_now = rospy.Publisher('drone/battery_now', BatteryState, queue_size=10)

        #Publisher of the current drone attitude
        self.pub_orientacion_quaternion = rospy.Publisher('drone/orient_quaternion',QuaternionStamped, queue_size=10)

    #METHODS

    #Method for updating the speed delivered by the navigation node

    def update_velocity(self,Vel_nav):
        self.Vx = Vel_nav.twist.linear.x
        self.Vy = Vel_nav.twist.linear.y
        self.Vz = Vel_nav.twist.linear.z
        self.heading = Vel_nav.twist.angular.z

        tol = 0
        #Tolerancia entre los datos que estan entre 357 a 3 grados
        if self.heading in range(357,361) or self.heading in range(0,4):

            Ang_problem = [357,358,359,360,0,1,2,3]

            if self.heading in Ang_problem:

                pos = Ang_problem.index(self.heading)

                infpos = pos - 3

                if infpos < 0:
                    infpos = 0

                suppos = pos + 3

                if suppos > len(Ang_problem):
                    suppos = len(Ang_problem)


                for i in range(infpos,suppos):
                    if int(self.vehicle.heading) == Ang_problem[i]:

                        tol = 1
                        break
                    else:
                        tol = 0



        if int(self.heading) in range((int(self.vehicle.heading) - 3),(int(self.vehicle.heading) + 3)) or tol==1:

            self.Vel_mat_rot_Z_Aut()
        else:

            self.condition_yaw_Aut()
        

    #Method to obtain current position (latitude, longitude and altitude)
    def publish_pos_gps(self):

        #Este mensaje tiene una estructura especial es por ellos que se formula de la siguiente manera
        msg =NavSatFix()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "drone/Data_GPS"
        msg.status.status = self.vehicle.gps_0.fix_type #Estado del GPS
        msg.status.service = self.vehicle.gps_0.satellites_visible #Cantidad de satelites conectados


        #Solicito posicion a la pixhawk
        msg.latitude =  self.vehicle.location.global_relative_frame.lat #Latitud
        msg.longitude =  self.vehicle.location.global_relative_frame.lon #Longitud
        msg.altitude =  self.vehicle.location.global_relative_frame.alt #Altura
        msg.position_covariance = (0,0,0,0,0,0,0,0,0) 
        msg.position_covariance_type = 0

        #Publico la informacion
        self.pub_pos_gps.publish(msg)

    #Method to publish general variables of the drone's sensors.
    def publish_status_drone(self):

        #Se solicita la información a la pixhawk
        # --> VELOCIDADES ACTUALES EL DRON
        
        vel_now = TwistStamped()

        vel_now.header.stamp = rospy.Time.now()
        vel_now.header.frame_id = "drone/Velocidades"
        vel_now.twist.linear.x = self.vehicle.velocity[0]
        vel_now.twist.linear.y = self.vehicle.velocity[1]
        vel_now.twist.linear.z = self.vehicle.velocity[2]
        vel_now.twist.angular.x = 0
        vel_now.twist.angular.y = 0
        vel_now.twist.angular.z = 0


        #--> ANGULO EN GRADOS DE LA ROTACION EN EL EJE Z (Respecto al norte)
        
        point_ang_z = PointStamped()
        
        point_ang_z.header.stamp = rospy.Time.now()
        point_ang_z.header.frame_id = "drone/angle_z"
        point_ang_z.point.x = 0
        point_ang_z.point.y = 0
        point_ang_z.point.z = self.vehicle.heading

        #--> ESTADO DE LA BATERIA DEL DRON

        battery_now = BatteryState()

        battery_now.header.stamp = rospy.Time.now()
        battery_now.header.frame_id = "drone/battery"
        battery_now.voltage = self.vehicle.battery.voltage
        battery_now.current = self.vehicle.battery.current
        battery_now.percentage = self.vehicle.battery.level


        #--> ACTITUD O ORIENTACION ACTUAL DE DRON

        self.phi = self.vehicle.attitude.roll
        self.theta = self.vehicle.attitude.pitch
        self.psi = self.vehicle.attitude.yaw

        # Convertimos los angulos fijos, angulos de euler, a Quaternion para posteriormente publicar la orientacion
                #Conversion de tomada de Meccanimo Complesso https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
        self.qw = math.cos(self.phi/2) * math.cos(self.theta/2) * math.cos(self.psi/2) + math.sin(self.phi/2) * math.sin(self.theta/2) * math.sin(self.psi/2)
        self.qx = math.sin(self.phi/2) * math.cos(self.theta/2) * math.cos(self.psi/2) - math.cos(self.phi/2) * math.sin(self.theta/2) * math.sin(self.psi/2)
        self.qy = math.cos(self.phi/2) * math.sin(self.theta/2) * math.cos(self.psi/2) + math.sin(self.phi/2) * math.cos(self.theta/2) * math.sin(self.psi/2)
        self.qz = math.cos(self.phi/2) * math.cos(self.theta/2) * math.sin(self.psi/2) - math.sin(self.phi/2) * math.sin(self.theta/2) * math.cos(self.psi/2)
               
        attitud_now = QuaternionStamped()
        
        attitud_now.header.stamp = rospy.Time.now()
        attitud_now.header.frame_id = "drone/Orientacion_quaternion"

        attitud_now.quaternion.x = self.qx
        attitud_now.quaternion.y = self.qy
        attitud_now.quaternion.z = self.qz
        attitud_now.quaternion.w = self.qw

        self.pub_vel_now.publish(vel_now)
        self.pub_orient_angle_z_now.publish(point_ang_z)
        self.pub_battery_now.publish(battery_now)
        self.pub_orientacion_quaternion.publish(attitud_now)


    #METHOD FOR THE ARMING OF DRONE

    def arm_drone(self,request):
        
        timeout = 60
        timeout_start = time.time()


        self.vehicle.mode = VehicleMode("GUIDED")
           

        while self.vehicle.is_armable==False and (time.time() < timeout_start + timeout):
            print("Esperando disponibilidad de armado")
            print("")
            time.sleep(2)


        if self.vehicle.is_armable==False:
            estatus = "Arm_fail"

        elif self.vehicle.is_armable == True:


            print("El dron esta listo para ser armado!")

            print("CUIDADO")
            time.sleep(3)

            self.vehicle.armed=True  ## Comando para armar el dron
        
            while self.vehicle.armed==False:
                print("Esperando que se arme el dron")
                time.sleep(1)
            print("------El dron esta armado!------")

            estatus = 'Arm_check'

        return armResponse(estatus)


    #METHOD FOR TAKE-OFF
    
    def takeoff(self,request):
        
        if self.vehicle.armed == True:

            ######### Enviamos accion de despegue ##########
            time.sleep(2)
            print("------Taking off-------")

            self.vehicle.simple_takeoff(request.alt_deseada)        

            #Mostramos la altura actual y el codigo se detiene hasta que no llegue a la altura deseada, esto se hace
            #debido a que si se ejecuta otro comando, inmendiatamente se interrumpe la acción anterior.

            
            while True:
                print(" Current height: ", self.vehicle.location.global_relative_frame.alt)

                #Verificamos que no exceda la altura deseada, cuando se cumple salimos de la funcion

                if self.vehicle.location.global_relative_frame.alt >= request.alt_deseada * 0.95:
                    print("")
                    print("------Desired height achieved-----")
                    break
                time.sleep(1)
            
            estatus = "Alt_check"

        else:
            self.msg ='Dronenoarmed'
            print("drone no armado")
            estatus = 'No_arm'

        return takeoffResponse(estatus)

    #METHOD FOR SENDING SPEED COMMANDS TO THE FLIGHT CONTROLLER IN ABSOLUTE COORDINATES

    #Default Dronekit library script. Taken from the dronekit API

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        
        self.vehicle.send_mavlink(msg)
           

    #METHOD FOR CONVERTING VELOCITIES WITH RELATIVE COORDINATES TO VELOCITIES WITH ABSOLUTE COORDINATES
    
    #I implement it to perform debugging by means of services.
    def Vel_mat_rot_Z(self,request):

        grados_act = self.vehicle.heading

        def sen(grados):
            seno = math.sin(math.radians(grados))
            return seno

        def cos(grados):
            coseno = math.cos(math.radians(grados))
            return coseno

        self.VxP = request.Vx*cos(grados_act) - request.Vy*sen(grados_act)
        self.VyP = request.Vx*sen(grados_act) + request.Vy*cos(grados_act)
        self.VzP = request.Vz

        self.send_ned_velocity(self.VxP,self.VyP,self.VzP,0.1)

        estatus = "Vel_send"
        return vel_linResponse(estatus)

    #It is implemented in the navigation node
    def Vel_mat_rot_Z_Aut(self):

        grados_act = self.vehicle.heading

        def sen(grados):
            seno = math.sin(math.radians(grados))
            return seno

        def cos(grados):
            coseno = math.cos(math.radians(grados))
            return coseno

        self.VxP = self.Vx*cos(grados_act) - self.Vy*sen(grados_act)
        self.VyP = self.Vx*sen(grados_act) + self.Vy*cos(grados_act)
        self.VzP = self.Vz

        self.send_ned_velocity(self.VxP,self.VyP,self.VzP,0.1)



    #METHOD TO REALIZE THE ROTATION OF THE DRONE IN THE Z-AXIS

    #I implement it to debug by means of services.
    def condition_yaw(self,request):

        relative = False

        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            request.heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        
        #Se envia el comando al dron 
        self.vehicle.send_mavlink(msg)

        
        while True:
        
            print(" Current orientation ", self.vehicle.heading, "- Orientation request: ", self.heading)

            #Nos metemos en un ciclo hasta que se cumpla el valor de ultima posicion del cabeceo
            #para que la funcion no sea interrumpida por otro comando

            if self.vehicle.heading > request.heading:
                if self.vehicle.heading <= request.heading * 0.94:
                    print("------Desired orientation achieved-----")
                    break
                time.sleep(1)
            else:
                if self.vehicle.heading >= request.heading * 0.94:
                    print("------Desired orientation achieved-----")
                    break
                time.sleep(1)

        estatus = "Rot_yaw_check"       
        return rot_yawResponse(estatus)


    #It is implemented by means of the state machine.
    def condition_yaw_Aut(self):

        relative = False

        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            self.heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        
        #Se envia el comando al dron 
        self.vehicle.send_mavlink(msg)

        
        while True:
        
            print("Current orientation: ", self.vehicle.heading, "- Orientation request: ", self.heading)

            #Nos metemos en un ciclo hasta que se cumpla el valor de ultima posicion del cabeceo
            #para que la funcion no sea interrumpida por otro comando

            if self.vehicle.heading > self.heading:
                if self.vehicle.heading <= self.heading * 0.94:
                    print("------Desired orientation achieved------")
                    break
                time.sleep(1)
            else:
                if self.vehicle.heading >= self.heading * 0.94:
                    print("------Desired orientation achieved------")
                    break
                time.sleep(1)


    #METHOD TO PERFORM A CONTROLLED LANDING.

    def aterrizaje(self,request):
       #Switch to landing mode
        
        self.vehicle.mode = VehicleMode("LAND")

        print("-------Landing-------")

        while True:
            print(" Current height: ", self.vehicle.location.global_relative_frame.alt)

            ##We verify that the desired height is exceeded, when it is fulfilled we exit the function.

            if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                print("--------Landing-------")
                break
            time.sleep(1)
        
        print("---------Landed----------")

        estatus = "Land_check"

        return landResponse(estatus)

def main():

    print("---------- Initialized node -------")
    rospy.init_node('drone')

    drone = Node_functions_drone("127.0.0.1:14550",5760) 

    #Sampling period
    rospy.Rate(33)


    while not rospy.is_shutdown():
        
        #We publish the position of the drone
        drone.publish_pos_gps()
        
        #We publish the status of the drone
        drone.publish_status_drone()



if __name__ == "__main__":
    main()

print("Script dead")