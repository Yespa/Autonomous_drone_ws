#  EXPERIMENTAL AUTONOMOUS DRONE 


## SOFTWARE DEPENDENCIES :file_cabinet:


* [ROS Melodic Morenia](http://wiki.ros.org/melodic) <br>

### PYTHON DEPENDENCIES

* [Python](https://www.python.org/) <br>
Python is a programming language that lets you work quickly and integrate systems more effectively.  

* [Dronekit](https://dronekit.io/) <br>
The API allows developers to create Python apps that communicate with vehicles over MAVLink. It provides programmatic access to a connected vehicle's telemetry, state and parameter information, and enables both mission management and direct control over vehicle movement and operations.

* [OpenCV](https://opencv.org/) <br>
OpenCV (Open Source Computer Vision Library) is a library of programming functions mainly aimed at real-time computer vision.

* [Pymavlink](https://github.com/ArduPilot/pymavlink)<br>
It is a Python adaptation of the MAVLink communication protocol.

* [Pyrealsense2](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html) <br>
It is a library that allows accessing and extracting information from the Intel RealSense D435I camera.

* [Tkinter](https://docs.python.org/es/3/library/tk.html)<br>
Package used for the creation of the graphical interface for navigation monitoring.

### EXPERIMENTAL DRON

The following elements were used to assemble the drone:

* [Readytosky® S500 Quadcopter Frame Stretch X FPV Drone Frame Kit PCB version with carbon fiber landing gear](https://www.amazon.com/gp/product/B01N0AX1MZ/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1)<br>

* [Readytosky Pixhawk PX4 flight controller Pix 2.4.8](https://www.amazon.com/gp/product/B07CHQ7SZ4/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1)<br>

* [Readytosky M8N GPS module Built-in compass with GPS antenna](https://www.amazon.com/gp/product/B01KK9A8QG/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)<br>

* [GARTT 2 pairs ml2312s 960 kv 2212 Brushless Motor CW CCW](https://www.amazon.com/gp/product/B06ZZQL33X/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)

* [Readytosky 40A ESC 2-4S brushless ESC electronic speed controller 5V/3A BEC](https://www.amazon.com/Readytosky-escobillas-controlador-electr%C3%B3nico-helic%C3%B3ptero/dp/B09G5S9YYG/ref=sr_1_1_sspa?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&keywords=esc+40+a&qid=1636741117&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFQRlFTUVg2V0hSREQmZW5jcnlwdGVkSWQ9QTEwMzkwNzMzNlFaVTkyOFZFNUtMJmVuY3J5cHRlZEFkSWQ9QTAzNTcwMzQ3VUJGOEY1NEJPUlgmd2lkZ2V0TmFtZT1zcF9hdGYmYWN0aW9uPWNsaWNrUmVkaXJlY3QmZG9Ob3RMb2dDbGljaz10cnVl)<br>

* [Pixhawk 5.3V BEC XT60 Power Module Connectors for APM2.8 2.6 Quadcopter](https://www.amazon.com/gp/product/B07PJRXHPY/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)<br>

* [Lutions Damping Plate Fiberglass Fiberglass Flight driver Anti-vibration](https://www.amazon.com/gp/product/B01KKB4SNI/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)<br>

* [Readytosky 6 pairs 10x4.5 1045 propeller CW CCW 10 inches](https://www.amazon.com/gp/product/B0823NNTKD/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)<br>

* [HRB 11.1V 5000mAh 3S 50C-100C LiPo battery](https://www.amazon.com/-/es/5000mAh-50C-100C-bater%C3%ADa-enchufe-escobillas/dp/B06XNTHQRZ/ref=sr_1_10?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=AJAC7BXK0JYD&keywords=3s+5000mah+lipo+battery&qid=1636741208&sprefix=Battery+LiPo+500%2Caps%2C243&sr=8-10)<br>

* [Flysky FS-i6X - 6-10 Channel Remote Control Transmitter](https://www.amazon.com/gp/product/B0744DPPL8/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1)<br>


The following Youtube link will show a tutorial of how this drone was assemble.

Youtube

The drone assembly before being integrated with the navigation system looks as follows:

Foto

And in these Youtube videos you will be able to observe how it behaves while flying.

* [Stabilization tests](https://youtu.be/OdA1hklx3Kw)<br>
* [First flight tests](https://youtu.be/eaa0WxqBArw)<br>


### NAVIGATION SYSTEM :flying_saucer:

The following components were used for the navigation system:

* [NVIDIA Jetson Nano Developer Kit](https://www.amazon.com/gp/product/B084DSDDLT/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)<br>
* [Wireless-AC8265 - Dual-mode wireless NIC module for Jetson Nano Developer Kit M.2 NGFF](https://www.amazon.com/gp/product/B07SM4SPLV/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)<br>
* [Intel RealSense - Depth camera D435i](https://www.amazon.com/-/es/Intel-RealSense-profundidad-plateado-82635D435IDK5P/dp/B07MWR2YJB/ref=sr_1_2?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&keywords=D435i&qid=1636746456&sr=8-2)<br>

An MDF structure is built to attach these elements to the drone and thus make use of the integration of all these technologies. 

The system aspect of the experimental drone is as follows.

A simple navigation system is developed directing the UAV to a previously defined terrestrial coordinate. The navigation script is in charge of taking off, orienting, moving and landing the drone to the destination point.

For the navigation to run it was necessary to create other scripts that run in parallel as ROS nodes in the Python framework and these are:



### GUI

## AUTHOR

## William Yesid Palencia Arismendy


