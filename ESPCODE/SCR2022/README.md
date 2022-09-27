# Origami Robot Cars with Actuation and Feedback Control

<img src="RoCoOrigamiRobotCars/differentialcar.jpg"  width="300" height="400">
<img src="RoCoOrigamiRobotCars/pivoitngcar.jpg"  width="300" height="400">

_Red Car = Differential car; Green Car = Pivoting car_

# Goal
- To create origami robot cars that can be used by a diversity of people. 
- To build two car designs that can run on varying surfaces -- a normal four-wheeled car and a pivoting four-wheeled car using [UCLA LEMUR’S RoCo](https://git.uclalemur.com/roco/rocolib).
- To incorporate robotic behaviours, actuation and feedback control, on the origami robot cars using a variety of hardware and Arduino.
<!-- blank line -->
----
<!-- blank line -->
**NOTE:** There are **two car designs in this repository: a differential steering car and a pivot steering car**. The cars don’t really have any difference (besides their steering type) when driven on a smooth surface. However, when placed in carpets or rougher surfaces, the differential steering car DOES NOT turn properly while the pivot steering car TURNS significantly. Thus, before jumping into building the cars, you must decide which car will benefit you more -- or you can just build both if you want to. Note that there is always a benefit from using one car over the other. For instance, it is easier to turn a differential steering car to an angle using an IMU than the pivoting car while it is easier to drive the pivoting car on rougher surfaces than the differential steering car.

If you are interested in the creation of the car designs using RoCo, click **TODO: INSERT LINK TO BUILDERS** here to see the documentation.
<!-- blank line -->
----
<!-- blank line -->
## Hardware
**NOTE:** The build of the car will depend on what functionalities of the car you want them to be. The following are the respective hardware for each functionality.

_Actuation_
- [Assembled Adafruit HUZZAH32 – ESP32 Feather Board - with Stacking Headers](https://www.adafruit.com/product/3619) 
- [8-Channel PWM or Servo FeatherWing Add-on For All Feather Boards](https://www.adafruit.com/product/2928)
- FS90R Continuous Servo Motors
- [Lipo batteries](https://www.amazon.com/Crazepony-Battery-PowerWhoop-Connector-Inductrix/dp/B07L9SHHFX/?_encoding=UTF8&pd_rd_w=7JfxW&pf_rd_p=205b21f0-8557-4ea8-a863-e58f77379cf8&pf_rd_r=JEQX8C0JQKHZC3YKTFGG&pd_rd_r=e30cfbcb-193d-43bf-be71-03e3e5a52767&pd_rd_wg=r5UnO&ref_=pd_gw_ci_mcx_mr_hp_atf_m) 
- [NeoPixel FeatherWing - 4x8 RGB LED Add-on For All Feather Boards](https://www.adafruit.com/product/2945)
- **ADDITIONAL FOR PIVOT STEERING CAR**: SG90 180 Degrees 9G Micro Servo Motor Tower and Super Glue

_Feedback Control_

IMU
- [Adafruit LSM6DSOX + LIS3MDL FeatherWing - Precision 9-DoF IMU](https://www.adafruit.com/product/4565) (I recommend using this IMU because it is stackable with the ESP32 Feather Board)

LIDAR
- [VL53L0X Lidar Sensor](https://www.amazon.com/Onyehn-VL53L0X-Breakout-GY-VL53L0XV2-Distance/dp/B07Q5Y3G4C/ref=sr_1_3?dchild=1&keywords=vl53l0x&qid=1630037809&s=electronics&sr=1-3)
- [FeatherWing Proto - Prototyping Add-on For All Feather Boards](https://www.adafruit.com/product/2884)

SWARM
- [Adafruit Radio FeatherWing - RFM69HCW 433MHz - RadioFruit](https://www.adafruit.com/product/3230)
- [Stacking Headers for Feather - 12-pin and 16-pin female headers](https://www.adafruit.com/product/2830) 

## Tutorial for creating the cars

The tutorial below is for a car with the actuation hardware and IMU sensor inside it. If you only want the car to have actuation and not feedback control go **TODO: INSERT LINK TO LIBRARY/BUILDERS** here to change the size of the car.

The files for generating the default Origami Robot Cars are in [here](https://git.uclalemur.com/arnhold/feedback-controlled-cars/-/tree/main/RoCoOrigamiRobotCars).

**TUTORIAL for Differential Steering Car**

**TUTORIAL for Pivot Steering Car** 

## Installing needed libraries for hardware

Adafruit Hardware:
1. [Instructions](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/using-with-arduino-ide) for setting up the **ESP32 Featherboard**.
2. [Instructions](https://learn.adafruit.com/adafruit-8-channel-pwm-or-servo-featherwing/using-the-adafruit-library) for setting up the **8-Channel PWM or Servo FeatherWing Add-on For All Feather Boards**.
3. [Instructions](https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-installation) for setting up the **NeoPixel FeatherWing - 4x8 RGB LED Add-on For All Feather Boards**.'
4. [Instructions](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/calibration-pre-check) for setting up the **Adafruit LSM6DSOX + LIS3MDL FeatherWing - Precision 9-DoF IMU**.'
5. [Instructions](https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code) for setting up the **VL53L0X Time of Flight Micro-LIDAR Distance Sensor**.'

**SWARM/Mesh Nerworking:**
Go to this [repository](https://git.uclalemur.com/arnhold/espmeshnetworking/-/tree/master/MeshCode) to download and install the needed libraries to enable communications between Origam Robot Cars (this repo is also needed even you are only controlling one car). All of the instructions are in the said repository.

**Camera:**
Install the _OpenMV Arduino RPC Interface Library_ in Arduino.

**Other Libraries**
Download [Libraries.zip](https://git.uclalemur.com/arnhold/feedback-controlled-cars/-/tree/main/Libraries) -> extract -> install or paste the libraries in your Arduino libraries directory.

## Uploading the codes + controlling the car

Go [here](https://git.uclalemur.com/arnhold/feedback-controlled-cars/-/tree/main/ArduinoCarCode) for the instructions and guide in uploading the codes and controlling the car or go to _ArduinoCarCode_ directory.

## Videos of the Cars
Once you read the guide from the link above (in the section _Uploading the codes + controlling the car_), the videos of the functionalities of the cars can be seen in this [google drive](https://drive.google.com/drive/folders/1-jRa_ZuXAgKMxjYW3UdKy5mry3GdDi6D?usp=sharing).
