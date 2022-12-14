
## Installation
- Python 3.9
- IDE: Pycharm(recommended) or Visual Studio
- git
- libraries in requirement.txt (Updated, 2/3/22)

Your Python has to be these listed versions above since Webots simulation (Used in the next lab) does not support other versions.
IDE is strongly recommended. 
All required libraries are in requirement.txt. IDE usually install all automatically, or 

```bash
pip install -r requirements.txt
```


## How to Run
- woodbot_run.py has an example of how to run the hardware version of middleware
- You can pick one of input options: FileInput, KeyboardInput, or JoystickInput
- You can have outputs as you want: FileOutput
- You can add as many robot as you want for simulations: KinematicsModel, WebotsModel, NOPModel
- If you use FileOutput, you can reproduce the result with FileInput

## How to Run Paper Cars
- ESPCODE folder has the Arudino IDE Code for the Adafruit Feather board
- You will need to input the router information and maybe the motor polarity by changing the drive function
- This software works in in tandem with the woodbot_run.py program

## Notes
- Input space
- Joystick
  - JoystickInput requires a joystick controller (not required if you don't have one), or throws an error
  - We are using pygame to deal with joysticks which suppose to work cross platform and on nearly all controllers
  - Depending on the vendor of the controllers, key mapping could be different (Defined in constants.py, class JOYSTICK)
  - We tested with Xbox and Switch controllers with no issues (Plug and Play)
  - Your controller should have 2 joysticks at least.
- matplotlib is for figures


## Webots and aruco maker
- Please update Webots Woodbot motor maxVelocity to 20.(HingeJoint > Device > Motor > maxVelocity)
- Aruco Marker system is added. 
- Aruco maker detection is another robot. you need to add the ArucoMaker().
  - aruco_track = ArucoMaker(track_id=1, display=False,camera_id=1)
  - camera id could be 0 or 1 if you have more than 1 camera
  - track_id is maker id on the robot
  - display True will slow down everything. Even false, you get woodbot.avi video output.

- If you like to use your own setup, you need: 
  - ID 255 (right top), 254, 253, 252 (right bottom) 5by5 50mm marker at https://chev.me/arucogen/.
  - Camera calibration board and follow advices in https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844
  - To run calibration, change your picture names to image0, image1, image2, ..., and then run ArucoHelper.py. You may need to change the box size (currently 0.022m)
  
## Run speed 
- Now you Python version MUST be 3.9. If you were using other versions, you meed to add path to Webots Python39 library and delete old path. 
- Now you have an option to run simulation either realtime or as fast as possible. If you add hardware woodbots, it runs at realtime.
- You may need to increase TEST.DT (in constants.py) if you get info saying actual DT is larger than specified DT. Hardware has communication latency of about 100-200ms. You can start with 0.25s and increase or decrease as necessary. 
- Let Webots run as fast as possible (not real speed). Woodbot may not move smoothly because it finishes running one time step faster than hardware. 
- Please follow Paperbot repo for how to connect the woodbot. 

## Process system
- ProcessSystem allows you to conduct a process after every timesteps
- returning a job at def process() creates a background job. (Either thread or multi process)
- you can set callback to know when the background task is done


## Files
```
kinematics
???   README.md
???   run_example.py                    (An example code)   
???
????????????doc
???   ???   function.md               (Software architecture)
???   ???   summary.md                (Lab summary)
???
????????????sim
    ???   fromulation.py            (You need to modify)
    ???   constants.py
    ???   Simulation.py 
    ???
    ????????????inputs
    ???     ???   InputSystem         (Base class)
    ???     ???   FileInput.py
    ???     ???   Keyboard.py
    ???     ???   JoystickInput.py 
    ???
    ????????????outputs
    ???     ???   OutputSystem        (Base class)
    ???     ???   FileOutput.py
    ???     ???   GraphOutput.py      (You need to implement)
    ???     ???   AnimationOutput.py  (You need to implement)
    ???
    ????????????robots
    ???     ???   RobotSystem.py      (Base class)
    ???     ???   KinematicsModel.py  (Kinematics Model of differential drive)
    ???     ???   NOPModel.py         (Do nothing)
    ???     ???   WebotsModel.py      Webots implementation of differntial drive
    ???     ???   WebsocketConnection.py  Differential drive robot hardware 
    ??? 
    ????????????PROCESS
          ???   ProcessSystem.py    (Base class)
          ???   TestProcess.py      (example showing how to create, submit and get callback from a job)

```  
