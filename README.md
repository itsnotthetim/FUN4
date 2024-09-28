# FUN4: Hello World 

## **Mission**
**Design the 3-DOF Manipulator Arm Control System by the following instruction**
- Part I Setup an Environment
  - Finding a workspace of manipulator arm along with proving the answer
  - Creat **Randomizer Node** for random an end-effector position inside off the workspace area
    and name the topic as **/target** (msg type: PoseStamped)
  - Send an end-effector position through the topic **/end_effector** (msg type: PoseStamped)
  
- Part II Controller
   - The manupulator has 3 control modes, consist of **Inverse Pose Kinematic Mode (IPK)**, **Tele-operation Mode       (Teleop)** and **Autonomous Mode (Auto)**. For changing the mode, The mode can be changed while it's running
     **Note:** Return the response of service as **True** when the mode has changes.

    **1. Inverse Pose Kinematic:** Calculate the configuration space to task space. Return **True** if the
  solition can be solve and operate the robot and vice versa

    **2. Teleoperation:** Operate the robot with **/cmd_vel** (msg type: Twist) from **teleop_twist_keyboaard**
      for calculated the joint angular velocity. There're 2 modes for /cmd_vel.
        2.1) /cmd_vel that refer from an **end-effector frame**
        2.2) /cmd_vel that refer from a **base frame**
      **If the robot is close to the Singularity, Stop the robot and publish something to warn the user**

    **3. Autonomous Mode:** Send the Taskspace from the **Randomizer Node** to the robot and the robot have to           move to the target position within 10 seconds. when finished,response will give the return as **True** then regenerating the new position.

## **Installation**

1. Create workspace
   ```bash
   mkdir ~/your_workspace
   cd ~/your_workspace
   mkdir src/
   ```
2. Clone this repository to your workspace:
   ```bash
   git clone https://github.com/itsnotthetim/fun4.git
   ```
3. Build your workspace by colcon build:
   ```bash
   cd ~/your_workspace
   colcon build
   ```
4. Install the Roboticstoolbox package and teleop_twist_keyboard
   ```bash
   pip install roboticstoolbox-python
   sudo apt install ros-humble-teleop-twist-keyboard
   ```
5. Source the workspace:
   ```bash
   . ~/your_workspace/install/setup.bash
   ```
6. Add your source command workspace into ~/.bashrc:
   ```bash
   echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
   ```
## Usage
- This package provides you  1 launch file `fun.launch.py`. Contains with  `controller.py` use to control and compute the manipulator arm and `random_node.py` use to random the taskspace of
  an **Autonomous mode**. And 2 custom services in fun4/srv folder that use for communicate between user and within node -- `CallRandomPos.srv` and `ControllerMode`.
  

  ### 1.) Launch the `fun.lauunch.py` First before doing anything (Not reccomended to call service before running this)
     ```bash 
     ros2 launch fun4 fun.lauunch.py
     ```
   
    - Then you have to use service call to choose a mode by the following instruction here:
        - Mode: 1 ; **Inverse Pose Kinematic Mode --- You have to select mode: 1 and set the mode1_pose which  -0.03 <= x,y,z <= 0.53**
      
        - Mode: 2 ; **Teleoperation Mode --- You have to select mode: 2 and set the mode2_pose which**
          - **True = refer from base to end-effector frame**
          - **False = refer directly to the end-effector frame**
        
        -  Mode: 3 ; **Autonomous Mode -- Select only mode: 3**
     
      **You can leave the setting which doesn't relate to that mode**
    
    ```bash 
       ros2 service call /change_mode fun4/srv/ControllerMode "{mode: 0, mode1_pose: {x: 0.0, y: 0.0, z: 0.0}, mode2_toggle: true}"
     ```
  
    **2.) Inverse Kinematic Pose Mode: Example Usage**
     ```bash
     ros2 service call /change_mode fun4/srv/ControllerMode "{mode: 1, mode1_pose: {x: 0.3, y: 0.2, z: 0.1}}"
     ```
   ![IPKMode](https://cdn.discordapp.com/attachments/718092584928411729/1289460430867267654/image.png?ex=66f8e71b&is=66f7959b&hm=014c317ae5193d7de413871dde3d81a80b0f948e887119e4f378a49a3fee42ed&)

    **3.) Teleoperation Mode: Example Usage**
    - Mode: Reference at Base frame
     ```bash
     ros2 service call /change_mode fun4/srv/ControllerMode "{mode: 2, mode2_toggle: true}"
     ```
   - Mode: Reference at the end-effector frame
     ```bash
     ros2 service call /change_mode fun4/srv/ControllerMode "{mode: 2, mode2_toggle: false}"
     ```
    **3.1) Then Run teleop_twist_keyboard (Stick in this terminal to control movement)**
    - Press q/z for increase++/decrease-- the linear velocity
    - Press t/b to control velocity at axis z+/z-
    - Press I/< (press **cap lock** before) to control velocity at axis x+/x-
    - Press J/L (press **cap lock** before) to control velocity at axis y-/y+
    - Press U/P (press **cap lock** before) to control velocirt at plane xy-/xy+
    - Press any key except the defination to stop moving
  
  ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
![teleoptwist](https://cdn.discordapp.com/attachments/718092584928411729/1289500744575356980/image.png?ex=66f90ca7&is=66f7bb27&hm=c1566456140cdcad7426bc9c10aab8d49ee2f129e50abb63cf245886bc51e7cb&)

![teleoptwist](https://cdn.discordapp.com/attachments/718092584928411729/1289500858920206376/image.png?ex=66f90cc2&is=66f7bb42&hm=20e2139eb1127e37712f78ef64b2854d9f23bb19cd08d5f5f56244c877cf512d&)

  **4.) Autonomous Mode: Example Usage**
    - The robot will continue moving around depending on the data from random_node 
    ```bash
     ros2 service call /change_mode fun4/srv/ControllerMode "{mode: 3}"
     ```
  
  ![autonomous](https://cdn.discordapp.com/attachments/718092584928411729/1289501791448338473/image.png?ex=66f90da0&is=66f7bc20&hm=576c81c93646ce1681bf8698dd12e3f6d9ff5d4cda76da1dd3f6ffe4c06b1be8&)

  ## Authors
  - Karanyaphas Chitsuebsai 65340500065

