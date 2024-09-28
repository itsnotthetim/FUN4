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
  

  ### 1.) Launch the `fun.lauunch.py` first before doing anything
   ```bash 
   ros2 launch fun4 fun.lauunch.py
   ```
   
    - Then you have to use service call to choose a mode by the following instruction here:
  ```bash 
     ros2 launch fun4 fun.lauunch.py
     ros2 service call /change_mode fun4/srv/ControllerMode "mode: 0
      mode1_pose:
        x: 0.0
        y: 0.0
        z: 0.0
      mode2_toggle: false" 
   ```
    **1.1) Inverse Kinematic Pose Mode: call the service**
   ```bash
   ros2 run god_turtle teleop_turtle.py
   
   ```
   ![teleopinterface](https://cdn.discordapp.com/attachments/1024674136758431752/1284653235877056543/image.png?ex=66e76a0f&is=66e6188f&hm=e0fd3f2f76b81f614ae65281782b43b9eec0a6f1478940a08d249cb4ce99a02c&)
  ![teleopturtle](https://cdn.discordapp.com/attachments/1024674136758431752/1284654197471449229/image.png?ex=66e76af4&is=66e61974&hm=fb5cb29c398c88b7ff0a335a06185ccc4178b5531522ffeefc730841d327e92a&)

  ### 3.) Parameter Configuration
  ```bash
   rqt
   
   ```
  ![dclrprm](https://cdn.discordapp.com/attachments/1024674136758431752/1284656084103069726/image.png?ex=66e76cb6&is=66e61b36&hm=520fd17b7e996939c32fd0670ddd0232894338535e98cff2fa57d10175e3fee7&)

  ## Authors
  - Manasawin Anekvisudwong 65340500049
  - Karanyaphas Chitsuebsai 65340500065

