# Report of the practical class 2

## 1) Getting the robot to move

In order to make the robot move, we have to knwo different things before. Let's note the important datas :

- The topic we will send data to : `/cmd_vel_mux/input/navi`
- The type of the topic : `geometry_msgs/Twist`

If we didn't know the name of the topic, we could have used `rostopic list` in order to get the list of all topics that we can manipulate. And as for the type, we could have used the command `rostopic type <topic_name>` in order to get the type of the given topic.

This type of topic need a particular format of data which is the following : $'[x, y, z]'~~'[\alpha, \beta, \gamma]'$ with x, y and z representing respectively the linear velocity according to the x, y and z axis and $\alpha$, $\beta$ and $\gamma$ representing respectively the angular velocity around the x, y and z axis. 

**Example :**

Let's say that we want to send the following data to the topic `/cmd_vel_mux/input/navi` of type `geometry_msgs/Twist` :

$$\left\{\begin{array}{lll}x = 2.0\\ y = 1.0\\ z = 0.0\end{array}\right.\quad \left\{\begin{array}{lll}\alpha = 0.0\\ \beta = 3.0\\ \gamma = 0.0\end{array}\right.$$

Our command will be : `rostopic pub /cmd_vel_mux/input/navi geometry_msgs/Twist '[2.0, 1.0, 0.0]' '[0.0, 3.0, 0.0]'`

---

## 2) Program a keyboard-based teleoperation

let's again note every important information :

- The topic name : `/cmd_vel_mux/input/navi`
- The topic type : `geometry_msgs/Twist`
- The `u` key will be used to go forward
- The `j` key will be used to go backward
- The `h` key will be used to turn left
- The `k` key will be used to turn right
- The `f` key will be used to accelerate the robot of 10%
- The `s` key will be used to decelerate the robot of 10%

We've seen in the first practical class how to create a package to allow ROS to read a Python file.
We've done the same, the following commands recall all the steps to create a package :

1. Create a folder and a `src` folder inside it : `mdkir -p tp_student/src`
2. Enter the following command : `catkin_make`
3. Being inside the `tp_student` folder in the terminal, enable the virtual environment : `source devel/setup.sh`
4. Go to the `src` folder and enter the following command : `catkin_create_pkg tp2_ros_package`
5. A new folder is now created, go into this folder and make a new directory called `scripts`: `mkdir scripts`
6. Create a new Python file that will interact with ROS : `touch my_python_file.py`
7. Don't forget to give the authorizations to the file : `chmod +x my_python_file.py`
8. Launch the python file with the following command : `rosrun tp2_ros_package my_python_file.py`

Our python_file completed is in the repository so we won't explain the code in this file. 


