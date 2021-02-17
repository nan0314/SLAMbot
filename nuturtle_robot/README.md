# Nuturtle Robot
* `<roslaunch nuturtle_robot basic_remote.launch>` to run nodes on the turtlebot
and setup low level control through ROS
# Example Usage
```
roslaunch nuturtle_robot odom_teleop.launch robot:=FranklinTheTurtle.local circle:=false
roslaunch nuturtle_robot odom_teleop.launch robot:=FranklinTheTurtle.local circle:=true
```

Final Location in format (th,x,y)

Experiment 1:

![Video](https://drive.google.com/uc?export=view&id=1G5f7t2TROddBYKYC4nAr3XS98TP48JIc)

![Screen Capture](https://drive.google.com/uc?expert=view&id=1U-PW0LDYVcQC6YnxOTa1a93SxeG1S2ds)


Final Location: (-0.0459,0.0287,0.0113)


Experiment 2:

![Video](https://drive.google.com/uc?export=view&id=1L2w76aoJRhgfUcVFXyzWBRBePrsNfkvL)

![Screen Capture](https://drive.google.com/uc?export=view&id=1RcUBjRK_J_VyxNtB4ZMG0LOEKA5v0ss7)

Final Location: (0.0025,-0.0011,0.0013)


Experiment 3:

![Video](https://drive.google.com/uc?export=view&id=1nOc-BzaVfCCQev37koYSX3fmLvrWme38)

![Screen Capture](https://drive.google.com/uc?export=view&id=17JNR4s0pC8VR8kCKUkX7Bvw5-gIqXRyo)

Final Location: (0.0158,-0.0219,0.0004)


Experiment 4:

In this experiment I tried to drive in circles manually. I was very unseccessful-- driving in circles is hard.
Best leave it to the software.

![Video](https://drive.google.com/uc?export=view&id=1e6_uEF-2goA1eQsf5_WwahHlRK5Qhz2Y)

![Screen Capture](https://drive.google.com/uc?export=view&id=1Jnu0Mzg6Y9QURchKUGiaHaDI5FIF9TyQ)

Final Location: (0.0203,0.1378,0.1105)
