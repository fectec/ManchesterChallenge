# Manchester Challenge

Main project developed in collaboration with Manchester Robotics, as part of the undergraduate courses **"Robotics Foundation"** and **"Intelligent Robotics Implementation."**

## First Stage: ROS2 DC Motor Control

The goal was to control the speed of a DC motor attached to one of the wheels of the Puzzlebot — Manchester Robotics’ educational, differential-drive mobile robot.

Although velocity control for the Puzzlebot’s wheels will later use pre-built facilities provided by Manchester Robotics, it is essential to first understand the core control principles.

One approach is open-loop control, where voltage is applied in proportion to the desired speed. However, this method may not be robust enough, since it lacks self-correction.

For this reason, a closed-loop controller was used: a PID. By applying proportional, integral, and derivative terms, the system can reduce error over time. This ensures that the actual speed converges to the desired value.
