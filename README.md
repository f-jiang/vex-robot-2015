# VEX 2015 Robot - Nothing But Net

Here is the code written for our VEX competition robot. The VEX robotics competition is a high school-level engineering challenge designed to foster
learning, teamwork, and an interest in STEM. 

## The Game

<img src="https://raw.githubusercontent.com/f-jiang/vex-robot-2015/master/images/field.png" width="400">

For 2015, the game's objective was to score miniature balls into nets located in the corners of the playing field.
At the beginning of each match, shooting is done autonomously, and the team with the most scored shots wins.

## The Robot

<img src="https://raw.githubusercontent.com/f-jiang/vex-robot-2015/master/images/robot.jpg" width="400">

Our goal was to create a robot that was maneuverable and could make accurate shots from any range. Thus we decided to use a holonomic x-drive,
which allowed the robot to freely move and turn in any direction. Using flywheels for the ball launcher enabled us to make both long and short
distance shots simply by adjusting the power delivered to the wheels. Although the drivetrain controls were counterintuitive at first, we found
the holonomic drive to be quite good at navigating tight spaces where sudden changes in direction were frequent and necessary.

The robot code is based on a loop in which we listen to controller input and perform the corresponding robot action. A scalable slew rate
controller was implemented in order to reduce gear wear. Also, the shooting distance can be adjusted manually or assigned to a number of commonly used presets. The code is powered by the [PROS Kernel for the VEX Cortex Microcontroller](https://github.com/purduesigbots/pros) rather than the more common RobotC.
