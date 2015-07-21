# Guardian-WAM Joystick Teleop #

The Guardian-WAM Joystick Teleoperation node is launched by default on Guardian startup.  This node allows for a user to control many aspects of the Guardian-WAM system using a Playstation 3 controller in conjunction with the joy package.

# Instructions #

Upon pairing the joystick with the Guardian-WAM system, the user will have the ability to control the Guardian-WAM system using the following commands.

For instructions on how to pair the PS3 Joystick to the Guardian-WAM system please see the following tutorial:
[Pairing PS3 Joystick Tutorial](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsPairingPS3Joystick)

This documentation assumes that you have already started the WAM node. The WAM portion of the joystick teleoperation will not be functional until the WAM node is launched.  The WAM node can be launched with the following commands:
```
ssh robot@WAM
#username: robot
#password: WAM

#Once you have a secure shell in the WAM onboard PC, you can launch it with the following command.

roslaunch wam_bringup wam_real.launch
```

**To control the WAM or Guardian you _MUST_ be pressing the cooresponding deadman switch**

The deadman switches were introduced to ensure safety and eliminate unwanted motion commands.

To control the Guardian you must be holding down `R1`.

To control the WAM you must be holding down **L1**.  For WAM orientation control you must be holding down **L1** & **L2**.

![http://i.imgur.com/B7hkI.png](http://i.imgur.com/B7hkI.png)

While holding `R1`, you can control the Guardian using the joystick

![http://i.imgur.com/j7wxV.png](http://i.imgur.com/j7wxV.png)

While holding L1, you can control the WAM using the joystick:

![http://i.imgur.com/u23Zg.png](http://i.imgur.com/u23Zg.png)