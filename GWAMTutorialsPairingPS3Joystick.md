# Pairing a PS3 Joystick to the Guardian-WAM System #

The following is a tutorial describing how to pair the Playstation 3 controller with the on-board Bluetooth dongle.


# Instructions #

Upon starting the Guardian system and the Guardian PC, a script is launched spawning all of the Guardian-WAM startup nodes.  One of these nodes is the joy node.  The joy node is a ROS node allowing for easy joystick use; in our case a PS3 joystick.

It is important to wait for an amount of time to allow for the joy node to be started before trying to pair your joystick to the Guardian-WAM system.  Upon the node launching it enters a search mode waiting for the user to pair the joystick with the on-board Bluetooth dongle.

**Pairing the Joystick** -
Press the PS3 Pairing Button shown below to pair your PS3 Controller with the Guardian-WAM.  Upon pairing the joystick will vibrate.

![http://i.imgur.com/kirFb.png](http://i.imgur.com/kirFb.png)

If the joystick will not pair, restart the Guardian-WAM system.  Wait 2-3 minutes before trying to pair the joystick again.

**Do the following only if you could not pair the joystick following the above instructions**

If you are still not successful pairing the PS3 Joystick with the Guardian-WAM System please do the following in a new terminal:
```
ssh guardian-wam@guardian-wam
#user: guardian-wam
#password: guardian
sudo hciconfig hci0 up
hciconfig
#leave this terminal open
```
This will give you the MAC address for the Bluetooth dongle onboard the Guardian.  ex.

```
hci0:   Type: USB
        BD Address: 00:22:B0:D0:5A:09 ACL MTU: 384:8 SCO MTU: 64:8
        UP RUNNING PSCAN
        RX bytes:1013623 acl:17474 sco:0 events:35 errors:0
        TX bytes:247 acl:10 sco:0 commands:16 errors:0
```

Where the MAC address is 00:22:B0:D0:5A:09

Plug in the PS3 Joystick using the supplied USB cable.

In the same terminal shell do the following:
```
sudo bash
rosrun ps3joy sixpair [MAC address]
```
This will result in the following:
```
Current Bluetooth master: 00:22:b0:d0:5a:09
Setting master bd_addr to 00:22:b0:d0:5a:09
```

At this time you should restart the entire Guardian-WAM System, waiting 2-3 minutes before pairing the PS3 Joystick again.

For more information and instructions on pairing a bluetooth joystick please see the following tutorial:

[ros.org/wiki/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle](http://www.ros.org/wiki/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle)