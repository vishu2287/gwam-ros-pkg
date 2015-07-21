# Initial Setup - User PC #

The Guardian-WAM software framework is ROS (Robot Operating System).  For more information on ROS please visit [ros.org](http://www.ros.org).

There are two primary communication techniques with the Guardian WAM:

### External PC Communication ###
**_The following is the preferred GWAM communication protocol._**

**Install ROS Electric:**

To use the Guardian-WAM in conjunction with an external pc, you must have installed a local copy of ROS Electric.  Installation instructions are available here: [ROS Electric Installation](http://www.ros.org/wiki/electric/Installation/Ubuntu).

**Configure ROS Network Setup:**  (_On your personal computer only!_)

You need to edit your **/etc/hosts** file with sudo permissions and add the following lines
```
192.168.2.101   guardian-wam
192.168.2.110   WAM
```

**Test Communication**
You should now be able to ping & ssh into the different PCs on the Guardian system.
```
#Ping the Guardian PC
ping guardian-wam
#Ping the WAM PC
ping WAM

#ssh into the Guardian PC
ssh guardian-wam@guardian-wam
#password: guardian
exit 


#ssh into the WAM
ssh robot@WAM
#password: WAM
exit 
```

**Create your ROS Workspace**
The following will create a folder which you can use as your ROS development workspace.  If you already have a working ROS environment workspace, please skip this instruction.
```
cd ~/
mkdir ros
```

**Edit your local .bashrc:**

The following adds a line to **~/.bashrc** file in order source a local environment description file for easy editing.
```
echo "source ~/ros/gwam_env.ros" >> ~/.bashrc
. ~/.bashrc
```

Now that we have created the link to our local environment file, we must create it in our favorite text editor.

Create the file ~/ros/gwam\_env.ros and paste in the following
```
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=~/ros:${ROS_PACKAGE_PATH}
export ROS_MASTER_URI=http://guardian-wam:11311;
#export ROS_MASTER_URI=http://localhost:11311;
```

Explanation:
We first start by sourcing the default ROS Environment.
```
source /opt/ros/electric/setup.bash
```
We then export our ROS\_PACKAGE\_PATH, this tells the environment where our workspace is located and appends the default ROS\_PACKAGE\_PATH.
```
export ROS_PACKAGE_PATH=~/ros:${ROS_PACKAGE_PATH}
```
The next export is where our system will look for the ROS Master.
```
export ROS_MASTER_URI=http://guardian-wam:11311
```
This means that by default your ROS environment will have the Guardian PC as the ROS Master.

The following commented piece of code is included for when you do **not** want the ROS Master to run on the Guardian PC (ex. when you want to run a robot simulation).  By uncommenting this and commenting the previous command we are setting our ROS Master to the local PC.
```
#export ROS_MASTER_URI=http://localhost:11311
```


A handy command to view your ROS Environment variables at any time is:
```
env | grep ROS
```

For further information on ROS Environment Variables please visit: http://www.ros.org/wiki/ROS/EnvironmentVariables

For further information on ROS Networking please visit:
http://www.ros.org/wiki/ROS/NetworkSetup

### Secure Shell Guardian PC Communication ###

Another alternative for controlling the Guardian-WAM is using the onboard PC of the Guardian as the working machine.

Although this is not suggested, it is an alternative for those who do not want to install ROS on a local machine.

This is done by connecting the local PC to a network and accessing the Guardian PC remotely using a secure shell (ssh) client or remote terminal software.

**_Linux_**

To use this communication approach, you must have access to the Guardian-WAM Router.  For instructions on how to connect your computer to the Guardian-WAM router please follow the tutorial:

[Connecting External PC to Guardian-WAM Router](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMTutorialsConnectPCtoRouter)


You need to edit your **/etc/hosts** file with sudo permissions and add the following lines
```
192.168.2.101   guardian-wam
192.168.2.110   WAM
```

In a new terminal, log into the Guardian PC using secure shell client software, ex.:
```
ssh guardian-wam@guardian-wam
#username: guardian-wam
#password: guardian
```

Using the secure shell client you now have all the ability to control the Guardian-WAM using ROS.

It is suggested that for any programs you create are located in the
~/Sources workspace that is already configured on the Guardian PC.

To log into the WAM PC (PC-104):
```
ssh robot@WAM
#username: robot
#password: WAM
```

All of the WAM ROS Software is located in the ~/ros workspace.  Please do not edit anything in the ~/ros/electric folder.

Now that you have finished setting up and communicating with your Guardian-WAM please continue to: [gwam-ros-pkg Repository Installation Instructions.](http://code.google.com/p/gwam-ros-pkg/wiki/GWAMROSInstallation)