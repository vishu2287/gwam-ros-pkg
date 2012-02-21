/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <string.h>
#include <vector>
#include <stdint.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
//#include <serial_port/lightweightserial.h>
//#include <cereal_port/CerealPort.h>
#include "/home/guardian-wam/Sources/Guardian/svn_guardian/trunk/robot/SerialDevice/include/SerialDevice/SerialDevice.h"


namespace guardian_hw_iface{

#ifndef GUARDIAN_HARDWARE_INTERFACE_H
#define GUARDIAN_HARDWARE_INTERFACE_H

// conection stuff
#define GUARDIAN_HW_IFACE_DEFAULT_PORT 		        "/dev/ttyS0" //"/dev/ttyUSB0"
#define GUARDIAN_HW_IFACE_DEFAULT_PARITY 		"even"
#define GUARDIAN_HW_IFACE_DEFAULT_BAUDRATE 	        9600
#define GUARDIAN_HW_IFACE_DEFAULT_DATA_SIZE 	        7

// component frequency
#define GUARDIAN_HW_IFACE_DEFAULT_HZ                   	10		

// Default max speeds
#define GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED 	        0x7F            // hexa
#define GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_REF_SPEED	0x7F            //

// CONTROL MODE
#define GUARDIAN_HW_IFACE_CONTROL_RS232			1
#define GUARDIAN_HW_IFACE_CONTROL_RC			2

// INTERNAL ERROR CODES
#define GUARDIAN_HW_IFACE_ERROR_NONE			0
#define GUARDIAN_HW_IFACE_ERROR_OPENING			1
#define GUARDIAN_HW_IFACE_ERROR_SERIALCOMM		2
#define GUARDIAN_HW_IFACE_ERROR_TIMEOUT			3


// ODOMETRY PARAMETERS
#define GUARDIAN_COUNTS_PER_REV		     	    	1200		//2000            // =500 x 4 (in quadrature mode)

#define GUARDIAN_ENCODER_TO_REV			    	0.2941		//0.04            // gearbox 1:25

#define GUARDIAN_DIAMETER_WHEEL			    	0.24429		// m (0.24)
#define GUARDIAN_RPM2MPS			        0.012790994   	// conversion value from rpm to mps PI*Diameter/60
#define GUARDIAN_D_TRACKS_M                 		1.0		//1.24           	// set wheels B   GIR (w,z)

// ROBOT SPEED LIMITS
#define GUARDIAN_MAX_XSPEED         			1.00     	// [m/s]  - max speed of each track = (1800rpm/25/60)*pi*0.25=0.95<1.00
#define GUARDIAN_MAX_YAWSPEED       			1.65     	// [rad/s] w = (vr-vl)/GUARDIAN_D_TRACKS_M =(0.95+0.95)/ 1.15

#define GUARDIAN_HW_IFACE_SERIAL_DELAY		        5000		// us between serial transmisions to the GUARDIAN_HW_IFACE controller
#define GUARDIAN_HW_IFACE_ABSOLUTE_ENCODERS		1	        // from GUARDIAN_HW_IFACE.h
#define GUARDIAN_HW_IFACE_RELATIVE_ENCODERS		2	       	// -//-
#define GUARDIAN_HW_IFACE_MAX_ENC_INC                   1500            // maximal increment that could be read in 1 period = maximal speed

typedef struct _pose {
        double vx;
        double vy;
        double va;
        double px;
        double py;
        double pa;
} pose;

enum States{
        INIT_STATE,
        READY_STATE,
        FAILURE_STATE,
};

typedef struct _data {
        float voltage;
        float voltage_internal;     	// value used
        int temperature[2];         	// 2 heatsink temperatures
        float analog_input[2];      	// 2 analog inputs
        bool digital_output;        	// 1 digital output (real value)
        bool digital_output_setvalue; 	// digital output (set value)
        bool digital_input[2];     	// 2 digital inputs
	int encoder_left;           	// left encoder pulses
	int encoder_right;          	// right encoder pulses
	int last_encoder_left;      	// left encoder pulses
	int last_encoder_right;     	// right encoder pulses
	double v_ref_mps;	    	// current reference linear speed [mps]
	double w_ref_rads; 	    	// current reference angular speed [rads]
        double actSpeedLmps;        	// current speed left
        double actSpeedRmps;        	// current speed right
        double desSpeedLmps;        	// reference speed left
        double desSpeedRmps;        	// reference speed right
        double max_linear_speed;   	// maximal linear speed
        double max_angular_speed;  	// maximal angular speed
        bool bMotorsEnabled;       	// motors enabled or not
	int controlMode;		// current control mode
	States currentState;		// curent State (0: init, 1: ready, 2: failure)
} data;

//! Class to operate the guardian_hardware_interface servo driver
class guardian_hardware_interface{

    private:

        //! Mutex for controlling the changes and access to encoder values
        pthread_mutex_t mutex_encoders;
        //! Mutex for controlling the changes and access to encoder values
        pthread_mutex_t mutex_odometry;
	//! Class for the serial communication
	//SerialDevice *serial;
	SerialDevice::SerialDevice * serial;
        //! Establish encoder's data process (Absolute or Relative)
        int encoders_mode;
	//! Control mode of the robot (RS232, R/C)
	int control_mode;
	//! Odometry data
	pose robot_pose;
	//! Robot data
	data robot_data;
	//! Error counter (just for debug)
	int err_counter;
	//! Internal error code used to select the apropiate recovery action
	int iErrorType;

    public:

        //! Public constructor
        guardian_hardware_interface(double hz, const char* device) ;
        //! Public destructor
        ~guardian_hardware_interface();
        //! Switches motor power on/off
        void ToggleMotorPower(unsigned char val);
        //!	Returns robot velocity and pose
       void GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa);
       //! Resets driver odometry
        void ResetOdometry();
        //! Set new odometry value (pose)
        void ModifyOdometry(  double px,  double py,  double pa );
        //! Set robot left and right track references from linear and angular values
        void SetRobotSpeed(double lin_speed, double ang_speed);
        //! Set robot linear and angular speed limits
        void SetRobotSpeedLimits(double lin_speed, double ang_speed);
        //! Return the robot voltage value measured internally
        float GetVoltage();
        //! Return the last value measured of encoder 'L'eft or 'R'ight
        float GetEncoder(char enc);
        //! Return the heat sink temperature measurement
        int GetTemp(int index);
        //!	Return the selected analog input measurement
        float GetAnalogInput(int index);
        //!	Set the digital output to the selected value
        void SetDigitalOutput(bool value);
        //!	Get Digital Output
        bool GetDigitalOutput();
        //! Return value of selected digital input (0..2)
        bool GetDigitalInput(int input);
        //! Actions for the default mode ReadyState
        void ReadyState(int token);
	//! Get current State
	States GetCurrentState();

    private:

	
        //!Open serial ports (called by Component::Setup)
        int Open(); //ReturnValue Open();                                     
        //! Closes serial port
        int Close(); //ReturnValue Close();        
        //!	Switches on/off the motor
        void TogleMotorPower(int val);
        //! Read battery's charge
        void ReadBatteryVoltage();
        //!	Read the internal temperature
        void ReadTemperature();
        //!	Read the control mode
        void ReadControlMode();
        //!	Write the control mode (mixed/separated axes, open/closed loop)
        void WriteControlMode(int cm);
        //! brief Read analog inputs
        void ReadAnalogInputs();
        //! Read digital inputs
        void ReadDigitalInputs();
        //! Controller software reset
        void ResetController();
        //! Configure controller to accept RS232 commands
        void EnterRS232Mode();
        //! Read digital inputs
        int WriteDigitalOutput(bool value);
        //! Convert an hexadecimal value into a decimal value.
        int HexToDec(char *value, int size);
        //! Convert an hexadecimal value into a decimal value, using Ca2 of the hexadecimal number
        int HexToDec(char *value);
        //! Transform read analog value into temperature value
        int ValToHSTemp(int AnaValue);
        //! Read encoders value
        int ReadEncoders();
        //! Resets the encoders counter with response confirmation
        int ResetEncoders();
        //! Calculates RPM
        double CalculateRPM(double *rpm_left, double *rpm_right);
        //! Updates robot's odometry
        void UpdateOdometry();
        //! Writes motors speed references
        void WriteMotorSpeed(double speedL, double speedR);
        //! GUARDIAN_HW_IFACE configured AB mixed, closed loop
        void SetVWRef();
        //! Actions in Failure State
        void FailureState();
        //! Actions in InitState
        void InitState();

};

};


#endif
