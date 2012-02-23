/** \file guardian_hardware_interface.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2010
 *
 * \brief guardian_hardware_interface class driver
 * Component to manage guardian_hardware_interface servo driver
 * (C) 2010 Robotnik Automation, SLL
*/
#include "guardian_hardware_interface/guardian_hardware_interface.h"

#include <math.h>
#include <cstdlib>
#include <string>

#define LOG_STRING_LENGTH 	7
#define REC_BUFFER 			10

#define GUARDIAN_HW_ERROR 	-2
#define GUARDIAN_HW_OK 		0
#define USEC_PER_SEC    	1000000L


using namespace guardian_hw_iface;


/*!	\fn guardian_hardware_interface::guardian_hardware_interface(double hz)
 * 	\brief Public constructor
*/
guardian_hardware_interface::guardian_hardware_interface(double hz, const char* device) {

	// Initialization of odometry mutex
	pthread_mutex_init(& mutex_odometry, NULL);

	// Initialization of encoder mutex
	pthread_mutex_init(& mutex_encoders, NULL);

	// Creates serial device
	serial = new SerialDevice::SerialDevice(device, GUARDIAN_HW_IFACE_DEFAULT_BAUDRATE, GUARDIAN_HW_IFACE_DEFAULT_PARITY, GUARDIAN_HW_IFACE_DEFAULT_DATA_SIZE);
	
	

	this->control_mode = GUARDIAN_HW_IFACE_CONTROL_RS232; 				//CNTRL_RC Testing-> By default control will be Radio Control
	this->encoders_mode = GUARDIAN_HW_IFACE_ABSOLUTE_ENCODERS;
	this->err_counter = 0;                      					//Debug counts number of communication errors

	this->robot_data.last_encoder_left = 0; this->robot_data.encoder_left = 0;
	this->robot_data.last_encoder_right= 0; this->robot_data.encoder_right = 0;
	this->iErrorType = GUARDIAN_HW_IFACE_ERROR_NONE;
	
	// Configure commands according to driver encoders mode
	if (this->encoders_mode==GUARDIAN_HW_IFACE_ABSOLUTE_ENCODERS) {
	    if(GUARDIAN_HW_IFACE_ENCODER_CONF == 1){
            sprintf(cmdEncLeft, "?q0\r"); // absolute encoders
            sprintf(cmdEncRight, "?q1\r");
	    }else{
            sprintf(cmdEncLeft, "?q1\r"); // absolute encoders
            sprintf(cmdEncRight, "?q0\r");
	    }
	}else {
	    if(GUARDIAN_HW_IFACE_ENCODER_CONF == 1){
            sprintf(cmdEncLeft, "?q4\r"); // relative encoders
            sprintf(cmdEncRight, "?q5\r");
	    }else{
            sprintf(cmdEncLeft, "?q5\r"); // relative encoders
            sprintf(cmdEncRight, "?q4\r");
	    }
    }

	this->Open();
	// Enter serial communication mode
    EnterRS232Mode();
	WriteMotorControlMode(MCM_MIXED_CLOSED);	// Set control mode to CLOSED LOOP, MIXED & CONTROL VELOCITY

	//this->ReadControlMode();
	//ROS_INFO("ControlMode: %d\n", robot_data.controlMode);

    	//this->Setup();									
	//this->Start();

	this->InitState();								// Executes InitState()
	robot_data.currentState = READY_STATE;						// Set the robot to READY_STATE

}


/*!	\fn guardian_hardware_interface::~guardian_hardware_interface()
 * 	\brief Public destructor
*/
guardian_hardware_interface::~guardian_hardware_interface(){

	serial->Close();

	// Delete serial port
	if (serial!=NULL) delete serial;

    	// Destroy odometry mutex
	pthread_mutex_destroy(& mutex_odometry );

    	// Destroy encoders mutex
	pthread_mutex_destroy(& mutex_encoders );

}

/*!	\fn ReturnValue guardian_hardware_interface::Open()
 * 	\brief Open device
 * 	\returns GUARDIAN_HW_ERROR
 * 	\returns GUARDIAN_HW_OK
*/
int guardian_hardware_interface::Open(){

	ROS_INFO("guardian_hardware_interface::Open");
	// Open serial port
	int resOpen = serial->Setup();

    if (resOpen == SERIAL_ERROR){
		ROS_ERROR("guardian_hardware_interface::Open: Error on Setup");
		this->iErrorType = GUARDIAN_HW_IFACE_ERROR_OPENING;
		this->FailureState();
    	return GUARDIAN_HW_ERROR;
    }else{
		ROS_WARN("guardian_hardware_interface::Open:: OK");
		return GUARDIAN_HW_OK;
	}
}

/*!	\fn ReturnValue guardian_hardware_interface::Close()
 * 	\brief Closes serial port
 * 	\returns GUARDIAN_HW_ERROR
 * 	\returns GUARDIAN_HW_OK
*/
int guardian_hardware_interface::Close(){

	if (serial!=NULL) {
    	serial->Close();
    	serial->~SerialDevice();
    }

	return GUARDIAN_HW_OK;
}

/*!	\fn void guardian_hardware_interface::ReadBatteryVoltage()
	* Read battery's charge.
	* Send the query to the controller.
	* Command = ?e o ?E
*/
void guardian_hardware_interface::ReadBatteryVoltage(){
	char cRecBuffer[REC_BUFFER]="";
	memset(cRecBuffer, 0, REC_BUFFER);
	int written_bytes=0;
	if(serial->WritePort("?e\r",&written_bytes, 3) != SERIAL_OK){
	//if(serial->WritePort((char*)"?e\r", 3) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::ReadBatteryVoltage: Error sending message");
        }

	// read response from ReadBatteryVoltage()
	char c;
	char buf[8];
	int n=0;			//Number of received bytes
	int j=0;			//Tokens' counter
	int i=0;
	memset(cRecBuffer, 0, REC_BUFFER);
	bool bBat=false, bBat1=false, bEndBat=false;

	memset(buf,0,8);
	while (!( bEndBat ) && j<100000) {	//50000
		j++;
                //ROS_ERROR("Abans del serial->ReadPort de ReadBatteryVoltage()");
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c, &n, 1);
		if(n>0){
			if (c!='\r') {
				buf[i]=c;
				i++;
                	} else { // return
				if (i>0) {
					if (bBat1) { // Process GUARDIAN_HW_IFACE internal voltage
						robot_data.voltage_internal = (float)HexToDec(buf,2)*0.111328125;
						bEndBat = true;
						bBat1 = false;
					} else
						if (bBat) { // Process batt voltage
							robot_data.voltage = (float)HexToDec(buf,2)*0.210347828;//0.21484375;
							bBat = false;
							bBat1 = true;
					} else if (!strcmp(buf,"?e")) bBat = true;
                	}
				i=0;
				memset(buf,0,8);
			}
		}
	}

	if(j==100000){  //50000
		printf("ErrCount: %d\n",this->err_counter);
		this->err_counter++;
		usleep(200 * GUARDIAN_HW_IFACE_SERIAL_DELAY);
	} else {
		usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
	}
}

/*!	\fn void void guardian_hardware_interface::ReadTemperature()
 * 	\brief Read the internal temperature.
 *  Send the query to the controller.
 *  Command = ?m o ?M
*/
void guardian_hardware_interface::ReadTemperature(){

    	int written_bytes=0;
	if(serial->WritePort((char*)"?M\r",&written_bytes,3) != GUARDIAN_HW_OK){
		puts("guardian_hardware_interface::ReadTemperature: Error sending message");
		ROS_ERROR("guardian_hardware_interface::ReadTemperature: Error sending message");
        }

    	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	// read response from GUARDIAN_HW_IFACE - temperatures
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    	char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                    robot_data.temperature[0]=ValToHSTemp(HexToDec(&buf[2],2));
                    robot_data.temperature[1]=ValToHSTemp(HexToDec(&buf[4],2));
                    //printf("Temperatures: (1)%d    (2)%d\n ", robot_data.temperature[0], robot_data.temperature[1] );
                }
            }
        }
	}

	if(j==50000){
		ROS_ERROR("guardian_hardware_interface::ReadTemperature: {%d} ErrData ",j);
		this->err_counter++;
    }
}

/*!	\fn void void guardian_hardware_interface::ReadControlMode()
 * 	\brief Read the control mode
 *  Send the query to the controller.
 *  Command = ^01
*/
void guardian_hardware_interface::ReadControlMode(){ 

//    	int written_bytes=0;

	/*if(serial->WritePort((char*)"^01\r", 4) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::ReadControlMode: Error sending message");
        }
	
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	// read response from ReadControlMode
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    	char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		n = serial->ReadPort(&c, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				printf("buf[%d]: %c\n", i, c);
				i++;
				if (i>=4) {
				    bEnd=true;
                    robot_data.controlMode=HexToDec(&buf[3]);
                }
            }
        }
	}

	if(j==50000){
		printf("{%d} ErrData ",j);
		this->err_counter++;
        }
*/

	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8], mode[3]="\0";
	memset(buf,0,8);
    int written_bytes=0;

	if(serial->WritePort((char*)"^01\r",&written_bytes, 4) != SERIAL_OK){
		ROS_ERROR("guardian_hardware_interface::ReadMotorControlMode: Error sending message");
    }
	
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	while (!( bEnd ) && j<5000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                }
            }
        }
	}

    if(bEnd){
        // Copy the value of the mode from the response
        mode[0] = buf[3];
        mode[1] = buf[4];
        //printf("ax3500::ReadMotorControlMode: mode = %s\n", mode);
        if(!strcmp(mode, "00")){
            motor_control_mode = MCM_SEPARATED_OPEN;
            printf("guardian_hardware_interface::ReadMotorControlMode: MCM_SEPARATED_OPEN\n");
        }else if(!strcmp(mode, "C5")){
            motor_control_mode = MCM_MIXED_CLOSED;
           	printf("guardian_hardware_interface::ReadMotorControlMode: MCM_MIXED_CLOSED\n");
        }else if(!strcmp(mode, "01")){
            motor_control_mode = MCM_MIXED_OPEN;
            printf("guardian_hardware_interface::ReadMotorControlMode: MCM_MIXED_OPEN\n");
        }
    }
	if(j> 5000){
		ROS_ERROR("guardian_hardware_interface::ReadMotorControlMode: {%d} ErrData ",j);
	}
}

/*!	\fn void void guardian_hardware_interface::WriteControlMode(int mode)
 * 	\brief Set the control mode
 *  Send the query to the controller.
 *  Command = ^01 C5 : mixed mode - open loop
              ^01 01 : mixed mode - open loop
              ^01 00 : separate mode - open loop
	      ^01 05 : mixed mode - closed loop - velocity control
*/
/*void guardian_hardware_interface::WriteControlMode(int mode){

	int written_bytes=0;
	switch (mode){

		case 5: if(serial->WritePort("^01 05\r", 7) == GUARDIAN_HW_ERROR){ //mix mode, closed loop, velocity control
	    			ROS_ERROR("guardian_hardware_interface::WriteControlMode: Error sending message.");
        		}
			usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
			robot_data.controlMode = 05;
			ROS_INFO("guardian_hardware_interface::WriteControlMode: Control Mode: MIX MODE, CLOSED LOOP, VELOCITY CONTROL.");
			break;

	  	default: if(serial->WritePort("^01 00\r", 7) == GUARDIAN_HW_ERROR){ //separate mode, open loop
	    			ROS_ERROR("guardian_hardware_interface::WriteControlMode: Error sending message.");
        		}
			usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
			robot_data.controlMode = 00;
			ROS_INFO("guardian_hardware_interface::WriteControlMode: Control Mode: SEPARATE MODE, OPEN LOOP.");
			break;

	}
	if(serial->WritePort("^FF\r", 4) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::WriteControlMode: Error sending message 2");
        }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
}
*/

/*!	\fn void guardian_hardware_interface::WriteMotorControlMode(MotorControlMode mcm)
 * 	\brief Set the control mode
 *  Send the query to the controller.
 *  Command = ^01 C5 : mixed mode - closed loop
              ^01 01 : mixed mode - open loop
              ^01 00 : separate mode - open loop
*/
void guardian_hardware_interface::WriteMotorControlMode(MotorControlMode mcm){
    int written_bytes=0;
    char cCommand[8] = "\0";
    std::string sMode("");
	int wait_cycles = 50000;

    // Create the string depending on the selected command
    switch(mcm){
        case MCM_MIXED_OPEN:
            strcpy(cCommand, "^01 01\r");
            sMode.assign("Mixed mode + open loop");
        break;
        case MCM_MIXED_CLOSED:
            strcpy(cCommand, "^01 C5\r");
            sMode.assign("Mixed mode + closed loop");
        break;
        case MCM_SEPARATED_OPEN:
            strcpy(cCommand, "^01 00\r");
            sMode.assign("Separated mode + open loop");
        break;
        case MCM_SEPARATED_CLOSED:
            strcpy(cCommand, "^01 C4\r");
            sMode.assign("Separated mode + open loop");
        break;
    }
	
    // Sends the command to configure the mode
    if(serial->WritePort(cCommand, &written_bytes, 7) == GUARDIAN_HW_ERROR){ //separate mode, open loop
        ROS_ERROR("guardian_hardware_interface::WriteMotorControlMode: Error setting %s", sMode.c_str());	    
	    return;
    }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
	// read response from AX3500 - command !
	bool bEnd=false;
	int j=0,n=0;
	char c;
	while (!( bEnd ) && j<wait_cycles) {	// wait until read '+'
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                }
            }
        }
	}

	if(j>=wait_cycles){
	    ROS_ERROR("guardian_hardware_interface::WriteMotorControlMode: Error reading response");
	   
		//printf("{%d} ErrData ",j);
		this->err_counter++;
		return;
    }

    // Sends the command to reset the device
	if(serial->WritePort((char*)"^FF\r",&written_bytes, 4) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::WriteMotorControlMode: Error reseting the device after configure %s", sMode.c_str());
	    
		return;
    }

	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
    bEnd = false;
    j= n = 0;
	while (!( bEnd ) && j<wait_cycles) {	// wait until read '+'
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                }
            }
        }
	}

	if(j>=wait_cycles){
	    ROS_ERROR("guardian_hardware_interface::WriteMotorControlMode: Error reading response after reseting");
		//printf("{%d} ErrData ",j);
		this->err_counter++;
		return;
    }

	ROS_INFO("guardian_hardware_interface::WriteMotorControlMode: configured succesfully on %s", sMode.c_str());
}



/*!	\fn void guardian_hardware_interface::ReadAnalogInputs()
 * 	\brief Read analog inputs
 * 	Send the query to the controller.
 * 	Command = ?p o ?P
*/
void guardian_hardware_interface::ReadAnalogInputs(){

	// send request to GUARDIAN_HW_IFACE, analog inputs
	int written_bytes=0;
	if(serial->WritePort((char*)"?P\r",&written_bytes, 3) != GUARDIAN_HW_OK){
	//if(serial->WritePort("?P\r", 3) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::ReadAnalogInputs: Error sending message");
        }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	// read response from GUARDIAN_HW_IFACE - analog input
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
        char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                    int aux1=HexToDec(&buf[2],2);
                    if(aux1==255) aux1=-127;
                    else if(aux1>127) aux1=127-aux1;
                    int aux2= HexToDec(&buf[4],2);
                    if(aux2==255) aux2=-127;
                    else if(aux2 > 127) aux2 = 127 - aux2;
                    //-127=0V, 0=2.5V, 127= 5V
                    // Lineal interpolation-> m= 0.019685039, b= 2.5
                    float m=0.019685039;
                    float b=2.5;
                    robot_data.analog_input[0] = m*(float) aux1+b;
                    robot_data.analog_input[1] = m*(float) aux2+b;
                    //printf("Anlog Inputs: (1)%f    (2)%f\n ", robot_data.analog_input[0], robot_data.analog_input[1]);
                }
            }
        }
	}

	if(j==50000){
		printf("{%d} ErrData ",j);
		this->err_counter++;
        }
}

/*!	\fn void guardian_hardware_interface::ReadDigitalInputs()
 * 	\brief Read digital inputs
 * 	Send the query to the controller.
 * 	Command = ?i o ?I
*/
void guardian_hardware_interface::ReadDigitalInputs(){

	int written_bytes=0;
	if(serial->WritePort("?I\r",&written_bytes, 3) != SERIAL_OK) {
	//if(serial->WritePort((char*)"?I\r", 3) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::ReadDigitalInputs: Error sending message");
     }

	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	// read response from GUARDIAN_HW_IFACE - analog input
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    	char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=8) {
				    bEnd=true;
                    //int aux1=HexToDec(&buf[2],2);
                    //int aux2=HexToDec(&buf[4],2);
                    //int aux3=HexToDec(&buf[6],2);
                    robot_data.digital_input[0] = (bool) HexToDec(&buf[2],2);
                    robot_data.digital_input[1] = (bool) HexToDec(&buf[4],2);
                    robot_data.digital_input[2] = (bool) HexToDec(&buf[6],2);
                    //printf("Anlog Inputs: (1)%f    (2)%f\n ", robot_data.analog_input[0], robot_data.analog_input[1]);
                }
            }
        }
	}

	if(j==50000){
		ROS_ERROR("guardian_hardware_interface::ReadDigitalInputs: {%d} ErrData ",j);
		this->err_counter++;
    }

}


/*!	\fn void guardian_hardware_interface::ResetController()
 * 	\brief Allows the controller to be reset in the same manner as if the reset button
 * 	was pressed.
 *  Command = %rrrrrr
*/
void guardian_hardware_interface::ResetController(){

	int written_bytes=0;

	WriteMotorSpeed(0.0,0.0); // Stop Motors

	ToggleMotorPower('0');  // Set flag to motors disabled

	if(serial->WritePort("%rrrrrr\r",&written_bytes, 8) != GUARDIAN_HW_OK){
	//if(serial->WritePort((char*)"%rrrrrr\r", 8) == GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::ResetController: Error sending message");
    }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
}

/*!	\fn void guardian_hardware_interface::EnterRS232Mode()
 * 	\brief If the controller is configured in R/C or Analog mode, it will not be able to accept and recognize
 * 	RS232 commands immediately. It will enter the serial mode after it has received 10 continuous
 * 	“Enter” (Carriage Return) characters
*/
void guardian_hardware_interface::EnterRS232Mode(){
	int written_bytes=0;
	for(int i=0; i < 10; i++)	{
		if(serial->WritePort("\r",&written_bytes, 1) != GUARDIAN_HW_OK)	{
		//if(serial->WritePort((char*)"\r", 1) == GUARDIAN_HW_ERROR) {
			ROS_ERROR("guardian_hardware_interface::EnterRS232Mode: Error sending message");
        }
		usleep(2*GUARDIAN_HW_IFACE_SERIAL_DELAY);
	}
	//serial->WritePort("^01 00\r", &written_bytes, 7 );
	//usleep(2*GUARDIAN_HW_IFACE_SERIAL_DELAY); 
	ROS_INFO("guardian_hardware_interface::EnterRS232Mode: Entering RS232 mode");
}

/*!	\fn int guardian_hardware_interface::WriteDigitalOutput(bool value)
 * 	\brief Set the digital output into the selected value.
 *		 Initial configuration: output 1..4 managed by usbdux device
 *								output 5 managed by GUARDIAN_HW_IFACE board
 * 	\param bit as int, output number from 1 to DIGITAL_OUTPUTS
 * 	\param value as int, output value
 * 	\return GUARDIAN_HW_ERROR if the output can't be set
 * 	\return GUARDIAN_HW_OK
*/
int guardian_hardware_interface::WriteDigitalOutput(bool value){

    int written_bytes=0;

    if(value) { // GUARDIAN_HW_IFACE Command= !C (true) or !c (false)
    if(serial->WritePort((char*)"!C\r",&written_bytes, 3) != SERIAL_OK){
	//if(serial->WritePort((char*)"!C\r", 3) == GUARDIAN_HW_ERROR){
	    ROS_ERROR("guardian_hardware_interface::WriteDigitalOutput: Error sending message");
            return GUARDIAN_HW_ERROR;
        }
    }else {
        if(serial->WritePort((char*)"!c\r",&written_bytes, 3) != GUARDIAN_HW_OK){
		//if(serial->WritePort((char*)"!c\r", 3) == GUARDIAN_HW_ERROR){
	    ROS_ERROR("guardian_hardware_interface::WriteDigitalOutput: Error sending message");
            return GUARDIAN_HW_ERROR;
        }
    }

	// read response from GUARDIAN_HW_IFACE - command !
	bool bEnd=false;
	int j=0,n=0;
	char c;
	while (!( bEnd ) && j<5000) {	// wait until read '+'
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                  robot_data.digital_output = value;
                }
            }
        }
	}

	if(j==5000){
		printf("{%d} ErrData ",j);
		this->err_counter++;
        }

	return GUARDIAN_HW_OK;
}

/*!	\fn int guardian_hardware_interface::HexToDec(char *value, int size)
	* Convert an hexadecimal value into a decimal value.
	* @param value as char *, is an string with the value
	* @param size as integer, is the size of the string
	* return -1 if error
*/
int guardian_hardware_interface::HexToDec(char *value, int size){
	int aux = 0;
	int aux_value;

	for(int i=0; i<size; i++) {
		//printf("value[%d]=%c   ", i, value[i]);
		aux_value = 0;
		switch(value[i]) { //transforms each character into an integer
			case '0':
				aux_value = 0;
			break;
			case '1':
				aux_value = 1;
			break;
			case '2':
				aux_value = 2;
			break;
			case '3':
				aux_value = 3;
			break;
			case '4':
				aux_value = 4;
			break;
			case '5':
				aux_value = 5;
			break;
			case '6':
				aux_value = 6;
			break;
			case '7':
				aux_value = 7;
			break;
			case '8':
				aux_value = 8;
			break;
			case '9':
				aux_value = 9;
			break;
			case 'A':
				aux_value = 10;
			break;
			case 'B':
				aux_value = 11;
			break;
			case 'C':
				aux_value = 12;
			break;
			case 'D':
				aux_value = 13;
			break;
			case 'E':
				aux_value = 14;
			break;
			case 'F':
				aux_value = 15;
			break;
			default:
				return -1;
			break;
		}

		aux+= (int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
	}

	return aux;
}

/*!	\fn int guardian_hardware_interface::HexToDec(char *value)
	* Convert an hexadecimal value into a decimal value, using Ca2 of the hexadecimal number
	* @param value as char *, is an string with the value
	* return converted value
*/
int guardian_hardware_interface::HexToDec(char *value){
	int aux = 0;
	int aux_value=0;
	int signal = 1;
	int size = 0;

	for(int i=0; i<8; i++) {//Calculate the size of the number
		switch(value[i]) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case 'A':
			case 'B':
			case 'C':
			case 'D':
			case 'E':
			case 'F':
				size++;
			break;
			default:
				i=8;
			break;
		}
	}

	// Is a negative number? More Significant Bit == 1 -> Negative
	if((value[0]=='F') || (value[0]=='E') || (value[0]=='D') || (value[0]=='C') || (value[0]=='B')
		|| (value[0]=='A') || (value[0]=='9') || (value[0]=='8'))
		signal = -1;

	if(signal==-1) { //If it's negative
		for(int i=0; i<size; i++) { //Apply Ca1
			aux_value = 0;
			switch(value[i]) { //transforms each character into an integer
				case '0':
					aux_value = 15;
				break;
				case '1':
					aux_value = 14;
				break;
				case '2':
					aux_value = 13;
				break;
				case '3':
					aux_value = 12;
				break;
				case '4':
					aux_value = 11;
				break;
				case '5':
					aux_value = 10;
				break;
				case '6':
					aux_value = 9;
				break;
				case '7':
					aux_value = 8;
				break;
				case '8':
					aux_value = 7;
				break;
				case '9':
					aux_value = 6;
				break;
				case 'A':
					aux_value = 5;
				break;
				case 'B':
					aux_value = 4;
				break;
				case 'C':
					aux_value = 3;
				break;
				case 'D':
					aux_value = 2;
				break;
				case 'E':
					aux_value = 1;
				break;
				case 'F':
					aux_value = 0;
				break;
			}

			aux+=(int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
		}
		aux+=1;//Ca1 + 1 = Ca2
		aux=-aux;
	}else {	//Positive
		for(int i=0; i<size; i++) {
			//printf("value[%d]=%c   ", i, value[i]);
			aux_value = 0;
			switch(value[i]) {//transforms each character into an integer
				case '0':
					aux_value = 0;
				break;
				case '1':
					aux_value = 1;
				break;
				case '2':
					aux_value = 2;
				break;
				case '3':
					aux_value = 3;
				break;
				case '4':
					aux_value = 4;
				break;
				case '5':
					aux_value = 5;
				break;
				case '6':
					aux_value = 6;
				break;
				case '7':
					aux_value = 7;
				break;
				case '8':
					aux_value = 8;
				break;
				case '9':
					aux_value = 9;
				break;
				case 'A':
					aux_value = 10;
				break;
				case 'B':
					aux_value = 11;
				break;
				case 'C':
					aux_value = 12;
				break;
				case 'D':
					aux_value = 13;
				break;
				case 'E':
					aux_value = 14;
				break;
				case 'F':
					aux_value = 15;
				break;
			}
			aux+= (int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
		}
	}

	return aux;
}

/*!	\fn int guardian_hardware_interface::ValToHSTemp(int AnaValue)
	* Transform read analog value into temperature value
*/
int guardian_hardware_interface::ValToHSTemp(int AnaValue){
	// Interpolation table. Analog readings at -40 to 150 oC, in 5o intervals
	int TempTable[39] ={248, 246, 243, 240, 235, 230, 224, 217, 208, 199, 188, 177,
						165, 153, 140, 128, 116, 104,93, 83, 74, 65, 58, 51, 45, 40, 35, 31, 27, 24, 21,
						19, 17, 15, 13, 12, 11, 9, 8};
	int LoTemp, HiTemp, lobound, hibound, temp, i;
	i = 38;

	while (TempTable[i] < AnaValue && i > 0) i--;

    if (i < 0) i = 0;

    if (i == 38)
        return 150;
    else {
        LoTemp = i * 5 - 40;
        HiTemp = LoTemp + 5;
        lobound = TempTable[i];
        hibound = TempTable[i+1];
        temp = LoTemp + (5 * ((AnaValue - lobound)*100/ (hibound - lobound)))/100;
        return temp;
    }
}

/*!	\fn void guardian_hardware_interface::ReadEncoders()
	* Read encoders value
*/
/*int guardian_hardware_interface::ReadEncoders(){
	int encL=0,encR=0;
	char c;
	char cRecBuffer[REC_BUFFER]="";
	char buf[8];
	int n=0;			//Number of received bytes
	int j=0;			//Tokens' counter
	int k=0;
	int i=0;
	memset(cRecBuffer, 0, REC_BUFFER);
	bool bQ0=false, bQ1=false, bEndQ0=false, bEndQ1=false;
    	int written_bytes=0;
    	char cmd1[8];
    	char cmd2[8];


	// Configure commands according to driver encoders mode
	if (this->encoders_mode==GUARDIAN_HW_IFACE_ABSOLUTE_ENCODERS) {
	    sprintf(cmd1, "?q0\r"); // absolute encoders
	    sprintf(cmd2, "?q1\r");
        }else {
	    sprintf(cmd1, "?q4\r"); // relative encoders
	    sprintf(cmd2, "?q5\r");
        }

	// send request to GUARDIAN_HW_IFACE, encoder0
    	if(serial->WritePort(cmd1,&written_bytes, 4) != GUARDIAN_HW_OK) {
   	//if(serial->WritePort(cmd1, 4) == GUARDIAN_HW_ERROR) {
		ROS_ERROR("guardian_hardware_interface::ReadEncoders: Error sending message for encoder 1");
        }

	// read response from GUARDIAN_HW_IFACE - encoder0
	memset(buf,0,8);
	while (!( bEndQ0 ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c, &n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
                }
			else { // return
				if (i>0) {
					if (bQ0) { // Process q0 data
						pthread_mutex_lock(&mutex_encoders);
							encR = HexToDec(buf);
							this->robot_data.encoder_right = encR;//-HexToDec(buf);
						pthread_mutex_unlock(&mutex_encoders);
						bEndQ0 = true;
						bQ0 = false;
					} else if ( !strcmp(buf,"?q0") || !strcmp(buf,"?q4") ) bQ0 = true;
				}
			i=0;
			memset(buf,0,5);
            }
        }
    }
	if(j==50000){
		printf("{%d} ErrData ",j);
		this->err_counter++;
        }

	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	if(serial->WritePort(cmd2, &written_bytes, 4) != GUARDIAN_HW_OK) {
	//if(serial->WritePort(cmd2, 4) == GUARDIAN_HW_ERROR) {
		ROS_ERROR("guardian_hardware_interface::ReadEncoders: Error sending message for encoder 2");
        }

	// read response from GUARDIAN_HW_IFACE - encoder1
	n=0; j=0; k=0; i=0;

	memset(buf,0,8);
	while (!( bEndQ1 ) && j<50000) {	// wait until read enc1 - Q1
		j++;
		//n = serial->ReadPort(&c, 1);
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received

			if (c!='\r') {
				buf[i]=c;
				i++;
                         }
			else { // return
				if (i>0) {
					if (bQ1) { // Process q1 data
						pthread_mutex_lock(&mutex_encoders);
							encL = HexToDec(buf);
							this->robot_data.encoder_left = encL;
						pthread_mutex_unlock(&mutex_encoders);
						bEndQ1 = true;
						bQ1 = false;
					} else if (!strcmp(buf,"?q1") || !strcmp(buf,"?q5") ) bQ1 = true;
				}
			i=0;
			memset(buf,0,5);
			}
		}
	}

	if(j==50000){
		printf("{%d} ErrData ",j);
		this->err_counter++;
	}

	if (bEndQ0 && bEndQ1){
		return GUARDIAN_HW_OK;
	}else{
		ROS_ERROR("-*****************************-READ ENCODERS FAILED-************************************-");
		return GUARDIAN_HW_ERROR;
	}
}
*/

/*!	\fn void guardian_hardware_interface::ReadEncoders()
	* Read encoders value
*/
int guardian_hardware_interface::ReadEncoders(){
	int encL=0,encR=0;
	char c;
	char cRecBuffer[REC_BUFFER]="";
	char buf[8];
	int n=0;			//Number of received bytes
	int j=0;			//Tokens' counter
	int k=0;
	int i=0;
	memset(cRecBuffer, 0, REC_BUFFER);
	bool bQ0=false, bQ1=false, bEndQ0=false, bEndQ1=false;
    int written_bytes=0;
	int wait_cycles = 50000;
	
	// send request to AX3500, right encoder
    if(serial->WritePort(cmdEncRight, &written_bytes, 4) != GUARDIAN_HW_OK) {
        ROS_ERROR("guardian_hardware_interface::ReadEncoders: Error sending message for encoder right");
    }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
	// read response from AX3500 - encoder0
	memset(buf,0,8);
	//printf("ENC_R:\n");
	while (!( bEndQ0 ) && j<wait_cycles) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				//printf("%c", c);
            }else { // return
				//if(i>=3)
					//printf("[%s]", buf);
				
				if (bQ0) { // Process q0 data
					pthread_mutex_lock(&mutex_encoders);
						encR = HexToDec(buf);
						this->robot_data.encoder_right = GUARDIAN_HW_IFACE_ENCODER_DIR * encR;//-HexToDec(buf);
					pthread_mutex_unlock(&mutex_encoders);
					bEndQ0 = true;
					bQ0 = false;
				} else if ( !strcmp(buf,"?q0") || !strcmp(buf,"?q1") || !strcmp(buf,"?q4") || !strcmp(buf,"?q5") )
                    bQ0 = true;
				
				i=0;
				memset(buf,0,8);
            }
        }
    }
	if( (j>= wait_cycles) ){
		ROS_ERROR("guardian_hardware_interface::ReadEncoders:(Right) {%d} ErrData ",j);
		//this->err_counter++;
    }
	
	//usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	if(serial->WritePort(cmdEncLeft, &written_bytes, 4) != GUARDIAN_HW_OK) {
		ROS_ERROR("guardian_hardware_interface::ReadEncoders: Error sending message for encoder left");
    }
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
	// read response from AX3500 - encoder1
	n=0; j=0; k=0; i=0;

	memset(buf,0,8);
	//printf("\nENC_L:\n");
	while (!( bEndQ1 ) && j<wait_cycles) {	// wait until read enc1 - Q1
		j++;
        serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received

			if (c!='\r') {
				buf[i]=c;
				i++;
				//printf("%c", c);
           	}
			else{ // return
				//if(i>=3)
					//printf("[%s]", buf);
				if (bQ1) { // Process q1 data
					pthread_mutex_lock(&mutex_encoders);
						encL = HexToDec(buf);
						this->robot_data.encoder_left = GUARDIAN_HW_IFACE_ENCODER_DIR * encL;
					pthread_mutex_unlock(&mutex_encoders);
					bEndQ1 = true;
					bQ1 = false;
				} else if ( !strcmp(buf,"?q0") || !strcmp(buf,"?q1") || !strcmp(buf,"?q4") || !strcmp(buf,"?q5") )
                    bQ1 = true;
				
				i=0;
				memset(buf,0,8);
			}
		}
	}
	//printf("\n");

	if( (j>=wait_cycles) ){
		ROS_ERROR("guardian_hardware_interface::ReadEncoders: (Left) {%d} ErrData ",j);
		//this->err_counter++;
	}
	//usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
	if (bEndQ0 && bEndQ1){
		//ROS_INFO("guardian_hardware_interface:ReadEncoders: L = %d, R = %d", this->robot_data.encoder_left, this->robot_data.encoder_right);
		return GUARDIAN_HW_OK;
	}else{
		ROS_ERROR("-*****************************-READ ENCODERS FAILED-************************************-");
		return GUARDIAN_HW_ERROR;
	}
}



/*! \fn void guardian_hardware_interface::ResetEncoders()
 *  \brief Resets the encoders counter with response confirmation
 *	\return GUARDIAN_HW_OK
 *	\return GUARDIAN_HW_ERROR
*/
int guardian_hardware_interface::ResetEncoders(){
	char cSendBuffer[5]="!q2\r";
	int j = 0, n = 0;
	char c;
	int counts = 200;
	int timer = 2*GUARDIAN_HW_IFACE_SERIAL_DELAY;
	bool bEnd = false;
	int attempts = 0, max_attempts = 5;
	int written_bytes=0;

	// Send command !q2
	while(!bEnd && (attempts < max_attempts)){	// Todos los mensajes de configuración se repetirán hasta que se reciban todas las respuestas de validación (+)
		
 		if(serial->WritePort(cSendBuffer, &written_bytes, 5) == SERIAL_ERROR) {
			ROS_ERROR("guardian_hardware_interface::ResetEncoders: Error sending %s to the controller", cSendBuffer);
			return GUARDIAN_HW_ERROR;
		}
		usleep(timer);

		j=0;
		serial->ReadPort(&c, &n, 1);
		while(c!='+' && j < counts){	//Wait for confirmation
			j++;
			serial->ReadPort(&c, &n, 1);
        }
		//if(c!= '\r') // printf("ax3500::ResetEncoders: c= %c, counts=%d\n", c, j);
		if(c =='+'){
			ROS_INFO("guardian_hardware_interface::ResetEncoders: command OK");
			bEnd = true;
		}
		attempts++;
		
		
	/*	j=0;
		serial->ReadPort(&c, &n, 1);
		while(c!='+' && j < counts){	//Wait for confirmation
			j++;
			serial->ReadPort(&c, 1);
            	}
		//if(c!= '\r') // printf("guardian_hardware_interface::ResetEncoders: c= %c, counts=%d\n", c, j);
		if(c =='+'){
			ROS_INFO("guardian_hardware_interface::ResetEncoders: OK");
			bEnd = true;
		}
		attempts++;*/
	}

	if(attempts >= max_attempts){
			ROS_ERROR("guardian_hardware_interface::ResetEncoders: Error: %d attempts to reset the encoders", attempts);
		return GUARDIAN_HW_ERROR;
	}

	pthread_mutex_lock(&mutex_encoders);
		this->robot_data.encoder_left = 0;
		this->robot_data.encoder_right = 0;
		this->robot_data.last_encoder_left = 0;
        this->robot_data.last_encoder_right = 0;
	pthread_mutex_unlock(&mutex_encoders);

	return GUARDIAN_HW_OK;
}


/*!	\fn double guardian_hardware_interface::CalculateRPM()
	* Calculates RPM
	* @return sample period in seconds
*/
double guardian_hardware_interface::CalculateRPM(double *rpm_left, double *rpm_right){
	static bool bfirst = true;
	//static struct timespec now, last;
	ros::Duration diff;
	static ros::Time current_time, last_time;
	int inc_left_counts = 0, inc_right_counts = 0;
	double secs=0.0;
	double rev_per_min_left = 0.0, rev_per_min_right=0.0;

	if(this->encoders_mode == GUARDIAN_HW_IFACE_ABSOLUTE_ENCODERS){
        pthread_mutex_lock(&mutex_encoders);
            inc_left_counts = robot_data.encoder_left - robot_data.last_encoder_left;
            inc_right_counts = robot_data.encoder_right - robot_data.last_encoder_right;
            robot_data.last_encoder_left = robot_data.encoder_left;
            robot_data.last_encoder_right = robot_data.encoder_right;
        pthread_mutex_unlock(&mutex_encoders);
        }
    	else {
    	// RELATIVE ENCODERS
        pthread_mutex_lock(&mutex_encoders);

            inc_left_counts = robot_data.encoder_left;
            inc_right_counts = robot_data.encoder_right;

			if(abs(inc_left_counts) > GUARDIAN_HW_IFACE_MAX_ENC_INC) {	// Check wrong data
				ROS_ERROR("guardian_hardware_interface::CalculateRPM:Relative: error reading left encoder: inc = %d\n", inc_left_counts);
				inc_left_counts = robot_data.last_encoder_left;
			}else{

				robot_data.last_encoder_left = robot_data.encoder_left;
            }

			if(abs(inc_right_counts) > GUARDIAN_HW_IFACE_MAX_ENC_INC){	//En caso de que llegase un dato anómalo
				ROS_ERROR("guardian_hardware_interface::CalculateRPM:Relative: error reading right encoder: inc = %d\n", inc_right_counts);
				inc_right_counts = robot_data.last_encoder_right;
			}else{

				robot_data.last_encoder_right = robot_data.encoder_right;
			}

			robot_data.encoder_left = 0;
			robot_data.encoder_right = 0;

        pthread_mutex_unlock(&mutex_encoders);
        }

	// clock_gettime(GetClock(), &now);
	current_time = ros::Time::now();

	if(bfirst) {	//First call
		bfirst = false;
		ROS_WARN("bfirst value changed!");
		last_time = current_time;
		return -1.0;
        }

	//Normalize in sec.ns
	//tsnorm(&now);
	//tsnorm(&last);

	//diff = calcdiff(now,last);
	//ROS_WARN("Current time: %f", current_time.toSec());
	//ROS_WARN("Last time: %f", last_time.toSec());
	diff = current_time - last_time; 
	//secs = (double)diff/(double)USEC_PER_SEC;
	secs = diff.toSec();
	//ROS_WARN("secs: %f", secs);
	//2000 0.04 (gear 1:25)
	*rpm_left= rev_per_min_left = ((inc_left_counts/secs)/GUARDIAN_COUNTS_PER_REV)*60.0*GUARDIAN_ENCODER_TO_REV;
	//ROS_ERROR("*RPM_LEFT: %d", ((inc_left_counts/secs)/GUARDIAN_COUNTS_PER_REV)*60.0*GUARDIAN_ENCODER_TO_REV);
	*rpm_right = rev_per_min_right = ((inc_right_counts/secs)/GUARDIAN_COUNTS_PER_REV)*60.0*GUARDIAN_ENCODER_TO_REV;
	//ROS_ERROR("*RPM_RIGHT: %d", ((inc_right_counts/secs)/GUARDIAN_COUNTS_PER_REV)*60.0*GUARDIAN_ENCODER_TO_REV);

	last_time = current_time;
	// ROS_WARN("New last_time for the next iteration: %f", last_time.toSec());
	return secs;
}


/*!	\fn void guardian_hardware_interface::UpdateOdometry()
	* Updates robot's odometry
*/
void guardian_hardware_interface::UpdateOdometry(){
	double fVelocityLeftRpm=0.0;
	double fVelocityRightRpm=0.0;
	double v_left_mps, v_right_mps;
	double fSamplePeriod = 0.0; // Default sample period
	double linearSpeedMps=0.0, angularSpeedRads= 0.0;

	fSamplePeriod = CalculateRPM(&fVelocityLeftRpm,&fVelocityRightRpm);


	if(fSamplePeriod < 0.0){
		//ROS_ERROR("Entra per fSamplePeriod < 0.0");
		return;
	}

	// Convert velocities from rpm to m/s
	v_left_mps = fVelocityLeftRpm * GUARDIAN_RPM2MPS;
	v_right_mps = fVelocityRightRpm * GUARDIAN_RPM2MPS;
	linearSpeedMps = (v_right_mps + v_left_mps) / 2.0;			   		    	// m/s
	angularSpeedRads = -(v_right_mps - v_left_mps) / GUARDIAN_D_TRACKS_M;    			//rad/s

	pthread_mutex_lock(&mutex_odometry);

                //Left and right track velocities
                robot_data.actSpeedLmps = v_left_mps;
                robot_data.actSpeedRmps = v_right_mps;

		//Velocity
		robot_pose.va = angularSpeedRads;
		robot_pose.vx = linearSpeedMps *  cos(robot_pose.pa);
		robot_pose.vy = linearSpeedMps *  sin(robot_pose.pa);

		//Position
		robot_pose.pa += angularSpeedRads * fSamplePeriod;
		robot_pose.px += robot_pose.vx * fSamplePeriod;
		robot_pose.py += robot_pose.vy * fSamplePeriod;

	pthread_mutex_unlock(&mutex_odometry);
}



/*!   \fn void guardian_hardware_interface::SetVWRef()
      * GUARDIAN_HW_IFACE state space control of V:Linear speed and W:Angular speed
        GUARDIAN_HW_IFACE configured AB mixed, closed loop 
*/ 
void guardian_hardware_interface::SetVWRef()
{
     // Linear and angular speed references
     double linear_ref = robot_data.v_ref_mps / GUARDIAN_MAX_XSPEED;
     double angular_ref = robot_data.w_ref_rads / GUARDIAN_MAX_YAWSPEED; 
     if (linear_ref > 1.0) linear_ref=1.0;
     if (linear_ref < -1.0) linear_ref=-1.0;
     if (angular_ref > 1.0) angular_ref=1.0;
     if (angular_ref < -1.0) angular_ref=-1.0;

     // additional flag check
     if (this->robot_data.bMotorsEnabled)
	WriteMotorSpeed(angular_ref*0x7F, linear_ref*0x7F);
     else
        WriteMotorSpeed(0.0,0.0);
}


/*! \fn void guardian_hardware_interface::WriteMotorSpeed( angular_speed or speedL, linear_speed or speedR )
 * 	\brief Sets motor speed input variables are different variables according to GUARDIAN_HW_IFACE cfg. 
*/
void guardian_hardware_interface::WriteMotorSpeed(double speedL, double speedR){
	int vel_right_ref = 0x00, vel_left_ref = 0x00;
	char c;
	int n;
	int j = 0;
	char cSendBuffer[6]="",cSendBuffer2[6]="";
	//	, cRecBuffer[REC_BUFFER];

	//Control max speed
	if((speedL < 0.0) && (speedL < -GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED) )
		speedL = -GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED;
	else if( (speedL > 0.0) && (speedL > GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED) )
		speedL = GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED;

	if((speedR < 0.0) && (speedR < -GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED) )
		speedR = -GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED;
	else if( (speedR > 0.0) && (speedR > GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED) )
		speedR = GUARDIAN_HW_IFACE_MOTOR_DEF_MAX_SPEED;

	//separate mode
	if(speedL >= 0.0)	{
		vel_left_ref =(int)(speedL);
		sprintf(cSendBuffer,"!a%02x\r",vel_left_ref);
	}else{
		vel_left_ref = -(int)(speedL);
		sprintf(cSendBuffer,"!A%02x\r",vel_left_ref);
	}

	if(speedR >= 0.0)	{
		vel_right_ref =(int)(speedR);
		sprintf(cSendBuffer2,"!b%02x\r",vel_right_ref);
	}else{
		vel_right_ref = -(int)(speedR);
		sprintf(cSendBuffer2,"!B%02x\r",vel_right_ref);
	}

    	//Sends the values
    int written_bytes=0;
    if(serial->WritePort(cSendBuffer, &written_bytes, 5)==GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::WriteMotorSpeed: Error sending message");
	}
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	serial->ReadPort(&c, &n, 1);
   	while(c!='+' && j < 5000){ //Wait for confirmation
        	j++;
        	serial->ReadPort(&c, &n, 1);
   	}
   	if(c !='+'){
		ROS_ERROR("guardian_hardware_interface::WriteMotorSpeed:Error in confirmation");
    }
	//memset(cRecBuffer, 0, REC_BUFFER);
	written_bytes=0;
    if(serial->WritePort(cSendBuffer2, &written_bytes, 5)==GUARDIAN_HW_ERROR){
		ROS_ERROR("guardian_hardware_interface::WriteMotorSpeed: Error sending message");
	}
	usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);

	j = n = 0;
   	serial->ReadPort(&c, &n, 1);
    	while(c!='+' && j < 5000){ //Wait for confirmation
        	j++;
        	serial->ReadPort(&c, &n, 1);
    	}
    	if(c !='+'){
        	//sprintf(cAux, "guardian_hardware_interface::WriteMotorSpeed:Error on %s command. No response", cSendBuffer2);
		ROS_ERROR("guardian_hardware_interface::WriteMotorSpeed:Error in confirmation");
    	}
	
}


/*! \fn void guardian_hardware_interface::InitState()
   * Actions for InitState
   * Periodic actions at component hz
*/
void guardian_hardware_interface::InitState()
{
    robot_data.currentState = INIT_STATE;


    // Reset Encoders (avoid initial odometry error due to wrong initialization
    
    if (ResetEncoders()!=GUARDIAN_HW_OK){
        this->iErrorType=GUARDIAN_HW_IFACE_ERROR_SERIALCOMM;  
		this->FailureState();
    }else{
		this->ResetOdometry();
		usleep(GUARDIAN_HW_IFACE_SERIAL_DELAY);
        this->SetRobotSpeed(0.0,0.0);
		this->ToggleMotorPower('1');
		this->iErrorType=GUARDIAN_HW_IFACE_ERROR_NONE; 
        sleep(1);
		this->ReadyState(0);
	
   }
}


// READY STATE ACÍ

/*!	\fn void guardian_hardware_interface::FailureState()
 * 	\brief Actions in Failure State
*/
void guardian_hardware_interface::FailureState(){
/*
	robot_data.currentState = FAILURE_STATE;

	int timer = 2500000; //useconds
	static int recovery_cycles = 0;

	recovery_cycles++;
	ROS_ERROR("RECOVERY CICLES: %d", recovery_cycles);
	if(recovery_cycles >= GUARDIAN_HW_IFACE_DEFAULT_HZ){ // Try to recover each second
		switch(iErrorType)	{

			ROS_WARN("guardian_hardware_interface::FailureState: Trying to recover..");

			case GUARDIAN_HW_IFACE_ERROR_OPENING: //Try to recover
				ROS_WARN("guardian_hardware_interface::FailureState: Recovering from failure state (GUARDIAN_HW_IFACE_ERROR_OPENING)");
				this->Close();
				usleep(timer);
				if (this->Open()==GUARDIAN_HW_OK) this->InitState(); //SwitchToState(INIT_STATE);
				break;

			case GUARDIAN_HW_IFACE_ERROR_SERIALCOMM:
				ROS_WARN("guardian_hardware_interface::FailureState: Recovering from failure state (GUARDIAN_HW_IFACE_ERROR_SERIALCOMM)");
				this->Close();
				usleep(timer);
				if (this->Open()==GUARDIAN_HW_OK) this->InitState(); //SwitchToState(INIT_STATE);
				break;
			

			case GUARDIAN_HW_IFACE_ERROR_TIMEOUT:
				log->AddError("guardian_hardware_interface::FailureState: Recovering from failure state(GUARDIAN_HW_IFACE_ERROR_TIMEOUT.)");
				printf("guardian_hardware_interface::FailureState: Recovering from failure state (GUARDIAN_HW_IFACE_ERROR_TIMEOUT.)\n");
				Close();
				usleep(timer);
				Open();
				break;


		}
		recovery_cycles = 0;
	}*/
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//
// PUBLIC FUNCTIONS / DATA ACCESS FUNCTIONS
//
///////////////////////////////////////////////////////////////////////////////////////////////////

/*! \fn void guardian_hardware_interface::ReadyState()
   * Component State Machine ReadyState
   * Periodic actions at component hz
*/
void guardian_hardware_interface::ReadyState(int token)
{
	robot_data.currentState = READY_STATE;

	// static int token = 0;

	// Send cmd and process response
	if ( ReadEncoders() == GUARDIAN_HW_OK){	
		UpdateOdometry();
	}else{
	   //ROS_ERROR("Algo va mal...");
	   this->iErrorType=GUARDIAN_HW_IFACE_ERROR_SERIALCOMM;
	   this->FailureState();
	}								

	if (robot_data.bMotorsEnabled) {
		// Control robot axes
		SetVWRef();         
	}else{
		// Set 0 references
		WriteMotorSpeed(0.0,0.0);
	}

 /*	
    // No time critical functions called iterativel
    //ROS_WARN("Abans del switch de ReadyState(%d).\n", token);

    switch (token) {
	
        case 0: ReadBatteryVoltage(); // Send cmd and process response  
//				ROS_INFO("Reading bateries (CASE:0)");
                break;

        case 1: ReadAnalogInputs();   // Read analog inputs
//				ROS_INFO("Reading analog inputs (CASE:1)");
                break;
	
        case 2: ReadTemperature();    // Read temperatures
//				ROS_INFO("Reading temperatures (CASE:2)");
                break;
	
        case 3: ReadDigitalInputs();  // Read digital inputs
//				ROS_INFO("Reading digital inputs (CASE:3)");
                break;
		
        case 4: WriteDigitalOutput( robot_data.digital_output_setvalue );
//				ROS_INFO("Writing digital output (CASE:4)");
                token=-1;
                break;
	
        default:token=-1;
//				ROS_INFO("token=-1 (CASE:DEFAULT)");
                break;
        }
*/

}

/*!	\fn void guardian_hardware_interface::ToggleMotorPower(unsigned char val)
 * 	\brief Switches on/off the motor
*/
void guardian_hardware_interface::ToggleMotorPower(unsigned char val)
{
	switch(val)	{

            case '1':	//Enable Motor
			//if(control_mode==CONTROL_RC){ //Changing the mode
            		//EnterRS232Mode();
            		this->robot_data.bMotorsEnabled=true;
			this->ReadyState(0);
            		//}
			ROS_INFO("guardian_hardware_interface::ToggleMotorPower: Motor enabled");
            		break;

	    case '0':	//Disable Motor
			ROS_INFO("guardian_hardware_interface::ToggleMotorPower: Motor disabled");
            		this->robot_data.bMotorsEnabled=false;
            		break;
	     default:   //Disable Motor
			ROS_ERROR("guardian_hardware_interface::ToggleMotorPower: Mode not allowed, motor dissabled!");
			this->robot_data.bMotorsEnabled=false;
            		break;
	}
}

/*!	\fn void guardian_hardware_interface::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
	* Returns robot velocity and pose
*/
void guardian_hardware_interface::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
{
    pthread_mutex_lock(&mutex_odometry);
        // Return values
        *vx = robot_pose.vx; *vy=robot_pose.vy; *va=robot_pose.va;
        *px=robot_pose.px; *py=robot_pose.py; *pa=robot_pose.pa;
    pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_hardware_interface::ResetOdometry()
	* Resets driver odometry
*/
void guardian_hardware_interface::ResetOdometry(){

	// Reset player local copy of the robot odometry
	pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = 0;
		robot_pose.py = 0;
		robot_pose.pa = 0;
		robot_pose.vx = 0;
		robot_pose.vy = 0;
		robot_pose.va = 0;
	pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_hardware_interface::ModifyOdometry()
	* Set new odometry value (pose)
*/
void guardian_hardware_interface::ModifyOdometry(  double px,  double py,  double pa )
{
	// Reset player local copy of the robot odometry
	pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = px;
		robot_pose.py = py;
		robot_pose.pa = pa;
	pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_hardware_interface::SetRobotSpeedLimits()
	* Set robot linear and angular speed limits
	* linear speed in m/s
	* angular speed in rad/s
*/
void guardian_hardware_interface::SetRobotSpeedLimits(double lin_speed, double ang_speed){

    robot_data.max_linear_speed = lin_speed;  // m/s
    robot_data.max_angular_speed = ang_speed; // rad/s
}

/*!	\fn void guardian_hardware_interface::SetRobotSpeed()
	* Set robot left and right track references from linear and angular values
*/
void guardian_hardware_interface::SetRobotSpeed(double lin_speed, double ang_speed){

	double delta;

	// convert (lin_speed,ang_speed) to (speedL,speedR)
	delta = ang_speed * GUARDIAN_D_TRACKS_M;
	pthread_mutex_lock(&mutex_odometry);
		robot_data.v_ref_mps = lin_speed;
		robot_data.w_ref_rads = ang_speed;
        	robot_data.desSpeedLmps = lin_speed - (delta/2);
        	robot_data.desSpeedRmps = lin_speed + (delta/2); // this values are input to PID
	pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_hardware_interface::GetVoltage()
	* Return the robot voltage value measured internally
*/
float guardian_hardware_interface::GetVoltage()
{
    return robot_data.voltage_internal;
}

/*!	\fn void guardian_hardware_interface::GetEncoder(char enc)
	* Return the last value measured of encoder 'L'eft or 'R'ight
*/
float guardian_hardware_interface::GetEncoder(char enc)
{
    float encoder_value=0;
    switch (enc) {
        case 'L':
        case 'l':
            pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_left;
            pthread_mutex_unlock(&mutex_encoders);
            break;
        case 'R':
        case 'r':
            pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_right;
            pthread_mutex_unlock(&mutex_encoders);
            break;
        }

    return encoder_value;
}

/*!	\fn int guardian_hardware_interface::GetTemperature(int index)
	* Return the heat sink temperature measurement
*/
int guardian_hardware_interface::GetTemp(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.temperature[index];
}

/*!	\fn float guardian_hardware_interface::GetTemperature(int index)
	* Return the selected analog input measurement
*/
float guardian_hardware_interface::GetAnalogInput(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.analog_input[index];
}

/*!	\fn float guardian_hardware_interface::SetDigitalOutput
	* Public function for setting /resetting digital output
*/
void guardian_hardware_interface::SetDigitalOutput( bool value )
{
   robot_data.digital_output_setvalue = value;
}

/*!	\fn float guardian_hardware_interface::GetDigitalOutput
	* Public function for reading digital output
*/
bool guardian_hardware_interface::GetDigitalOutput()
{
   return robot_data.digital_output;
}

/*!	\fn bool guardian_hardware_interface::GetDigitalInput
	* Public function for reading digital inputs 0..2
*/
bool guardian_hardware_interface::GetDigitalInput(int input)
{
   if ((input>=0) && (input<=2))
      return robot_data.digital_input[input];
   else
      return false;
}

/*!	\fn bool guardian_hardware_interface::GetDigitalInput
	* Public function for reading digital inputs 0..2
*/
States guardian_hardware_interface::GetCurrentState()
{
	return robot_data.currentState;
}

 // namespace
