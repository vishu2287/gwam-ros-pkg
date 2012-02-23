/** \file SerialDevice.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2010
 *
 * \brief SerialDevice Class
 * (C) 2009 Robotnik Automation, SLL
 * Class to manage a serial connection
*/

#include "guardian_hardware_interface/SerialDevice.h"
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>
#include <errno.h>   /* Error number definitions */
#include <stropts.h>

/*! \fn SerialDevice::SerialDevice(void)
 	* Constructor by default
*/
SerialDevice::SerialDevice(){
	strcpy(cDevice,DEFAULT_PORT);
	strcpy(cParity,DEFAULT_PARITY);
	iBaudRate=DEFAULT_TRANSFERRATE;
	iBitDataSize = DEFAULT_DATA_SIZE;
	bCanon = false;
    iOpenMode = 2;
	bInitialized =  false;
}

/*! \fn SerialDevice::SerialDevice(void)
 	* Constructor by default
*/
SerialDevice::SerialDevice(const char *device, int baudrate, const char *parity, int datasize){
    strcpy(cDevice,device);
	//std::cout << "SerialDevice::SerialDevice: " << cDevice << " Parity= " << parity
	//<< " DataSize=" << datasize <<" BaudRate=" << baudrate << std::endl;
	strcpy(cParity,parity);
	iBaudRate=baudrate;
	iBitDataSize = datasize;
	bCanon = false;
	iOpenMode = 2;
	bInitialized =  false;
}

/*! \fn SerialDevice::~SerialDevice(void)
 	* Destructor by default
*/
SerialDevice::~SerialDevice(void){
	//std::cout << "SerialDevice::~SerialDevice" << std::endl;
}
/*
agvservicios:/robotrans# stty -F /dev/ttyS0 -a
speed 19200 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^D; eol = <undef>;
eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R;
werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 5;
parenb parodd cs7 -hupcl -cstopb cread clocal -crtscts
ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff
-iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt
-echoctl -echoke

agvservicios:/robotrans/src# stty -F /dev/ttyS1 -a
speed 19200 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^D; eol = <undef>;
eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R;
werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 5;
parenb parodd cs7 -hupcl -cstopb cread clocal -crtscts
ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff
-iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt
-echoctl -echoke

*/

/*! \fn int SerialDevice::Configure()
 * Configures devices and performance of the component
 *  - Configures Baudrate
 *  - Configures Parity
 *  - Configures Bit data size
 * \return SERIAL_OK
 * \return SERIAL_ERROR if the configuration process fails
*/
int SerialDevice::Configure(){
	struct termios options;
	// set up comm flags
	//memset(&options, 0,sizeof(options));
	switch(iOpenMode){
        case 1:
			printf("SerialDevice::Configure: Configuring port %s on mode %d\n", cDevice, iOpenMode);
            // Get the current options for the port...
            tcgetattr(fd, &options);
            // Set the baud rates to X
            switch(iBaudRate){
                case 9600:
                    cfsetispeed(&options, B9600);
                    cfsetospeed(&options, B9600);
                break;
                case 19200:
                    cfsetispeed(&options, B19200);
                    cfsetospeed(&options, B19200);
                break;
                case 38400:
                    cfsetispeed(&options, B38400);
                    cfsetospeed(&options, B38400);
                break;
                case 115200:
                    cfsetispeed(&options, B115200);
                    cfsetospeed(&options, B115200);
                break;
                default:
                    cfsetispeed(&options, B19200);
                    cfsetospeed(&options, B19200);
                break;
            }
            // Enable the receiver and set local mode...
            options.c_cflag |= CLOCAL | CREAD;
            options.c_cflag &= ~HUPCL;
            //
            // PARITY
            if(!strcmp(cParity, "none")){
                //parity = NONE
                options.c_cflag &= ~PARENB;
                options.c_cflag &= ~PARODD;
                //printf("SerialDevice::InitPort: Parity = none\n");
            } else if(!strcmp(cParity, "even")){
                //parity = EVEN
                options.c_cflag |= PARENB;
                //printf("SerialDevice::InitPort: Parity = even\n");
            } else if(!strcmp(cParity, "odd")){
                options.c_cflag |= PARENB;
                options.c_cflag |= PARODD;
                //printf("SerialDevice::InitPort: Parity = odd\n");
            } else {
                //parity = NONE
                options.c_cflag &= ~PARENB;
                options.c_cflag &= ~PARODD; // NONE
                //printf("SerialDevice::InitPort: Parity = none\n");
            }

            options.c_cflag &= ~CSTOPB;// 1 Stop bit
            options.c_cflag &= ~CSIZE;
            //
            // Character size
            switch(iBitDataSize){
                case 5:
                    options.c_cflag |= CS5;
                break;
                case 6:
                    options.c_cflag |= CS6;
                break;
                case 7:
                    options.c_cflag |= CS7;
                break;
                case 8:
                    options.c_cflag |= CS8;
                break;
                default:
                    options.c_cflag |= CS7;
                break;
            }
            /*
            -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts
            ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
            -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
        -   isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke
             */
            options.c_cflag &= ~CRTSCTS;	//Disables hardware flow control

            options.c_iflag |= IGNBRK;
            options.c_iflag &= ~ICRNL;
            options.c_iflag &= ~IXON;
            options.c_oflag &= ~OPOST;
            options.c_oflag &= ~ONLCR;
            options.c_lflag &= ~ISIG;
            options.c_lflag &= ~IEXTEN;
            options.c_lflag &= ~ECHOK;
            options.c_lflag &= ~ECHOCTL;
            options.c_lflag &= ~ECHOKE;
            options.c_lflag &= ~ECHO;
            options.c_lflag &= ~ECHOE;
            //
            // Entrada canónica-> La entrada canónica es orientada a línea. Los caracteres se meten en un buffer hasta recibir un CR o LF.
            if(bCanon)
                options.c_lflag |= ICANON;
            else
                options.c_lflag &= ~ICANON;

            // carácteres de control
            //options.c_cc[VMIN] = (cc_t)1;
            options.c_cc[VMIN] = (cc_t)1;
            options.c_cc[VTIME] = (cc_t)5;

            tcflush(fd, TCIFLUSH);
            // Set the new options for the port...
            tcsetattr(fd, TCSANOW, &options);

            fcntl(fd,F_SETFL, FNDELAY);	//TEST
        break;
        case 2:

            // set up comm flags
            memset(&options, 0,sizeof(options));

            switch(iBitDataSize)
            {
                case 5:
                    options.c_cflag = CS5 | CREAD;
                break;
                case 6:
                    options.c_cflag = CS6 | CREAD;
                break;
                case 7:
                    options.c_cflag = CS7 | CREAD;
                break;
                case 8:
                    options.c_cflag = CS8 | CREAD;
                break;
                default:
                    //std::cout << "SerialDevice::OpenPort: unknown datasize (" << iBitDataSize << " bits) : setting default datasize ("<< DEFAULT_DATA_SIZE <<" bits)" << std::endl;
                    iBitDataSize = 8;
                    options.c_cflag = CS8 | CREAD;
                break;
            }

            options.c_iflag = INPCK;
            options.c_oflag = 0;
            options.c_lflag = 0;

            if(!strcmp(cParity,"even"))
                options.c_cflag |= PARENB;//Even parity
            else if(!strcmp(cParity,"odd"))
                options.c_cflag |= PARODD;//Odd parity
            else if(!strcmp(cParity,"none")){	//No parity
                options.c_cflag &= ~PARODD;
                options.c_cflag &= ~PARENB;
            }

            options.c_lflag &= ~ICANON;	// TEST

            tcsetattr(fd, TCSANOW, &options);
            tcflush(fd, TCIOFLUSH);


            if (SetTermSpeed(iBaudRate) == SERIAL_ERROR){
                printf("SerialDevice::Configure: Error in %s setting terminal speed (%d) \n",cDevice, iBaudRate);
                return SERIAL_ERROR;

            }

            // Make sure queue is empty
            tcflush(fd, TCIOFLUSH);
            usleep(1000);
            tcflush(fd, TCIFLUSH);

        break;

	}

	printf("SerialDevice::Configure: Port %s initialized with baudrate = %d, Parity = %s and Data size = %d\n",cDevice, iBaudRate,cParity, iBitDataSize);

	return SERIAL_OK;
}

/*!	\fn int SerialDevice::SetTermSpeed(int speed)
 * Set serial communication speed.
 * Valid values: 9600, 19200, 38400, 115200
 * @return SERIAL_OK
 * @return SERIAL_ERROR
*/
int SerialDevice::SetTermSpeed(int speed){
	struct termios term;
	int term_speed;

	switch(speed){
		case 9600:
			term_speed = B9600;
			break;
		case 19200:
			term_speed = B19200;
			break;
		case 38400:
			term_speed = B38400;
			break;
		case 115200:
			term_speed = B115200;
			break;
		default:
			term_speed = B19200;
			printf("SerialDevice::SetTermSpeed: Unknown speed %d speedL. Setting default value to 19200\n", speed);
			//std::cout << "SerialDevice::SetTermSpeed: Unknown speed ("<< speed <<") speedL Setting default value ("<< term_speed <<")"<< std::endl;
			break;
	}

	switch(term_speed)	{
		case B9600:
		case B19200:
		case B38400:
		case B115200:
			if( tcgetattr( fd, &term ) < 0 ) {
				//std::cout << "SerialDevice::SetTermSpeed: Unable to get device attributes" << std::endl;
				return SERIAL_ERROR;
			}

		  //cfmakeraw( &term );
			if(cfsetispeed( &term, term_speed ) < 0 || cfsetospeed( &term, term_speed ) < 0) {
                printf("SerialDevice::SetTermSpeed: Failed to set serial baudrate\n");
				return SERIAL_ERROR;
			}

			if( tcsetattr( fd, TCSAFLUSH, &term ) < 0 )	{
                printf("SerialDevice::SetTermSpeed: Unable to set device attributes\n");
				return SERIAL_ERROR;
			}
			//std::cout << "SerialDevice::SetTermSpeed: Communication rate changed to " << speed << std::endl;
			return SERIAL_OK;
		break;

        default:
		//std::cout << "SerialDevice::SetTermSpeed: Unknown speed "<< speed << std::endl;
            printf("SerialDevice::SetTermSpeed:  Unknown speed\n");
            return SERIAL_ERROR;
        break;

	}
	return SERIAL_OK;
}

/*! \fn int SerialDevice::Open()
 *  \brief Opens serial port for communication
 *  \return SERIAL_OK
 *  \return SERIAL_ERROR
*/
int SerialDevice::Open(){
	//struct termios newtio;

    switch(iOpenMode){
        case 1:
            fd = open(cDevice,  O_RDWR | O_NOCTTY);
        break;

        case 2:
            fd = open(cDevice,  O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
		break;
    }


	if(fd == -1){
		//std::cout << "SerialDevice::OpenPort: Error opening " << cDevice << " port" << std::endl;
		printf("SerialDevice::Open: Error opening %s port on mode %d\n", cDevice, iOpenMode);
		return SERIAL_ERROR;  // invalid device file
	}else{
		//std::cout << "SerialDevice::Open: " << cDevice << " opened properly" << std::endl;
		printf("SerialDevice::Open: %s opened properly on mode %d\n", cDevice, iOpenMode);
		return SERIAL_OK;
	}
	//bReady = true;

	return SERIAL_OK;
}

/*! \fn int SerialDevice::Close()
 *  Closes serial port
 *  \return SERIAL_OK
 *  \return SERIAL_ERROR
*/
int SerialDevice::Close(){

	if(close(fd) != 0){
        printf("SerialDevice::ClosePort: Error closing port %s", cDevice);
        return SERIAL_ERROR;
	}else{
        printf("SerialDevice::ClosePort: port %s closed",cDevice);
		bInitialized = false;
        return SERIAL_OK;
	}
}


/*! \fn int SerialDevice::Setup()
 * Configures and initializes the component
 * \return SERIAL_OK
 * \return INITIALIZED if the component is already intialized
 * \return SERIAL_ERROR
*/
int SerialDevice::Setup(){
	
	// Checks if has been initialized
	if(bInitialized){
		printf("SerialDevice::Setup: Already initialized");
		return INITIALIZED;
	}

	// Opens devices used by the component
	if(Open() != SERIAL_OK){
		printf("SerialDevice::Setup: Error in Open\n");
		return SERIAL_ERROR;
	}
	printf("SerialDevice::Setup: Open OK\n");
	//
	// Configure the component
	if(Configure() != SERIAL_OK){
		printf("SerialDevice::Setup: Error in Configure\n");
		return SERIAL_ERROR;
	}
	printf("SerialDevice::Setup: Configure OK\n");
	bInitialized = true;

	return SERIAL_OK;
}

/*!	\fn int SerialDevice::WritePort(char *chars, int *written_bytes, int length)
 * @brief Sends data to the serial port
 * @param chars as char*, string for sending
 * @param length as int, length of the string
 * @param written_bytes as int*, number of written bytes
 * @return number of sent bytes
*/
int SerialDevice::WritePort(char *chars, int *written_bytes, int length) {
	
	if(bInitialized){
        *written_bytes = write(fd, chars, length);//strlen(chars));
		//printf("SerialDevice::WritePort: sending [%s]\n", chars);		
        if (written_bytes < 0) {
            printf("SerialDevice::WritePort: Error writting on %s\n", cDevice);
            return SERIAL_ERROR;
        }
	}else{
        printf("SerialDevice::WritePort: Device %s not ready\n", cDevice);
        return NOT_INITIALIZED;
	}

	return SERIAL_OK;
}

/*!	\fn int SerialDevice::ReadPort(char *result, int *read_bytes, int num_bytes)
 * @brief Reads serial port
 * @param result as char *, output buffer
 * @param read_bytes as int, number of read bytes
 * @param num_bytes as *int, number of desired bytes to read
 * @return SERIAL_OK
 * @return NOT_INITIALIZED
 * @return SERIAL_ERROR
*/
int SerialDevice::ReadPort(char *result, int *read_bytes, int num_bytes) {
	int n = 0;

    if(bInitialized) {
        n =  read(fd, result, num_bytes);
        *read_bytes = n;

        if(n < 0){
            if (errno == EAGAIN) {
                //printf("SerialDevice::ReadPort: Read= %d, SERIAL EAGAIN SERIAL_ERROR, errno= %d\n",iIn, errno);
                return SERIAL_ERROR;
            } else {
                printf("SerialDevice::ReadPort: SERIAL read error %d %s", errno, strerror(errno));
                return SERIAL_ERROR;
            }
		}
    } else {
        printf("SerialDevice::ReadPort: Device %s not ready", cDevice);
        return NOT_INITIALIZED;
    }

	return SERIAL_OK;
}

/*!	\fn int SerialDevice::Flush()
 * Clean port buffer
 * @return SERIAL_OK
 * @return SERIAL_ERROR
*/
int SerialDevice::Flush(){
	if (tcflush( fd, TCIOFLUSH ) < 0 )
        return SERIAL_OK;
	else
        return SERIAL_ERROR;
}

/*!	\fn char *SerialDevice::GetDevice()
 * @return opened device
*/
char *SerialDevice::GetDevice(){
	return cDevice;
}

/*!	\fn int SerialDevice::Flush()
 * Sets the canonical input. False by default
*/
void SerialDevice::SetCanonicalInput(bool value){
	bCanon = value;
}
