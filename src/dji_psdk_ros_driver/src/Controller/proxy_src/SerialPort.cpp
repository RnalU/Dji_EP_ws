#include "SerialPort.h"
#include "Log.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/serial.h>

#define B12000000	0010017
static int get_baud(int rate) {
	int speed;
	switch(rate) {
		case 4800:   speed = B4800;   break;
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		case 230400: speed = B230400; break;
		case 460800: speed = B460800; break;
		case 921600: speed = B921600; break;
		case 2000000: speed = B2000000; break;
		case 2500000: speed = B2500000; break;
		case 3000000: speed = B3000000; break;
		case 3500000: speed = B3500000; break;
		case 4000000: speed = B4000000;break;
		case 12000000: speed = B12000000;break;
		default:
			speed = B115200;
		break;
	}
	return speed;
}

SerialPort::SerialPort(const char* name, const char* device, int baudrate, int timeo, char databits, char stopbits, char parity) :
mName(name),
mFd(-1),
mDevice(device),
mBaudrate(baudrate),
mDataBits(databits),
mStopBits(stopbits),
mParity(parity),
mKeepDelaySec(timeo),
mByteSendCnt(0),
mByteToSendCnt(0),
mValid(false)
{
}


SerialPort::~SerialPort()
{
}

const char* SerialPort::name()
{
	return mName.c_str();
}
bool SerialPort::valid()
{
	return mValid;
}
int SerialPort::connect()
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	mFd = open(mDevice.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(mFd != -1){
		int err = 0;

		fcntl(mFd, F_SETFL, 0);
		if(mBaudrate < 4000000){
			err = setupSerial(mBaudrate, mDataBits, mStopBits, mParity);
		}else{
			err = setupSerial(mBaudrate);
		}
		
		if(err == 0){
			mValid = true;
			Log::i("Serial connection <%s> build, keep time %d ms", mName.c_str(), mKeepDelaySec);
			return ERROR_NONE;
		}else{
			Log::w("setup serial failed %s:%s", mDevice.c_str(), strerror(errno));
			return ERROR_SERIAL_ERROR;
		}
	}else{
		Log::w("open serail failed %s:%s", mDevice.c_str(), strerror(errno));
		return ERROR_SERIAL_ERROR;
	}
}


int SerialPort::setupSerial(int baud, char databits, char stopbits, char parity)
{
	if (!isatty(mFd)) {
		Log::w("file descriptor %d is NOT a serial port", mFd);
		return -1;
	}

	tcflush(mFd, TCIOFLUSH);

	// Read file descritor configuration
	struct termios config;
	if (tcgetattr(mFd, &config) < 0) {
		Log::w("could not read configuration of fd %d", mFd);
		return -2;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	ONOCR | OFILL | OPOST);

#ifdef OLCUC
	config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
	config.c_oflag &= ~ONOEOT;
#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input

	switch(parity)            //设置校验位
    {
    case 'n':
    case 'N':
        config.c_cflag &= ~PARENB;        //清除校验位
        config.c_iflag &= ~INPCK;        //enable parity checking
        break;
    case 'o':
    case 'O':
        config.c_cflag |= PARENB;        //enable parity
        config.c_cflag |= PARODD;        //奇校验
        config.c_iflag |= INPCK;            //disable parity checking
        break;
    case 'e':
    case 'E':
        config.c_cflag |= PARENB;        //enable parity
        config.c_cflag &= ~PARODD;        //偶校验
        config.c_iflag |= INPCK;            //disable pairty checking
        break;
    case 's':
    case 'S':
        config.c_cflag &= ~PARENB;        //清除校验位
        config.c_cflag &= ~CSTOPB;        //??????????????
        config.c_iflag |= INPCK;            //disable pairty checking
        break;
    default:
        Log::w("Unsupported parity.\n");
        return false;    
    }

    switch(stopbits)        //设置停止位
    {
    case 1:
        config.c_cflag &= ~CSTOPB;
        break;
    case 2:
        config.c_cflag |= CSTOPB;
        break;
    default:
        Log::w("Unsupported stopbits.\n");
        return false;
    }

	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN] = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	int baud_flag = get_baud(baud);
	if(cfsetispeed(&config, baud_flag) < 0 || cfsetospeed(&config, baud_flag) < 0){
		Log::w("Could not set desired baud rate of %d Baud", baud);
		return -3;
	}

	// Finally, apply the configuration
	if (tcsetattr(mFd, TCSAFLUSH, &config) < 0) {
		Log::w("could not set configuration of fd %d", mFd);
		return -4;
	}

	tcflush(mFd, TCIOFLUSH);

	// Done!
	return 0;
}

int SerialPort::setupSerial(int baud){
	struct termios   opt;
 	struct serial_struct serial;

 	tcgetattr(mFd, &opt);
 	tcflush(mFd, TCIOFLUSH);
 	cfmakeraw(&opt);
 	Log::i("set baud 38400");
 	cfsetspeed(&opt, B38400);
 	tcflush(mFd, TCIOFLUSH);
 	if(tcsetattr(mFd, TCSANOW, &opt) != 0){
		Log::e("tcsetattr serial : %s", strerror(errno));
		return -1;
 	}

 	if((ioctl(mFd, TIOCGSERIAL, &serial)) < 0){
		Log::e("Fail to get Serial: %s", strerror(errno));
		return -1;
 	}
 	Log::i("baud_base: %d ,divisor : %d", serial.baud_base, serial.custom_divisor);
 	serial.flags = ASYNC_SPD_CUST;
	// if(baud > serial.baud_base){
	// 	baud = 115200;
	// 	Log::e("!!!!!!!baud(%d) bigger than baud base(%d), set baud to 115200!!!!!!!!", baud, serial.baud_base);
	// }
 	serial.custom_divisor = serial.baud_base/baud;
 	Log::i("new divisor : %d", serial.custom_divisor);
 	if((ioctl(mFd, TIOCSSERIAL, &serial))<0){
		Log::e("Fail to set Serial:%s", strerror(errno));
		return -1;
 	}

 	ioctl(mFd, TIOCGSERIAL, &serial);
 	Log::i("success set baud to %d,custom_divisor=%d,baud_base=%d", baud, serial.custom_divisor, serial.baud_base);
 	return 0;
}

int SerialPort::send(const char* data, int size, int peer)
{
	if (!valid()) return ERROR_INVALID;
	int err = sendStateless(data, size);
	if (err <= 0) {
		Log::d("send fail to invalid err=%d", err);
		mValid = false;
	}
	return err;
}

int SerialPort::sendStateless(const char* data, int size)
{
	if (!valid()) return ERROR_INVALID;
	int sendsize = write(mFd, data, size);
	mByteSendCnt += sendsize;
	mByteToSendCnt += size;
	if (-1 == sendsize) {
		return ERROR_SERIAL_ERROR;
	}
	if (sendsize != size) return ERROR_SEND_FAIL;
	return sendsize;
}

int SerialPort::recv(char* data, int size, int *peer)
{
	if (!valid()) return ERROR_INVALID;
	*peer = 0;
	int err = readStateless(data, size);
	if (err <= 0) {
		Log::d("SerialPort read fail to invalid err=%d", err);
		mValid = false;
	}
	return err;
}

int SerialPort::readStateless(char* data, int size)
{
	if (!valid()) return ERROR_INVALID;
	struct timeval tv_timeout;
	tv_timeout.tv_sec = mKeepDelaySec/1000;
	tv_timeout.tv_usec = mKeepDelaySec%1000*1000;

	fd_set fs_read;
	FD_ZERO(&fs_read);
	FD_SET(mFd, &fs_read);
	int ret = select(mFd + 1, &fs_read, NULL, NULL, &tv_timeout);
	if (ret > 0) {
		if (FD_ISSET(mFd, &fs_read)) {
			int recvsize = read(mFd, data, size);
			if (-1 == recvsize) {
				return ERROR_SERIAL_ERROR;
			}
			return recvsize;
		} else {
			// error
			return ERROR_SERIAL_ERROR;
		}
	} else if (ret == 0) {
		//  -4 : timeout
		return ERROR_RECV_TIMEOUT;
	} else {
		// -3 : error
		return ERROR_SERIAL_ERROR;
	}
}
int SerialPort::disconnect()
{
	if(mFd != -1){
		close(mFd);
		mFd = -1;
		mValid = false;
	}
	return ERROR_NONE;
}
