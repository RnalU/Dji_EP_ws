#pragma once
#include "Port.h"
#include <string>
class SerialPort : public Port
{
public:
	SerialPort(const char* name, const char* device, int baudrate, int timeo = 4000, char databits = 8, char stopbits = 1, char parity = 'N');
	virtual ~SerialPort();
public:
	virtual const char* name();
	virtual bool valid();
	virtual int connect();
	virtual int send(const char* data, int size, int peer);
	virtual int recv(char* data, int size, int *peer);
	virtual int disconnect();
private:
	int sendStateless(const char* data, int size);
	int readStateless(char* data, int size);
private:
	int setupSerial(int baudrate, char databits, char stopbits, char parity);
	int setupSerial(int baud);
private:
	std::string mName;
	int mFd;
	std::string mDevice;
	int mBaudrate;
	char mDataBits;
	char mStopBits;
	char mParity;
	bool mValid;
	int mKeepDelaySec;
	uint32_t mByteSendCnt;
	uint32_t mByteToSendCnt;
};

