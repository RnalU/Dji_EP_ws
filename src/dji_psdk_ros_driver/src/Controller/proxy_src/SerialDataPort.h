#pragma once
#include "DataPort.h"
#include "SerialPort.h"
#include "DataPortKeeperThread.h"

class SerialDataPort : public DataPort
{
public:
	SerialDataPort(const char* name, const char* port, int baudrate, int mode, int timeo=4000);
	SerialDataPort(const char* name, const char* port, int baudrate, char databits, char stopbits, char parity, int mode, int timeo=4000);
	virtual ~SerialDataPort();

	virtual int start();
	virtual int stop();
	virtual void reconnect();
	virtual const char* name();
private:
	std::string mName;
	SerialPort mPort;
	int mMode;
	DataPortKeeperThread *mKeeperThread;
};

