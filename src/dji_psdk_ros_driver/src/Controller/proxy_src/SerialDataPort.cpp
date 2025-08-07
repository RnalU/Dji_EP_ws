#include "SerialDataPort.h"

SerialDataPort::SerialDataPort(const char* name, const char* port, int baudrate, int mode, int timeo) :
	mPort(name, port, baudrate, timeo), 
	mMode(mode),
	mKeeperThread(NULL)
{
	mName = name;
}

SerialDataPort::SerialDataPort(const char* name, const char* port, int baudrate, char databits, char stopbits, char parity, int mode, int timeo) :
	mPort(name, port, baudrate, timeo, databits, stopbits, parity),
	mMode(mode),
	mKeeperThread(NULL)
{
	mName = name;
}


SerialDataPort::~SerialDataPort()
{
	if(mKeeperThread){
		delete mKeeperThread;
	}
}

const char* SerialDataPort::name()
{
	return mName.c_str();
}

int SerialDataPort::start()
{
	mKeeperThread = new DataPortKeeperThread(this, &mPort, mMode);
	return mKeeperThread->start();
}

int SerialDataPort::stop()
{
	int err = mKeeperThread->stop();
	delete mKeeperThread;
	mKeeperThread = NULL;
	return err;
}

void SerialDataPort::reconnect(){
	mPort.disconnect();
	mPort.connect();
}
