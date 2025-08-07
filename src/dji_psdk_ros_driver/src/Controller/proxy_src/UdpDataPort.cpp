#include "UdpDataPort.h"

UdpDataPort::UdpDataPort(const char* name, int localPort, const char* remoteIp, int remotePort, int mode, int timeo) :
	mPort(name, localPort, remoteIp, remotePort, mode, timeo)
{
	mKeeperThread = new DataPortKeeperThread(this, &mPort, mode);
	mName = name;
}

UdpDataPort::UdpDataPort(const char* name, const char* remoteIp, int remotePort, int mode, int timeo) :
	mPort(name, 0, remoteIp, remotePort, mode, timeo)
{
	mKeeperThread = new DataPortKeeperThread(this, &mPort, mode);
	mName = name;
}

UdpDataPort::UdpDataPort(const char* name, int listenPort, int mode, int timeo) :
	mPort(name, listenPort, mode, timeo)
{
	mKeeperThread = new DataPortKeeperThread(this, &mPort, mode);
	mName = name;
}



UdpDataPort::~UdpDataPort()
{
	delete mKeeperThread;
}

const char* UdpDataPort::name()
{
	return mName.c_str();
}

int UdpDataPort::start()
{
	return mKeeperThread->start();
}

int UdpDataPort::stop()
{
	return mKeeperThread->stop();
}
