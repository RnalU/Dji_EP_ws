#include "TcpDataPort.h"

TcpDataPort::TcpDataPort(const char* name, const char* remoteIp, int remotePort, int mode, int timeo):
	mPort(name, remoteIp, remotePort, timeo),
	mMode(mode)
{
	mName = name;
}

TcpDataPort::~TcpDataPort()
{
	delete mKeeperThread;
}

const char* TcpDataPort::name()
{
	return mName.c_str();
}

int TcpDataPort::start()
{
	mKeeperThread = new DataPortKeeperThread(this, &mPort, mMode);
	return mKeeperThread->start();
}

int TcpDataPort::stop()
{
	int err = mKeeperThread->stop();
	delete mKeeperThread;
	mKeeperThread = NULL;
	return err;
}

void TcpDataPort::reconnect(){
	mPort.disconnect();
	mPort.connect();
}
