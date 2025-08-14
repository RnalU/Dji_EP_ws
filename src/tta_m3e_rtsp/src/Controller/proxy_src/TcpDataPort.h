#pragma once
#include "DataPort.h"
#include "TcpPort.h"
#include "DataPortKeeperThread.h"
class TcpDataPort : public DataPort
{
public:
	TcpDataPort(const char* name, const char* remoteIp, int remotePort, int mode, int timeo=4000);
	virtual ~TcpDataPort();

	virtual int start();
	virtual int stop();
	virtual void reconnect();
	virtual const char* name();

private:
	std::string mName;
	TcpPort mPort;
	int mMode;
	DataPortKeeperThread *mKeeperThread;
};

