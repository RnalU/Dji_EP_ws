#pragma once
#include "DataPort.h"
#include "UdpPort.h"
#include "DataPortKeeperThread.h"

class UdpDataPort : public DataPort
{
public:
	UdpDataPort(const char* name, int localPort, const char* remoteIp, int remotePort, int mode, int timeo=12000);
	UdpDataPort(const char* name, const char* remoteIp, int remotePort, int mode, int timeo=12000);
	UdpDataPort(const char* name, int listenPort, int mode, int timeo=12000);
	virtual ~UdpDataPort();
	virtual int start();
	virtual int stop();
	virtual const char* name();
private:
	std::string mName;
	UdpPort mPort;
	DataPortKeeperThread *mKeeperThread;
};

