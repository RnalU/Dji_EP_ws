#pragma once
#include "Port.h"
#include "SocketUtils.h"
#include <string>
class TcpPort : public Port
{
public:
	TcpPort(const char* name, const char* remoteIp, int remotePort, int timeo=4000);
	virtual ~TcpPort();
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
	std::string mName;
	std::string mRemoteIp;
	int mRemotePort;
	socket_t		mSocket;
	SOCKADDR_IN		mRemoteAddr;
	bool mValid;
	int mKeepDelaySec;
};

