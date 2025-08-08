#pragma once
#include "Port.h"
#include <string>
#include <map>

#include "SocketUtils.h"

class UdpPort : public Port
{
public:
	UdpPort(const char* name, int localPort, int mode, int timeo=4000);
	UdpPort(const char* name, int localPort, const char* remoteIp, int remotePort, int mode, int timeo=4000);
	virtual ~UdpPort();
public:
	virtual const char* name();
	virtual bool valid();
	virtual int connect();
	virtual int send(const char* data, int size, int peer);
	virtual int recv(char* data, int size, int *peer);
	virtual int disconnect();

	static const int MODE_MULTICAST = 0x04;
	static const int MODE_BOARDCAST = 0x08;

	static const int MODE_MULTIPEER = 0x10;
	static const int MODE_NO_RESET =  0X20;
private:
	int sendStateless(const char* data, int size);
	int readStateless(char* data, int size);
	int sendStateless(const char* data, int size, int peer);
	int readStateless(char* data, int size, int *peer);
	bool isSockAddrIn(SOCKADDR_IN sa);
	int getPeerCnt(SOCKADDR_IN sa);
private:
	std::string mName;
	int mLocalPort;
	std::string mRemoteIp;
	int mRemotePort;
	socket_t		mSocket;
	SOCKADDR_IN		mLocalAddr;
	SOCKADDR_IN     mRemoteAddr;
	bool mValid;
	int mKeepDelaySec;
	int mPeerCnt;
	std::map<int, SOCKADDR_IN> mClientSockAddrMap;
	uint32_t mByteRecvCnt;
	uint32_t mByteSendCnt;
};

