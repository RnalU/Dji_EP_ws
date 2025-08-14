#pragma once
#include <cstdint>
class Port
{
public:
	Port();
	virtual ~Port();
	virtual bool valid() = 0;
	virtual int connect() = 0;
	virtual int send(const char* data, int size, int peer) = 0;
	virtual int recv(char* data, int size, int *peer) = 0;

	virtual int disconnect() = 0;
	virtual const char* name() = 0;

	int mMode;
public:
	static const int ERROR_NONE = 0;
	static const int ERROR_INVALID = -1;
	static const int ERROR_SOCKET_ERROR = -2;
	static const int ERROR_SERIAL_ERROR = -3;
	static const int ERROR_RECV_TIMEOUT = -4;
	static const int ERROR_RECV_FAIL = -5;
	static const int ERROR_SEND_FAIL = -6;
	static const int ERROR_SEND_PEER = -7;
};

