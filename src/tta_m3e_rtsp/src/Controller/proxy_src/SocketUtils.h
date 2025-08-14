#pragma once

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
#include <WinSock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_t;
typedef int socklen_t;
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#define SOCKET_ERROR -1
#define INVALID_SOCKET -1
typedef int socket_t;
typedef struct sockaddr_in SOCKADDR_IN;
#endif
#include <string>
#include <list>
#include <map>
class Iface
{
public:
	std::string name;
	std::string addr;
	std::string netmask;
};
class SocketUtils
{
private:
	SocketUtils() {};
	virtual ~SocketUtils() {};
public:
	static int StartUp();
	static int CleanUp();
	static socket_t CreateSocket(int af, int type, int protocol);
	static int ConnectSocket(socket_t sockfd, const struct sockaddr *addr, socklen_t addrlen);
	static int CloseSocket(socket_t s);
	static int ShutdownSocket(socket_t s, int how);
	static int SocketLastError();
	static std::list<Iface> GetAllIfaces();
    static std::map<std::string, uint32_t> getAllIfacesIpInShort();
	static int Ping(std::string dst_addr, int timeout_ms);
	static int Ping(std::string ifacename, std::string dst_addr, int timeout_ms);
	static int Ping(std::string ifacename, std::string dst_addr, std::string backupDstAddr, int timeout_ms);
	static int getRequest204(std::string url, const std::string card);
	static int getRequest204(std::string url);
	static uint64_t GetMacAddress(std::string ifacename);
	static int GetNetcardUpDown(std::string ifacename);
	static int getipbyname(const char *name, char *ip, int ip_len);
	static int getNetlinkStatus(const char *if_name);

private:
	static void HandleSig(int signo);

public:
	static const int PING_STATE_INVALID_DSTADDR = -1;
	static const int PING_STATE_INVALID_SOCKET = -2;
	static const int PING_STATE_INVALID_IFACE = -3;
	static const int PING_STATE_SEND_FAIL = -4;
	static const int PING_STATE_RECV_FAIL = -5;
	static const int PING_STATE_RECV_TIMEOUT = -6;
	static const int PING_STATE_RECV_WRONG = -7;
};

