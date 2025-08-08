#include "UdpPort.h"
#include "Log.h"
#include <cstring>
#include <stdio.h>

UdpPort::UdpPort(const char* name, int localPort, int mode, int timeo):
	mName(name),
	mLocalPort(localPort),
	mRemoteIp(""),
	mRemotePort(-1),
	mSocket(-1),
	mKeepDelaySec(timeo),
	mValid(false),
	mByteRecvCnt(0),
	mByteSendCnt(0),
	mPeerCnt(0)
{
	mMode=mode;
}

UdpPort::UdpPort(const char* name, int localPort, const char* remoteIp, int remotePort, int mode, int timeo) :
	mName(name),
	mLocalPort(localPort),
	mRemoteIp(remoteIp),
	mRemotePort(remotePort),
	mSocket(-1),
	mKeepDelaySec(timeo),
	mValid(false),
	mByteRecvCnt(0),
	mByteSendCnt(0),
	mPeerCnt(0)
{
	mMode=mode;
}

UdpPort::~UdpPort()
{

}
const char* UdpPort::name()
{
	return mName.c_str();
}
bool UdpPort::valid()
{
	return mValid;
}

bool UdpPort::isSockAddrIn(SOCKADDR_IN sa){
	for( auto x : mClientSockAddrMap){
		if((x.second.sin_addr.s_addr == sa.sin_addr.s_addr) && (x.second.sin_port == sa.sin_port)){
			return true;
		}
	}

	return false;
}

int UdpPort::getPeerCnt(SOCKADDR_IN sa){
	for( auto x : mClientSockAddrMap){
		if((x.second.sin_addr.s_addr == sa.sin_addr.s_addr) && (x.second.sin_port == sa.sin_port)){
			return x.first;
		}
	}

	mClientSockAddrMap[mPeerCnt] = sa;
	return mPeerCnt++;
}

int UdpPort::connect()
{
	memset(&mLocalAddr, 0, sizeof(mLocalAddr));
	mLocalAddr.sin_family = AF_INET;
	mLocalAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	if((mMode & MODE_MULTICAST) || (mMode & MODE_BOARDCAST)){
		Log::i("set local boardcast port:%d", mRemotePort);
		mLocalAddr.sin_port = htons(mRemotePort);
	}else{
		mLocalAddr.sin_port = mLocalPort > 0 ? htons(mLocalPort) : 0;
	}

	memset(&mRemoteAddr, 0, sizeof(mRemoteAddr));
	mRemoteAddr.sin_family = AF_INET;
	if(mMode & MODE_BOARDCAST){
		Log::i("set remote boardcast address any");
		mRemoteAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	}else{
		mRemoteAddr.sin_addr.s_addr = inet_addr(mRemoteIp.c_str());
	}
	mRemoteAddr.sin_port = htons(mRemotePort);

	if ((mSocket = SocketUtils::CreateSocket(AF_INET, SOCK_DGRAM, 0)) >= 0) {
		if((mMode & MODE_BOARDCAST) || (mMode & MODE_MULTICAST) ){
			Log::i("set udp port reuseaddr/reuseport");
			int one = 1;
			if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0){
				perror("setsockopt()");
			}
			if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one)) < 0){
				perror("setsockopt()");
			}
		}

		if(mMode & MODE_MULTICAST){
			Log::i("set udp port join membership %s", mRemoteIp.c_str());
			struct ip_mreq mreq;
			mreq.imr_multiaddr.s_addr=inet_addr(mRemoteIp.c_str());
			mreq.imr_interface.s_addr=htonl(INADDR_ANY);
			setsockopt(mSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
			Log::i("set multcast to %s:%d", mRemoteIp.c_str(), mRemotePort);
		}

		if (bind(mSocket, (struct sockaddr *) &mLocalAddr, sizeof(mLocalAddr)) >= 0) {
			struct sockaddr_in ss;
			socklen_t len;
			len = sizeof(ss);
			char localip[20];

			if(getsockname(mSocket, (struct sockaddr *)&ss, &len) < 0){
				Log::i("get sock name failed");
			}
			inet_ntop(AF_INET, &ss.sin_addr, localip, sizeof(localip));
			// mLocalPort = ntohs(ss.sin_port);
			// Log::i("update local port to %d", mLocalPort);

			char loop = 0;
			setsockopt(mSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
			int nRecvBuf = 512 * 1024;
			setsockopt(mSocket, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBuf, sizeof(int));

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif	//SIO_UDP_CONNRESET
			BOOL bNewBehavior = FALSE;
			DWORD dwBytesReturned = 0;
			WSAIoctl(mSocket, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);
#endif
			/*
			int err = setsockopt(mSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mMulticastReq, sizeof(mMulticastReq));
			if (err >= 0) {
				mValid = true;
				return ERROR_NONE;
			}
			else {
				Log::e("addMcastMembership err=%d", SocketUtils::SocketLastError());
				return ERROR_SOCKET_ERROR;
			}
			*/
			mValid = true;
			Log::i("UDP connection <%s> build, keep time %d ms", mName.c_str(), mKeepDelaySec);
			return ERROR_NONE;
		}
		else {
			close(mSocket);
			Log::e("bind socket %s err: %d", name(), SocketUtils::SocketLastError());
			return ERROR_SOCKET_ERROR;
		}
	}
	else {
		Log::e("create socket err: %d", SocketUtils::SocketLastError());
		return ERROR_SOCKET_ERROR;
	}
}
int UdpPort::sendStateless(const char* data, int size)
{
	if (!valid()) return ERROR_INVALID;
	int sendsize = sendto(mSocket, data, size, 0, (struct sockaddr *) &mRemoteAddr, sizeof(mRemoteAddr));
	if (SOCKET_ERROR == sendsize) {
		Log::e("sendto err : %d", SocketUtils::SocketLastError());
		return ERROR_SOCKET_ERROR;
	}
	if (sendsize != size) return ERROR_SEND_FAIL;
	return sendsize;
}

int UdpPort::send(const char* data, int size, int peer)
{
	if (!valid()) return ERROR_INVALID;

	if((mPeerCnt != 0) && (peer >= mPeerCnt)){ // 0 is a initial value for peerNo
		Log::d("invalid peer number %d, max:%d", peer, mPeerCnt-1);
		return ERROR_SEND_PEER;
	}

	int err = sendStateless(data, size, peer);
	if (err <= 0) {
		Log::d("send fail to invalid err=%d", err);
		if(mMode & MODE_NO_RESET){
			Log::d("udp with no reset flag, send to error ignored");
		}else{
			mValid = false;
		}
	}
	return err;
}
int UdpPort::sendStateless(const char* data, int size, int peer)
{
	if (!valid()) return ERROR_INVALID;

	SOCKADDR_IN remote;

	if(mMode & MODE_MULTIPEER){
		remote = mClientSockAddrMap[peer];
	}else{
		remote = mRemoteAddr;
	}

	// printf("udp sendto %s:%d-%d\n", inet_ntoa(remote.sin_addr), ntohs(remote.sin_port), peer);

	int sendsize = sendto(mSocket, data, size, 0, (struct sockaddr *) &remote, sizeof(remote));
	if (SOCKET_ERROR == sendsize) {
		Log::e("sendto err : %d", SocketUtils::SocketLastError());
		return ERROR_SOCKET_ERROR;
	}
	if (sendsize != size) return ERROR_SEND_FAIL;
	return sendsize;
}

int UdpPort::readStateless(char* data, int size)
{
	if (!valid()) return ERROR_INVALID;
	struct timeval tv_timeout;
	tv_timeout.tv_sec = mKeepDelaySec/1000;
	tv_timeout.tv_usec = mKeepDelaySec%1000*1000;

	fd_set fs_read;
	FD_ZERO(&fs_read);
	FD_SET(mSocket, &fs_read);
	int ret = select(mSocket + 1, &fs_read, NULL, NULL, &tv_timeout);
	if (ret > 0) {
		if (FD_ISSET(mSocket, &fs_read)) {
			int recvsize = recvfrom(mSocket, data, size, 0, NULL, 0);  // we don't care about peer infomations
			if (SOCKET_ERROR == recvsize) {
				Log::e("recvfrom err : %d", SocketUtils::SocketLastError());
				return ERROR_SOCKET_ERROR;
			}
			mByteRecvCnt += recvsize;
			return recvsize;
		}
		else {
			// error
			return ERROR_RECV_FAIL;
		}
	}
	else if (ret == 0) {
		//  0 : timeout
		return ERROR_RECV_TIMEOUT;
	}
	else {
		// -1 : error
		return ERROR_RECV_FAIL;
	}
}

int UdpPort::recv(char* data, int size, int *peer)
{
	if (!valid()) return ERROR_INVALID;
	int err = readStateless(data, size, peer);
	if (err <= 0) {
		Log::d("UdpPort read fail to invalid err=%d", err);

		printf("UdpPort read fail to invalid err=%d \r\n", err);
		if(mMode & MODE_NO_RESET){
			Log::d("udp with no reset flag, recv error ignored");
		}else{
			mValid = false;
		}
	}
	return err;
}
int UdpPort::readStateless(char* data, int size, int *peer)
{
	if (!valid()) return ERROR_INVALID;
	struct timeval tv_timeout;
	tv_timeout.tv_sec = mKeepDelaySec/1000;
	tv_timeout.tv_usec = mKeepDelaySec%1000*1000;
	// Log::d("Udp Port %s keep time %d seconds", mName.c_str(), mKeepDelaySec);

	//printf("Udp Port %s keep time %d seconds  \r\n", mName.c_str(), tv_timeout.tv_sec);


	fd_set fs_read;
	FD_ZERO(&fs_read);
	FD_SET(mSocket, &fs_read);
	int ret = select(mSocket + 1, &fs_read, NULL, NULL, &tv_timeout);
	if (ret > 0) {
		if (FD_ISSET(mSocket, &fs_read)) {
			SOCKADDR_IN peerAddr;
			socklen_t len = sizeof(peerAddr);

			int recvsize = recvfrom(mSocket, data, size, 0, (struct sockaddr *) &peerAddr, &len);
			if (SOCKET_ERROR == recvsize) {
				Log::e("recvfrom err : %d", SocketUtils::SocketLastError());
				return ERROR_SOCKET_ERROR;
			}
			*peer = getPeerCnt(peerAddr);
			// printf("udp recvfrom %s:%d-%d-%d\n", inet_ntoa(peerAddr.sin_addr), ntohs(peerAddr.sin_port), *peer, mPeerCnt);

			mByteRecvCnt += recvsize;
			return recvsize;
		}
		else {
			// error
			return ERROR_RECV_FAIL;
		}
	}
	else if (ret == 0) {
		//  0 : timeout
		return ERROR_RECV_TIMEOUT;
	}
	else {
		// -1 : error
		return ERROR_RECV_FAIL;
	}
}

int UdpPort::disconnect()
{
	if (mSocket >= 0) {
		/*
		int err = setsockopt(mSocket, IPPROTO_IP, IP_DROP_MEMBERSHIP, (const char*)&mMulticastReq, sizeof(mMulticastReq));
		if (err < 0) {
			Log::e("drop membership error: err = %d", err);
		}
		*/
		SocketUtils::CloseSocket(mSocket);
		mSocket = -1;
		mValid = false;
	}else {
		Log::e("disconnect error: mSocket = %d", mSocket);
	}
	return ERROR_NONE;
}
