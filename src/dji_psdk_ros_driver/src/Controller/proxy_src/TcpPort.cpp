#include "TcpPort.h"
#include "Log.h"
#include <cstring>
#include <fcntl.h>
#include "SocketUtils.h"

TcpPort::TcpPort(const char* name, const char* remoteIp, int remotePort, int timeo) :
	mName(name),
	mRemoteIp(remoteIp),
	mRemotePort(remotePort),
	mSocket(INVALID_SOCKET),
	mKeepDelaySec(timeo),
	mValid(false)
{
}


TcpPort::~TcpPort()
{
}

const char* TcpPort::name()
{
	return mName.c_str();
}
bool TcpPort::valid()
{
	return mValid;
}

int TcpPort::connect()
{
	if(mSocket != INVALID_SOCKET){
		return ERROR_NONE;
	}else{
		if (mRemotePort >= 0) {
			memset(&mRemoteAddr, 0, sizeof(mRemoteAddr));
			mRemoteAddr.sin_family = AF_INET;
			mRemoteAddr.sin_addr.s_addr = inet_addr(mRemoteIp.c_str());
			mRemoteAddr.sin_port = htons(mRemotePort);

			if ((mSocket = SocketUtils::CreateSocket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET) {
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
				u_long iMode = 1;
				int iResult = ioctlsocket(mSocket, FIONBIO, &iMode);
				if (iResult != NO_ERROR) {
					Log::w("ioctlsocket failed with error: %ld", iResult);
				}
#else
				int flags = fcntl(mSocket, F_GETFL, 0);
				if (fcntl(mSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
					Log::w("set socket to nonblock fail");
				}
#endif
				SocketUtils::ConnectSocket(mSocket, (struct sockaddr *) &mRemoteAddr, sizeof(mRemoteAddr));

				struct timeval tv_timeout;
				tv_timeout.tv_sec = 4;
				tv_timeout.tv_usec = 0;

				fd_set fs_write;
				FD_ZERO(&fs_write);
				FD_SET(mSocket, &fs_write);

				int rv;
				if ((rv = select(mSocket + 1, NULL, &fs_write, NULL, &tv_timeout)) == -1) {
					Log::e("connect error occurred on select function.");
					return ERROR_SOCKET_ERROR;
				}
				else if (rv == 0) {
					Log::e("connect time out occurred.");
					return ERROR_SOCKET_ERROR;
				}
				else {
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
					u_long iMode = 0;
					int iResult = ioctlsocket(mSocket, FIONBIO, &iMode);
					if (iResult != NO_ERROR) {
						Log::w("ioctlsocket failed with error: %ld", iResult);
					}
#else
					flags = fcntl(mSocket, F_GETFL, 0);
					if (fcntl(mSocket, F_SETFL, flags & ~O_NONBLOCK) < 0) {
						Log::w("set socket back to block fail");
					}
#endif
					Log::i("TCP connection <%s> build, keep time %d ms", mName.c_str(), mKeepDelaySec);
					mValid = true;
					return ERROR_NONE;
				}
			}
			else {
				Log::e("create socket err: %d", SocketUtils::SocketLastError());
				return ERROR_SOCKET_ERROR;
			}
		}
		else {
			Log::e("create socket params wrong.");
			return ERROR_INVALID;
		}
	}
}

int TcpPort::disconnect()
{
	if (mSocket != INVALID_SOCKET) {
		SocketUtils::CloseSocket(mSocket);
		mSocket = INVALID_SOCKET;
		Log::i("disconnect ok");
	}
	else {
		Log::e("disconnect error: mSocket = %d", mSocket);
	}
	mValid = false;
	return ERROR_NONE;
}

int TcpPort::recv(char* data, int size, int *peer)
{
	if (!valid()) return ERROR_INVALID;
	*peer = 0;
	int err = readStateless(data, size);
	if (err <= 0) {
		Log::d("TcpPort read fail to invalid err=%d", err);
		mValid = false;
	}
	return err;
}

int TcpPort::readStateless(char* data, int size)
{
	socket_t socket = mSocket;
	if (socket == INVALID_SOCKET) return ERROR_INVALID;

	struct timeval tv_timeout;
	tv_timeout.tv_sec = mKeepDelaySec/1000;
	tv_timeout.tv_usec = mKeepDelaySec%1000*1000;
	// Log::d("Tcp Port %s keep time %d seconds", mName.c_str(), mKeepDelaySec);

	fd_set fs_read;
	FD_ZERO(&fs_read);
	FD_SET(socket, &fs_read);
	int ret = select(socket + 1, &fs_read, NULL, NULL, &tv_timeout);
	if(ret > 0){
		if (FD_ISSET(socket, &fs_read)){
			socklen_t len = sizeof(mRemoteAddr);
			int recvsize = recvfrom(socket, data, size, 0, (struct sockaddr *) &mRemoteAddr, &len);
			if (SOCKET_ERROR == recvsize) {
				//LogE("recvfrom err : %d", my_socket_last_error());
				return ERROR_SOCKET_ERROR;
			}
			return recvsize;
		}else {
			// error
			return ERROR_RECV_FAIL;
		}
	}else if (ret == 0){
		//  0 : timeout
		return ERROR_RECV_TIMEOUT;
	}else{
		// -1 : error
		return ERROR_RECV_FAIL;
	}
}

int TcpPort::send(const char* data, int size, int peer)
{
	if (!valid()) return ERROR_INVALID;
	int err = sendStateless(data, size);
	if (err <= 0) {
		Log::d("send fail to invalid err=%d", err);
		mValid = false;
	}
	return err;
}

int TcpPort::sendStateless(const char* data, int size)
{
	socket_t socket = mSocket;
	if (socket == INVALID_SOCKET) return ERROR_INVALID;
	// int sendsize = sendto(socket, data, size, 0, (struct sockaddr *) &mRemoteAddr, sizeof(mRemoteAddr));
	struct sockaddr_in ss;
	socklen_t len;
	len = sizeof(ss);
	char localip[32];

	// if(getsockname(socket, (struct sockaddr *)&ss, &len) < 0){
	// 	Log::i("get sock name failed");
	// }
	// inet_ntop(AF_INET, &ss.sin_addr, localip, sizeof(localip));
	// int llport = ntohs(ss.sin_port);
	// Log::i("write address %s local port to %d", localip, llport);


	int sendsize = write(socket, data, size);

	if (SOCKET_ERROR == sendsize) {
		//LogE("sendto err : %d", my_socket_last_error());
		return ERROR_SOCKET_ERROR;
	}
	if (sendsize != size) return ERROR_SEND_FAIL;
	return sendsize;
}
