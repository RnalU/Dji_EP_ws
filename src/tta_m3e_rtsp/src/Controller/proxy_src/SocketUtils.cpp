#include "SocketUtils.h"


#include <signal.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <net/if.h> 
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <Log.h>

#include  <stdio.h>
#include  <stdlib.h>
#include  <fcntl.h>
#include  <errno.h>
#include  <sys/ioctl.h>
#include  <sys/types.h>
#include  <sys/socket.h>
// #include  <linux/if.h>
#include  <linux/sockios.h>
#include  <linux/ethtool.h>

#define PING_PACKET_SIZE 4096

void SocketUtils::HandleSig(int signo)
{
	Log::i("recv signal %d: %s", signo, strerror(errno));
	switch(signo){
		case SIGINT:
			Log::i("recv signal SIGINT");
			Log::i("\033[0;31mprogram exit abnormally!\033[0;39m");
			break;
		case SIGTERM:
			Log::i("recv signal SIGINT");
			Log::i("\033[0;31mprogram exit abnormally!\033[0;39m");
			break;
		case SIGTSTP:
			Log::i("recv signal SIGINT");
			Log::i("\033[0;31mprogram exit abnormally!\033[0;39m");
			break;
	}

    exit(0);
}

int SocketUtils::StartUp()
{
	signal(SIGINT, HandleSig);
    signal(SIGTERM, HandleSig);
	signal(SIGPIPE, SIG_IGN);
	signal(SIGSEGV,HandleSig);
    signal(SIGBUS,HandleSig);
    signal(SIGFPE,HandleSig);
    signal(SIGHUP,HandleSig);
    signal(SIGILL,HandleSig);
	return 0;
}

int SocketUtils::CleanUp()
{
	signal(SIGPIPE, SIG_DFL);
	return 0;
}

socket_t SocketUtils::CreateSocket(int af, int type, int protocol)
{
	return socket(af, type, protocol);
}

int SocketUtils::ConnectSocket(socket_t s, const struct sockaddr *addr, socklen_t addrlen)
{
	return connect(s, addr, addrlen);
}

int SocketUtils::CloseSocket(socket_t s)
{
	return close(s);
}

int SocketUtils::ShutdownSocket(socket_t s, int how)
{
	return close(s);
}

int SocketUtils::SocketLastError()
{
	return errno;
}

std::list<Iface> SocketUtils::GetAllIfaces()
{
	std::list<Iface> ifaces;
	struct ifaddrs* ifaddr = NULL;
  	int ret = getifaddrs(&ifaddr);
	if (ret==0) {
		for (struct ifaddrs* ifp = ifaddr; ifp != NULL; ifp = ifp->ifa_next) {
			if (ifp->ifa_addr && ifp->ifa_addr->sa_family == AF_INET) {
				Iface iface;
				iface.name = ifp->ifa_name;
				iface.addr = inet_ntoa(((struct sockaddr_in*)ifp->ifa_addr)->sin_addr);
				iface.netmask = inet_ntoa(((struct sockaddr_in*)ifp->ifa_netmask)->sin_addr);
				ifaces.push_back(iface);
			}
		}
  	}
	return ifaces;
}

std::map<std::string, uint32_t> SocketUtils::getAllIfacesIpInShort(){
    std::map<std::string, uint32_t> ifaces;
    struct ifaddrs* ifaddr = NULL;
  	int ret = getifaddrs(&ifaddr);
	if (ret==0) {
		for (struct ifaddrs* ifp = ifaddr; ifp != NULL; ifp = ifp->ifa_next) {
            if (ifp->ifa_addr && ifp->ifa_addr->sa_family == AF_INET) {
                ifaces[ifp->ifa_name]=((struct sockaddr_in*)ifp->ifa_addr)->sin_addr.s_addr;
            }else{
                std::map<std::string, uint32_t>::iterator it = ifaces.find(ifp->ifa_name);
				if(it == ifaces.end()){
                    ifaces.insert(std::pair<std::string, uint32_t>(ifp->ifa_name, 0));
                }
            }
        }
    }

    return ifaces;
}

static uint16_t icmp_cal_chksum(uint16_t* addr, int count)
{
    int32_t sum = 0;
    while( count > 1 )  {
        /*  This is the inner loop */
        sum += *addr++;
        count -= 2;
    }

    /*  Add left-over byte, if any */
    if( count > 0 ){
        sum += *(uint8_t*)addr;
	}

    /*  Fold 32-bit sum to 16 bits */
    while (sum>>16){
        sum = (sum & 0xffff) + (sum >> 16);
	}

    uint16_t checksum = ~sum;

	return checksum;
}

static void tv_sub(struct timeval *recvtime, struct timeval *sendtime)
{
    long sec = recvtime->tv_sec - sendtime->tv_sec;
    long usec = recvtime->tv_usec - sendtime->tv_usec;
    if(usec >= 0){
        recvtime->tv_sec = sec;
        recvtime->tv_usec = usec;
    }else{
        recvtime->tv_sec = sec - 1;
        recvtime->tv_usec = -usec;
    }
}

int SocketUtils::Ping(std::string dst_addr, int timeout_ms)
{
	struct sockaddr_in dest_addr;
	struct sockaddr_in from_addr;
	bzero(&dest_addr,sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
	if(inet_addr(dst_addr.c_str()) == INADDR_NONE) {
		struct hostent *host;
        if((host = gethostbyname(dst_addr.c_str())) == NULL) {
            return PING_STATE_INVALID_DSTADDR;
        }
        memcpy((char *)&dest_addr.sin_addr,host->h_addr,host->h_length);
    } else {
        dest_addr.sin_addr.s_addr = inet_addr(dst_addr.c_str());
    }

	pid_t pid = getpid();

	int rtt = PING_STATE_INVALID_SOCKET;

	socket_t socket = INVALID_SOCKET;
	if((socket=CreateSocket(AF_INET, SOCK_RAW, IPPROTO_ICMP))!=INVALID_SOCKET){
		// set recvbuff size
		int size = 50 * 1024;
		setsockopt(socket, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));

		
		char sendpacket[PING_PACKET_SIZE];
		char recvpacket[PING_PACKET_SIZE];

		// pack icmp
		struct icmp *icmp;
		struct timeval *tval;
		icmp = (struct icmp*)sendpacket;
		icmp->icmp_type = ICMP_ECHO;
		icmp->icmp_code = 0;
		icmp->icmp_cksum = 0;
		icmp->icmp_seq = 0;
		icmp->icmp_id = pid;
		int packsize = 64; // icmp pack size 64
		tval = (struct timeval *)icmp->icmp_data;
		gettimeofday(tval,NULL);        // send time
		icmp->icmp_cksum =  icmp_cal_chksum((uint16_t*)icmp, packsize); 

		// send
		if(sendto(socket, sendpacket, packsize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) >= 0) {
			// recv
			struct timeval tv_timeout;
			tv_timeout.tv_sec = timeout_ms / 1000;
			tv_timeout.tv_usec = timeout_ms % 1000 * 1000;

			fd_set fs_read;
			FD_ZERO(&fs_read);
			FD_SET(socket, &fs_read);
			int ret = select(socket + 1, &fs_read, NULL, NULL, &tv_timeout);
			if (ret > 0) {
				if (FD_ISSET(socket, &fs_read)) {
					socklen_t fromlen = sizeof(from_addr);
					int recvsize = recvfrom(socket, recvpacket, sizeof(recvpacket), 0, (struct sockaddr *)&from_addr, &fromlen);
					if (recvsize>0) {
						// recv time
						struct timeval tvrecv;
						gettimeofday(&tvrecv,NULL);

						// unpack icmp
						struct ip *ip;
						ip = (struct ip *)recvpacket;
						int iphdrlen = ip->ip_hl << 2;
						icmp = (struct icmp *)(recvpacket + iphdrlen);
						int len = recvsize - iphdrlen;
						if(len >= 8){
							if((icmp->icmp_type == ICMP_ECHOREPLY) && (icmp->icmp_id == pid)) {

								// compute rtt
								struct timeval *tvsend = (struct timeval *)icmp->icmp_data;
								tv_sub(&tvrecv, tvsend);
								rtt = (int)(((int64_t)tvrecv.tv_sec)*1000 + tvrecv.tv_usec/1000);
							}else{
								rtt = PING_STATE_RECV_WRONG;
							}
						}else{
							rtt = PING_STATE_RECV_WRONG;
						}
					}else{
						rtt = PING_STATE_RECV_FAIL;
					}
				}else{
					rtt = PING_STATE_RECV_FAIL;
				}
			}else if (ret == 0) {
				//  0 : timeout
				rtt = PING_STATE_RECV_FAIL;
			}
			else {
				// -1 : error
				rtt = PING_STATE_RECV_TIMEOUT;
			}
		}else{
			rtt = PING_STATE_SEND_FAIL;
		}
		
		CloseSocket(socket);
	}else{
		perror("create socket");
	}
	return rtt;
}

int SocketUtils::Ping(std::string ifacename, std::string dst_addr, int timeout_ms)
{
	struct sockaddr_in dest_addr;
	struct sockaddr_in from_addr;
	bzero(&dest_addr,sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
	if(inet_addr(dst_addr.c_str()) == INADDR_NONE) {
		struct hostent *host;
        if((host = gethostbyname(dst_addr.c_str())) == NULL) {
            return PING_STATE_INVALID_DSTADDR;
        }
        memcpy((char *)&dest_addr.sin_addr,host->h_addr,host->h_length);
    } else {
        dest_addr.sin_addr.s_addr = inet_addr(dst_addr.c_str());
    }

	pid_t pid = getpid();

	int rtt = PING_STATE_INVALID_SOCKET;

	socket_t socket = INVALID_SOCKET;
	if((socket=CreateSocket(AF_INET, SOCK_RAW, IPPROTO_ICMP))!=INVALID_SOCKET){
		struct ifreq interface;
		strncpy(interface.ifr_ifrn.ifrn_name, ifacename.c_str(), ifacename.length());
		if (setsockopt(socket, SOL_SOCKET, SO_BINDTODEVICE, (char *)&interface, sizeof(interface))  == 0) {
			// set recvbuff size
			int size = 50 * 1024;
			setsockopt(socket, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));

			
			char sendpacket[PING_PACKET_SIZE];
			char recvpacket[PING_PACKET_SIZE];

			// pack icmp
			struct icmp *icmp;
			struct timeval *tval;
			icmp = (struct icmp*)sendpacket;
			icmp->icmp_type = ICMP_ECHO;
    		icmp->icmp_code = 0;
    		icmp->icmp_cksum = 0;
    		icmp->icmp_seq = 0;
    		icmp->icmp_id = pid;
			int packsize = 64; // icmp pack size 64
    		tval = (struct timeval *)icmp->icmp_data;
    		gettimeofday(tval,NULL);        // send time
			icmp->icmp_cksum =  icmp_cal_chksum((uint16_t*)icmp, packsize); 

			// send
			if(sendto(socket, sendpacket, packsize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) >= 0) {
            	

				// recv
				struct timeval tv_timeout;
				tv_timeout.tv_sec = timeout_ms / 1000;
				tv_timeout.tv_usec = timeout_ms % 1000 * 1000;

				fd_set fs_read;
				FD_ZERO(&fs_read);
				FD_SET(socket, &fs_read);
				int ret = select(socket + 1, &fs_read, NULL, NULL, &tv_timeout);
				if (ret > 0) {
					if (FD_ISSET(socket, &fs_read)) {
						socklen_t fromlen = sizeof(from_addr);
						int recvsize = recvfrom(socket, recvpacket, sizeof(recvpacket), 0, (struct sockaddr *)&from_addr, &fromlen);
						if (recvsize>0) {
							// recv time
							struct timeval tvrecv;
							gettimeofday(&tvrecv,NULL);

							// unpack icmp
							struct ip *ip;
							ip = (struct ip *)recvpacket;
							int iphdrlen = ip->ip_hl << 2;
							icmp = (struct icmp *)(recvpacket + iphdrlen);
							int len = recvsize - iphdrlen;
							if(len >= 8){
								if((icmp->icmp_type == ICMP_ECHOREPLY) && (icmp->icmp_id == pid)) {

									// compute rtt
									struct timeval *tvsend = (struct timeval *)icmp->icmp_data;
									tv_sub(&tvrecv, tvsend);
									rtt = (int)(((int64_t)tvrecv.tv_sec)*1000 + tvrecv.tv_usec/1000);
								}else{
									rtt = PING_STATE_RECV_WRONG;
								}
							}else{
								rtt = PING_STATE_RECV_WRONG;
							}
						}else{
							rtt = PING_STATE_RECV_FAIL;
						}
					}else{
						rtt = PING_STATE_RECV_FAIL;
					}
				}else if (ret == 0) {
					//  0 : timeout
					rtt = PING_STATE_RECV_FAIL;
				}
				else {
					// -1 : error
					rtt = PING_STATE_RECV_TIMEOUT;
				}
			}else{
				rtt = PING_STATE_SEND_FAIL;
			}
		}else{
			rtt = PING_STATE_INVALID_IFACE;
		}
		CloseSocket(socket);
	}
	return rtt;
}

int SocketUtils::Ping(std::string ifacename, std::string dst_addr, std::string backupDstAddr, int timeout_ms){
	int ms = Ping(ifacename, dst_addr, timeout_ms);
	if(ms < 0){
		ms = Ping(ifacename, backupDstAddr, timeout_ms);
	}
	
	return ms;
}

int SocketUtils::getRequest204(std::string url, const std::string card){
	std::string responseStr;
	long code=0;



	return -1;
}

int SocketUtils::getRequest204(std::string url){


	return -1;
}

uint64_t SocketUtils::GetMacAddress(std::string ifacename)
{
	struct ifreq interface;
	uint8_t macaddr[ETH_ALEN];
	strncpy(interface.ifr_ifrn.ifrn_name, ifacename.c_str(), ifacename.length());
	socket_t socket = SocketUtils::CreateSocket(AF_INET, SOCK_DGRAM, 0);
	uint64_t mac = 0;
	if(socket != INVALID_SOCKET) {
		if(0==ioctl(socket, SIOCGIFHWADDR, &interface)) {
			memcpy(macaddr, interface.ifr_hwaddr.sa_data, ETH_ALEN);
			for(int i=0; i<ETH_ALEN&&i<8; i++){
				mac = mac << 8;
				mac = mac | macaddr[i] & 0xffL;
			}
		}
		SocketUtils::CloseSocket(socket);
	}
	return mac;
}

int SocketUtils::GetNetcardUpDown(std::string ifacename){
	int skfd = 0;
	struct ifreq ifr;
	int ret = 0;

	skfd = socket(AF_INET, SOCK_DGRAM, 0);
	if(skfd < 0) {
		Log::e("Open socket error: %s", strerror(errno));
		return -1;
	}

	strcpy(ifr.ifr_name, ifacename.c_str());

	if(ioctl(skfd, SIOCGIFFLAGS, &ifr) <0 ) {
		Log::e("IOCTL error : %s", strerror(errno));
		Log::e("Maybe ethernet inferface %s is not valid!", ifr.ifr_name);
		close(skfd);
		return -1;
	}

	if(ifr.ifr_flags & IFF_RUNNING) {
		ret = 0;
	} else {
		ret = 1;
	}	

	close(skfd);

	return ret;
}

int SocketUtils::getipbyname(const char *name, char *ip, int ip_len){
    struct hostent *hptr;

    if( (hptr = gethostbyname(name)) == NULL){
        Log::e("gethostbyname error, %s:%s\n", name, hstrerror(h_errno));
        return -1;
    }

    if(inet_ntop(hptr->h_addrtype, hptr->h_addr_list[0], ip, ip_len) == NULL){
        Log::e("inet_ntop error\n");
        return -1;
    }

    return 0;
}

// if_name like "ath0", "eth0". Notice: call this function
// need root privilege.
// return value:
// -1 -- error , details can check errno
// 1 -- interface link up
// 0 -- interface link down.
int SocketUtils::getNetlinkStatus(const char *if_name)
{
    int skfd;
    struct ifreq ifr;
    struct ethtool_value edata;
    edata.cmd = ETHTOOL_GLINK;
    edata.data = 0;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;
    if (( skfd = socket( AF_INET, SOCK_DGRAM, 0 )) == 0)
        return -1;
    if(ioctl( skfd, SIOCETHTOOL, &ifr ) == -1)
    {
        close(skfd);
        return -1;
    }
    close(skfd);
    return edata.data;
}
