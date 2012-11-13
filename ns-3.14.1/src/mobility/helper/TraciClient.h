#ifndef TRACI_CLIENT_H
#define TRACI_CLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h> 
#include <iostream>

namespace ns3 {

class TraciClient
{
private:
	int serverPort;
	char serverIP[50];
	
	int sockfd;
	struct sockaddr_in sendAddr, recvAddr;
	struct hostent* server;
	unsigned int sendAddrLen, recvAddrLen;

public:
	TraciClient();
	void start();
	void stop();
	int sendData(char* buf, int buflen);
	int recvData(char* buf, int buflen);
	void setRecvTimeout(int timeSecs, int timeMicroSecs);
};

} // namespace ns3

#endif /* TRACI_CLIENT_H */
