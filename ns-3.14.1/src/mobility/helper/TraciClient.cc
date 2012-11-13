#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h> 
#include <iostream>
#include "TraciClient.h"

using namespace std;

namespace ns3 {

TraciClient::TraciClient()
{	
	serverPort = 2004;
	strcpy(serverIP, "localhost");
	sendAddrLen = sizeof(sendAddr);
}

void TraciClient::start()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        cout << "ERROR opening socket" << endl;
        exit(1);
    }
    
    // Set Receive Timeout
    setRecvTimeout(1, 0);

    server = gethostbyname(serverIP);
    if (server == NULL) {
        cout << "ERROR, no such host" << endl;
        exit(1);
    }
    
    bzero((char *) &sendAddr, sizeof(sendAddr));
    sendAddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&sendAddr.sin_addr.s_addr,
    		server->h_length);
    sendAddr.sin_port = htons(serverPort);	
}

void TraciClient::stop()
{
	close(sockfd);
}

int TraciClient::recvData(char* buf, int buflen)
{
	bzero((char *) &recvAddr, sizeof(recvAddr));
	if(buf == NULL)
		return -1;
	
	int n = recvfrom(sockfd, buf, buflen, 0, 
						(struct sockaddr *)&recvAddr, &recvAddrLen);
	return n;
}

int TraciClient::sendData(char* buf, int buflen)
{
	int n = sendto(sockfd, buf, buflen, 0, 
					(const struct sockaddr*)&sendAddr, sendAddrLen);
	return n;
}

void TraciClient::setRecvTimeout(int timeSecs, int timeMicroSecs)
{
	struct timeval tv;
	tv.tv_sec = timeSecs;
	tv.tv_usec = timeMicroSecs;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
}

} // namespace ns3
