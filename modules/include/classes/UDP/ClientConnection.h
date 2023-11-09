
#include <iostream>
#include <sstream>
#include <unistd.h> 
#include <string.h> 
#include <sys/socket.h> 		
#include <netinet/in.h> 
#include <arpa/inet.h> 
#include <fcntl.h>
#include <boost/log/trivial.hpp>
		
#define INVALID_SOCKET -1
#define IS_TCP 0

using namespace std;

class Connection
{
public:
	Connection( int BufLen = 128, bool bTransportProtocol = true ) 
		{ theSocket = INVALID_SOCKET;bIsTCP = bTransportProtocol; nBufLen = BufLen; }
	int Initialize(void);
	int Send(char* pBuffer, int nLen);
	int Receive(char* pBuffer, int& nLen);
	void SetTransportProtocol( bool bistcp ){ bIsTCP = bistcp;}
	int GetSocket(){ return theSocket; }
protected:
	int nBufLen;
	int theSocket;
	struct sockaddr_in addr;
	bool bIsTCP;
};

int Connection::Initialize(){
	int nRet = 0;
	return nRet;
}

/*
int Connection::Receive(char* pBuffer, int& nLen)
Attempt to receive nLen characters
* returns 0 on success, -1 on fail  nLen contains number of bytes received
*/
int Connection::Receive(char* pBuffer, int& nLen)
{
	int nRet = 0;
	int iResult = 0;
	if (nLen <= 0)
	{
		//~ ROS_INFO_STREAM("Can't receive -0 bytes");
		nRet = -1;
	}
	else
	{
		int nBytesRequested = nLen;
		if( bIsTCP == true )
		{
			iResult = recv(theSocket, pBuffer, nLen, 0);		
		}
		else		
		{
			socklen_t addrlen = sizeof(addr);		
			iResult = recvfrom(theSocket, pBuffer, nLen, 0, (struct sockaddr*) & addr, &addrlen);		
		}
		if (iResult == nLen)
		{	/* SUCCESS do nothing*/
			//~ ROS_INFO_STREAM("SUCCESS");
		}
		else if (iResult > 0 && iResult < nLen)
		{
			//~ nRet = -1;
			/*ROS_INFO_STREAM("INCOMPLETE MESSAGE");
			ROS_INFO_STREAM("Received:" << iResult);
			ROS_INFO_STREAM("Sent: " << nLen);*/
			nRet = nLen - iResult;
			
			// ROS_INFO_STREAM("Lost: " << nRet);
			//~ ROS_INFO_STREAM("<- (size: " << iResult << " bytes): " << pBuffer);
		}
		else if (iResult == 0)
		{
			ROS_INFO_STREAM("Connection closed");
			nRet = -1000;
		}
		else
		{
			ROS_INFO_STREAM("Receive recv failed ");
			nRet = -1000;
		}		
	}
	nLen = iResult; // how many bytes were actually read?	
	return nRet;
}

int Connection::Send(char* pBuffer, int nBytes)
{
	int nRet = 0;
	int iResult = 0;
	char* pLocal = pBuffer;
	int nSend;
	while (nBytes > 0){	
		if (nBytes > nBufLen){
			nSend = nBufLen;
		}else{
			nSend = nBytes;
		}
		nSend = nBytes;
		if( bIsTCP == true )
			iResult = send(theSocket, pLocal, nSend, 0);		
		else{			
			iResult = sendto(theSocket, pLocal, nSend, 0, (struct sockaddr*) & addr, sizeof(addr));
			char *IPaddr = inet_ntoa(addr.sin_addr);			
		}
		if (iResult == -1){
			ROS_INFO_STREAM("Connection::Send failed");
			nRet=-1;
			break;
		}
		nBytes -= nSend;
		pLocal += nSend;
	}
	return nRet;
}

class ClientConnection : public Connection
{
public:
	ClientConnection( std::string Address, int nPort, int BufLen = 128, bool bTransportProtocol = true , int rcvMaxBuf = 10650);
	int Initialize(void);
	int DeInitialize(void);
	void SetPort(int inport){ port = inport;}
protected:
	std::string IPaddress;
	int port;
	int soRcvMaxBuf;
private:
};

/*
ClientConnection::ClientConnection( char *pAddress, int nPort, int BufLen = 128, bool bTransportProtocol = true ) : Connection( bTransportProtocol ) 


*/
ClientConnection::ClientConnection( string pAddress, int nPort, int BufLen, bool bTransportProtocol , int rcvMaxBuf) : Connection( BufLen, bTransportProtocol ) 
{ 
	IPaddress = pAddress; // una IP tiene por lo mas 15 caracteres mas crlf o lf o 0
	port = nPort;
	soRcvMaxBuf = rcvMaxBuf;
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=1000;
	setsockopt(theSocket,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
}

/*
int ClientConnection::Initialize(void)

Attempts to connect theSocket to server
success == 0, fail == -1

*/
int ClientConnection::Initialize(void)
{
	int nRet = 0;
	int iResult;
		
	iResult = Connection::Initialize();

	if (iResult == 0)
	{
		//~ ROS_INFO_STREAM("bIsTCP: ");
		//~ ROS_INFO_STREAM(bIsTCP);
		if( bIsTCP == true )
			theSocket = socket(AF_INET, SOCK_STREAM, 0);
		else
			theSocket = socket(AF_INET, SOCK_DGRAM, 0);
		
		socklen_t recv_buffer_len = sizeof(soRcvMaxBuf);
		nRet = setsockopt(theSocket,SOL_SOCKET,SO_RCVBUF,&soRcvMaxBuf,recv_buffer_len);
		if (nRet)
			ROS_ERROR_STREAM("------ nret = " << nRet << " SO_RCVBUF can't be set with: " << soRcvMaxBuf << " bytes");
		
		nRet = 0;
		//~ ROS_INFO_STREAM("theSocket: ");
		//~ ROS_INFO_STREAM(theSocket);
		if(theSocket != 0) 
		{
			fcntl(theSocket, F_SETFL, O_NONBLOCK);
			addr.sin_family = AF_INET; 
			addr.sin_port = htons(port); 
			//~ ROS_INFO_STREAM("Connection IP address: "+IPaddress);
			//~ ROS_INFO_STREAM("Connection port: "+std::to_string(port));
       
       		if( bIsTCP == true )
       		{
				int inet_pton_add = inet_pton(AF_INET, IPaddress.c_str(), &(addr.sin_addr));
       		
				// Convert IPv4 and IPv6 addresses from text to binary form 
				if(inet_pton_add > 0)  
				{ 
					if (connect(theSocket, (struct sockaddr *) &addr, sizeof(addr)) == 0) 
					{ 
						// SUCCESS CONNECTED
						ROS_INFO_STREAM("Connected!");
					} 
					else
					{
						ROS_INFO_STREAM("Connection failed!");
						nRet = -1;
					}
				} 
				else
				{
					ROS_INFO_STREAM("Server address invalid");
					nRet = -1;
				}
			}	
			else
			{	// TODO: What do we do with the address?
				int inet_pton_add = inet_pton(AF_INET, IPaddress.c_str(), &(addr.sin_addr.s_addr));
				//~ ROS_INFO_STREAM("inet_pton_add: ");
				//~ ROS_INFO_STREAM(inet_pton_add);
			}	
		}
		else 
		{ 
			ROS_INFO_STREAM("Couldn't socket()");
			nRet =- -1; 
		} 
	}
	else
	{
		ROS_INFO_STREAM("ClientConnection initialization failed!");
		nRet = -1;
	}
	return nRet;
}

int ClientConnection::DeInitialize( void )
{
	int iResult;

	// shutdown the connection since no more data will be sent
	iResult = close( theSocket );
	
	return iResult;
}
