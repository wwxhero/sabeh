/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1999 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: sockcode.cxx,v 1.36 2016/10/28 20:47:57 IOWA\dheitbri Exp $
// Author:       Yiannis Papelis
// Date:         August, 1999
//
// Description:  A "portable" interface to sockets.
//
/////////////////////////////////////////////////////////////////////////////
#include "hcsmconnect.h"

// socket specific includes
#ifdef _WIN32

#elif __sgi
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <errno.h>
typedef bool BOOL;
#define TRUE true
#define FALSE false

#elif _PowerMAXOS

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/fcntl.h>
#include <errno.h>
typedef bool BOOL;
#endif

#undef _HCSM_DEBUG_SOCK_
//#define _HCSM_DEBUG_SOCK_

////////////////////////////////////////////////////////////////////////////
//
// Description: connect to a socket using the specified port
//
// Remarks:
// This function attempts to connect to a socket using the specified
// port, on the machine whose IP name or IP address is specified
// in the first argument.
//
// The function will block until a connection is made.
//
// Arguments:
// IpAddr   - the ip address or ip name of the machine to connect to
// port     - the port to connect to
//
// Return value:
// The function returns -1 to indicate an error or the socket number
// if a connection was established
//
SOCKET
ConnectToSocket(const char IpAddr[], int port)
{
	unsigned long RemAddr = inet_addr((char *)IpAddr);
	sockaddr_in   sinRem;
	SOCKET        sock;

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "->Enter ConnectToSocket(%s, %d)\n", IpAddr, port);
#endif

	if ( RemAddr == INADDR_NONE ) {
		hostent  *pHE = gethostbyname((char *)IpAddr);
		if ( pHE == 0 ) {
			return -1;
		}
		RemAddr = *((unsigned long *)pHE->h_addr_list[0]);
	}
	if ( RemAddr == INADDR_NONE ) {
		return -1;
	}


#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "Resolved address ok.\n");
#endif

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if ( sock < 0 ) {
#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "<-socket(..) failed, (err=%d) return -1\n",
			WSAGetLastError());
#else
		fprintf(stderr, "<-socket(..) failed, (err=%d) return -1\n", 
			errno);
#endif
#endif
		return -1;
	}

#ifdef _WIN32
	BOOL nOptionValue=TRUE;
	char *pOpt = reinterpret_cast<char*>(&nOptionValue);
 	if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, 
							pOpt, sizeof(BOOL))==SOCKET_ERROR) {
#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "Warning! Error in setting TCP_NODELAY to TRUE, err=%d.\n",
				WSAGetLastError());
#else
		fprintf(stderr, "Warning! Error in setting TCP_NODELAY to TRUE, err=%d\n",
				errno);
#endif
#endif
		return -1;
	}
#endif

	sinRem.sin_family = AF_INET;
	sinRem.sin_addr.s_addr = RemAddr;
	sinRem.sin_port = htons(port);
	if ( connect(sock, (sockaddr *)&sinRem, sizeof(sockaddr_in)) <0 ) {
#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "<-Connect failed,(err=%d) return -1\n",
			WSAGetLastError());
#else
		fprintf(stderr, "<-Connect failed, (err=%d) return -1\n", 
			errno);
#endif
#endif
		return -1;
	}

	return sock;
}


////////////////////////////////////////////////////////////////////////////
//
// This function creates a socket that is ready to receive connections
// but creates the socket as non blocking and does not wait for
// it to get a connection before returning.  The PollSocketForConnection
// function should be used to check for connections.
//
SOCKET
CreateNonBlockingSocket(int port)
{
	SOCKET       listenSock;
	sockaddr_in  myAddr = { 0 };

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "->Enter CreateNonBlockingSocket, port = %d\n", port);
#endif

	//
	// create & bind the socket
	//
	listenSock = socket(AF_INET, SOCK_STREAM, 0);
	if ( listenSock < 0 ) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "<-Exit with -1, socket() call failed\n");
#endif
		return -1;
	}

	myAddr.sin_family      = AF_INET;
	myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myAddr.sin_port        = htons(port);

#ifdef _WIN32
	BOOL nOptionValue=TRUE;
	char *pOpt = reinterpret_cast<char*>(&nOptionValue);
 	setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, 
			pOpt, sizeof(BOOL));
#else
	int doit = 1;
	setsockopt( listenSock, SOL_SOCKET, SO_REUSEADDR, &doit, sizeof(int));
#endif

	if ( ::bind(listenSock, (sockaddr *)&myAddr, sizeof(myAddr)) < 0 ) {
#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "<-Exit with -1, bind() call failed: %d\n",
				WSAGetLastError());
#else
		fprintf(stderr, "<-Exit with -1, bind() call failed: %d\n", errno );
#endif
#endif
		return -1;
	}

	//
	// Listen for connections.  We are actually waiting for a single 
	// connection from the client library.  Once a connection is
	// activated, we simply monitor for messages and implement what
	// the messages say.
	//
	if ( listen(listenSock, 2) < 0 ) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "<-Exit with -1, listen() call failed\n");
#endif
		return -1;
	}
	
#ifdef _WIN32
	unsigned long setit = 1;
	if ( ioctlsocket(listenSock, FIONBIO, &setit) == SOCKET_ERROR ) {
#elif __sgi
	int setit = 1;
	if ( ioctl(listenSock, FIONBIO, &setit) < 0 ) {
#elif _PowerMAXOS
	int setit = 1;
	//if ( fcntl(listenSock, O_NONBLOCK, &setit) < 0 ) {
	if ( fcntl(listenSock, F_SETFL,  O_NDELAY, &setit) < 0 ) {
#else
	syntax_error
#endif

#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "<-Exit with -1, ioctl() call failed (err=%d)\n", 
			WSAGetLastError());
#else
		fprintf(stderr, "<-Exit with -1, ioctl() call failed (err=%d)\n", 
			errno);
#endif
#endif
		return -1;
	}

#ifdef _WIN32
	nOptionValue=TRUE;
	pOpt = reinterpret_cast<char*>(&nOptionValue);
 	if (setsockopt(listenSock, IPPROTO_TCP, TCP_NODELAY, 
			pOpt, sizeof(BOOL))==SOCKET_ERROR) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "Warning! Error in setting TCP_NODELAY to TRUE, err=%d.\n",
				WSAGetLastError());
#endif
		return -1;
	}
#endif


#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "<-Exit with %d (OK)\n", listenSock);
#endif
	return listenSock;
}


SOCKET
PollSocketForConnection(int sock)
{
	SOCKET       cmdSock;
	int          cliLen  = sizeof(sockaddr_in);
	sockaddr_in  clientAddr;

	cmdSock = accept(sock, (sockaddr *)&clientAddr, &cliLen);
	if ( cmdSock == INVALID_SOCKET ) {
#ifdef _WIN32
		if ( WSAGetLastError() == WSAEWOULDBLOCK ) {
#else
		if ( errno == EWOULDBLOCK ) {
#endif
			return 0;
		}
		else {
#ifdef _WIN32
#else
			fprintf(stderr, "PollSocket: accept failed: %d\n", errno);
#endif
			return -1;
		}
	}

	return cmdSock;
}


SOCKET
WaitForConnection(int port)
{
	SOCKET       listenSock;
	SOCKET       cmdSock;
	sockaddr_in  myAddr = { 0 };
	int          cliLen  = sizeof(sockaddr_in);
	sockaddr_in  clientAddr;

	//
	// create & bind the socket
	//
	listenSock = socket(AF_INET, SOCK_STREAM, 0);
	if ( listenSock < 0 ) {
		cerr << "Cannot create listening socket." << endl;
		return -1;
	}

	myAddr.sin_family      = AF_INET;
	myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myAddr.sin_port        = htons(port);


	if ( bind(listenSock, (sockaddr *)&myAddr, sizeof(myAddr)) < 0 ) {
		cerr << "Bind failed." << endl;
		return -1;
	}

	//
	// Listen for connections.  We are actually waiting for a single 
	// connection from the client library.  Once a connection is
	// activated, we simply monitor for messages and implement what
	// the messages say.
	//
	if ( listen(listenSock, 2) < 0 ) {
		cerr << "Listen failed." << endl;
		return -1;
	}
	
	cmdSock = accept(listenSock, (sockaddr *)&clientAddr, &cliLen);
	if ( cmdSock < 0 ) {
		cerr << "Accept failed." << endl;
		return -1;
	}

#ifdef _WIN32
	BOOL nOptionValue=TRUE;
	char *pOpt = reinterpret_cast<char*>(&nOptionValue);
 	if (setsockopt(cmdSock, IPPROTO_TCP, TCP_NODELAY, 
			pOpt, sizeof(BOOL))==SOCKET_ERROR) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "Warning! Error in setting TCP_NODELAY to TRUE, err=%d.\n",
				WSAGetLastError());
#endif
		return -1;
	}
#endif

	return cmdSock;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: read the header of a message from a socket
//
// Remarks:
// Read a message header from the specified socket.  Also, make
// sure the header is a valid HCSM communication message header,
// convert the data to adjust for big/little endian.
//
// Returns:
// A negative number in case of an error or the number of
// bytes read.  If the socket is marked as non-blocking
// and the read fails because there is just not data, the function
// returns 0
//
int
ReadHeader(SOCKET sock, TMsgHeader &head)
{
	int bytesRead ;

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "->Enter ReadHeader(%d,)\n", sock);
#endif

	bytesRead = recv(sock, (char *)&head, sizeof(head), 0);
	if ( bytesRead < 0 ) {
#ifdef _WIN32
		if ( WSAGetLastError() == WSAEWOULDBLOCK ) return 0;
#else
		if ( errno == EWOULDBLOCK ) return 0;
#endif
#ifdef _HCSM_DEBUG_SOCK_
#ifdef _WIN32
		fprintf(stderr, "Last error is %d\n", WSAGetLastError());
#else
		fprintf(stderr, "Last error is %d\n", errno);
#endif
		fprintf(stderr, "<-Exit with -1\n");
#endif
		return -1;
	}
	if ( bytesRead != sizeof(head) ) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr,"<-Exit with -1, expected %d, got %d\n",
					sizeof(head), bytesRead);
#endif
		return -1;
	}

	head.m_OpCode    = ntohs(head.m_OpCode);
	head.m_MsgLen    = ntohl(head.m_MsgLen);
	head.m_StartMark = ntohs(head.m_StartMark);

	if ( head.m_StartMark != (short)START_MARK ) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "<-Exit with -1, read header ok, but start "
							"mark is wrong.");
		fprintf(stderr, "It is 0x%X, should be 0x%X\n",
				head.m_StartMark, START_MARK);
#endif
		return -1;
	}

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "ReadHeader, read header with cmd = %d, msglen=%d\n",
					head.m_OpCode, head.m_MsgLen);
#endif
	return bytesRead;
}


////////////////////////////////////////////////////////////////////////////
//
// Description: read a message from a socket
//
// Remarks:
// Read a message from the specified socket.  The message header
// has already been read and is provided as an argument.  The
// expected size of the message is read from the header.
// The message contents is stored in the specified structure.
//
// Returns:
// A negative number in case of an error or the number of
// bytes read.  If the socket is marked as non-blocking
// and the read fails because there is just not data, the function
// returns 0
//
int
ReadMessage(SOCKET sock, const TMsgHeader &head, TMessage &msg)
{
	int     bytesRead = 0;
	int     bytesToRead = head.m_MsgLen - sizeof(head);

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "Enter ReadMessage, bytes2read = %d\n", bytesToRead);
#endif

	while ( bytesRead < bytesToRead ) {
		int packet = recv(sock, (char *)msg.data.bytes + bytesRead, 
					bytesToRead - bytesRead, 0);

		if ( packet < 0 ) {
#ifdef _WIN32
		if ( WSAGetLastError() == WSAEWOULDBLOCK ) return 0;
#else
		if ( errno == EWOULDBLOCK ) return 0;
#endif
#ifdef _HCSM_DEBUG_SOCK_
			fprintf(stderr, "Read on message failed.\n");
#endif
			return -1;
		}

		bytesRead += packet;
	}

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "ReadMessage returns ok\n");
#endif
	return bytesRead;
}


///////////////////////////////////////////////////////////////////
//
// Description: send a message to the specified socket
//
// Remarks:
// This function creates and sends a message using the specified
// parameters.  Messages have a command code and a buffer of
// data.  Each message consists of a header and the body.
// The function creates the appropriate header, and performs
// any necessary network-order conversions, then sends the
// header and the body in a single message.
//
// If the size is specified as 0, the function will not 
// dereference the pointer and will only send the header.
//
// Return value:
// The function returns true to indicate that the message
// was sent, otherwise it returns false.  A return of false almost
// always indicate some problem with the socket connection.
//
// Arguments:
// sock - the socket to use for sending the message
// cmd  - the command to associate with the message
// pBuf - pointer to the data for the message
// size - the size of the data pointed to by pBuf
//
bool 
SendMessage(SOCKET sock, int cmd, const void *pBuf, int size)
{
	TMessage   msg;
	int        length;

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "->Enter SendMessage(sock=%d, cmd=%d, size=%d)\n",
				sock, cmd, size);
#endif

	length = size + sizeof(TMsgHeader);

	msg.header.m_StartMark = htons(START_MARK);
	msg.header.m_OpCode    = htons(cmd);
	msg.header.m_MsgLen    = htonl(length);

	// some messages may have zero length in which case the
	// pointer can be NULL so we don't even risk dereferencing
	int BytesSent;

	if ( size > 0 )  {
		memcpy(msg.data.bytes, pBuf, size);
	}

	BytesSent = send(sock, (char *)&msg, length, 0);

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "<-Exit SendMessage; BytesSent = %d, length=%d\n", 
						BytesSent, length);
#ifdef _WIN32
	fprintf(stderr, "Last error is %d\n", WSAGetLastError() );
#else
	fprintf(stderr, "Last error is %d\n", errno );
#endif
#endif

	return (BytesSent == length);
}


/////////////////////////////////////////////////////////////////////
//
// Description: read a message
//
// Remarks:
// This function reads a message from the specified socket.  The
// header is read first, and if there is a body, it is also
// read.
//
// Return value:
// The function returns -1 to indicate an error,  0 to
// indicate that the operation would block, or 1 to indicate
// that a message was read successfully.  
//
// Arguments:
// sock - the socket to use for reading the message
// msg  - the message that was read
//
int
RecvMessage(SOCKET sock, TMessage &msg )
{

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "->Enter RecvMessage(sock=%d,): ", sock);
#endif
	int code = ReadHeader(sock, msg.header);

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "ReadHead returned %d, msg size = %d\n", code,
					msg.header.m_MsgLen);
#endif
	if ( code <= 0 ) return code;

	// if the body length is zero, don't issue a read
	if ( msg.header.m_MsgLen == sizeof(TMsgHeader) ) {
#ifdef _HCSM_DEBUG_SOCK_
		fprintf(stderr, "Size of body is zero, return 1\n");
#endif
		return 1;
	}
	code = ReadMessage(sock, msg.header, msg);

#ifdef _HCSM_DEBUG_SOCK_
	fprintf(stderr, "<- Exit ReadMessage() with %d\n", code);
#endif
	return code;
}


/////////////////////////////////////////////////////////////////////
//
// Description: close a socket
//
// Remarks:
// This function simply closes the socket.  The primary reason
// for having this function is to ensure portability across
// the different platforms.
//
// Return value:
// True for success, or false
//
// Arguments:
// sock - the socket to use for reading the message
//
bool
CloseSocket(SOCKET sock)
{
#ifdef _WIN32
	if (closesocket(sock) == 0) {
		return true;
	}
#else
	if (close(sock) == 0) {
		return true;
	}
#endif
	else {
#ifdef _WIN32
		printf("Close(%d) failed:%d\n", sock, WSAGetLastError());
#else
		printf("Close(%d) failed:%d\n", sock, errno);
#endif
	}
	return false;
}


static void htonf( float input, float& output )
{
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = htonl( tmp );
	memcpy( &output, &tmp, sizeof(tmp) );
}



static void htonf( float input, double& outputD )
{
	float output;
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = htonl( tmp );
	memcpy( &output, &tmp, sizeof(tmp) );
	outputD = static_cast<double>(output);
}



static void htonf( double inputD, double& outputD )
{
	float input = static_cast<float>(inputD);
	float output;
	unsigned long tmp;
	memcpy( &tmp, &input, sizeof(tmp) );
	tmp = htonl( tmp );
	memcpy( &output, &tmp, sizeof(tmp) );
	outputD = static_cast<double>(output);
}



void
CvedDataToCommData( const cvTObjState& cvState, TCommObjState& coState, cvEObjType objType, CCved* pCved, int id )
{
	htonf(cvState.anyState.position.x, coState.position.x);
	htonf(cvState.anyState.position.y, coState.position.y);
	htonf(cvState.anyState.position.z, coState.position.z);

	htonf(cvState.anyState.tangent.i, coState.tangent.i);
	htonf(cvState.anyState.tangent.j, coState.tangent.j);
	htonf(cvState.anyState.tangent.k, coState.tangent.k);

	htonf(cvState.anyState.lateral.i, coState.lateral.i);
	htonf(cvState.anyState.lateral.j, coState.lateral.j);
	htonf(cvState.anyState.lateral.k, coState.lateral.k);

	htonf(cvState.anyState.boundBox[0].x, coState.boundBox[0].x);
	htonf(cvState.anyState.boundBox[0].y, coState.boundBox[0].y);
	htonf(cvState.anyState.boundBox[0].z, coState.boundBox[0].z);

	htonf(cvState.anyState.boundBox[1].x, coState.boundBox[1].x);
	htonf(cvState.anyState.boundBox[1].y, coState.boundBox[1].y);
	htonf(cvState.anyState.boundBox[1].z, coState.boundBox[1].z);

	htonf(cvState.anyState.vel, coState.vel);

	// first copy the visualState flag, as most objects will use it
	coState.specific.visualState = htons( pCved->GetVehicleVisualState( id ) );
	switch ( objType ){
	case eCV_TRAFFIC_LIGHT:
		{
//		cerr << "TLstate is " << pCved->GetTrafficLightState(id) << endl;
		coState.specific.state = (eCVTrafficLightState)htonl(pCved->GetTrafficLightState(id));
//		cerr << "htonl(TLstate) is " << coState.specific.state << endl;
		}
		break;

	default:
//		cerr << "Default: Type is " << objType << endl;
		break;
	}
}



