#include "TCPConnection.h"



CTCPConnection::CTCPConnection()
{
	state = 0;
	connected = false;

	actionResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (state != 0)
	{
		printf("WSAStartup failed with error: %d\n", actionResult);
		state = 1;
	}
}


CTCPConnection::~CTCPConnection()
{
	if (connected)
		closeConnection();
}

int CTCPConnection::setupConnection(char* address, char* port)
{
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	actionResult = getaddrinfo(address, port, &hints, &result);
	if (actionResult != 0)
	{
		printf("getaddrinfo failed with error: %d\n", actionResult);
		WSACleanup();
		state = 2;
		return 0;
	}

	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET)
		{
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			state = 3;
			return 0;
		}

		// Connect to server.
		actionResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (actionResult == SOCKET_ERROR)
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET)
	{
		printf("Unable to connect to server!\n");
		WSACleanup();
		state = 4;
		return 0;
	}

	connected = true;
	return 1;
}

int CTCPConnection::sendData(const char* data)
{
	if (!connected)
	{
		state = 7;
		return 0;
	}
	actionResult = send(ConnectSocket, data, (int)strlen(data), 0);
	if (actionResult == SOCKET_ERROR)
	{
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		state = 5;
		return 0;
	}

	return 1;
}

int CTCPConnection::closeConnection()
{
	if (!connected)
	{
		state = 7;
		return 0;
	}
	actionResult = shutdown(ConnectSocket, SD_SEND);
	if (actionResult == SOCKET_ERROR)
	{
		printf("shutdown failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		state = 6;
		return 0;
	}

	connected = false;
	return 1;
}
