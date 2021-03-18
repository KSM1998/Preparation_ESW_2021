//#include "stdafx.h"
#include <stdio.h>
#include<winsock.h>					//소켓을 사용하기 위한 헤더파일

#pragma comment(lib, "ws2_32")		//위에서 선언한 헤더파일들을 가져다 쓰기위한 링크

#define PORT 4578
#define PACKET_SIZE 1024
#define SERVER_IP "211.32.63.121"

int main() 
{
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	SOCKET hSocket;
	hSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	SOCKADDR_IN tAddr = {};
	tAddr.sin_family = AF_INET;
	tAddr.sin_port = htons(PORT);
	tAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

	connect(hSocket, (SOCKADDR*)&tAddr, sizeof(tAddr));

	char cMsg[] = "Client Send";
	send(hSocket, cMsg, strlen(cMsg), 0);

	char cBuffer[PACKET_SIZE] = {};
	recv(hSocket, cBuffer, PACKET_SIZE, 0);

	printf("RECV Msg : %s\n", cBuffer);

	closesocket(hSocket);

	WSACleanup();

	return 0;
}