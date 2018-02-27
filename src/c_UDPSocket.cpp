#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

#include <signal.h>

#include <iostream>

#define SERVER_ID   "127.0.0.1"
#define SERVER_PORT 19215
#define BUFFER_SIZE 1024

bool work = true;
void signalInterrupt(int sig) {
    work = false;
}

int main(int argc, char ** argv)
{
    struct sockaddr_in serverAddr;
    struct sockaddr_in remouteAddr;
    socklen_t addrlen = sizeof(remouteAddr);
    int receiveLen = 0;
    int fd = 0;
    unsigned char buff[BUFFER_SIZE];

    // Register signals
    signal(SIGINT, signalInterrupt);

    std::cout << "Start RSI Server." << std::endl;

    // Creating socket file descriptor
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        // TODO Add pretty error mesage
        std::cout << "Error: Can't create socket!" << std::endl;
    }

    // Identify a socket
    std::memset((char *)&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_ID);
    serverAddr.sin_port = htons(SERVER_PORT);

    if (bind(fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cout << "Error: Bind failed!" << std::endl;
        return -1;
    }

    while(work) {
        std::cout << "Waiting of port " << SERVER_PORT << "..." << std::endl;
        receiveLen = recvfrom(fd, buff, BUFFER_SIZE, 0, (struct sockaddr *)&remouteAddr, &addrlen);
        std::cout << "Received message length: " << receiveLen << std::endl;
        if (receiveLen > 0) {
            buff[receiveLen] = 0;
            std::cout << "Received message : " << buff << std::endl;
        }
    }

    return 0;
}
