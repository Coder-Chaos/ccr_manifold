#pragma once
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>

namespace common
{
    enum protocal_type
    {
        TCP,
        UDP
    };

    enum type
    {
        SERVER,
        CLIENT
    };

    class Net
    {
    private:
        struct sockaddr_in address_;

        // socket fd
        int sockfd_;

        // server socket
        // int sockfd_server_;

        // protocal type
        protocal_type p_type_;

        // server or client?
        type type_;

        // server ip address
        const char *server_ip_addr_;

        // client ip address
        const char *client_ip_addr_;

        // port
        int port_;

    public:
        Net(){};
        ~Net()
        {
            close(sockfd_);
        };

        // construct with type, server ip address and port
        explicit Net(const protocal_type &p_type, const type &type, const char *serv_ip_addr, const int &port);

        // udp send
        int UdpSendtoServer(const void *buf, int len);
    };

} // namespace common
