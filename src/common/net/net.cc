#include "common.hpp"
#include "net.hpp"

namespace common
{
    Net::Net(const protocal_type &p_type, const type &type, const char *serv_ip_addr, const int &port) : p_type_{p_type}, type_{type}, server_ip_addr_{serv_ip_addr}, port_{port}
    {
        // check the type
        if (p_type_ == TCP)
        {
            /* code */
            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        }
        else if (p_type_ == UDP)
        {
            /* code */
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        }

        // set the address
        address_.sin_family = AF_INET;
        address_.sin_port = htons(port_);
        if (type_ == CLIENT)
        {
            // set dest ip address
            address_.sin_addr.s_addr = inet_addr(server_ip_addr_);
        }
        else if (type_ == SERVER)
        {
            // bind local ip address
            address_.sin_addr.s_addr = INADDR_ANY;
        }

        // bind if server
        if (type_ == SERVER)
        {
            int ret = bind(sockfd_, (struct sockaddr *)&address_, sizeof(address_));

            if (ret < 0)
            {
                /* code */
                perror("\rbind fail!\n");
                close(sockfd_);
                exit(-1);
            }
            else
            {
                printf("\rnetwork server created !\n");
                printf("\rserver address:%s, port:%d\n", server_ip_addr_, port_);
            }
        }
        else
        {
            printf("\rnetwork client created, port:%d.\n", port_);
        }
    }

    int Net::UdpSendtoServer(const void *buf, int len)
    {
        return sendto(sockfd_, buf, len, 0, (struct sockaddr *)&address_, sizeof(address_));
    }

} // namespace common