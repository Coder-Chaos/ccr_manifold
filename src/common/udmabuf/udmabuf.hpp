#pragma once
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/mman.h>

namespace common
{

    class udmabuf
    {
    private:
        /* data */

        // fd
        int fd_;

        // buf virtual address
        char *virtual_addr_ = nullptr;

        // device name
        const std::string device_name_;

        // mmap size
        int mmp_size_;

        // u-dma-buf pyhsical address
        unsigned long phy_addr_;

        // u-dma-buf size
        unsigned long total_buf_size_;

    public:
        udmabuf(const std::string dev_name, int mmp_size);
        udmabuf(/* args */) {}
        ~udmabuf();

        // get u-dma-buf physical address
        unsigned long GetPhyAddr(void) const
        {
            return phy_addr_;
        }

        // get u-dma-buf virtual address.
        char *GetVirtualAddr(void) const
        {
            return virtual_addr_;
        }

        // get buf size
        unsigned long GetBufTotalSize(void) const
        {
            return total_buf_size_;
        }
    };
} // namespace common