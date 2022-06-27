#include "udmabuf.hpp"

namespace common
{

    udmabuf::udmabuf(const std::string dev_name, int mmp_size) : device_name_(dev_name), mmp_size_(mmp_size)
    {
        unsigned char attr[1024];
        int fd;

        // set phy address path

        std::string phy_addr_path = "/sys/class/u-dma-buf/" + device_name_ + "/phys_addr";

        if ((fd = open(phy_addr_path.c_str(), O_RDONLY)) != -1)
        {
            read(fd, attr, 1024);
            sscanf((const char *)attr, "%lx", &phy_addr_);
            close(fd);
        }

        // set size address path
        std::string size_path = "/sys/class/u-dma-buf/" + device_name_ + "/size";

        if ((fd = open(size_path.c_str(), O_RDONLY)) != -1)
        {
            read(fd, attr, 1024);
            sscanf((const char *)attr, "%lx", &total_buf_size_);
            close(fd);
        }

        // set device path
        std::string device_path = "/dev/" + device_name_;

        // mmap
        fd_ = open(device_path.c_str(), O_RDWR);
        virtual_addr_ = (char *)mmap(NULL, mmp_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    }

    udmabuf::~udmabuf()
    {
        munmap(virtual_addr_, mmp_size_);
        close(fd_);
    }

} // namespace common