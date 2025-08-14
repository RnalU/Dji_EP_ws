#pragma once

#include <string>
#include <stdint.h>

class diskInfo{
public:
    diskInfo(std::string path);
    ~diskInfo();

    int getDiskInfo();
    uint64_t getDiskSize();
    uint64_t getDiskFreeSize();
    std::string getDiskFileSystemType();

private:
    std::string mPath;
    uint64_t mDiskSize;
    uint64_t mDiskFreeSize;
    uint64_t mDiskFileSystemType;
};