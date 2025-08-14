#include "diskInfo.h"
#include <sys/statfs.h>
#include <sys/vfs.h>
#include <errno.h>
#include <string.h>

diskInfo::diskInfo(std::string path):
    mPath(path),
    mDiskSize(0),
    mDiskFreeSize(0),
    mDiskFileSystemType(0)
{
    

}

diskInfo::~diskInfo(){

}

int diskInfo::getDiskInfo(){
    struct statfs diskInfo;
    int ret = 0;
    uint64_t total=0, avail=0, free=0;

    if((ret = statfs(mPath.c_str(), &diskInfo)) == -1){
        printf("get filesystem info failed:%s", strerror(errno));
        return -1;
    }

    mDiskFileSystemType = diskInfo.f_type;
    mDiskSize = diskInfo.f_blocks * diskInfo.f_bsize;
    avail = diskInfo.f_bavail * diskInfo.f_bsize;

    return 0;
}

std::string diskInfo::getDiskFileSystemType(){
    
}

uint64_t diskInfo::getDiskFreeSize(){
    return mDiskFreeSize;
}

uint64_t diskInfo::getDiskSize(){
    return mDiskSize;
}