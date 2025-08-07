#include "Lock.h"


Lock::Lock():
mName("")
{
	pthread_mutex_init(&mMutex, NULL);
}


Lock::~Lock()
{
	pthread_mutex_destroy(&mMutex);
}

void Lock::lock()
{
	pthread_mutex_lock(&mMutex);
	if(mName != ""){
		printf("%s locked\n", mName.c_str());
	}
}

void Lock::unlock()
{
	pthread_mutex_unlock(&mMutex);
	if(mName != ""){
		printf("%s unlocked\n", mName.c_str());
	}
}

void Lock::lock(int line)
{
	pthread_mutex_lock(&mMutex);
	if(mName != ""){
		printf("%s locked @ %d\n", mName.c_str(), line);
	}
}

void Lock::unlock(int line)
{
	pthread_mutex_unlock(&mMutex);
	if(mName != ""){
		printf("%s unlocked %d\n", mName.c_str(), line);
	}
}

void Lock::setName(std::string name){
	mName = name;
}