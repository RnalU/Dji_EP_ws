#pragma once
#include <pthread.h>
#include <string>

class Lock
{
public:
	Lock();
	virtual ~Lock();
	void lock();
	void unlock();
	void lock(int line);
	void unlock(int line);
	void setName(std::string name);
private:
	pthread_mutex_t mMutex;
	std::string mName;
};

