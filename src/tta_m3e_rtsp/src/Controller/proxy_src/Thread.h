#pragma once
#include <cstdbool>
#include <atomic>
#include "Lock.h"
class Thread
{
public:
	Thread();
	virtual ~Thread();
	int start();
	int stop();
	bool isStarted();
	bool isRunning();
protected:
	bool done();
	virtual void threadFunc();
	virtual void beforeThreadCreateLocked();
	virtual void afterThreadCreateLocked();
	virtual void beforeThreadJoinLocked();
	virtual void afterThreadJoinLocked();
private:
	static void* ThreadWrapper(void* me);


private:
	std::atomic<bool> mRunning;
	std::atomic<bool> mStarted;
	std::atomic<bool> mDone;
	Lock mActionLock;
	pthread_t mThread;
};

