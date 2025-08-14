#include "Thread.h"
#include "Log.h"

Thread::Thread() :
	mRunning(false),
	mStarted(false),
	mDone(true),
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
	mThread({0, 0})
#else
	mThread(NULL)
#endif
{
}


Thread::~Thread()
{
}


int Thread::start()
{
	//Log::d("Thread start ...");
	int err = 0;
	mActionLock.lock();
	if (mStarted) {
		err = -1;
		Log::d("Thread start fail : alread started.");
	}
	else {
		mDone = false;
		beforeThreadCreateLocked();
		mRunning = true;
		pthread_attr_t attr;
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		pthread_create(&mThread, &attr, ThreadWrapper, this);
		pthread_attr_destroy(&attr);
		afterThreadCreateLocked();
		mStarted = true;
		// Log::d("Thread start done");
	}
	mActionLock.unlock();
	return err;
}


int Thread::stop()
{
	//Log::d("Thread stop ...");
	int err = 0;
	mActionLock.lock();
	if (mStarted) {
		mDone = true;
		beforeThreadJoinLocked();
		pthread_join(mThread, NULL);
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
		mThread = { 0, 0 };
#else
		mThread = NULL;
#endif
		afterThreadJoinLocked();
		mStarted = false;
		// Log::d("Thread stop done");
	}
	else {
		err = -1;
		Log::d("Thread stop fail : not started yet.");
	}
	mActionLock.unlock();
	return err;
}

bool Thread::done()
{
	return mDone;
}
bool Thread::isStarted()
{
	return mStarted;
}
bool Thread::isRunning()
{
	return mRunning;
}
void Thread::threadFunc()
{
	while (!done()) {

	}
}
void Thread::beforeThreadCreateLocked()
{

}
void Thread::afterThreadCreateLocked()
{

}
void Thread::beforeThreadJoinLocked()
{

}
void Thread::afterThreadJoinLocked()
{

}

void* Thread::ThreadWrapper(void* me)
{
	Thread *th = static_cast<Thread *>(me);
	th->threadFunc();
	th->mDone = true;
	th->mRunning = false;
	return NULL;
}
