#pragma once
#include "Thread.h"
#include "DataPort.h"
#include "Port.h"
#include "DataPortReadThread.h"
#include "DataPortSendThread.h"
class DataPortKeeperThread : public Thread
{
public:
	DataPortKeeperThread(DataPort* dataport, Port* port, int mode);
	virtual ~DataPortKeeperThread();
protected:
	virtual void threadFunc();
private:
	void startThreads();
	void stopThreads();
private:
	DataPort* mDataPort;
	Port* mPort;
	int mMode;
	DataPortReadThread* mReadThread;
	DataPortSendThread* mSendThread;
};

