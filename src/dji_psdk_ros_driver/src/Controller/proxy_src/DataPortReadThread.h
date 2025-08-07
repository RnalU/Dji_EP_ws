#pragma once
#include "Thread.h"
#include "DataPort.h"
#include "Port.h"
class DataPortReadThread : public Thread
{
public:
	DataPortReadThread(DataPort* dataport, Port* port);
	virtual ~DataPortReadThread();
protected:
	virtual void threadFunc();
private:
	DataPort* mDataPort;
	Port* mPort;
};

