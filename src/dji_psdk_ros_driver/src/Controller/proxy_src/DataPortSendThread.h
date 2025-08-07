#pragma once
#include "Thread.h"
#include "DataPort.h"
#include "Port.h"
class DataPortSendThread : public Thread
{
public:
	DataPortSendThread(DataPort* dataport, Port* port);
	virtual ~DataPortSendThread();
protected:
	virtual void threadFunc();
private:
	DataPort* mDataPort;
	Port* mPort;
};

