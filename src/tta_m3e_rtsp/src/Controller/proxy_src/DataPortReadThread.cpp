#include "DataPortReadThread.h"
#include "Utils.h"
#include "Log.h"

DataPortReadThread::DataPortReadThread(DataPort* dataport, Port* port) :
	mDataPort(dataport),
	mPort(port)
{
}

DataPortReadThread::~DataPortReadThread()
{
}

void DataPortReadThread::threadFunc()
{
	char buf[1024];
	int peerNo;

	while (!done()) {
		if (mPort && mDataPort) {
			int ret = mPort->recv(buf, 1024, &peerNo);
			if(ret > 0){
				BufferItem* item = new BufferItem(buf, ret, peerNo);
				mDataPort->income(item);
			}else {
				Utils::Delay(10);
			}
		}else {
			Utils::Delay(1000);
		}
	}
}
