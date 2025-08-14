#include "DataPortSendThread.h"
#include "Utils.h"
#include "Log.h"

DataPortSendThread::DataPortSendThread(DataPort* dataport, Port* port) :
	mDataPort(dataport),
	mPort(port)
{
}

DataPortSendThread::~DataPortSendThread()
{
}

void DataPortSendThread::threadFunc()
{
	while (!done()) {
		if(mPort && mDataPort){
			BufferItem* item = mDataPort->outcome();
			if(item){
				int ret = mPort->send(item->data(), item->size(), item->mPeerNo);
				delete item;

				Utils::Delay(10);
			}else{
				Utils::Delay(10);
			}
		}else{
			Utils::Delay(1000);
		}
	}
}
