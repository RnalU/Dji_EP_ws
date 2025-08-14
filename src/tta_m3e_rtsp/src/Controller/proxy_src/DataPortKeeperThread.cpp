#include "DataPortKeeperThread.h"
#include "Utils.h"
#include "Log.h"
#include "stdio.h"

// 数据端口数据转发进程解析函数
DataPortKeeperThread::DataPortKeeperThread(DataPort* dataport, Port* port, int mode):
	mDataPort(dataport),
	mPort(port),
	mMode(mode)
{
		printf("new --------------------------------------DataPortKeeperThread\r\n");
}

// 数据端口数据转发进程析构函数
DataPortKeeperThread::~DataPortKeeperThread()
{
	if(mReadThread){
		delete mReadThread;
		mReadThread = NULL;
	}

	if(mSendThread){
		delete mSendThread;
		mSendThread = NULL;
	}
}

void DataPortKeeperThread::startThreads()
{
	if (mMode & DataPort::MODE_INCOME) {
		mReadThread = new DataPortReadThread(mDataPort, mPort);
		mReadThread->start();
	}

	if (mMode & DataPort::MODE_OUTCOME) {
		mSendThread = new DataPortSendThread(mDataPort, mPort);
		mSendThread->start();
	}
}

void DataPortKeeperThread::stopThreads()
{
	if(mMode & DataPort::MODE_INCOME) {
		mReadThread->stop();
		delete mReadThread;
		mReadThread = NULL;
	}

	if(mMode & DataPort::MODE_OUTCOME) {
		mSendThread->stop();
		delete mSendThread;
		mSendThread = NULL;
	}
}

void DataPortKeeperThread::threadFunc()
{
	startThreads();
	while (!done()) {
		if(mPort){
			if(!mPort->valid()){
				stopThreads();
				mPort->disconnect();
				mPort->connect();
				startThreads();
			}
		}
		Utils::Delay(1000);
	}
	stopThreads();
	Log::i("stop read and send thread ok.");
}
