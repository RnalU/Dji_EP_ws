#pragma once
#include "Buffer.h"
class DataPort
{
public:
	DataPort();
	virtual ~DataPort();
	virtual int start() = 0;
	virtual int stop() = 0;
	virtual const char* name() = 0;
	BufferItem* recv();
	void send(BufferItem* item);
	int getIncomeSize();
	int getOutcomeSize();
private:
	void income(BufferItem* item);
	BufferItem* outcome();
private:
	Buffer mIncomeBuffer;
	Buffer mOutcomeBuffer;
	int mMode;
public:
	static const int MODE_NONE = 0;
	static const int MODE_INCOME = 0x01;
	static const int MODE_OUTCOME = 0x02;

	static const int MODE_MULTICAST = 0x04;
	static const int MODE_BOARDCAST = 0x08;

	static const int MODE_MULTIPEER = 0x10;
	static const int MODE_NO_RESET = 0X20;

	friend class DataPortReadThread;
	friend class DataPortSendThread;
};

