#include "DataPort.h"

DataPort::DataPort()
{
}

DataPort::~DataPort()
{
}

// 从输入Buffer中获取一阵数据
BufferItem* DataPort::recv()
{
	return mIncomeBuffer.pop();
}

// 添加一个数据到输出Buffer中
void DataPort::send(BufferItem* item)
{
	mOutcomeBuffer.push(item);
}
// 从输出Buffer中获取一帧数据
BufferItem* DataPort::outcome()
{
	return mOutcomeBuffer.pop();
}

// 添加一帧数据到输入Buffer中
void DataPort::income(BufferItem* item)
{
	mIncomeBuffer.push(item);
}

// 获取输入数据的大小
int DataPort::getIncomeSize()
{
	return mIncomeBuffer.getSize();
}

// 获取输出数据的大小
int DataPort::getOutcomeSize()
{
	return mOutcomeBuffer.getSize();
}
