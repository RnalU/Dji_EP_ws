#pragma once
#include <list>
#include "BufferItem.h"
#include "Lock.h"

class Buffer
{
public:
	Buffer();
	Buffer(int capacity);
	virtual ~Buffer();
	void push(BufferItem* item);
	BufferItem* pop();
	void clear();
	int getSize();
	int getCapacity();
private:
	void dropLocked();
	void pushLocked(BufferItem* item);
	BufferItem* popLocked();
private:
	std::list<BufferItem*> queue;
	int capacity;	// 缓冲区最大大小
	int size;		// 对象BufferItem的大小：缓冲区*一帧数据
	int mQueueSize;	// 缓冲区当前数据的大小
	Lock lock;
};

