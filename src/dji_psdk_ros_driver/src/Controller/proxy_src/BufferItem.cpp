#include "BufferItem.h"

// Buffer存储数据(初始化buffer的vector[长度为size])
BufferItem::BufferItem(const char* data, int size):
	buffer(data, data+size),
	mPeerNo(0)
{
}

BufferItem::BufferItem(const char* data, int size, int remoteAddr):
	buffer(data, data+size),
	mPeerNo(remoteAddr)
{

}


BufferItem::~BufferItem()
{
}

// 获取Buffer的vector数据的第一个地址
char* BufferItem::data()
{
	return buffer.data();
}

// 获取Buffer的vector的大小
int BufferItem::size()
{
	return buffer.size();
}

// 将新的数据保存到Buffer的最后
int BufferItem::mAdd(char *p, int len){
	for(int i=0; i<len; i++){
		buffer.push_back(p[i]);
	}

	return 0;
}

// 在Buffer中Vector的最后一个结束符'0/'
void BufferItem::endString(){
	buffer.push_back(0);
}
