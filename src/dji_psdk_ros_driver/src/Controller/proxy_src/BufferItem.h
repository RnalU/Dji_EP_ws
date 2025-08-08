#pragma once
#include <vector>
class BufferItem
{
public:
	BufferItem(const char* data, int size);
	BufferItem(const char* data, int size, int peerNo);
	virtual ~BufferItem();
	int size();
	char* data();
	int mPeerNo;
	int mAdd(char *p, int len);
	void endString();
private:
	std::vector<char> buffer;
};

