#pragma once
#include <stdint.h>
#include <string>

typedef struct _MD5Context {
	uint32_t A, B, C, D;
	uint32_t data[16];
	uint64_t num;
} MD5Context;

class CryptUtils
{
private:
	CryptUtils();
	virtual ~CryptUtils();
public:
	static int MD5_Init(MD5Context *c);
	static int MD5_Update(MD5Context *c, const void *data, size_t len);
	static int MD5_Final(unsigned char *md, MD5Context *c);
public:
	static std::string ToHexString(const uint8_t* data, const int size);
public:
	static void MD5FromFile(const std::string& path, uint8_t outmd[16]);
	static std::string MD5FromFile(const std::string& path);
	static std::string MD5FromData(const char* data, const int size);
	static std::string MD5FromString(const std::string& str);
};

