#include "CryptUtils.h"
#include <string.h>
#include <stdio.h>

#define INIT_DATA_A (uint32_t)0x67452301L
#define INIT_DATA_B (uint32_t)0xefcdab89L
#define INIT_DATA_C (uint32_t)0x98badcfeL
#define INIT_DATA_D (uint32_t)0x10325476L

static const uint8_t S[64] = {
 7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,
 5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,
 4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,
 6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,
};

static const uint32_t K[64] = {
0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
};

static const char HEX[] = "0123456789abcdef";

static void MD5_Main_Loop(MD5Context *c, uint32_t M[16])
{
	uint32_t A = c->A;
	uint32_t B = c->B;
	uint32_t C = c->C;
	uint32_t D = c->D;
	
	for (int i = 0; i < 64; i++) {
		uint32_t F, g;
		if (0 <= i && i < 16) {
			F = (B & C) | ((~B) & D);
			g = i;
		}
		else if (16 <= i && i < 32) {
			F = (D & B) | ((~D) & C);
			g = (5 * i + 1) % 16;
		}
		else if (32 <= i && i < 48) {
			F = B ^ C ^ D;
			g = (3 * i + 5) % 16;
		}
		else if (48 <= i && i < 64) {
			F = C ^ (B | (~D));
			g = (7 * i) % 16;
		}
		F = F + A + K[i] + M[g];
		A = D;
		D = C;
		C = B;
		B = B + ((F << S[i]) | (F >> (32 - S[i])));
	}
	c->A += A;
	c->B += B;
	c->C += C;
	c->D += D;
}

CryptUtils::CryptUtils()
{
}


CryptUtils::~CryptUtils()
{
}

int CryptUtils::MD5_Init(MD5Context *c)
{
	memset(c, 0, sizeof(*c));
	c->A = INIT_DATA_A;
	c->B = INIT_DATA_B;
	c->C = INIT_DATA_C;
	c->D = INIT_DATA_D;
	return 1;
}
int CryptUtils::MD5_Update(MD5Context *c, const void *data, size_t len)
{
	if (c) {
		int n = c->num % 64;
		int k = n + len;
		int offset = 0;
		while (k >= 64) {
			memcpy(((uint8_t*)c->data) + n, ((uint8_t*)data)+offset, 64 - n);
			MD5_Main_Loop(c, c->data);
			n = 0;
			offset += 64 - n;
			k -= 64;
		}
		if (k > 0) {
			memcpy(((uint8_t*)c->data) + n, ((uint8_t*)data) + offset, k);
			n = k;
		}
		c->num += len;
		return 1;
	}
	else {
		return 0;
	}
}
int CryptUtils::MD5_Final(unsigned char *md, MD5Context *c)
{
	if (c) {
		int n = c->num % 64;
		uint8_t fillbyte = 0x80;
		if (n >= 56) {
			for (int i = n; i < 64; i++) {
				*(((uint8_t*)c->data) + i) = fillbyte;
				if (fillbyte) {
					fillbyte = 0x00;
				}
			}
			MD5_Main_Loop(c, c->data);
			n = 0;
		}
		for (int i = n; i < 56; i++) {
			*(((uint8_t*)c->data) + i) = fillbyte;
			if (fillbyte) {
				fillbyte = 0x00;
			}
		}
		uint64_t numBits = c->num << 3;
		*(((uint8_t*)c->data) + 56) = numBits & 0xff;
		*(((uint8_t*)c->data) + 57) = (numBits >> 8) & 0xff;
		*(((uint8_t*)c->data) + 58) = (numBits >> 16) & 0xff;
		*(((uint8_t*)c->data) + 59) = (numBits >> 24) & 0xff;
		*(((uint8_t*)c->data) + 60) = (numBits >> 32) & 0xff;
		*(((uint8_t*)c->data) + 61) = (numBits >> 40) & 0xff;
		*(((uint8_t*)c->data) + 62) = (numBits >> 48) & 0xff;
		*(((uint8_t*)c->data) + 63) = (numBits >> 56) & 0xff;
		MD5_Main_Loop(c, c->data);
		md[0] = c->A & 0xff;
		md[1] = (c->A >> 8) & 0xff;
		md[2] = (c->A >> 16) & 0xff;
		md[3] = (c->A >> 24) & 0xff;
		md[4] = c->B & 0xff;
		md[5] = (c->B >> 8) & 0xff;
		md[6] = (c->B >> 16) & 0xff;
		md[7] = (c->B >> 24) & 0xff;
		md[8] = c->C & 0xff;
		md[9] = (c->C >> 8) & 0xff;
		md[10] = (c->C >> 16) & 0xff;
		md[11] = (c->C >> 24) & 0xff;
		md[12] = c->D & 0xff;
		md[13] = (c->D >> 8) & 0xff;
		md[14] = (c->D >> 16) & 0xff;
		md[15] = (c->D >> 24) & 0xff;
		MD5_Init(c);
		return 1;
	}
	else {
		return 0;
	}
}
std::string CryptUtils::ToHexString(const uint8_t* data, const int size)
{
	std::string str(size * 2, 0);
	char ch = 0;
	unsigned char t;
	for (int i = 0; i < size; i++)
	{
		t = (data[i] >> 4) & 0x0f;
		str[i * 2] = HEX[t];
		t = data[i] & 0x0f;
		str[i * 2 + 1] = HEX[t];	
	}
	return str;
}

void CryptUtils::MD5FromFile(const std::string& path, uint8_t outmd[16])
{
	MD5Context ctx;
	memset(outmd, 0, sizeof(outmd));
	uint8_t buf[1024];
	MD5_Init(&ctx);
	FILE* fp = fopen(path.c_str(), "rb");
	if (fp) {
		int r = 0;
		while ((r = fread(buf, 1, 1024, fp)) > 0) {
			MD5_Update(&ctx, buf, r);
		}
		fclose(fp);
	}
	MD5_Final(outmd, &ctx);
}

std::string CryptUtils::MD5FromFile(const std::string& path)
{
	uint8_t outmd[16];
	MD5FromFile(path, outmd);
	return ToHexString(outmd, 16);
}
std::string CryptUtils::MD5FromData(const char* data, const int size)
{
	MD5Context ctx;
	unsigned char outmd[16];
	memset(outmd, 0, sizeof(outmd));
	MD5_Init(&ctx);
	MD5_Update(&ctx, data, size);
	MD5_Final(outmd, &ctx);
	return ToHexString(outmd, 16);
}
std::string CryptUtils::MD5FromString(const std::string& str)
{
	MD5Context ctx;
	unsigned char outmd[16];
	memset(outmd, 0, sizeof(outmd));
	MD5_Init(&ctx);
	MD5_Update(&ctx, str.c_str(), str.length());
	MD5_Final(outmd, &ctx);
	return ToHexString(outmd, 16);
}
