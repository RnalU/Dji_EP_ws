#pragma once
#include <string>
#include <cstdint>
class Utils
{
private:
	Utils() {}
	~Utils() {}
public:
	// static std::string utf16_to_utf8(std::wstring utf16_string);
	// static std::wstring utf8_to_utf16(std::string utf8_string);
	static int64_t NowMills();
	static std::string GetDateTimeStringShort();
	static std::string GetDateTimeString(int64_t ms);
	static std::string GetDateTimeString();
	static std::string GetDateTimeStringFormatZhidao();
	static std::string GetSimpleDateTimeString();
	static std::string GetSimpleDateTimeString(int64_t ms);
	static void Delay(uint32_t ms);
	static std::string TrimString(const std::string& str);
	static std::string GetStringFromBuffer(const char* buf, int size);
	static void FillStringToBuffer(char* buf, int size, const std::string str);
	static long getSystemRunTime();
	static long getSystemRunTime_us();
	static void setDataTime(uint32_t s);
};

