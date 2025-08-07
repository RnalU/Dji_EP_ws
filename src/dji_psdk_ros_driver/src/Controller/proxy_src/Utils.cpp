#include "Utils.h"
#include <time.h>
#include <locale>
// #include <codecvt>
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
#include <windows.h>
#include <chrono>
#else
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#endif

// #if _MSC_VER >= 1900
// std::string Utils::utf16_to_utf8(std::wstring utf16_string)
// {
// 	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> convert;
// 	auto p = reinterpret_cast<const wchar_t *>(utf16_string.data());
// 	return convert.to_bytes(p, p + utf16_string.size());
// }
// std::wstring Utils::utf8_to_utf16(std::string utf8_string)
// {
// 	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> convert;
// 	auto p = reinterpret_cast<const char *>(utf8_string.data());
// 	auto str = convert.from_bytes(p, p + utf8_string.size());
// 	std::wstring u16_str(str.begin(), str.end());
// 	return u16_str;
// }
// #else

// std::string Utils::utf16_to_utf8(std::wstring utf16_string)
// {
// 	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> convert;
// 	return convert.to_bytes(utf16_string);
// }
// std::wstring Utils::utf8_to_utf16(std::string utf8_string)
// {
// 	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> convert;
// 	return convert.from_bytes(utf8_string);
// }
// #endif

std::string Utils::TrimString(const std::string& str)
{
	int b = 0;
	int e = str.length() - 1;
	while (b <= e) {
		char ch = str[b];
		if (ch == ' ' || ch == '\n') {
			b++;
		}
		else {
			break;
		}
	}
	while (b <= e) {
		char ch = str[e];
		if (ch == ' ' || ch == '\n') {
			e--;
		}
		else {
			break;
		}
	}
	if (b <= e) {
		return str.substr(b, e - b + 1);
	}
	else {
		return "";
	}
}

int64_t Utils::NowMills()
{
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
	int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	int64_t ts = ((int64_t)tv.tv_sec) * 1000 + tv.tv_usec / 1000;
#endif
	return ts;
}

std::string Utils::GetDateTimeStringShort() {
	int64_t ms = NowMills();
	time_t rawtime = ms / 1000; 
	int mills = ms % 1000;
	struct tm *t;
	char line[32];
	t = localtime(&rawtime);
	if (t!=NULL) {
		sprintf(line, "%02d%02d-%02d%02d%02d%02d", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, mills);
		return line;
	}
	else {
		return "0000-00-00_00:00:00-000";
	}
}

std::string Utils::GetDateTimeString(int64_t ms) {
	time_t rawtime = ms / 1000; 
	int mills = ms % 1000;
	struct tm *t;
	char line[32];
	t = localtime(&rawtime);
	if (t!=NULL) {
		sprintf(line, "%04d-%02d-%02d_%02d:%02d:%02d#%03d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, mills);
		return line;
	}
	else {
		return "0000-00-00_00:00:00-000";
	}
}

std::string Utils::GetDateTimeStringFormatZhidao() {
	int64_t ms = NowMills();
	time_t rawtime = ms / 1000; 
	int mills = ms % 1000;
	struct tm *t;
	char line[32];
	t = localtime(&rawtime);
	if (t!=NULL) {
		sprintf(line, "%04d-%02d-%02d %02d:%02d:%02d.%03d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, mills);
		return line;
	}
	else {
		return "0000-00-00_00:00:00-000";
	}
}

std::string Utils::GetDateTimeString()
{
	return GetDateTimeString(NowMills());
}

std::string Utils::GetSimpleDateTimeString()
{
	return GetSimpleDateTimeString(NowMills());
}
std::string Utils::GetSimpleDateTimeString(int64_t ms)
{
	time_t rawtime = ms / 1000;
	int mills = ms % 1000;
	struct tm *t;
	char line[32];
	t = localtime(&rawtime);
	if (t!=NULL) {
		sprintf(line, "%04d-%02d-%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
		return line;
	}
	else {
		return "0000-00-00";
	}
}


void Utils::Delay(uint32_t ms)
{
#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

std::string Utils::GetStringFromBuffer(const char* buf, int size)
{
	std::string str;
	for(int i=0; i<size; i++){
		if(buf[i]){
			str.append(1, buf[i]);
		}else{
			break;
		}
	}
	return TrimString(str);
}

void Utils::FillStringToBuffer(char* buf, int size, const std::string str)
{
	int i = 0 ;
	while(i<str.length() && i<size) {
		buf[i] = str[i];
		i++;
	}
	if(i<size) {
		buf[i]=0;
	}
}

long Utils::getSystemRunTime(){
  struct timespec times = {0, 0};
  long time;

  clock_gettime(CLOCK_MONOTONIC, &times);

  return times.tv_sec*1000 + times.tv_nsec/1000000;
}

long Utils::getSystemRunTime_us(){
  struct timespec times = {0, 0};
  long time;

  clock_gettime(CLOCK_MONOTONIC, &times);

  return times.tv_sec*1000000 + times.tv_nsec/1000;
}

void Utils::setDataTime(uint32_t s){
	time_t rawtime = s; 
	struct tm *t;
	char line[32];
	t = localtime(&rawtime);
	if (t!=NULL) {
		snprintf(line, sizeof(line), "date -s \"%d-%d-%d %d:%d:%d\"", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
		system(line);
	}
}