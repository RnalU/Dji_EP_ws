#include "Log.h"
#include "Utils.h"

#include <cstdarg>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "FileUtils.h"

#define LOG_ENABLE 1

#define LOG_MAX_BLOCK_COUNT 20
#define LOG_MAX_BLOCK_SIZE 1000000

#ifdef	ANDROID
#include <android/log.h>
#endif

std::string Log::LogDir(".");
std::string Log::mTag("GW");

LogThread Log::LogThreadIns;

void Log::SetLogDir(const char* path)
{
	LogDir = path;
}

void Log::WriteToFile(const char* tag, const char* level, const char* line)
{
	if(LOG_ENABLE){
		if (LogThreadIns.isStarted()) {
			LogItem* item = new LogItem();
			// item->tag = tag == NULL ? "null" : tag;
            item->tag = Log::mTag;
			item->ts = Utils::NowMills();
			item->level = level == NULL ? "N" : level;
			item->msg = line;
			LogThreadIns.push(item);
		}
	}else{
		std::cout << Utils::GetDateTimeStringShort() << " " << level << " " << line << std::endl;
	}
}

void Log::WriteToFile(const char* level, const char* line)
{
	if(LOG_ENABLE){
		if (LogThreadIns.isStarted()) {
			LogItem* item = new LogItem();
            item->tag = Log::mTag;
			item->ts = Utils::NowMills();
			item->level = level == NULL ? "N" : level;
			item->msg = line;
			LogThreadIns.push(item);
		}
	}else{
		std::cout << Utils::GetDateTimeStringShort() << " " << level << " " << line << std::endl;
	}
}

void Log::i(const char * fmt, ...)
{
#ifdef LOG_ENABLE 
	const int STR_BUF_LEN = 1024;
	char str_buf[STR_BUF_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(str_buf, STR_BUF_LEN, fmt, args);
	va_end(args);
#ifdef	ANDROID
	__android_log_print(ANDROID_LOG_INFO, tag, "%s", str_buf);
#else
	//printf("I %s\t%s\r\n",tag,str_buf);
	WriteToFile("I", str_buf);
#endif
#endif
}

void Log::d(const char * fmt, ...)
{
#ifdef LOG_ENABLE 
	const int STR_BUF_LEN = 1024;
	char str_buf[STR_BUF_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(str_buf, STR_BUF_LEN, fmt, args);
	va_end(args);
#ifdef	ANDROID
	__android_log_print(ANDROID_LOG_INFO, tag, "%s", str_buf);
#else
	//printf("I %s\t%s\r\n",tag,str_buf);
	WriteToFile("D", str_buf);
#endif
#endif
}

void Log::w(const char * fmt, ...)
{
#ifdef LOG_ENABLE 
	const int STR_BUF_LEN = 1024;
	char str_buf[STR_BUF_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(str_buf, STR_BUF_LEN, fmt, args);
	va_end(args);
#ifdef	ANDROID
	__android_log_print(ANDROID_LOG_INFO, tag, "%s", str_buf);
#else
	//printf("I %s\t%s\r\n",tag,str_buf);
	WriteToFile("W", str_buf);
#endif
#endif
}

void Log::e(const char * fmt, ...)
{
#ifdef LOG_ENABLE 
	const int STR_BUF_LEN = 1024;
	char str_buf[STR_BUF_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(str_buf, STR_BUF_LEN, fmt, args);
	va_end(args);
#ifdef	ANDROID
	__android_log_print(ANDROID_LOG_INFO, tag, "%s", str_buf);
#else
	//printf("I %s\t%s\r\n",tag,str_buf);
	WriteToFile("E", str_buf);
#endif
#endif
}

int Log::StartLog()
{
	return LogThreadIns.start();
}

int Log::StartLog(std::string tag){
    mTag = tag;
    return LogThreadIns.start();
}

int Log::StopLog()
{
	return LogThreadIns.stop();
}


#define MAX_LOGS 10000

LogThread::LogThread() :
	mSize(0)
{

}
LogThread::~LogThread()
{

}

void LogThread::threadFunc()
{
	while (!done()) {
		LogItem* item = pop();
		if (item) {
			writeLog(item);
			delete item;
		}
		else {
			Utils::Delay(10);
		}
	}
}

void LogThread::push(LogItem* item) {
	mLock.lock();
	pushLocked(item);
	dropLocked();
	mLock.unlock();
}

void LogThread::pushLocked(LogItem* item)
{
	if (item) {
		mLogItemQueue.push_back(item);
		mSize++;
	}
}
void LogThread::dropLocked()
{
	while (mLogItemQueue.size() > MAX_LOGS) {
		LogItem* item = popLocked();
		if (item) {
			delete item;
		}
		else {
			break;
		}
	}
}

LogItem* LogThread::pop() {
	LogItem* item = NULL;
	mLock.lock();
	item = popLocked();
	mLock.unlock();
	return item;
}

LogItem* LogThread::popLocked()
{
	if (mLogItemQueue.size() > 0) {
		LogItem* front = mLogItemQueue.front();
		mLogItemQueue.pop_front();
		mSize--;
		return front;
	}
	else {
		return NULL;
	}
}

void LogThread::writeLog(LogItem* item)
{
	if (item) {
		std::string pathstr(Log::LogDir.c_str());
		// pathstr.append(FileUtils::SEPARATOR);
		// pathstr.append(item->tag);
		// pathstr.append(FileUtils::SEPARATOR);
		// pathstr.append(Utils::GetSimpleDateTimeString(item->ts));

		// FileUtils::MakeDirs(pathstr);

		std::string dir = pathstr;

		pathstr.append(FileUtils::SEPARATOR);
		pathstr.append(item->tag);
		pathstr.append(".log");

		deliverLogFiles(dir, item->tag, pathstr);

		std::ofstream out;
		out.open(pathstr, std::ios::app);
		if (out.is_open()) {
			out.seekp(0, std::ios::end);
			out << Utils::GetDateTimeString(item->ts) << " " << item->level << " " << item->msg << "\n";
			out.close();
		}
	}
}

size_t LogThread::getFileSize(const std::string& path)
{
	std::ifstream in;
	in.open(path, std::ios::in | std::ios::binary);
	if (in.is_open()) {
		in.seekg(0, std::ios::beg);
		size_t pb = in.tellg();
		in.seekg(0, std::ios::end);
		size_t pe = in.tellg();
		in.close();
		return pe-pb;
	}else{
		return 0;
	}
}

void LogThread::deliverLogFiles(const std::string& dir, const std::string& name, const std::string& path)
{
	size_t size = getFileSize(path);
	if(size > LOG_MAX_BLOCK_SIZE){
		int i = 0;
		char buf[64];
		std::string p;

		for(; i<=LOG_MAX_BLOCK_COUNT; i++){
			snprintf(buf, sizeof(buf), "%s-%02d.log", name.c_str(), i);
			p = dir + FileUtils::SEPARATOR + buf;
			if(access(p.c_str(), 0)!=0){
				// printf("%s not exist\n", buf);
				break;
			}
			// printf("%s exist\n", buf);
		}

		if(i > LOG_MAX_BLOCK_COUNT){
			// printf("log files overflow, delete the oldest file\n");

			std::string pSrc, pDst;
			i=0;
			snprintf(buf, sizeof(buf), "%s-%02d.log", name.c_str(), i);
			pDst = dir + FileUtils::SEPARATOR + buf;
			// printf("the oldest file is %s\n", pDst.c_str());
			if(access(pDst.c_str(), 0) == 0){
				// printf("remove %s\n", pDst.c_str());
				remove(pDst.c_str());
			}

			for(;i<LOG_MAX_BLOCK_COUNT;i++){
				snprintf(buf, sizeof(buf), "%s-%02d.log", name.c_str(), i+1);
				pSrc = dir + FileUtils::SEPARATOR + buf;
				if(access(pSrc.c_str(), 0) == 0){
					// printf("%s->%s\n", pSrc.c_str(), pDst.c_str());
					rename(pSrc.c_str(), pDst.c_str());
				}
				if(access(pSrc.c_str(), 0) == 0){
					// printf("remove %s\n", pSrc.c_str());
					remove(pSrc.c_str());
				}
				pDst = pSrc;
			}
		}else{
			// printf("log files increas, %s->%s\n", path.c_str(), p.c_str());
			rename(path.c_str(), p.c_str());
		}
	}
}
