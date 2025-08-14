#pragma once
#include <string>
#include <list>
#include "Thread.h"

#define LOG_IN_1	1

class LogThread;
class Log
{
private:
	Log() {}
	virtual ~Log() {}
public:
	static std::string LogDir;
    static std::string mTag;
	static int StartLog();
    static int StartLog(std::string tag);
	static int StopLog();
private:
	static LogThread LogThreadIns;
private:
	static void WriteToFile(const char* tag, const char* level, const char* line);
    static void WriteToFile(const char* level, const char* line);
public:
	static void SetLogDir(const char* path);
    static void i(const char * format, ...);
	static void d(const char * format, ...);
	static void w(const char * format, ...);
	static void e(const char * format, ...);
};
class LogItem {
public:
	int64_t ts;
	std::string tag;
	std::string level;
	std::string msg;
};
class LogThread : public Thread
{
public:
	LogThread();
	virtual ~LogThread();
protected:
	virtual void threadFunc();
public:
	void push(LogItem* item);
	LogItem* pop();
private:
	void pushLocked(LogItem* item);
	void dropLocked();
	size_t getFileSize(const std::string& path);
	void deliverLogFiles(const std::string& dir, const std::string& name, const std::string& path);
	LogItem* popLocked();
	void writeLog(LogItem* item);
private:
	std::list<LogItem*> mLogItemQueue;
	Lock mLock;
	int mSize;
};
