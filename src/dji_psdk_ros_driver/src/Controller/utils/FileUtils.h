#pragma once
#include <string>
#include <vector>
#include <stdint.h>
#include <map>

class File 
{
public:
	std::string name;
	std::string path;
	int type;

public:
	static const int TYPE_FLIE = 1;
	static const int TYPE_DIR = 2;
};

class parameter
{
public:
	std::string key;
	double value;
};

class FileUtils
{
private:
	FileUtils(){};
	virtual ~FileUtils(){}
public:
	static int GetFileLength(const std::string& path);
	static int ReadFileData(const std::string& path, void* buffer, int offset, int size);
	static int WriteFileData(const std::string& path, void* buffer, int size);
	static int WriteFileDataAppended(const std::string& path, void* buffer, int size);
	static int ListFiles(const std::string& dirpath, std::vector<File>& list, bool hide = true);
	static int MakeDirs(const std::string& dirpath);
	static int CreateFile(const std::string& path);
	static int removeFile(const std::string& path);
	static std::string Basename(const std::string& path);
	static std::string Dirname(const std::string& path);
	static std::string readAll(const std::string& path);
	static std::string readline(const std::string& path);
	static void splitString(std::string& s, std::vector<std::string>& v, const std::string& c);

	static int readParameters(const std::string& path, std::vector<parameter> & paras);
	static int readParameters2map(const std::string& path, std::map<std::string, double> &params);
	static int writeParameters(const std::string& path, std::vector<parameter> & paras);
	static double getParameterValue(const std::string& path, std::string key);
	static int setParameterValue(const std::string& path, std::string key, double value);

	// static int getParameterValue(const std::string& path, std::vector<std::string> keys, std::vector<parameter> &parameters);
	// static int setParameterValue(const std::string& path, std::vector<parameter> parameters);
	static int getSystemBuildTime();
	static int writePid(const std::string &name);

    static int popen2(char *command, char *response, size_t size);
	
	// static double getParameterValue(const std::string& path, uint16_t index);
	// static double getParameterValue(const std::string& path, std::string key);

public:
	static const char* SEPARATOR;
	static const char  SEPARATOR_CHAR;
};
