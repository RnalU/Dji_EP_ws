#include "FileUtils.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <libgen.h>
#include <iostream>
#include <fstream>

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
const char* FileUtils::SEPARATOR = "\\";
const char  FileUtils::SEPARATOR_CHAR = '\\';
#else
const char* FileUtils::SEPARATOR = "/";
const char  FileUtils::SEPARATOR_CHAR = '/';
#endif

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
#define MKDIR(x) _mkdir(x)
#else
#define MKDIR(x) mkdir(x, S_IRWXU | S_IRWXG | S_IRWXO)
#endif

int FileUtils::GetFileLength(const std::string& path)
{
	struct stat buf;
	int err = stat(path.c_str(), &buf);
	if(err==0) {
		if(buf.st_mode & S_IFREG) {
			return buf.st_size;
		}else{
			return -1;
		}
	}else {
		return -1;
	}
}
int FileUtils::ReadFileData(const std::string& path, void* buffer, int offset, int size)
{
	FILE* fp = fopen(path.c_str(), "rb");
	if(fp) {
		int r = 0;
		fseek(fp, offset, SEEK_SET);
		r = fread(buffer, 1, size, fp);
		fclose(fp);
		return r;
	}else{
		return -1;
	}
}
int FileUtils::WriteFileData(const std::string& path, void* buffer, int size)
{
	FILE* fp = fopen(path.c_str(), "w");
	if(fp) {
		int w = fwrite(buffer, 1, size, fp);
		fclose(fp);
		return w;
	}else{
		return -1;
	}
}

int FileUtils::WriteFileDataAppended(const std::string& path, void* buffer, int size){
	FILE* fp = fopen(path.c_str(), "a");
	if(fp) {
		int w = fwrite(buffer, 1, size, fp);
		fclose(fp);
		return w;
	}else{
		return -1;
	}
}

int FileUtils::ListFiles(const std::string& dirpath, std::vector<File>& list, bool hide)
{
	DIR* dir = NULL;
	struct dirent* ptr = NULL;
	if((dir=opendir(dirpath.c_str())) == NULL){
		return -1;
	}
	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name, ".")==0 || strcmp(ptr->d_name, "..")==0){
			continue;
		}else if(ptr->d_type == 8){ //file
			if(!hide || ptr->d_name[0]!='.'){
				File file;
				file.name = ptr->d_name;
				file.path = dirpath + SEPARATOR + file.name;
				file.type = File::TYPE_FLIE;
				list.push_back(file);
			}
		}else if(ptr->d_type == 10){ //link file
			continue;
		}else if(ptr->d_type == 4){ //dir
			if(!hide || ptr->d_name[0]!='.'){
				File file;
				file.name = ptr->d_name;
				file.path = dirpath + SEPARATOR + file.name;
				file.type = File::TYPE_DIR;
				list.push_back(file);
			}
		}
	}
	closedir(dir);
	return 0;
}

int FileUtils::MakeDirs(const std::string& dirpath)
{
	if(dirpath.length()<=0 || dirpath.length()>PATH_MAX){
		return -1;
	}
	size_t t = -1;
	while((t=dirpath.find(SEPARATOR_CHAR, t+1))!=std::string::npos){
		std::string str = dirpath.substr(0, t);
		MKDIR(str.c_str());
	}
	MKDIR(dirpath.c_str());
	return 0;
}

int FileUtils::CreateFile(const std::string& path)
{
	std::string dir = Dirname(path);
	MakeDirs(dir);
	FILE* fp = fopen(path.c_str(), "w");
	if(fp) {
		fclose(fp);
		return 0;
	}else{
		return -1;
	}
}
std::string FileUtils::Basename(const std::string& path)
{
	char *basec, *bname;
    basec = strdup(path.c_str());
	bname = basename(basec);
	std::string ret = bname;
	free(basec);
	return ret;
}
std::string FileUtils::Dirname(const std::string& path)
{
	char *dirc, *dname;
    dirc = strdup(path.c_str());
	dname = dirname(dirc);
	std::string ret = dname;
	free(dirc);
	return ret;
}
int FileUtils::removeFile(const std::string& path){
	return remove(path.c_str());
}
std::string FileUtils::readAll(const std::string& path){
	std::ifstream in(path);
    std::istreambuf_iterator<char> begin(in);
    std::istreambuf_iterator<char> end;
    std::string content(begin, end);
    return content;
}

void FileUtils::splitString(std::string& s, std::vector<std::string>& v, const std::string& c)
{
  std::string::size_type pos1, pos2;

  v.clear();
  pos2 = s.find(c);
  pos1 = 0;
  while(std::string::npos != pos2)
  {
    v.push_back(s.substr(pos1, pos2-pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if(pos1 != s.length())
    v.push_back(s.substr(pos1));
}

int FileUtils::readParameters(const std::string& path, std::vector<parameter> & paras){
	std::ifstream in(path);
	std::string line;
	parameter para;
	std::vector<std::string> t;

	const std::string sep = ":";

	// Log::i("read parameters:");

	while(in >> line){
		// Log::i("%s", line.c_str());
		splitString(line, t, sep);

		if(t.size() == 2){
			para.key = t[0];
			para.value = atof(t[1].c_str());
			paras.push_back(para);
		}else{

		}
	}

	in.close();

	return 0;
}

int FileUtils::readParameters2map(const std::string& path, std::map<std::string, double> &params){
	std::ifstream in(path);
	std::string line;
	std::vector<std::string> t;

	const std::string sep = ":";

	while(in >> line){
		splitString(line, t, sep);

		if(t.size() == 2){
			params[t[0]] = atof(t[1].c_str());
		}
	}

	in.close();

	return 0;
}

int FileUtils::writeParameters(const std::string& path, std::vector<parameter> & paras){
	std::ofstream out(path);
	std::string line;
	char value[32]={0};

	for(auto x : paras){
		snprintf(value, sizeof(value), "%f", x.value);
		line = x.key + ":" + value;
		out << line << std::endl;
	}

	out.close();
	sync();

	return 0;
}

double FileUtils::getParameterValue(const std::string& path, std::string key){
	std::vector<parameter> paras;

	readParameters(path, paras);

	for(auto x : paras){
		if(x.key == key){
			return x.value;
		}
	}

	return -1;
}

int FileUtils::setParameterValue(const std::string& path, std::string key, double value){
	std::vector<parameter> paras;
	std::vector<int> a;

	printf("set parameter %s:%f\n", key.c_str(), value);
	readParameters(path, paras);
	for(std::vector<parameter>::iterator it=paras.begin(); it!=paras.end(); it++){
		if(it->key == key){
			it->value = value;
		}
	}
	writeParameters(path, paras);

	return 0;
}

int FileUtils::getSystemBuildTime(){
	std::ifstream in("/proc/version");
	std::string line;
	std::vector<std::string> lines;
	int i=0;

	while(in >> line){
		lines.push_back(line);
	}

	struct tm tm_time;
	std::string buildTime = lines[17] + "-" + lines[13] + "-" + lines[14] + " " + lines[15];
	std::cout << buildTime << std::endl;
	strptime(buildTime.c_str(), "%Y-%b-%d %H:%M:%S", &tm_time);
	time_t t = mktime(&tm_time);
	return t;
}

int FileUtils::writePid(const std::string &name){
	// char s_pid[16] = {0};

	// pid_t pid = getpid();
	// std::string pidFile = LOGDIR;
	// pidFile += "/";
	// pidFile += name;

	// bzero(s_pid, sizeof(s_pid));
	// snprintf(s_pid, sizeof(s_pid), "%d", pid);
	// WriteFileData(pidFile, s_pid, strlen(s_pid));
	return 0;
}

int FileUtils::popen2(char *command, char *response, size_t size){
    FILE    *fp;
    int ret;

    fp = popen(command, "r");
    if(NULL == fp){
        printf("popen failure.\n");
        return -1;
    }

    ret=fread(response, 1, size, fp);
    if(ret < 0){
        printf("Read from fp failed:%s\n",strerror(errno));
        return -1;
    }
    response[ret] = 0;

    pclose(fp);

    return 0;
}
