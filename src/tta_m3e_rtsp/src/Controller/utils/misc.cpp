#include "misc.h"

misc::misc(){

}

misc::~misc(){

}

std::string misc::urlJoint(std::string &domain, int &port, const char *tail){
    char portstr[32];
	sprintf(portstr, "%d", port);
	return "http://" + domain + ":" + portstr + tail;
}