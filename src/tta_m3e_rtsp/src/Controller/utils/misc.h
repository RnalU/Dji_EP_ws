#pragma once
#include <string>

class misc
{
public:
    misc();
    ~misc();
    static std::string urlJoint(std::string &domain, int &port, const char *tail);

};