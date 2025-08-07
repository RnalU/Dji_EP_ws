#include "Port.h"

Port::Port()
{
}

Port::~Port()
{
}

int Port::send(const char* data, int size, int peer){   // for single peer connection this virtual function is not needed

}

int Port::recv(char* data, int size, int *peer){  // for single peer connection this virtual function is not needed

}
