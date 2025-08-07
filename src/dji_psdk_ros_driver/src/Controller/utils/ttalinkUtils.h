#pragma once
#include "DataPort.h"
#include <string>
#include "common/ttalink.h"

class ttalinkUtils
{
private:
    ttalinkUtils(){}
    virtual ~ttalinkUtils(){}
public:
    static void SendRequestModuleVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t module_type);
    static void SendResponseModuleVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t module_type, const char* version);
    static void SendGeneralMessage(DataPort* port, uint8_t dst_addr, uint8_t src_addr, ttalink_general_message_t* gm);

    static void SendHeartBeat(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type);
    static void SendHeartBeat_Reserve(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint32_t reserve);
    static void SendHeartBeat(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint8_t mode);
    static void SendHeartBeat(DataPort* port, int peer, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, std::string s);

    static void SendGeneralStatus(DataPort* port, uint8_t dst_addr, uint8_t src_addr, std::string name, double value, int64_t sn, uint8_t type, int peer=0);
    static void SendGeneralCommand(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint16_t cmd, float param1=0, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0);

    static void SendGeneralRequestVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t deviceType); 
    static void SendGeneralRequestVersionAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, const char* version, int64_t sn, uint8_t type);  
    static void SendGeneralCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type);
    static void SendGeneralCommandAckRes(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type, int reslutParam);
    static void SendGeneralParamterValue(DataPort* port, uint8_t dst_addr, uint8_t src_addr, \
                                        const char *name, double value, uint16_t count, uint16_t index, int64_t sn, uint8_t type);
    static void SendSwitchControl(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t cmd);
    static void SendRepack(DataPort* port, ttalink_message_t &msg);
    static void SendJsonPassString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, const std::string &str);
    static void SendJsonRequestCameraInfo(DataPort *port);

    static void SendPostion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, int32_t lat, int32_t lon, int32_t alt);

    static void SendRepack(DataPort* port, ttalink_message_t &msg, int peer);
    static void SendJsonPassString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, const std::string &str, int peer);
    static void SendGeneralCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type, int peer);
    static void RecalaulateCrc(ttalink_message_t *msg);

    static void SendCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int peer);

    static void SendGeneralCommandJsonACKString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, uint8_t subcmd, int64_t sn, uint8_t type, const std::string &str, int peer);

    static void rosuav_ctrl_loopinput(DataPort* port, uint8_t dst_addr, uint8_t src_addr, ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input, int peer);
};
