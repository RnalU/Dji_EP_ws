#include "ttalinkUtils.h"
#include "CJsonObject.hpp"

void ttalinkUtils::SendRequestModuleVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t module_type)
{

}

void ttalinkUtils::SendResponseModuleVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t module_type, const char* version)
{

}

void ttalinkUtils::SendGeneralMessage(DataPort* port, uint8_t dst_addr, uint8_t src_addr, ttalink_general_message_t* gm)
{
    if(port && gm){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_message_encode(dst_addr, src_addr, &msg, gm);
		ttalink_to_send_buffer(buffer, &msg);
		int sendcount = ttalink_get_send_buffer_length(&msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, sendcount);
		port->send(ack_item);
	}
}

void ttalinkUtils::SendHeartBeat(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_heartbeat_t hb;
		int len=0;

		bzero(&hb, sizeof(hb));
		hb.device_type = type;
		hb.sn = sn;
		ttalink_heartbeat_encode(dst_addr, src_addr, &msg, &hb);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}

void ttalinkUtils::SendHeartBeat_Reserve(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint32_t reserve)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_heartbeat_t hb;
		int len=0;

		bzero(&hb, sizeof(hb));
		hb.device_type = type;
		hb.sn = sn;
		hb.reserve = reserve;
		ttalink_heartbeat_encode(dst_addr, src_addr, &msg, &hb);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}

void ttalinkUtils::SendHeartBeat(DataPort* port, int peer, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, std::string s){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_heartbeat_t hb;
		int len=0;

		bzero(&hb, sizeof(hb));
		hb.device_type = type;
		hb.sn = sn;
		ttalink_heartbeat_encode(dst_addr, src_addr, &msg, &hb);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len, peer);
		port->send(item);
	}
}

void ttalinkUtils::SendHeartBeat(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint8_t mode)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_heartbeat_t hb;
		int len=0;

		bzero(&hb, sizeof(hb));
		hb.device_type = type;
		hb.sn = sn;
		hb.mode = mode;
		ttalink_heartbeat_encode(dst_addr, src_addr, &msg, &hb);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}
}

void ttalinkUtils::SendGeneralStatus(DataPort* port, uint8_t dst_addr, uint8_t src_addr, std::string name, double value, int64_t sn, uint8_t type, int peer)
{
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_status_t gs;
		int len=0;

		gs.device_type = type;
		gs.sn = sn;
		memset(gs.status_id, 0, sizeof(gs.status_id));
		memcpy(gs.status_id, name.c_str(), name.size());
		gs.status_value = value;
		ttalink_general_status_encode(dst_addr, src_addr, &msg, &gs);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len, peer);
		port->send(item);
	}
}

void ttalinkUtils::SendGeneralCommand(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t type, uint16_t cmd, float param1, float param2, float param3, float param4, float param5, float param6, float param7){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_t gc;
		int len=0;

		gc.device_type = type;
		gc.sn = sn;
		gc.cmd = cmd;
		gc.param1 = param1;
		gc.param2 = param2;
		gc.param3 = param3;
		gc.param4 = param4;
		gc.param5 = param5;
		gc.param6 = param6;
		gc.param7 = param7;
		ttalink_general_command_encode(dst_addr, src_addr, &msg, &gc);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len);
		port->send(item);
	}	
}

void ttalinkUtils::SendGeneralRequestVersion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, uint8_t deviceType){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_request_version_t grv;
		int len=0;

		grv.sn = sn;
		grv.device_type = deviceType;
		ttalink_general_request_version_encode(dst_addr, src_addr, &msg, &grv);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}
}

void ttalinkUtils::SendGeneralRequestVersionAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, const char* version, int64_t sn, uint8_t type){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_request_version_ack_t grva;
		int len=0;

		grva.sn = sn;
		grva.device_type = type;
		grva.ack = 0;
		memset(grva.version, 0, sizeof(grva.version));
		strncpy((char*)grva.version, version, sizeof(grva.version));
		ttalink_general_request_version_ack_encode(dst_addr, src_addr, &msg, &grva);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}	
}

void ttalinkUtils::SendGeneralCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_ack_t gca;
		int len=0;

		memset(&gca, 0, sizeof(gca));
		gca.sn = sn;
		gca.device_type = type;
		gca.cmd = cmd;
		gca.result = 0;
		ttalink_general_command_ack_encode(dst_addr, src_addr, &msg, &gca);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}	
}

void ttalinkUtils::SendGeneralCommandAckRes(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type, int reslutParam){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_ack_t gca;
		int len=0;

		memset(&gca, 0, sizeof(gca));
		gca.sn = sn;
		gca.device_type = type;
		gca.cmd = cmd;
		gca.result = 0;
		gca.result_param = reslutParam;
		ttalink_general_command_ack_encode(dst_addr, src_addr, &msg, &gca);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}	
}

void ttalinkUtils::SendGeneralParamterValue(DataPort* port, uint8_t dst_addr, uint8_t src_addr, \
                                        const char *name, double value, uint16_t count, uint16_t index, int64_t sn, uint8_t type){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_param_value_t gpv;
		int len=0;

		gpv.sn = sn;
		gpv.device_type = type;
		gpv.param_index = index;
		gpv.param_count = count;
		gpv.param_type = 0;
		gpv.param_value = value;
		memset(gpv.param_id, 0, sizeof(gpv.param_id));
		strncpy((char*)gpv.param_id, name, sizeof(gpv.param_id));
		ttalink_general_param_value_encode(dst_addr, src_addr, &msg, &gpv);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len);
		port->send(ack_item);
	}	
}

void ttalinkUtils::SendSwitchControl(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint8_t cmd){
	
}

void ttalinkUtils::SendRepack(DataPort* port, ttalink_message_t &msg){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		int len = 0;

		ttalink_to_send_buffer(buffer, &msg);
		len = ttalink_get_send_buffer_length(&msg);
		BufferItem* item = new BufferItem((char*)buffer, len);
		port->send(item);
	}	
}



void ttalinkUtils::SendJsonPassString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, const std::string &str){

}



void ttalinkUtils::SendJsonRequestCameraInfo(DataPort *port){

}

void ttalinkUtils::SendPostion(DataPort* port, uint8_t dst_addr, uint8_t src_addr, int64_t sn, int32_t lat, int32_t lon, int32_t alt){

}

void ttalinkUtils::SendRepack(DataPort* port, ttalink_message_t &msg, int peer){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		int len = 0;

		ttalink_to_send_buffer(buffer, &msg);
		len = ttalink_get_send_buffer_length(&msg);
		BufferItem* item = new BufferItem((char*)buffer, len, peer);
		port->send(item);
	}	
}

void ttalinkUtils::SendJsonPassString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, const std::string &str, int peer){

}


void ttalinkUtils::SendGeneralCommandJsonACKString(DataPort *port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, uint8_t subcmd, int64_t sn, uint8_t type, const std::string &str, int peer){
	if(port){
		
	}
}

void ttalinkUtils::SendGeneralCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int64_t sn, uint8_t type, int peer){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		ttalink_general_command_ack_t gca;
		int len=0;

		memset(&gca, 0, sizeof(gca));
		gca.sn = sn;
		gca.device_type = type;
		gca.cmd = cmd;
		gca.result = 0;
		ttalink_general_command_ack_encode(dst_addr, src_addr, &msg, &gca);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len, peer);
		port->send(ack_item);
	}	
}

void ttalinkUtils::RecalaulateCrc(ttalink_message_t *msg){
	ttalink_finalize_message_chan(msg, msg->dst_addr, msg->src_addr, TTALINK_CHANNEL_DEFAULT, msg->len, msg->len, 0, false);
}


void ttalinkUtils::SendCommandAck(DataPort* port, uint8_t dst_addr, uint8_t src_addr, uint16_t cmd, ACK_TYPE ack, int peer){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		 ttalink_com_ack_t com_ack;
		int len=0;

		memset(&com_ack, 0, sizeof(com_ack));

		com_ack.com_msgid = cmd;
		com_ack.ack = ack;

		ttalink_com_ack_encode(dst_addr, src_addr, &msg, &com_ack);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* ack_item = new BufferItem((char*)buffer, len, peer);
		port->send(ack_item);
	}	
}



void ttalinkUtils::rosuav_ctrl_loopinput(DataPort* port, uint8_t dst_addr, uint8_t src_addr, ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input, int peer){
	if(port){
		uint8_t buffer[TTALINK_MAX_PACKET_LEN];
		ttalink_message_t msg;
		int len=0;

		ttalink_rosuav_ctrl_loop_input_encode(dst_addr, src_addr, &msg, rosuav_ctrl_loop_input);
		len = ttalink_to_send_buffer(buffer, &msg);
		BufferItem* item = new BufferItem((char *)buffer, len, peer);
		port->send(item);
	}
}
