#include "uavData.h"
#include "Utils.h"
#include <mutex>

#include "ros/ros.h"

std::mutex g_mutex;

uavData::uavData(int localport):
mFcConnction(NULL),
mDataCount(0),
mlocalport(localport)
{
	memset(&mFeedback_data, 0, sizeof(mFeedback_data));
}

uavData::~uavData()
{
	if(mFcConnction){
		mFcConnction->stop();
		delete mFcConnction;
		mFcConnction = NULL;
	}
}

int uavData::rosuavFeedbackHandle(ttalink_rosuav_ctrl_feed_back_t * feedback_data)
{
    std::unique_lock<std::mutex> lock(g_mutex);

	mFeedback_data.latit = feedback_data->latit;
	mFeedback_data.longi = feedback_data->longi;
	mFeedback_data.altit = feedback_data->altit;

    mFeedback_data.velN = feedback_data->velN;
	mFeedback_data.velE = feedback_data->velE;
	mFeedback_data.velD = feedback_data->velD;

	mFeedback_data.atti_pitch = feedback_data->atti_pitch;
	mFeedback_data.atti_roll = feedback_data->atti_roll;
	mFeedback_data.atti_yaw = feedback_data->atti_yaw;

	mFeedback_data.gyro_pitch = feedback_data->gyro_pitch;
	mFeedback_data.gyro_roll = feedback_data->gyro_roll;
	mFeedback_data.gyro_yaw = feedback_data->gyro_yaw;

	mFeedback_data.accN = feedback_data->accN;
	mFeedback_data.accE = feedback_data->accE;
	mFeedback_data.accD = feedback_data->accD;

	mFeedback_data.quat[0] = feedback_data->quat[0];
	mFeedback_data.quat[1] = feedback_data->quat[1];
	mFeedback_data.quat[2] = feedback_data->quat[2];
	mFeedback_data.quat[3] = feedback_data->quat[3];

	lock.unlock();

	return 0;
}

int uavData::ttalinkMsgHandle(ttalink_message_t *msg)
{
	ttalink_rosuav_ctrl_feed_back_t  rosuav_ctrl_feed_back = {0};
	switch(msg->msgid)
	{
		case TTALINK_MSG_ID_ROSUAV_CTRL_FEED_BACK:
			ttalink_rosuav_ctrl_feed_back_decode(msg, &rosuav_ctrl_feed_back);
			rosuavFeedbackHandle(&rosuav_ctrl_feed_back);
		    break;
		default:
		    break;
	}


	return 0;
}

DataPort* uavData::getDataPort()
{
	if(mFcConnction)
	{
		return mFcConnction;
	}

	return NULL;
}

int uavData::proxy()
{
	// mFcConnction = new UdpDataPort("fc",mlocalport, "127.0.0.1", 10086, DataPort::MODE_OUTCOME | DataPort::MODE_INCOME , 0xffffff);

	mFcConnction = new TcpDataPort("fc", "127.0.0.1", 10086, DataPort::MODE_OUTCOME | DataPort::MODE_INCOME , 0xffffff);
	mFcConnction->start();

	doProxy();

	return 0;
}

int uavData::doProxy()
{
	BufferItem* item = NULL;
	int64_t now = Utils::getSystemRunTime();
	int64_t last_hb_ts = 0;
	ttalink_message_t msg;
	ttalink_status_t status;

	for(;;){
		int count = 0;
		now = Utils::getSystemRunTime();

		if(mFcConnction){
			item = mFcConnction->recv();
			if (item) {
				count ++;
				int bufsize = item->size();
				char* bufdata = item->data();
				uint8_t* udata = (uint8_t*)bufdata;
				for(int i=0;i<bufsize;i++){
					if(ttalink_parse_char(TTALINK_CHANNEL_FC_EMBE, udata[i], &msg, &status) == TTALINK_FRAMING_OK){
						printf("get msgid %d form FC\n",msg.msgid);

						ttalinkMsgHandle(&msg);
					}
                }
				delete item;
				item = NULL;
			}
		}


		if(count==0) {
			Utils::Delay(10);
		}

		mDataCount++;
	}
	return 0;
}

void uavData::threadFunc(){

    proxy();

}
