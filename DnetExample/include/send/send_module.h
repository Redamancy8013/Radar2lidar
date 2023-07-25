/**
 * @file
 * @brief  Head file for class SendModule.
 *
 * All rights reserved. Reproduction in whole or in part without the
 * written consent of CNGIC UVC is prohibited.
 */
#ifndef __SEND_MODULE_H__
#define __SEND_MODULE_H__


#include <common/ugv_topics.h>
#include <shudao.h>

using namespace cngicuvc::messages;
using namespace std;

namespace cngicuvc
{
	class SendModule : public dnet_task_t
	{	
		public:
			SendModule();
			~SendModule();			
			void task0_loop();		

		private:
			void onInit();
			void taskSendPackage();				
			dnet_socket_t mPublisher;		
			dnet_msg_t mMsgHead, mMsgData;		
			cngicuvc::messages::PlatformSteeringCommand mPlatformSteeringCommand;
			int mTimes;
	};	
}
#endif

