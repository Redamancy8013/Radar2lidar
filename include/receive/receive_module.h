/**
 * @file
 * @brief  Head file for class ReceiveModule.
 *
 * All rights reserved. Reproduction in whole or in part without the
 * written consent of CNGIC UVC is prohibited.
 */
#ifndef __RECEIVE_MODULE_H__
#define __RECEIVE_MODULE_H__

#include <common/ugv_topics.h>
#include <common/shudao.h>

using namespace cngicuvc::messages;
using namespace std;

namespace cngicuvc
{
	class ReceiveModule : public dnet_task_t
	{	
		public:
			ReceiveModule();
			~ReceiveModule();
			void task0_loop();

		private:
			void onInit();
			void taskReceivePackage();			
			
			dnet_socket_t subscriber;
			dnet_rwbuf_t mCmdRaw;
			cngicuvc::messages::PlatformSteeringCommand mPlatformSteeringCommand;
		
			dnet_msg_t mMsgHead, mMsgData;
			dnet_timer_t timer;
	};

}//end cngicuvc
#endif

