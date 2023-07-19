#ifndef SENSOR_TOPPICS_H
#define SENSOR_TOPPICS_H

#include "ugv_topics.h"
#include "radardriver.h"

#define  ADDR_RADARDATA_IPC "ipc:///tmp/ADDR_RADARDATA_IPC"
#define  ADDR_LIDARDATA_IPC "ipc:///tmp/ADDR_3DLIDARDATA_IPC"
#define  ADDR_GRLIDARDATA_IPC "ipc:///tmp/ADDR_3DGROUNDLIDARDATA_IPC"


struct PointXYZIR{
	float x = -1;
	float y = -1;
	float z = -1;
	float intensity = 0;
	uint16_t ring = 0;
};



//############# sensors ############################
namespace cngicuvc
{
	namespace messages
	{

		struct RadarData{
			Header header;
			RadarDataNode data[64];
		};


		struct PointClouds{
			Header header;
			int size = 0;
			PointXYZIR points[95000]; //for cfans or rslidar m1
		};
	}
}

#endif

