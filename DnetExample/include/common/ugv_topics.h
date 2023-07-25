/**
 * @file
 * @brief  Definition for UGV related topics.
 *
 * All rights reserved. Reproduction in whole or in part without the
 * written consent of CNGIC UVC is prohibited.
 */

#ifndef CNGICUVC_UGV_TOPICS_H
#define CNGICUVC_UGV_TOPICS_H

#include <sys/types.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "timestamp.h"

#include<pcl/register_point_struct.h>		//add by LN 20191114

typedef float float32_t;
typedef double float64_t;
/*
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int int16_t;
typedef unsigned short int uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef long int int64_t;
typedef unsigned long uint64_t;
*/
#define LASER3DSCAN_TYPE 100
#define LOCATORDATA_TYPE 101
#define SINGLELINE_LIDAR_VOXELCOSTMAP_TYPE 102
#define MULTILINE_LIDAR_VOXELCOSTMAP_TYPE 103
#define MISSION_TYPE 104

#define VEHICLE_FRAME 200
#define SINGLE_LINE_LIDAR_FRAME 201
#define MULTIPLE_LINE_LIDAR_FRAME 202
#define VOXEL_COST_MAP_FRAME 203


#define  ADDR_SITUATION_ENHANCEMENT_IPC    "ipc:///tmp/ADDR_SITUATION_ENHANCEMENT_IPC"
#define  ADDR_OBJECT_DETECT_CAMBRICON_IPC  "ipc:///tmp/ADDR_OBJECT_DETECT_CAMBRICON_IPC" //axy
#define  ADDR_OBSTACLE_ZONE_BOUNDARY_IPC  "ipc:///tmp/ADDR_OBSTACLE_ZONE_IPC" //axy

#define  ADDR_INSALIGNMENT_PUB_IPC   "ipc:///tmp/ADDR_INSALIGNMENT_PUB_IPC"
#define  ADDR_LASER3DSCAN_DRIVER_TCP  "tcp://192.168.3.200:6666"
#define  ADDR_LASER3DSCAN_DRIVER_IPC  "ipc:///tmp/ADDR_LASER3DSCAN_DRIVER_IPC"
#define  ADDR_LASER3DSCAN_PUB_IPC  "ipc:///tmp/ADDR_LASER3DSCAN_PUB_IPC"
#define  ADDR_PLAN_PUB_IPC  "ipc:///tmp/ADDR_PLAN_PUB_IPC"
#define  ADDR_PERSON_TRACKING_IPC  "ipc:///tmp/ADDR_PERSON_TRACKING_IPC"
#define  ADDR_LOCATOR_DRIVER_IPC  "ipc:///tmp/ADDR_XWGI_DRIVER_IPC"
#define  PERSONTRACKING_TCP  "tcp://192.1.2.22:8000"

#define ADDR_VOXELCOSTMAPDATA_3DLIDAR_IPC "ipc:///tmp/ADDR_VOXELCOSTMAP_3DLIDAR_IPC"
#define ADDR_HAS_PEOPLE_AROUND_VEHICLE_IPC "ipc:///tmp/ADDR_HAS_PEOPLE_NEAR_VEHICLE_IPC"

#define ADDR_PATHPLANNING_IPC "ipc:///tmp/ADDR_PATHPLANNING_IPC"

#define ADDR_MISSION_PATH_IPC "ipc:///tmp/ADDR_MISSION_PATH_IPC"
#define ADDR_NETSTATUS_NAVIMODE_IPC "ipc:///tmp/ADDR_NET_NAVIMODE_IPC"

#define ADDR_AUTOCONTROL_COMMAND_IPC "ipc:///tmp/ADDR_AUTOCONTROL_COMMAND_IPC"
#define ADDR_STOP_CONTROL_IPC "ipc:///tmp/ADDR_STOP_CONTROL_IPC"

#define ADDR_LASER3D_STATE_IPC "ipc:///tmp/ADDR_3DLIDAR_STATE_IPC"
#define ADDR_LOCATOR_STATE_IPC "ipc:///tmp/ADDR_XWGI_STATE_IPC"
#define ADDR_PPC_STATE_IPC "ipc:///tmp/ADDR_PPC_MODULE_STATE_IPC"

#define  ADDR_3DLIDAR_MODULE_EVENT_IPC  "ipc:///tmp/ADDR_LASER3DSAMPLE_EVENT_IPC"
#define  ADDR_MLIDARSENSE_MODULE_EVENT_IPC  "ipc:///tmp/ADDR_MLIDARSENSE_EVENT_IPC"

#define  ADDR_HEART_BEAT_IPC  "ipc:///tmp/ADDR_HEART_BEAT_IPC" //接受模块心跳
#define  ADDR_MODULE_STATUS_IPC "ipc:///tmp/ADDR_MODULE_STATUS_IPC"//模块状态信息

#define  ADDR_PLATFORM_DATA_IPC "ipc:///tmp/ADDR_PLATFORM_DATA_IPC"//platform data

#define  ADDR_LASER3DSCAN2_DRIVER_IPC  "ipc:///tmp/ADDR_LASER3DSCAN2_DRIVER_IPC" //add by LN persontracking
#define  ADDR_LASER3DSCAN2_DRIVER_IPC  "ipc:///tmp/ADDR_LASER3DSCAN2_DRIVER_IPC"	//add by LN persontracking


// start define
#define  ADDR_INS_PUBLISH_IPC  "ipc:///tmp/ADDR_INS_PUBLISH_IPC" //接受模块心跳

#define  ADDR_PLANRESULT_IPC  "ipc:///tmp/ADDR_PLANRESULT_IPC"
// #define  ADDR_INSINFO_IPC  "ipc://ADDR_INSINFO_IPC"
#define  ADDR_GISINFO_IPC  "ipc:///tmp/ADDR_GISINFO_IPC"
// #define  ADDR_TRACKTARGET_IPC  "ipc:///tmp/ADDR_TRACKTARGET_IPC"
#define  ADDR_SCANINFO_IPC  "ipc:///tmp/ADDR_SCANINFO_IPC"
// #define  ADDR_PLATINFO_IPC  "ipc:///tmp/ADDR_PLATINFO_IPC"
// #define  ADDR_CONTROLRESULT_IPC "ipc:///tmp/ADDR_CONTROLRESULT_IPC"
// #define  ADDR_MISSIONPATH_IPC  "ipc:///tmp/ADDR_MISSIONPATH_IPC"
// #define  ADDR_DRIVEMODE_IPC  "ipc:///tmp/ADDR_DRIVEMODE_IPC"
// end define

#define  ADDR_PLATFROM_TO_HMI_IPC  "ipc:///tmp/ADDR_PLATFROM_TO_HMI_IPC"	//platform发送给HMI优先级

#define  ADDR_SITUATION_ENHANCEMENT_IPC  "ipc:///tmp/ADDR_SITUATION_ENHANCEMENT_IPC"
#define  ADDR_PERCEPTION_PUB_IPC  "ipc:///tmp/ADDR_PERCEPTION_PUB_IPC"   //add by LN 20200116


struct pandar40Point			//pandar40p结构
  {
      PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
      float intensity;                  //改为int类型
      float laserID;                    //改为int类型
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
  }EIGEN_ALIGN16;

  POINT_CLOUD_REGISTER_POINT_STRUCT (pandar40Point,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, laserID, laserID)
)

struct PointXYZIR{
	float x = -1;
	float y = -1;
	float z = -1;
	float intensity = 0;
	uint16_t ring = 0;
};

struct RadarDataNode
{
    float range;
    float angle;    //angle 0~180
    float velocity;
    float accel;
    float lat_rate;
    float width;
    bool flag=0;
    int dnyprop;
    double localX;
    double localY;
    int px,py;
    int times;
    double lastX;
    double lastY;
    double globalX;
    double globalY;
    bool update;
    int idx;
};

struct PointCfans{
    PCL_ADD_POINT4D;
    float intensity;
    int laserid;
    float timeflag;
    float hangle;
    float vangle;
    float range;
    unsigned char mirrorid;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCfans,
    (float,x,x)
    (float,y,y)
    (float,z,z)
    (float,intensity,intensity)
    (int, laserid, laserid)
    (float, timeflag, timeflag)
    (float, hangle, hangle)
    (float, vangle, vangle)
    (float, range, range)
    (unsigned char, mirrorid, mirrorid)
)


using namespace std;
namespace cngicuvc
{
namespace messages
{
//**************************Begin the definition of elements in the message***********************//
struct MessageType
{
	enum messageType 
	{
		 LASER2D_SCAN              			= 200
		,LASER3D_SCAN              			= 201
		,LOCATOR_DATA              			= 202
		,SINGLELINE_LIDAR_VOXELCOSTMAP		= 203
		,MULTILINE_LIDAR_VOXELCOSTMAP		= 204
		,MISSION_PATH	       	  			= 205
		,NET_STATUS_NAVI_MODE	   			= 206
		,LOCALPATH_SPEED_PROFILE   			= 207
		,PLANNER_RETURNED_VALUE	   			= 208
		,PLATFORM_STEERING_COMMAND 			= 209
		,HAS_PEOPLE_NEAR_VEHICLE   			= 210
		,LASER3D_SCAN2             			= 211

		,PERCEPTION_HEART_BEAT    			= 160
		,PLANCONTROL_HEART_BEAT   			= 170

        ,LOG_GIS_HEART_BEAT                 = 180
		,PLAN_HEART_BEAT   					= 171
		,CONTROL_HEART_BEAT   				= 172
		,LOGGIS_HEART_BEAT   				= 173

		,MONITOR_SEND_TO_HMI				= 100
		,PALTFORM_STEERING_HEART_BEAT 		= 101
		,HMI_HEART_BEAT						= 103
		,XWGI_HEART_BEAT					= 106
		,LOG_GIS_SEND_TO_PLATFORM			= 110
		,MONITOR_SEND_TO_PLATFORM			= 150
		,SITUATION_ENHANCEMENT				= 151
		,PERSON_TRACKING                    = 80
        ,PERCEPTION_RESULT                  = 81    //ADD BY LN 20191123
		,PATH_RECORD_FRAME                  = 120    //录点记录减速帧
		,SITUATIONAL_ENHANCEMENT            = 130    //态势增强帧
		,PLATFORM_SEND_TO_HMI               = 131    //cannet反馈优先级状
        ,SUTENG_LIDAR_HEART_BEAT            = 108
		,HEASI_LIDAR_HEART_BEAT             = 132
        ,GISInfoType 						= 233
        ,PlanResultType 					= 235
        ,ScanInfoType 						= 237
        ,TrackTargetType 					= 239
        ,DriveModeType 						= 232 
        ,MileageInfo						= 233
        ,INSOPERATION						= 234
        ,RadarData   						= 212
        ,PointClouds 						= 213
        ,OBJECT_DETECT_CAMBRICON            = 152
        ,OBJECT_DETECT_NEG                  = 153
        ,OBJECT_DETECT_POS                  = 154
        ,PASS_ZONE                          = 155
        ,ROAD_BOUNDARY_LIFT                 = 156
        ,ROAD_BOUNDARY_RIGHT                = 157
        ,ZONE_ROAD_BOUNDARY                 = 158
		,IMAGE_DATA_RAW_TW                  = 159
		,SITUATION_HEART_BEAT               = 162
	};	
};


struct Header
{	
	uint8_t message_type;	
	uint64_t seq;					//sequence ID	
	timestamp_t  time_stamp;		//the data acquisition time		
	uint8_t frame_id;				//the frame this data is associated with
	Header()
	: seq(0)	
	{

	}
	Header operator=(const Header& header) 
	{ 
		message_type = header.message_type;
		seq = header.seq;
		time_stamp = header.time_stamp;
		frame_id = header.frame_id;
		return *this; 
	}
};

struct PointClouds
{
	Header header;
	int size = 0;
	PointXYZIR points[95000]; //for cfans or rslidar m1
};

struct RadarData
{
	Header header;
	RadarDataNode data[128];
};

// start define
// struct ControlResult{
//     cngicuvc::messages::Header header;
//     float trans_vel;
//     float curvature;
// };
struct DriveMode{
    cngicuvc::messages::Header header;
    enum Mode{
        // 路点记录模式
        LOG_GIS = 1,
        // 轨迹跟踪模式
        GUIDE_TRAJECTORY_FOLLOWING = 2,
        // 一键返航模式
        ONE_KEY_TO_RETURN = 3,
        // 人员低速跟随模式
        PERSON_SLOW_FOLLOWING = 4,
        // 人员绕桩跟踪模式
        PERSON_RAOZHUANG_FOLLOWING = 5,
        // 人员目标远距离探测
        PERSON_TARGET_ROMOTE_DETECTION = 6,
        // 车辆目标跟随模式
        VEHICLE_TRAJECTORY_FOLLOWING = 7,
        // 恢复自动驾驶功能
        RESTORE_AUTOPOILOT = 9,
        // 停止自动驾驶功能
        STOP_AUTOPOILOT = 10
    };
    enum NetStatus{
        // 网络连接状态
        DISCONNECTED = 0,
        NORMAL = 1
    };
    char control_mode;
    char net_state;
    char deratedflag;
    char circling;
};
struct GISInfo{
    cngicuvc::messages::Header header;

    // 行驶模式标志位
    unsigned char status;

    // 降级标志位
    unsigned char deratedflag;

    // 局部GIS路径
    double x[301];
    double y[301];

    // 最高速度
    double maxspeed;

    // 距终点距离
    double enddistance;

    double startdistance;

    // 绕圈标志位
    unsigned char circling;

    // 障碍物标志位
    unsigned char obstacle;

    // 弹坑标志位
    unsigned char hole_location;

    // 油门标志位
    char accelerator;

    // 转向标志位
    int turn;

};
struct Point2D{
    double x;
    double y;
};
// reluctant
/*struct MissionPath{
    cngicuvc::messages::Header header;
    unsigned int num_navi_point;
    // TODO how big for these array
    vector<Point2D> navi_points;
    vector<double> target_speed;
    vector<char> obstacle;
    vector<char> hole_location;
    vector<char> accelerator;
    vector<int> turn;
    // array end
    double MaxSpeed;
    double TargetSpeed;
};*/
struct PlanResult{
    cngicuvc::messages::Header header;
    // 限制速度
    double SpeedLimit;
    // 规划路径
    Point2D RoadPoints[301];
    // 降级标志位
    unsigned char deratedflag;
    // 行驶模式标志位
    unsigned char gis_status;
    double startdis;


    // 增加规划状态和跟踪偏差 feng 2020/03/22
    // 规划状态  无路经:0 有路径:1
    char Planner_status;

    // 跟踪偏差
    char tracking_deviation;

};
// end define

struct plantoHMI
{
	unsigned char planresult;
	short int tracking_deviation;
};

struct ModuleStatus
{
	u_char HMIHeartBeat_status; 
	u_char PerceptionProHeartBeat_status;
	u_char XWGIHeartBeat_status;
	u_char PlanControlHeartBeat_status;	
	u_char PlatformHeartBeat_status;
	u_char PlanHeartBeat_status;//axy	
	u_char ControlHeartBeat_status;	//axy
	//lzd
	u_char HesaiHeartBeat_status;
	u_char SutengHeartBeat_status;	   
	u_char Ladar3DHeartBeat_status;
	u_char LOG_GISHeartBeat_status;
	u_char DecisionHeartBeat_status;	
	u_char BlgControlHeartBeat_status;
	u_char VelodyneHeartBeat_status;	   

	u_char Bit_module_status;
	
	bool operator == (const ModuleStatus& moduleStatus)
	{
		return ((HMIHeartBeat_status == moduleStatus.HMIHeartBeat_status) && \
		(PerceptionProHeartBeat_status == moduleStatus.PerceptionProHeartBeat_status) && \
		(XWGIHeartBeat_status == moduleStatus.XWGIHeartBeat_status) && \
		(PlanControlHeartBeat_status == moduleStatus.PlanControlHeartBeat_status) && \
		(PlatformHeartBeat_status == moduleStatus.PlatformHeartBeat_status));
	}
};

struct PointXYZ
{
	float64_t x;
	float64_t y;
	float64_t z;


	//add by lh 
	float speedgoal;
	char status1; //是否避障标志位
	char status2; //扩展标志位1
	char status3; //扩展标志位2
	float param1; //扩展参数1
	float param2; //扩展参数2
};


struct LonLat
{
	float64_t longitude;
	float64_t latitude;
};

struct Quaternion
{
	float64_t x;
	float64_t y;
	float64_t z;
	float64_t w;
};

struct PositionOrientation
{
	PointXYZ position;
	Quaternion orientation;
};

//This represents a pose in free space with uncertainty.
struct PoseWithCovariance
{
	PositionOrientation pose;
	/*
	# Row-major representation of the 6x6 covariance matrix\n\
	# The orientation parameters use a fixed-axis representation.\n\
	# In order, the parameters are:\n\

	# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
	*/
	float64_t covariance[36];
};

//This represents a vector in free space. 
struct Vector3
{
	float64_t x;
	float64_t y;
	float64_t z;
};

//This expresses velocity in free space broken into its linear and angular parts.
struct Twist
{
	Vector3  linear;
	Vector3  angular;
};

//This expresses velocity in free space with uncertainty.
struct TwistWithCovariance
{
	Twist twist;
	/*
	# Row-major representation of the 6x6 covariance matrix\n\

	# The orientation parameters use a fixed-axis representation.\n\
	# In order, the parameters are:\n\
	# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
	*/
	float64_t covariance[36];
};

//velocity hint
struct VelocityGiven
{
    float32_t length;
    float32_t velocity;
};

struct SpeedProfileGenerationParameters
{
	float32_t acceleration;				// the maximal acceleration [m/s^2]
	float32_t deceleration;				// the absolute maximal deceleration in standard situations [unsigned m/s^2]
	float32_t emergency_deceleration;   // the absolute maximal deceleration in emergency situations [unsigned m/s^2]
	float32_t max_velocity;             // the upper bound of the velocity [m/s]
	SpeedProfileGenerationParameters()
		: acceleration(0.0)
		, deceleration(0.0)
		, emergency_deceleration(0.0)
		, max_velocity(0.0)
	{

	}
};

struct VelocityGivenProfile
{
	float32_t start_velocity;					// the start velocity which must be the actual velocity of the robot
	unsigned int num_velocity_given;
	std::vector<VelocityGiven> velocity_given;	// a list of velocity given
	float32_t end_length;						// the total length of the trajectory
	SpeedProfileGenerationParameters velocity_profile_generation_parameters;
};

struct HeartBeat
{
	
	float32_t received_time;	//???
	int received_beat_number;
	u_char status;
};

struct Path
{
	unsigned int num_navi_point;
	std::vector<PointXYZ> navi_points;
};

struct PlannedResult
{
	std::vector<PointXYZ> local_path;
	uint8_t planner_return;
};

struct NaviPoint
{
	PointXYZ position;
	//TODO: add other attributes
};

//This hold information about the characterists of grid map
struct MapMetaData
{
	//timestamp_t  load_time;	// the time at which the map was loaded
	float32_t resolution;		// the map resolution [m/cell]
	uint16_t width;				// map width [cells]
	uint16_t height;			// map height [cells]
	PointXYZ origin;			// the origin of the map [m, m, rad].
};

//how to send and interaction???
struct EmergencyStop
{
	timestamp_t time_stamp;	
	unsigned char stopped;
};

struct VehicleState
{
	float32_t pitch;
	float32_t roll;
	float32_t yaw;
	float32_t longitude;
	float32_t latitude;
	float32_t velocity;

	VehicleState operator=(const VehicleState& state) 
	{ 
		pitch = state.pitch;
		roll = state.roll;
		yaw = state.yaw;
		longitude = state.longitude;
		latitude = state.latitude;
		velocity = state.velocity;
		return *this; 
	}
};

struct SituationType
{
	unsigned short pedestrian;  //行人
	unsigned short vehicle;     //车
	unsigned short P_obstacle;  //正障碍
	unsigned short N_obstacle;  //负障碍
	unsigned short traffic_area;//可通行区域
	unsigned short road_boundary; //道路边界
};

struct VertexPosition
{
	unsigned short position_x;
	unsigned short position_y;
};


struct SituationEnhancementHeader
{
	Header header;
	SituationType type;
};

//**************************End the definition of elements in the message*************************//

//**************************Begin the definition of messages**************************************//

struct SituationEnhancement
{
	SituationEnhancementHeader header;
	vector<VertexPosition> position;
};

struct ZoneBoundaryResult
{
	Header header;      
	int num_obs;
	int num_neg_obs;
	int num_zone_points;
	int num_boundary_points;
	vector<VertexPosition> position;
};

struct ImageDataTW //axy
{
    Header header;
    int channels;
    int rows;
    vector<unsigned char> image_vec;
};

struct dynamicRect//axy
{
    int left;
    int top;
    int right;
    int bottom;
    int type;   //0 车辆,2行人
    float prob; //概率
};



struct objectCambrion//axy
{
    Header header;
    vector<dynamicRect> objectData;

};


struct Laser2DScan
{
	Header header;      				
	float32_t angle_min;        		// start angle of the scan [rad]
	float32_t angle_max;        		// end angle of the scan [rad]
	float32_t angle_increment; 			// angular distance between measurements [rad]	
	float32_t time_increment;   		// time between measurements [seconds] - if your scanner
		                 				// is moving, this will be used in interpolating position of 3d points
	float32_t scan_time;        		// time between scans [seconds]	
	float32_t range_min;        		// minimum range value [m]
	float32_t range_max;        		// maximum range value [m]	
	std::vector<float32_t> ranges;      // range data [m] (Note: values < range_min or > range_max should be discarded)
	std::vector<float32_t> intensities;	// intensity data [device-specific units]
};

struct ScanInfo{
	Header header;
	int ScanInfoX[720];
	int ScanInfoY[720];
};

struct Laser3DScan
{
	Header header; 
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
};

struct Laser3DScan2		//add by LN 20191114
{
    Header header;
    pcl::PointCloud<pandar40Point> point_cloud;
};

#pragma pack(push,2)
struct LocatorData
{			
	Header header;
	float64_t wc_x;
	float64_t wc_y;
	float64_t wc_z;
	// unit:meter
	float64_t lc_x;
	float64_t lc_y;
	float64_t lc_z;
	int area_number;			//Beijing=20
	float64_t longitude;
	float64_t latitude;
	float64_t altitude;
	float32_t yaw;
	float32_t pitch;
	float32_t roll;
	float32_t velocity_x;
	float32_t velocity_y;
	float32_t velocity_z;
	float32_t velocity_all;
	float32_t omega_x;
	float32_t omega_y;
	float32_t omega_z;		

	//增加
	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;

	bool GPS_valid;
	bool Time_valid;
	bool Yaw_valid;
	bool Speed_valid;

	unsigned short GPS_state;
	unsigned char  work_state;
    unsigned char  nav_state;
    uint32_t error_state;
	unsigned char  bversion;
	unsigned char  mversion;
    unsigned char  lversion;
	unsigned char  ver_year;
	unsigned char  ver_month;
    unsigned char  ver_day;
	unsigned char  ver_check;
	int32_t mileage;
};
#pragma pack(pop)

struct Odometry
{
	Header header;       
	std::string child_frame_id;
	PoseWithCovariance pose;		// be specified in the coordinate frame given by header.frame_id
	TwistWithCovariance twist;		// be specified in the coordinate frame given by the child_frame_id
};

struct Image
{
	Header header;        				/*# header timestamp should be acquisition time of image\n\
										# header frame_id should be optical frame of camera\n\
										# origin of frame should be optical center of cameara\n\
										# +x should point to the right in the image\n\
										# +y should point down in the image\n\
										# +z should point into to plane of the image\n\
										# If the frame_id here and the frame_id of the CameraInfo\n\
										# message associated with the image conflict\n\
										# the behavior is undefined*/
	unsigned int height;     			// image height, that is, number of rows
	unsigned int width;      			// image width, that is, number of columns
	std::string encoding;     			/*# Encoding of pixels -- channel meaning, ordering, size
		                 				# taken from the list of strings in include/sensor_msgs/image_encodings.h*/

	bool is_bigendian; 					// is this data bigendian?
	unsigned int  step;        			// Full row length in bytes
	std::vector<unsigned char> data;  	// actual matrix data, size is (step * rows)
};

struct Mission
{
	enum controlMode 
	{
		 GUIDE_TRAJECTORY_FOLLOWING    =  1
		,PERSON_FOLLOWING              =  2
		,PERSON_TRAJECTORY_FOLLOWING   =  3
		,ONE_KEY_BACK                  =  4
		,STOP                          =  5
	};	
	enum netState 
	{
		 NORMAL    					   =  1
		,DISCONNECTED             	   =  0
	};
	Header header;
	uint16_t mission_id;
	controlMode control_mode;
	netState net_state;
	std::vector<NaviPoint> navi_points;
};

struct LocalPathAndSpeedProfile
{
	Header header;
	Path local_path;
	VelocityGivenProfile speed_profile;
	uint8_t planner_return;
};

struct MissionPath
{
	Header header;
	uint16_t mission_id;
	vector<NaviPoint> navi_points;
};

	


struct NetStatusAndNavigateMode
{

//	enum controlMode
//	{
//		 PATH_RECORD_START                      =0x0501
//		,PATH_RECORD_STOP                       =0x0601
//		,PATH_TRACKING_NOMAL                    =0x0102
//		,PATH_TRACKING_DOWNGRADE                =0X0202
//		,PATH_TRACKING_CIRCLE                   =0X0302
//		,PATH_TRACKING_DOWNGRADE_CIRCLE         =0X0402
//		,ONE_RETURN_NOMAL                       =0X0103
//		,ONE_RETURN_DOWNGRADE                   =0X0203
//		,ONE_RETURN_CIRCLE                      =0X0303
//		,ONE_RETURN_DOWNGRADE_CIRCLE            =0X0403
//		,PEOPLE_LOWSPEED_FOLLOW                 =0X0104
//		,PEOPLE_AROUND_PILE_FOLLOW              =0X0105
//		,PEOPLETARGET_LONGDISTANCE_DETECTION    =0X0106
//		,VEHICLE_TARGET_FOLLOW                  =0X0107
//		,TIME_OUT                               =0X0108
//		,RESUME_AUTOPILOT                       =0X0109
//		,STOP_AUTOPILOT                         =0X010a
//		,VOICE_GUIDED_FOLLOW                    =0X010b
//		,GESTURE_GUIDED_FOLLOW                  =0X010c
	
//	};

    enum controlMode
    {
	     PATH_RECORD_START                      =0x0501
        ,PATH_RECORD_STOP                       =0x0601
        ,PATH_RECORD_CLEAR                      =0X0701	
        ,PATH_TRACKING_NORMAL                   =0X0102
        ,PATH_TRACKING_DOWNGRADE                =0X0202
        ,PATH_TRACKING_CIRCLE                   =0X0302
        ,PATH_TRACKING_DOWNGRADE_CIRCLE         =0X0402
        ,ONE_RETURN_NOMAL                       =0X0103
        ,ONE_RETURN_DOWNGRADE                   =0X0203
        ,ONE_RETURN_CIRCLE                      =0X0303
        ,ONE_RETURN_DOWNGRADE_CIRCLE 	        =0X0403
        ,PERSON_FOLLOWING_NORMAL                =0X0104
        ,PERSON_FOLLOWING_DOWNGRADE             =0X0204
        ,PERSON_FOLLOWING_NORMAL_BACK           =0X0304
        ,PERSON_FOLLOWING_DOWNGRADE_BACK        =0X0404
        ,VEHICLE_FOLLOWING_NORMAL               =0X0107
        ,VEHICLE_FOLLOWING_DOWNGRADE	        =0X0207
        ,RESUME_AUTOPILOT                       =0X0109
        ,STOP_AUTOPILOT                         =0X010a
    };

	enum netState 
	{
		DISCONNECTED             	   =  0
		, NORMAL    				   =  1
		
	};
	Header header;
	controlMode control_mode;
	netState net_state;
	ushort MissionCount;
	float speed_limit;
	unsigned char deratedflag;
};

struct PlannerReturnedResult
{
	Header header;
	int8_t result;
};

struct PlatformSteeringCommand
{	
	Header header;
	float32_t trans_vel;     		// in m/s	
	float32_t curvature;			// in 1/m
};

struct HasPeopleNearVehicle
{
	Header header;
	int8_t has_people;
};

//2-D grid map, in which each cell represents the probability of occupancy.
struct OccupancyGridMap
{
	Header header;	
	MapMetaData info;				//MetaData for the map	
	std::vector<uint8_t> data;		//The map data, in row-major order, starting with (0,0).  
									//Occupancy probabilities are in the range [0,100].  Unknown is -1.
};

struct VoxelCostMap
{
	Header header;  
	MapMetaData info;				//MetaData for the map       
	std::vector<float32_t> data;	//vector of costs

	VoxelCostMap operator=(const VoxelCostMap& val) 
	{ 
		header = val.header;
		info = val.info;
		data.clear();
		data.resize(val.data.size());
		data = val.data;
		return *this; 
	}
};

struct HeightMap
{
	Header header;  
	MapMetaData info;				//MetaData for the map       
	std::vector<float32_t> data;	//vector of heights
}; 

/*struct PersonTracking
{
	Header header;
	float position_x;
	float position_y;
	float position_z;
	float velocity_x;
	float velocity_y;
	float velocity_z; 
};*/
struct PersonTracking
{
	Header header;
	float position_x;
	float position_y;
	float position_z;
	float velocity_total;
	char  track_type;    //1 person 2 car 
	char  track_state;  //0:未初始化 1:初始化失败 2：初始化成功 3：正在跟踪 4：目标丢失 5:急停
	float Millimeter_x;   //毫米波找车坐标x
	float Millimeter_y;   //毫米波找车坐标y  200代表没有检测到目标
	float target_speed;   //目标车速
	float Relative_velocity; //与前车相对速度
	int eStopCount;          //紧急停车信号,ADD 2020-05-20
};



 
struct ModuleState
{
	enum deviceState 
	{
		 NORMAL    					   =  0
		,ABNORMAL               	   =  1
	};
	enum plannedResult 
	{
		 GOAL_REACHED              =  2
		,SHORTER_PATH_FOUND        =  1
		,OK                        =  0
		,NO_PATH_FOUND             = -1
		,ERR_STARTPOSE_OUTSIDE_MAP = -2
		,ERR_EMPTY_GUIDANCE        = -3
		,ERR_INTERNAL              = -4
	};
	Header header;
	uint8_t state;
};

	struct PathRecordFrame   //录点记录减速帧
	{
		Header header;
		uint16_t slowFlag;    //减速标记
		uint16_t slowFlagCount; //减速标记累加数
	};

	struct SituationalEnhancement  //态势增强帧
	{
		Header header;
		uint16_t conmand;
	};

	struct PersionInit
	{
		cngicuvc::messages::Header persionHead;
		float persion_x;
		float persion_y;
		float radius;  
		//add by lh
		ushort missionType;
		ushort planMaxSpeed;
		
	};

	struct Priority    //底盘发送给HMI的优先级
	{
		cngicuvc::messages::Header PriorityHead;
		unsigned char Priority;
	};

	struct MileageFeedback //axy自主导航计算机向车载操控终端和便携式操控终端反馈里程、时间、以及软件版本等信
	{

		cngicuvc::messages::Header header;
		uint32_t mileage;
		uint32_t working_time_total;
		uint32_t working_time_power;
		uint32_t working_time_generation;
		uint32_t working_time_load;
        //unsigned char version_imu[8];//unknown
        //unsigned char version_navigation[8];//unknown
        //unsigned char version_reserved[8];//unknown

    };

    struct InsOperation
    {
        Header header;
        unsigned char operation_num;  // 0 不装订 1 装订
        unsigned char flag;// 0 停止， 1 自检 2 寻北
        unsigned int longitude;
        unsigned int latitude;
        unsigned int altitude;
        unsigned char param_num;
        unsigned int param;
    };




//**************************End the definition of messages****************************************//

}
}  //namespace cngicuvc

#endif  //CNGICUVC_UGV_TOPICS_H
