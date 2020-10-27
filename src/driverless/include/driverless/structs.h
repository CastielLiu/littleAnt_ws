#ifndef STRUCTS_H_
#define STRUCTS_H_

/*@brief 路径点信息
*/
typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
}gpsMsg_t;

typedef struct
{
	double x,y;
}point_t;


/*@brief 车辆控制信息
*/
typedef struct ControlCmd
{
	ControlCmd()
	{
		validity = false;
		speed = 0.0;
		roadWheelAngle = 0.0;
		turnLight = 0;
	}
	bool  validity;
	float speed;
	float roadWheelAngle;
	uint8_t turnLight; // 0 关灯,1左转,2右转
	uint8_t stopLight; // 0 关灯,
	
} controlCmd_t;

/*@brief 停车点信息
 */
typedef struct ParkingPoint
{
	ParkingPoint()
	{
		index = 0;
		parkingDuration = 0;
		isParking = false;
	}
	ParkingPoint(size_t _index,float _duration)
	{
		index = _index;
		parkingDuration = _duration;
		isParking = false;
	}
	
	size_t index; //停车点在全局路径中的索引
	float  parkingDuration; //停车时长s,若为0,则表示一直停车
	double parkingTime;     //停车时刻
	bool   isParking;       //正在停车
} parkingPoint_t;

/*@brief 路径转向信息
 */
typedef struct TurnRange
{
	enum TurnType
	{
		TurnType_Left = -1,
		TurnType_None = 0,
		TurnType_Right = 1,
	};

	int type;
	size_t start_index;
	size_t end_index;

	TurnRange(int _type, size_t _start_index, size_t _end_index)
	{
		type = _type;
		start_index = _start_index;
		end_index = _end_index;
	}

} turnRange_t;

/*@brief 车辆参数
*/
typedef struct VehicleParams
{
	float max_roadwheel_angle;
	float max_speed;
	float wheel_base;
	float wheel_track;
	float width;
	float length;

	bool validity;
	VehicleParams()
	{
		validity = false;
	}
}vehicleParams_t;

/*@brief 车辆状态
*/
typedef struct VehicleState
{
	float speed;

} vehicleState_t;


#endif
