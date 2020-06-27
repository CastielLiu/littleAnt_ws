#ifndef STRUCTS_H_
#define STRUCTS_H_

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


#endif
