#ifndef STRUCTS_H_
#define STRUCTS_H_

typedef struct ControlCmd
{
	ControlCmd()
	{
		validity = false;
		speed = 0.0;
		roadWheelAngle = 0.0;
	}
	bool validity;
	float speed;
	float roadWheelAngle;
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


#endif
