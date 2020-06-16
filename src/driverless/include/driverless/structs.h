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


#endif
