#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <atomic>
#include <boost/thread/locks.hpp>    
#include <boost/thread/shared_mutex.hpp>    

typedef boost::shared_mutex SharedMutex;
typedef boost::unique_lock<SharedMutex> WriteLock;
typedef boost::shared_lock<SharedMutex> ReadLock;

typedef struct
{
	double x,y;
}point_t;


/*@brief 车辆控制信息*/
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

/*@brief 停车点信息*/
class ParkingPoint
{
public:
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
};

class ParkingPoints
{
public:
	std::vector<ParkingPoint> points;
	size_t next_index = 0;
	bool sorted = false;

	size_t size() const {return points.size();}
	void push_back(const ParkingPoint& point)
	{
		points.push_back(point);
	} 
	const ParkingPoint& operator[](size_t i)const  {return points[i];}
	ParkingPoint& operator[](size_t i)             {return points[i];}

	bool available() const { return next_index  < points.size();}

	void sort() //停车点由小到大排序
	{
		std::sort(points.begin(),points.end(),
			[](const ParkingPoint& point1,const ParkingPoint& point2)
			{return point1.index < point2.index;});
		sorted = true;
	}

	bool isSorted() const {return sorted;}

	void print(const std::string& prefix) const 
	{
		for(auto &point:points)
			printf("[%s] parking point index: %lu  duration: %.1f",prefix.c_str(), point.index,point.parkingDuration);
	}
	
	ParkingPoint& next()
	{
		if(!available())
			next_index = 0;

		return points[next_index];
	}

};


/*@brief 路径转向区间信息 */
class TurnRange
{
public:
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
};

class TurnRanges
{
public:
	std::vector<TurnRange> ranges;

	size_t size() const {return ranges.size();}

};


/*@brief 位姿信息*/
class Pose
{
public:
	double x, y, z, yaw;
};

/*@brief 路径点信息*/
class GpsPoint : public Pose
{
public:
	double longitude;
	double latitude;
	float curvature;
};

class Path
{
public:
	std::vector<GpsPoint> points;
	float resolution;

	std::atomic<size_t> pose_index;    //距离车辆最近路径点的索引
	size_t final_index;                //终点索引

	ParkingPoints park_points;         //停车点信息
	TurnRanges    turn_ranges;		   //转向取件信息

public:
	size_t size() const {return points.size();}
	const GpsPoint& operator[](size_t i) const {return points[i];}
	GpsPoint& operator[](size_t i)             {return points[i];}

};

/*@brief 车辆参数 */
class VehicleParams
{
public:
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
};

/*@brief 车辆状态 内部状态+外部状态
 * 更新车辆状态的线程利用类方法进行更新
 * 读取车辆状态的线程先创建副本，然后直接访问副本成员
*/
#define LOCK true
#define UNLOCK  false
class VehicleState 
{
public:
	float speed;        //车速
	float steer_angle;  //前轮转角
	Pose  pose;         //车辆位置

	bool speed_validity = false;
	bool steer_validity = false;
	bool pose_validity  = false;

	SharedMutex wr_mutex;//读写锁

	void setSpeed(const float& val)
	{
		WriteLock writeLock(wr_mutex);
		speed = val;
	}

	void setSteerAngle(const float& val)
	{
		WriteLock writeLock(wr_mutex);
		steer_angle = val;
	}

	void setPose(const Pose& val)
	{
		WriteLock writeLock(wr_mutex);
		pose = val;
	}

	float getSpeed(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return speed;
		}
		return speed;
	}
	float getSteerAngle(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return steer_angle;
		}
		return steer_angle;
	}

	Pose getPose(bool lock = UNLOCK)
	{
		if(lock)
		{
			ReadLock readLock(wr_mutex);
			return pose;
		}
		return pose;
	}

	VehicleState(){} //当定义了拷贝构造函数时，编译器将不提供默认构造函数，需显式定义

	VehicleState(const VehicleState& obj)
	{
		ReadLock readLock(wr_mutex);
		this->speed       = obj.speed;
		this->steer_angle = obj.steer_angle;
		this->pose        = obj.pose;
		this->speed_validity    = obj.speed_validity;
		this->steer_validity    = obj.steer_validity;
		this->pose_validity     = obj.pose_validity;
	};
	const VehicleState& operator=(const VehicleState& obj)
	{
		ReadLock readLock(wr_mutex);
		this->speed       = obj.speed;
		this->steer_angle = obj.steer_angle;
		this->pose        = obj.pose;
		this->speed_validity    = obj.speed_validity;
		this->steer_validity    = obj.steer_validity;
		this->pose_validity     = obj.pose_validity;
		return *this;
	};

	bool validity(std::string& info)
	{
		bool ok = true;
		if(!speed_validity)
		{
			info += "Vehicle speed is validity!\t";
			ok = false;
		}
		if(!steer_validity)
		{
			info += "Vehicle steer angle is validity!\t";
			ok = false;
		}
		if(pose.x <100 || pose.y <100) //the pose from gps is invailed!
		{
			info += "Vehicle pose is validity!";
			ok = false;
			pose_validity = false;
		}
		else
			pose_validity = true;
		return ok;
	}


};


#endif
