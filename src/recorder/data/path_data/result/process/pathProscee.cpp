#include<iostream>
#include<cmath>
#include<vector>
#include<cstdio>
#include<stdint.h>

using namespace std;

#define _XY 0
#define _LATLON 1

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
	float curvature;
	
	float maxOffset_left;
	float maxOffset_right;
	uint8_t traffic_sign;
	uint8_t other_info;
	
}gpsMsg_t;

bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points,bool is_lon_lat)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		printf("open %s failed\r\n",file_path.c_str());
		return false;
	}
	printf("open %s ok \r\n",file_path.c_str());
	
	gpsMsg_t point;
	
	while(!feof(fp))
	{
		if(is_lon_lat == _LATLON)
			fscanf(fp,"%lf\t%lf\n",&point.longitude,&point.latitude);
		else
		{
			fscanf(fp,"%lf\t%lf\t%lf\t%f\t%f\t%f\t%d\t%d\n",&point.x,&point.y,&point.yaw,&point.curvature,
															&point.maxOffset_left,&point.maxOffset_right,
															&point.traffic_sign,&point.other_info);
			/*printf("%lf\t%lf\t%lf\t%f\t%f\t%f\t%d\t%d\n",point.x,point.y,point.yaw,point.curvature,
															point.maxOffset_left,point.maxOffset_right,
															point.traffic_sign,point.other_info);*/
		}
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
}

bool dumpPathPoints(std::string file_path,std::vector<gpsMsg_t>& points) //xy
{
	FILE *fp = fopen(file_path.c_str(),"w");
	
	if(fp==NULL)
	{
		printf("open %s failed\r\n",file_path.c_str());
		return false;
	}
	printf("open %s ok \r\n",file_path.c_str());
	gpsMsg_t point;
	
	for(size_t i=0;i<points.size();i++)
	{
		point = points[i];
		fprintf(fp,"%.3f\t%.3f\t%.3f\t%.5f\t%.3f\t%.3f\t%d\t%d\r\n",point.x,point.y,point.yaw,point.curvature,
														point.maxOffset_left,point.maxOffset_right,
														point.traffic_sign,point.other_info);
		fflush(fp);
	}
	return true;
}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}


float dis2Points_latLon(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude);
	float y = (point1.latitude - point2.latitude ) *111000;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

size_t findNearestPointIndex(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& point,bool is_lonlat)
{
	float min_dis = 99999999;
	float dis;
	size_t index = 0;
	for(size_t i=0;i<path_points.size();i++)
	{
		if(is_lonlat == _LATLON)
			dis = dis2Points_latLon(path_points[i],point,true);
		else
			dis = dis2Points(path_points[i],point,true);
		if(dis < min_dis)
		{
			min_dis = dis;
			index = i;
		}
	}
	cout << "mindis:" << min_dis << endl;
	return index;
}

void pathPointsProcess(std::vector<gpsMsg_t>& path_points)
{
	for(size_t i=0; i<path_points.size(); i++)
	{
		;
	
	
	
	}

}

void pointOffset(gpsMsg_t& point,float offset)
{
	point.x =  offset * cos(point.yaw) + point.x;
	point.y = -offset * sin(point.yaw) + point.y;
}

size_t findIndexByDis(const vector<gpsMsg_t>& path_points,size_t currentIndex, float t_dis, bool is_up)
{
	float dis;
	if(is_up)
	{
		for(size_t i=currentIndex; i< path_points.size();i++)
		{
			dis = dis2Points(path_points[i],path_points[currentIndex],true);
			if(dis > t_dis)
				return i;
		}	
	}
	else
	{
		for(size_t i=currentIndex; i> 0;i--)
		{
			dis = dis2Points(path_points[i],path_points[currentIndex],true);
			if(dis > t_dis)
				return i;
		}	
	}
	return 0;
}


int main()
{
	vector<gpsMsg_t> path_points_xy;
	vector<gpsMsg_t> path_points_lonlat;
	
	loadPathPoints("_path515.txt",path_points_xy,_XY); //x,y
	//loadPathPoints("test.txt",path_points_xy,_XY); //x,y
	loadPathPoints("path515_lonlat.txt",path_points_lonlat,_LATLON); //lon lat
	
	//cout << "size:" << path_points_xy.size() << "\t"<< path_points_lonlat.size()<<endl;
	/*
	gpsMsg_t pointA;
	pointA.x = 530384.784996;	
	pointA.y = 4324070.80623;

	size_t nearest_index = findNearestPointIndex(path_points_xy,pointA,_XY);  //nearest index
	size_t end_index = findIndexByDis(path_points_xy,nearest_index,10.0,true);
	size_t start_index = findIndexByDis(path_points_xy,nearest_index,10.0,false);
	*/
	
	gpsMsg_t point_lonlat;
	point_lonlat.latitude = 117.;
	point_lonlat.longitude = 31.;
	
	size_t nearest_index = findNearestPointIndex(path_points_lonlat, point_lonlat,_LATLON);  //nearest index
	size_t end_index = findIndexByDis(path_points_xy,nearest_index,10.0,true); //up
	size_t start_index = findIndexByDis(path_points_xy,nearest_index,10.0,false); //down
	
	cout << "nearest_index:"<< nearest_index <<"\t start_index:"<<start_index <<"\t end_index:"<<end_index<<endl;
	
	for(size_t i=start_index;i<=end_index;i++)
	{
		pointOffset(path_points_xy[i],2.5);  /////offset
		cout << i <<endl;
	}
	
	dumpPathPoints("b.txt",path_points_xy);
	
	return 0;
	
}


