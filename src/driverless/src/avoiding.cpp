#include"avoiding.h"

Avoiding::Avoiding():
{
	danger_distance_front_ = 3.0;
	safety_distance_front_=20.0;
}

bool Avoiding::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("objects_topic", objects_topic_, "");
	nh_private.param<float>("deceleration_cofficient",deceleration_cofficient_,50);
	nh_private.param<float>("safety_distance_side",safety_distance_side_,0.4);
	nh_private.param<float>("danger_distance_side",danger_distance_side_,0.25);
	nh_private.param<float>("max_deceleration",max_deceleration_,5.0); 
	
	return true;
}

void Avoiding::publishDiagnostics(uint8_t level,const std::string& msg)
{
	diagnostic_msg_.level = level;
	diagnostic_msg_.message = msg;
	pub_diagnostic_.publish(diagnostic_msg_);
}

bool PathTracking::setGlobalPath(const std::vector<gpsMsg_t>& path)
{
	if(path_points_.size()!=0)
		return false;
	path_points_ = path;
	return true;
}

void Avoiding::start()
{
	sub_objects_ = nh.subscribe(objects_topic_,1,&Avoiding::objects_callback,this);
}

void Avoiding::updateStatus(const gpsMsg_t& pose,const float& speed, size_t nearest_index)
{

	danger_distance_front_ = generateDangerDistanceBySpeed(speed);  
	safety_distance_front_ = generateSafetyDisByDangerDis(danger_distance_front_);
}

void Avoiding::objects_callback(const euclidean_cluster::ObjectPolygonArray::ConstPtr& objects)
{
	size_t n_object = objects->polygons.size();
	
	if(n_object==0)
	{
		return;
	}


	
	size_t *indexArray = new size_t[n_object];
	float * dis2vehicleArray = new float[n_object];
	float * dis2pathArray = new float[n_object];
	
	jsk_recognition_msgs::BoundingBox object;
	
	//object position in vehicle coordination (x,y)
	//object position in  world  coordination (X,Y)
	float x,y;  double X,Y;
	
	for(size_t i=0; i< n_object; i++)
	{
		object =  objects->boxes[i];
		
		x = - object.pose.position.y;
		y =   object.pose.position.x;
		
		X =  x * cos(current_point_.yaw) + y * sin(current_point_.yaw) + current_point_.x;
		Y = -x * sin(current_point_.yaw) + y * cos(current_point_.yaw) + current_point_.y;
		
		indexArray[i] = i;
		dis2vehicleArray[i] = sqrt(x * x + y * y);
		try{
			dis2pathArray[i] = calculateDis2path(X,Y,path_points_,target_point_index_);
		}catch(const char* str){
			ROS_INFO("%s\t rosShutdown!",str);
			ros::shutdown();
		}		
		//printf("target x:%f\ty:%f\t X:%f\tY:%f\t\n",x,y,X,Y);
		//printf("car X:%f\t Y:%f\t yaw:%f\n",current_point_.x,current_point_.y,current_point_.yaw);
		//ROS_INFO("dis2path:%f\t dis2vehicle:%f\t x:%f  y:%f",dis2pathArray[i],dis2vehicleArray[i],x,y);
	}
	bubbleSort(dis2vehicleArray,indexArray,n_object);
	if(is_dangerous(objects, dis2vehicleArray,indexArray, dis2pathArray,n_object))
	{
		this->emergencyBrake();
		return ;
	}
	
	if(offset_msg_.data!=0.0 && is_backToOriginalLane(objects,dis2vehicleArray,indexArray,dis2pathArray,n_object))
	{
		backToOriginalLane();
	}
	
	if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_Ambulance)
	{
		bool is_ambulance = false;
		bool is_changLaneSafety = true;
		jsk_recognition_msgs::BoundingBox object; 
		for(size_t i=0; i<n_object; i++)
		{
			object = objects->boxes[indexArray[i]];
			float dis2path = dis2pathArray[indexArray[i]];
			float safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_;
			if(object.pose.position.x < -2.0 && dis2path < 1.0) //ambulance
			{
				is_ambulance = true;
			}
			else if(object.pose.position.x <20.0 && dis2path >0.0 &&
			       dis2path-3.0 < safety_center_distance_x)
			{
				is_changLaneSafety  = false; 
				break;
			}
		}
		if(is_ambulance && is_changLaneSafety)
		{
			offset_msg_.data = 2.5;
			pub_avoid_msg_to_gps_.publish(offset_msg_);
		}
	}
	else if(path_points_[nearest_point_index_].traffic_sign != TrafficSign_PickUp &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_TempStop &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_UTurn &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_Avoid)
	{
		//dis2vehicleArray was sorted but dis2pathArray not!
		decision(objects, dis2vehicleArray,indexArray, dis2pathArray,n_object);
	}
	
	delete [] indexArray;
	delete [] dis2vehicleArray;
	delete [] dis2pathArray;
}

inline void Avoiding::decision(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
				const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int n_object)
{
	jsk_recognition_msgs::BoundingBox object; 
	float try_offest[2] ={0.0,0.0};
	float safety_distance_front;
	float safety_center_distance_x;
	float dis2path;
	float dis2vehicle;
	
	std::vector<size_t> vehicleObstacle_indexArray; //for car following
	
	//float lateral_err = calculateDis2path(current_point_.x,current_point_.y,path_points_,target_point_index_);
	
	for(size_t i=0; i<n_object; i++)
	{
		dis2vehicle = dis2vehicleArray[i];
		object = objects->boxes[indexArray[i]];
		safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_;
		//dis2path:    distance from the object to the current path 
		//dis2pathArray[indexArray[i]] : distance from the object to the origin path
		//obstacle avoidance offset has been set in last time
		dis2path = dis2pathArray[indexArray[i]] - offset_msg_.data;
		
		if(try_offest[0] != 0.0)
		{
			safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			dis2path -= try_offest[0];
		}
		else
			safety_distance_front =safety_distance_front_;
		
		float expand_safty_width = 0.0;
		if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_IllegalPedestrian)
			expand_safty_width = 0.6;
			
		//object is outside the avoding area
		if(dis2vehicle >= safety_distance_front || object.pose.position.x <0 || fabs(dis2path) >= safety_center_distance_x +expand_safty_width)
			continue;
		else if(fabs(dis2path) <= (safety_center_distance_x + expand_safty_width) &&
				object.pose.position.x > 1.0 &&
				object.label == Person)
		{
			std::stringstream ss;
			ss << "pedestrian crossing..... dis2vehicle:" << dis2vehicle <<"  dis2path:"<< dis2path << " safe_x:" <<safety_center_distance_x  << " "
				<< expand_safty_width;
			
			if(dis2vehicle <= danger_distance_front_*2)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.cmd2.set_brake = 80.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 3.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			else if(dis2vehicle <= danger_distance_front_*3)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.cmd2.set_brake = 60.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 8.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is inside the true avoiding area
			else if(dis2vehicle <= safety_distance_front_)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.cmd2.set_brake = 40.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 10.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is just in the false avoid area, so pass
			// when test_offset not zero: safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			continue;
		}
		//object is other type, start to avoid
		float offset_increment = dis2path - safety_center_distance_x;//try avoid in the left
		if(offset_increment<0 )
			try_offest[0] += offset_increment; //try avoid in the left	
		//ROS_INFO("left try offset:%f\t dis2path:%f safety_dis:%f",try_offest[0],dis2path,safety_center_distance_x);
		//ROS_INFO("trueOffset:%f\t x:%f\t y:%f\t width:%f",
		//			offset_msg_.data,object.pose.position.x,object.pose.position.y,object.dimensions.y);
		//ROS_INFO("lateral_err:%f \t dis2truepath:%f",lateral_err,dis2pathArray[indexArray[i]]);
	}
	
	for(size_t i=0; i<n_object; i++)
	{
		dis2vehicle = dis2vehicleArray[i];
		object = objects->boxes[indexArray[i]];
		safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_;
		dis2path = dis2pathArray[indexArray[i]] - offset_msg_.data;
		
		if(try_offest[1] != 0.0)
		{
			safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			dis2path -= try_offest[1];
		}
		else
			safety_distance_front = safety_distance_front_;
		
		//object is outside the avoding area
		if(dis2vehicle >= safety_distance_front || object.pose.position.x <0 || fabs(dis2path) >= safety_center_distance_x)
			continue;
		
		if(object.label != Person)
			vehicleObstacle_indexArray.push_back(indexArray[i]);
			
		//object is other type, start to avoid
		float offset_increment = dis2path + safety_center_distance_x; //try avoid in the right
		if(offset_increment>0 )
			try_offest[1] += dis2path + safety_center_distance_x;
	}
	
	try_offest[0] += offset_msg_.data;
	try_offest[1] += offset_msg_.data;
	
	if(path_points_[nearest_point_index_].traffic_sign ==TrafficSign_AccidentArea)
	{
		maxOffset_left_ = 0.0;
		maxOffset_right_ =0.0;
	} 
	else if(path_points_[nearest_point_index_].traffic_sign ==TrafficSign_JamArea)
	{
		maxOffset_left_ = -3.5;
		maxOffset_right_= 3.5;
	}
	else if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_AvoidStartingCar)
	{
		maxOffset_left_ = -0.3;
		maxOffset_right_ = 0.0;
	}
	else if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_Ambulance)
	{
		maxOffset_left_ = 0.0;
		maxOffset_right_ = 3.0;
	}
	
	else if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_Avoid)
	{
		maxOffset_left_ = -3.5;
		maxOffset_right_ = 0.0;
	}
	else if(path_points_[nearest_point_index_].traffic_sign ==TrafficSign_BusStop)
	{
		maxOffset_left_  =0.0;
		maxOffset_right_ = 0.0;
	}
	else
		maxOffset_left_ = maxOffset_right_ = 0.0;
	
	if(try_offest[0] >0)
		try_offest[0] = -10.0;  //a big num
	if(try_offest[1]<0)
		try_offest[1] = 10.0;
		
	//no avoid message
	if(try_offest[0]==0.0 && try_offest[1] ==0.0) 
	{
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		return;
	}
	//avoid message is invalid ,must slow down ,perhaps not brake!
	else if(try_offest[0] < maxOffset_left_ && try_offest[1] > maxOffset_right_)
	{
		if(is_carFollow_ == false)
		{
			float _min_distance=100.0;  //a big number
			jsk_recognition_msgs::BoundingBox box;
			for(size_t i=0; i< vehicleObstacle_indexArray.size(); i++)
			{
				box = objects->boxes[vehicleObstacle_indexArray[i]];
				float _dis = box.pose.position.x - box.dimensions.x/2;
				if(_dis < _min_distance)
					_min_distance = _dis;
				
			}
			float t_deceleration = 0.5*vehicle_speed_*vehicle_speed_/(_min_distance-5.0);
			
			if(t_deceleration < 0.0)
			{
				avoid_cmd_.cmd2.set_speed = 0.0;
			}
			else
			{
				avoid_cmd_.cmd2.set_speed = 15.0;
				avoid_cmd_.cmd2.set_brake = t_deceleration/g_max_deceleration * 60 + 40.0;
				ROS_DEBUG("t_deceleration:%f\t set_brake:%f",t_deceleration,avoid_cmd_.cmd2.set_brake);
			}
			
			avoid_cmd_.status = true;
			if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_CarFollow)
				requestCarFollowing(objects,vehicleObstacle_indexArray);
		}
		else
		{
			avoid_cmd_.status = false;
			if(path_points_[nearest_point_index_].traffic_sign != TrafficSign_CarFollow)
				is_carFollow_ = false;
		}
		pub_avoid_cmd_.publish(avoid_cmd_);
	}
	//avoid message is valid
	//left offest is smaller,so avoid from left side
	else if(( -try_offest[0] <= try_offest[1] && try_offest[0] > maxOffset_left_) ||
			(-try_offest[0] > try_offest[1] && try_offest[0] > maxOffset_left_ && 
			try_offest[1] >maxOffset_right_))
	{
		//assuming that no deceleration is required for avoidance
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		
		offset_msg_.data = try_offest[0];
		pub_avoid_msg_to_gps_.publish(offset_msg_);
	}
	//right offest is smaller,so avoid from right side
	else
	{
		//assuming that no deceleration is required for avoidance
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		
		offset_msg_.data = try_offest[1];
		pub_avoid_msg_to_gps_.publish(offset_msg_);
	}
	
	std::stringstream debug_msg;
	debug_msg << "try_offest_L: " <<    try_offest[0] << "  max_L: "<< maxOffset_left_;
	debug_msg << "  try_offest_R: " <<  try_offest[1] << "  max_R: "<< maxOffset_right_ ;
	debug_msg << "  offset: " << offset_msg_.data;
}


inline bool Avoiding::is_backToOriginalLane(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object)
{
	float safety_center_distance_x; //the safety distance along x axis
	
	bool is_ok = true;
	
	for(size_t i=0; i<n_object; i++)
	{
		safety_center_distance_x = g_vehicle_width/2 + objects->boxes[indexArray[i]].dimensions.y/2 + safety_distance_side_;
		
		if(dis2vehicleArray[i] < safety_distance_front_*1.2 && 
		   objects->boxes[indexArray[i]].pose.position.x > -20.0 &&
		   fabs(dis2pathArray[indexArray[i]]) <= safety_center_distance_x)
		{
			is_ok = false;
			break;
		}
	}
	return is_ok;
}

inline bool Avoiding::is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
					const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object)
{
	if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_UTurn ||
	   nearest_point_index_ > 4210)
		return false;
		
	float safety_center_distance_x; //the safety distance along x axis
	float safety_center_distance_y; //the safety distance along y axis
	float x,y;
	for(size_t i=0; i<n_object; i++)
	{
		safety_center_distance_x = g_vehicle_width/2 + objects->boxes[indexArray[i]].dimensions.y/2 + danger_distance_side_;
		safety_center_distance_y = g_vehicle_length/2+ objects->boxes[indexArray[i]].dimensions.x/2 + danger_distance_front_;
		x = -objects->boxes[indexArray[i]].pose.position.y;
		y = objects->boxes[indexArray[i]].pose.position.x;
		
		if(y <= safety_center_distance_y && y > -g_vehicle_length/2  && fabs(x) <= safety_center_distance_x )
		{
			std::stringstream ss ;
			ss << "safety_x:"<<safety_center_distance_x <<"  safety_y:"<<safety_center_distance_y<<"  x:"<<x <<"  y:"<<y;
			return true;
		}
	}
	return false;
}

std::pair<double,double> Avoiding::vehicleToWorldCoordination(float x,float y)
{
	//object position in world coordination
	double X =  x * cos(current_point_.yaw) + y * sin(current_point_.yaw) + current_point_.x;
	double Y = -x * sin(current_point_.yaw) + y * cos(current_point_.yaw) + current_point_.y;
	return std::pair<double,double>(X,Y);
}


float Avoiding::calculate_dis2path(const double& X_,const double& Y_)
{
	//ROS_INFO("path_points_.size:%d\t target_point_index_:%d",path_points_.size(),target_point_index_);
	
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path_points_[target_point_index_].x - X_, 2) + 
					   pow(path_points_[target_point_index_].y - Y_, 2) ;
	
	float dis2next_target = pow(path_points_[target_point_index_+1].x - X_, 2) + 
							pow(path_points_[target_point_index_+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path_points_[target_point_index_-1].x - X_, 2) + 
					        pow(path_points_[target_point_index_-1].y - Y_, 2) ;
	
	//std::cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) << std::endl;
	
	float first_dis ,second_dis ,third_dis;  //a^2 b^2 c^2
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index_;
	
	size_t direction = 1;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		direction = -1;
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index_-i;
			
			second_dis = pow(path_points_[second_point_index].x - X_, 2) + 
						 pow(path_points_[second_point_index].y - Y_, 2) ;
			
			if(second_dis < first_dis) //continue 
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else if(dis2next_target < dis2target && dis2last_target > dis2target) //upward
	{
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index_ + i;
			second_dis = pow(path_points_[second_point_index].x - X_, 2) + 
						 pow(path_points_[second_point_index].y - Y_, 2) ;

			if(second_dis < first_dis) //continue
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else //midile
	{
		first_point_index = target_point_index_-1;
		second_point_index = target_point_index_ +1;
	}
	
	//the direction of side c 
	//float yaw_of_c = (path_points_[first_point_index].yaw + path_points_[second_point_index].yaw)/2;
	float yaw_of_c = direction * atan2(path_points_[second_point_index].x-path_points_[first_point_index].x,
									   path_points_[second_point_index].y-path_points_[first_point_index].y);
						   
	//object : world coordination to local coordination
	float x = (X_-path_points_[first_point_index].x) * cos(yaw_of_c) - (Y_-path_points_[first_point_index].y) * sin(yaw_of_c);
	//float y = (X_-path_points_[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points_[first_point_index].y) * cos(yaw_of_c);
	
	return x;
}

void Avoiding::get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
								size_t objectIndex,whatArea_t *obstacleArea,
								float ** obstacleVertex_x_y,
								float *obstacleDistance, 
								size_t *obstacleIndex,
								size_t &obstacleSequence)
{
	float core_x = objects->boxes[objectIndex].pose.position.x;
	float core_y = objects->boxes[objectIndex].pose.position.y;
	
	float x = core_x;
	float y = core_y;
	
	float delta_x = objects->boxes[objectIndex].dimensions.x/2;
	float delta_y = objects->boxes[objectIndex].dimensions.y/2;
	
	if(x>0&&y>0) //left front
	{
		x -= delta_x;
		y -= delta_y;
	}
	else if(x>0&&y<0) //right front
	{
		x -= delta_x;
		y += delta_y;
	}
	else if(x<0&&y>0) //left rear
	{
		x += delta_x;
		y -= delta_y;
	}
	else //right rear
	{
		x += delta_x;
		y += delta_y;
	}
	
	whatArea_t area = which_area(x,y);

	if( area != SafetyArea)
	{
		obstacleVertex_x_y[obstacleSequence][0] = x;
		obstacleVertex_x_y[obstacleSequence][1] = y;
		obstacleArea[obstacleSequence] = area;
		
		obstacleDistance[obstacleSequence] = sqrt(x*x+y*y);
		obstacleIndex[obstacleSequence] = objectIndex;
		obstacleSequence ++;
	}
}

whatArea_t Avoiding::which_area(float& x,float& y)
{
	if(y>safety_distance_side_ || y< -safety_distance_side_ || x>safety_distance_front_ || x< 1.0)
	{
		if(x<safety_distance_front_ && x> safety_distance_front_/4 &&  //行人检测区域限定
			y < pedestrian_detection_area_side_ && y> -pedestrian_detection_area_side_) //行人检测区
			return PedestrianDetectionArea;
		else
			return SafetyArea;//safety
	}
	else if(y>danger_distance_side_ || y<-danger_distance_side_ || x>danger_distance_front_)
		return AvoidingArea; //avoiding
	else
		return DangerArea; //danger!
}


float Avoiding::deceleration_2_brakingAperture(const float & deceleration)
{
	float brakingAperture = (deceleration * deceleration_cofficient_);
	return brakingAperture>40? 40:brakingAperture;
}

float Avoiding::brakingAperture_2_deceleration(const float & brakingAperture)
{
	return brakingAperture / deceleration_cofficient_;
}

void Avoiding::bubbleSort(const float * distance, size_t * index, size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length- i - 1; j++)
		{
			if (distance[j] > distance[j + 1])
			{
				size_t ind_temp = index[j];
				index[j] = index[j + 1];
				index[j + 1] = ind_temp;
			}
		}
	}
}

inline void Avoiding::emergencyBrake()
{
	avoid_cmd_.status = true;
	avoid_cmd_.cmd2.set_brake = 100.0;  //waiting test
	avoid_cmd_.cmd2.set_speed = 0.0;
	pub_avoid_cmd_.publish(avoid_cmd_);
	//ROS_ERROR("dangerous! emergency brake!!!!!");
}
