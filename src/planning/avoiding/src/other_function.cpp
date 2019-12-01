//calculate the minimum distance from a given point to the path
//heron's formula , but no direction.... //just record here
float Avoiding::dis2path2(const double& X_,const double& Y_)
{
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path_points_[target_point_index_].x - X_, 2) + 
					   pow(path_points_[target_point_index_].y - Y_, 2) ;
	
	float dis2next_target = pow(path_points_[target_point_index_+1].x - X_, 2) + 
							pow(path_points_[target_point_index_+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path_points_[target_point_index_-1].x - X_, 2) + 
					        pow(path_points_[target_point_index_-1].y - Y_, 2) ;
	
//	cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) <<endl;
	
	//heron's formula :a,b,c is three sides of triangle and p is half of its circumference
	float p,a,b,c;
	
	float first_dis ,second_dis ,third_dis;  //a^2 b^2 c^2
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index_;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
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
			{
				third_dis = pow(path_points_[second_point_index].x - path_points_[first_point_index].x , 2) + 
						    pow(path_points_[second_point_index].y - path_points_[first_point_index].y , 2) ;
				a = sqrt(first_dis);
				b = sqrt(second_dis);
				c = sqrt(third_dis);
				break;
			}
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
			{
				third_dis = pow(path_points_[second_point_index].x - path_points_[first_point_index].x , 2) + 
						    pow(path_points_[second_point_index].y - path_points_[first_point_index].y , 2) ;
				a = sqrt(first_dis);
				b = sqrt(second_dis);
				c = sqrt(third_dis);
				break;
			}
		}
	}
	else //midile
	{
		a = sqrt(dis2last_target);
		b = sqrt(dis2next_target);
		c = sqrt( pow(path_points_[target_point_index_+1].x - path_points_[target_point_index_-1].x , 2) + 
			      pow(path_points_[target_point_index_+1].y - path_points_[target_point_index_-1].y , 2)) ;
	}
	
	p = (a+b+c)/2;

	return sqrt(p*(p-a)*(p-b)*(p-c))*2/c;
}

/*
void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	//printf("danger_distance_front_:%f\tsafety_distance_front_:%f\t",danger_distance_front_,safety_distance_front_);
	//printf("danger_distance_side_:%f\tsafety_distance_side:%f\n\n",danger_distance_side_,safety_distance_side_);
	
	size_t n_object = objects->boxes.size();
	
	//ROS_INFO("n_object:%d",n_object);
	
	if(n_object==0)
	{
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		return;
	}
	
	float *obstacleDistances = new float[n_object]; //存放障碍物的距离
	
	float **obstacleVertex_x_y = new float *[n_object];
	for(size_t i=0;i<n_object;i++)
		obstacleVertex_x_y[i] = new float[2];
	
	size_t *obstacleIndices = new size_t[n_object]; //存放障碍物在目标中的索引
	
	whatArea_t *obstacleAreas = new whatArea_t[n_object]; //存放障碍物所在区域
	
//	bool *is_vetex_and_core_sameSide_ = new bool[n_object]; //障碍物顶点与其中心是否在同一侧
	
	size_t obstacleSequence = 0; //记录障碍物序号,用于按远近排序
	
	for(size_t i=0;i<n_object;i++)
	{
		this->get_obstacle_msg(objects,i,obstacleAreas,obstacleVertex_x_y,obstacleDistances,obstacleIndices,obstacleSequence);
		
	}
	
	//std::cout << "obstacleSequence:" <<obstacleSequence <<std::endl;
	bubbleSort(obstacleDistances,obstacleIndices,obstacleSequence); //障碍物由近到远排序
	
	if(obstacleSequence==0)  //所有目标物均在安全区
	{
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
//ROS_ERROR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
		goto Delete_memory;
	}
	
	//size_t nearest_obstacleIndex = obstacleIndices[0];
	
	//object_type_t nearest_obstacleType = object_type_t(objects->boxes[nearest_obstacleIndex].label);

	
	for(size_t i =0;i<obstacleSequence;i++)
	{
		whatArea_t area = obstacleAreas[i];
		if(area == DangerArea)  //dangerous!! brake right now!!
		{
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd2.set_speed =0.0;
			avoid_cmd_.cmd2.set_brake = 40;
			ROS_INFO("Danger!! x: %f\ty: %f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1]);
			break; 
		}
		else if(area == AvoidingArea)//AvoidingArea
		{
			object_type_t objectType = object_type_t(objects->boxes[obstacleIndices[i]].label);
			if(objectType==Person)//行人进入避障区  危险!
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate =true;
				avoid_cmd_.cmd2.set_speed =0.0;
				avoid_cmd_.cmd2.set_brake = 40;  //deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5));
				ROS_INFO("Person  just_decelerate  x: %f\ty: %f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1]);
				break;
			}
			else //other obstacle ,start to avoiding
			{
				start_avoidingFlag_.data = 1;
				///////////////pub_avoid_msg_to_gps_.publish(start_avoidingFlag_);
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate  = false;
		//ROS_ERROR("avoid_speed_:%f",avoid_speed_);
				avoid_cmd_.cmd2.set_brake = 0.0;
				
				float t_roadWheelAngle;
				if(obstacleVertex_x_y[i][0] > danger_distance_front_)
				{
					//float _x = objects->boxes[obstacleIndices[i]].pose.position.x/2;// 型心水平点
					float _x = obstacleVertex_x_y[i][0]/2; //避障区内_x > 0
				
					float _y = (fabs(obstacleVertex_x_y[i][1]) + safety_distance_side_)/2;
					float _theta = 2*atan(2*_x/_y);
					float turning_radius = _x / sin(_theta);
					t_roadWheelAngle = asin(vehicle_axis_dis_/turning_radius)*180/M_PI *5;
				}
				else
				{
					t_roadWheelAngle = 5.0;
				}
				
				t_roadWheelAngle = limit_steeringAngle(t_roadWheelAngle,15.0);
				
				avoid_cmd_.cmd2.set_speed = avoid_speed_;
				if(objects->boxes[obstacleIndices[i]].pose.position.y > 0.5) //障碍物在左侧
				{
					avoid_cmd_.cmd2.set_steeringAngle = -t_roadWheelAngle * 200.0/15;
ROS_INFO("car  avoiding  x: %f\ty: %f\t t_roadWheelAngle:%f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],-t_roadWheelAngle);
				}
				else  //right
				{
					avoid_cmd_.cmd2.set_steeringAngle = t_roadWheelAngle *200.0/15;
ROS_INFO("car  avoiding  x: %f\ty: %f\t t_roadWheelAngle:%f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],t_roadWheelAngle);
				}
				
				break;
			}
		}
		else if(area == PedestrianDetectionArea && objects->boxes[obstacleIndices[i]].label ==Person) 
		{
			float dis_x = objects->boxes[obstacleIndices[i]].pose.position.x;
			
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd2.set_speed = 0.0;
			if(dis_x>5)
				avoid_cmd_.cmd2.set_brake = deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5)); // -5 is safty distance!
			else
				avoid_cmd_.cmd2.set_brake = 40;
				
			ROS_INFO("Pedestrian....x: %f\ty: %f \tbrake:%f ",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],avoid_cmd_.cmd2.set_brake);
			//ROS_ERROR("speed: %f\tbrake: %f",avoid_cmd_.cmd2.set_speed,avoid_cmd_.cmd2.set_brake);
			break;
		}
		
	
	}
Delete_memory:
	//printf("set_brake:%f\t set_speed:%f\t speed:%f\t safety_distance_front_:%f\t danger_distance_front_:%f\n",
	//			avoid_cmd_.cmd2.set_brake,avoid_cmd_.cmd2.set_speed,vehicleSpeed_,safety_distance_front_,danger_distance_front_);
	pub_avoid_cmd_.publish(avoid_cmd_);
	
	
	delete [] obstacleDistances;
	for(size_t i=0;i<n_object;i++)
	{
		delete [] obstacleVertex_x_y[i];
	}
	delete [] obstacleVertex_x_y;
	delete [] obstacleIndices;
	delete [] obstacleAreas;
	//ROS_INFO("end");
}
*/
