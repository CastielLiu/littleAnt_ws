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
