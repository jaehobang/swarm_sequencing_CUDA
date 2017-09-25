#include "ros/ros.h"
#include "custom_messages/C2R.h"
#include "custom_messages/R2C.h"

#include "visualization_msgs/MarkerArray.h"
#include "smha_headers.hpp"

#include <fstream>
#include <sstream>

using namespace std;

RETURN testmain(PARAM* parametereter, int is_aided, std::vector<float> time_array, std::vector<int> sequence_array, std::vector<uint8_t> isFixed);
//int testmain();
RETURN return_struct; 
ros::Publisher ci_publisher;
ros::Publisher cr_publisher;
ros::Publisher cr1_publisher;
PARAM* parameter;
int N = 10;
int MAP_NUM = 0;
std::vector<int> map_sequence;

int errorCheckR2C(const custom_messages::R2C::ConstPtr& msg)
{
  if(msg->time_array.size() > 10) {
    ROS_INFO("Too many switch times selected....\n");
    return 1;
  }
  if(msg->is_aided == 0 && msg->sequence_int_array.size() != msg->time_array.size())
  {
    ROS_INFO("Number of switch times does not match number of sequence for unaided case....\n");
    return 1;
  }
}

void publishC2R()
{
  custom_messages::C2R c2r;  

  for(int i = 0; i < return_struct.sequence_end_indices.size(); i++)
  {
    printf("sequence_end_indices for [%d] is %d\n", i,
      return_struct.sequence_end_indices[i]);
  }

  c2r.cost_of_path = return_struct.cost_of_path;
  if(return_struct.is_valid_path == 0)
    c2r.cost_of_path = 100000;
  c2r.is_valid_path = return_struct.is_valid_path;
  c2r.is_complete = return_struct.is_complete;
  c2r.sequence_string_array = return_struct.sequence_string_array;
  c2r.eot = 0;  
  c2r.map_number = map_sequence[MAP_NUM];
  ci_publisher.publish(c2r);
	ros::spinOnce();
 
	return;
}

void getRGB(int* r, int* g, int* b, string behavior)
{ 
// ROS_INFO("Inside getRGB, behavior is %s", behavior.c_str());
	if(behavior == "Rendezvous"){//rendezvous - cyan
			*r=0;
			*g=255;
			*b=255;
	} else if(behavior == "Flocking"){ //flocking - purple
			*r=160;
			*g=32;
			*b=240;
	}	else if(behavior == "Flock East"){ //flock_east - gray
			*r=128;
			*g=128;
			*b=128;
	}	else if(behavior == "Flock North"){ //floack_north - brown
			*r=165;
			*g=42;
			*b=42;
	}	else if(behavior ==  "Flock West"){ //flock_west - darkgreen
			*r=0;
			*g=100;
			*b=0;
	}	else if(behavior ==  "Flock South"){ //flock_south - orange
			*r=255;
			*g=165;
			*b=0;
	} else if(behavior == "Antirendezvous"){ //antirendezvous - pink
			*r=255;
			*g=20;
			*b=147;
	}
	return;
}

void deleteMarkerArray()
{
  visualization_msgs::MarkerArray mk_arr;
  visualization_msgs::Marker mk;
  
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.action = visualization_msgs::Marker::DELETEALL;
  mk.ns = "delete";
  mk.id = 0;
  mk_arr.markers.push_back(mk);
  cr_publisher.publish(mk_arr);


}


void publishMarkerArray()
{
  //Delete all trajectories on the map first

  deleteMarkerArray();

	ros::Rate i_Rate(1);
  visualization_msgs::MarkerArray mk_arr;
  visualization_msgs::Marker mk;
  geometry_msgs::Point p;

  i_Rate.sleep();

	int sub_i = 0;
	int behav_i = 0;
  int pos_size = return_struct.sequence_end_indices[return_struct.sequence_end_indices.size() - 1] + 1;
	ROS_INFO("pos_size is %d", pos_size);

	for(int i = 0; i < return_struct.sequence_string_array.size(); i++)
  {
    std::cout << return_struct.sequence_string_array[i] << std::endl;
  }

  ROS_INFO("sequence_end_indices.size() is %d", return_struct.sequence_end_indices.size());

  /* We can have maximum of 10 marker array messages 
     Each marker array message will ha	r	param->fix_count = fix_count;
  param->fix_count = fix_count;
  ve 10 markers 
     Delete all */
  /*
  ros::Rate tmp_rate(10);
  for(int i = 0; i < 10; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			mk.ns = "robot" + std::to_string(j);
			if( j == 0) mk.id = 0;
			else mk.id++;
			mk.action = visualization_msgs::Marker::DELETE;
      mk.lifetime = ros::Duration(0.1);
		  mk_arr.markers.push_back(mk);
		}
		cr_publisher.publish(mk_arr);
		tmp_rate.sleep();
	}
  */



  for(int pos_i = 0; pos_i < pos_size; pos_i++)
	{
		for(int robot_i = 0; robot_i < N; robot_i++)
		{
  		//if pos_i == 0 add the marker to marker array
			//whenever pos_i becomes an index in sequence_end_indices, publish the message
	
      if(pos_i == 0)
			{
				mk.header.stamp = ros::Time::now();
				mk.header.frame_id = "map";
				mk.ns = "robot" + std::to_string(robot_i);
				mk.id = 0;
				mk.type = visualization_msgs::Marker::LINE_STRIP;
				mk.action = visualization_msgs::Marker::ADD;
				mk.scale.x = 0.2;
				mk.scale.y = 0.2;
				mk.scale.z = 0.2;
				mk.lifetime = ros::Duration();
				int r,g,b,a;
				getRGB(&r, &g, &b, return_struct.sequence_string_array[behav_i]);
				mk.color.r = (float) r / 255;
				mk.color.g = (float) g / 255;
				mk.color.b = (float) b / 255;
				mk.color.a = 1.0;
				mk_arr.markers.push_back(mk);
			}
			//do whatever
			p.x = return_struct.robot_positions[pos_i].robot_pos[robot_i][0];
			p.y = return_struct.robot_positions[pos_i].robot_pos[robot_i][1];
			p.z = 0;
	  	//if(pos_i == pos_size -1)
			//{
			//	ROS_INFO("for robot[%d] x,y = %f, %f", robot_i, p.x, p.y);
			//}


			mk_arr.markers[robot_i].points.push_back(p);
		}
		if(pos_i == return_struct.sequence_end_indices[sub_i])
		{
			//publish the message
			//debugging
			//ROS_INFO("about to publish the message");			

			cr_publisher.publish(mk_arr);
      //ROS_INFO("publish complete!");
			//ROS_INFO("Number of markers are...%d", mk_arr.markers.size());
      //for(int i = 0; i < mk_arr.markers.size(); i++)
		//	{
			//	ROS_INFO("Number of points in %d is %d", i, mk_arr.markers[i].points.size());
			//}
			behav_i++;
			sub_i++;
			ROS_INFO(" pos_i is %d", pos_i);
			ROS_INFO(" behav_i = %d, sub_i = %d", behav_i, sub_i);
			if(sub_i != return_struct.sequence_end_indices.size())
      {

				for(int robot_j = 0; robot_j < N; robot_j++)
				{
					mk_arr.markers[robot_j].points.clear();
					mk_arr.markers[robot_j].id++;
					int r,g,b;
					//ROS_INFO("new sequence is %s", return_struct.sequence_string_array[behav_i]);
					getRGB(&r, &g, &b, return_struct.sequence_string_array[behav_i]);
					//ROS_INFO("returned from getRGB");
					mk_arr.markers[robot_j].color.r = (float) r / 255;
		  		mk_arr.markers[robot_j].color.g = (float) g / 255;
					mk_arr.markers[robot_j].color.b = (float) b / 255;
				}
			}
      ROS_INFO("update mk_arr complete");
			ros::spinOnce();
			i_Rate.sleep();

		}
	}		
}

void processParam(std::vector<std::string> tokens, PARAM* parameter)
{
  // Assign parametereters according to the parametereter name
  if (tokens[0] == "N")
    parameter->N = std::stof(tokens[1]);
  else if (tokens[0] == "robot_radius")
    parameter->robot_radius = std::stof(tokens[1]);
  else if (tokens[0] == "M")
    parameter->M = std::stof(tokens[1]);
  else if (tokens[0] == "H")
    parameter->H = std::stof(tokens[1]);
  else if (tokens[0] == "H2")
    parameter->H2 = std::stof(tokens[1]);
  else if (tokens[0] == "q_count")
    parameter->q_count = std::stof(tokens[1]);
  else if (tokens[0] == "mapsize")
    parameter->mapsize = std::stof(tokens[1]);
  else if (tokens[0] == "ti")
    parameter->ti = std::stof(tokens[1]);
  else if (tokens[0] == "tf")
    parameter->tf = std::stof(tokens[1]);
  else if (tokens[0] == "dt")
    parameter->dt = std::stof(tokens[1]);
  else if (tokens[0] == "dT")
    parameter->dT = std::stof(tokens[1]);
  else if (tokens[0] == "target_radius")
    parameter->target_radius = std::stof(tokens[1]);

}


void generateRandomMapSequence()
{
  //training maps
  map_sequence.push_back(0);
  map_sequence.push_back(1);
  map_sequence.push_back(2);
  map_sequence.push_back(3);
  map_sequence.push_back(4);
  
  //test maps
  std::vector<int> basket = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
  std::srand(std::time(0));
  std::random_shuffle( basket.begin(), basket.end() );

  for(int i = 0; i < basket.size(); i++)
  {
    printf(" %d", basket[i]);
    map_sequence.push_back(basket[i]);
  }
  printf("\n");
  return;

}

void parseMap()
{
  ROS_INFO("BEFORE parameter->N = %d, parameter->M = %d", parameter->N, parameter->M);
  parameter->N = 10;
	parameter->H = 1;
  parameter->mapsize = 40;
	parameter->ti = 0;
	parameter->dt = 0.1;
	parameter->tf = 60;
	parameter->dT = 5;
	parameter->robot_radius = 0.5;
	parameter->q_count = 2;
	parameter->H2 = 1;
	
  string filename = "/home/jaeho-linux/hri2017/src/backend_C/maps"; //This needs to be modified for each computer.....
	filename += "/map" + std::to_string(map_sequence[MAP_NUM]) + ".txt";

	ifstream f(filename.c_str());
  if(f.good() == 0) {
		ROS_INFO("File does not exist!!!");
		return;
	}

  printf("OPENING MAP %d!!\n", map_sequence[MAP_NUM]);

	fstream file(filename);
  string str;
	unsigned int line = 1;
  size_t comment;

	 while (std::getline(file, str)) {
    comment = str.find('#', 0);
    if (comment != std::string::npos)
    {
      continue;
    }

    // String working variables
    std::istringstream iss(str);
    std::vector<std::string> tokens;

    // Place the parametereter and value into the token array
    copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));

    // Ensure valid line before processing parametereter
    if (tokens.size() == 2) {
      processParam(tokens, parameter);
    }

		if (tokens.size() == 1) {
      if (tokens[0] == "robot_pos")
      {
        int count = 0; 
        while (count < parameter->N)
        {
          std::getline(file, str);
          // String working variables
          std::istringstream iss(str);
          std::vector<std::string> tokens;

          // Place the parametereter and value into the token array
          copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
          if (tokens.size() != 3)
          {
            printf("Robot pos format wrong in file\n");
            return;
          }
          parameter->robot_pos[count][0] = std::stof(tokens[0]);
          parameter->robot_pos[count][1] = std::stof(tokens[1]);
          parameter->robot_pos[count][2] = std::stof(tokens[2]);
          count++;
        }
      }
      else if (tokens[0] == "obstacle_pos")
      {
        int count = 0;
        while (count < parameter->M)
        {
          std::getline(file, str);
          // String working variables
          std::istringstream iss(str);
          std::vector<std::string> tokens;

          // Place the parametereter and value into the token array
          copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
          if (tokens.size() != 3)
          {
            printf("obstacle pos format wrong in file\n");
            return;
          }
          parameter->obstacle_pos[count][0] = std::stof(tokens[0]);
          parameter->obstacle_pos[count][1] = std::stof(tokens[1]);
          parameter->obstacle_pos[count][2] = std::stof(tokens[2]);
          count++;
        }
      }
			else if (tokens[0] == "target_center")
      {
        std::getline(file, str);
        // String working variables
        std::istringstream iss(str);
        std::vector<std::string> tokens;

        // Place the parametereter and value into the token array
        copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
        if (tokens.size() != 2)
        {
          printf("target pos format wrong in file\n");
          return;
        }
        parameter->target_center[0] = std::stof(tokens[0]);
        parameter->target_center[1] = std::stof(tokens[1]);

      }
    }

    // Move to the next line in the parametereters file
    line++;
  }
	
  ROS_INFO("Making sure parameter is parse correctly...");
	ROS_INFO("Number of robots = %d, Number of obstacles = %d", parameter->N, parameter->M);
  /*
  for(int i = 0; i < parameter->N; i++)
	{
		ROS_INFO("Robot[%d] initial position is %f %f", i, parameter->robot_pos[i][0], parameter->robot_pos[i][1]);
	}
	for(int i = 0; i < parameter->M; i++)
	{
		ROS_INFO("Obstacle[%d] position = %f %f , radius = %f", i, parameter->obstacle_pos[i][0], parameter->obstacle_pos[i][1],
						parameter->obstacle_pos[i][2]);
  }
  ROS_INFO("target center = %f %f , radius = %f", parameter->target_center[0], parameter->target_center[1], 
						parameter->target_radius);
  */

}


void updateMap()
{
	while (cr1_publisher.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }//1. Parse the txt file that represents the map
	parseMap();
  MAP_NUM++;
	//2. Publish series of visualization_marker message to rviz for robot initial positions, obstacles, and goal
	//	Also send a visualization_marker_array message to erase the shown trajectory
  ros::Rate cr1_rate(10);
  visualization_msgs::Marker mk;
  visualization_msgs::MarkerArray mk_arr;
  mk.header.frame_id = "map";

	//2.0 eraseall
  
  deleteMarkerArray();
	mk.ns = "deleteall";
  mk.id = 0;
  mk.action = visualization_msgs::Marker::DELETEALL;
  cr1_publisher.publish(mk);
  ros::spinOnce();
  cr1_rate.sleep();

  mk_arr.markers.push_back(mk);
  cr_publisher.publish(mk_arr);
  ros::spinOnce();
  cr1_rate.sleep();
	

  //2.1 obstacles
	mk.ns = "obstacle";
  mk.id = 0;
  mk.type = visualization_msgs::Marker::CYLINDER;
  mk.action = visualization_msgs::Marker::ADD;
  printf("Number of obstacles!!!! is %d\n", parameter->M);
  for(int i = 0; i < parameter->M; i++)
	{
    mk.id++;
		float x = parameter->obstacle_pos[i][0];
    float y = parameter->obstacle_pos[i][1];
		float r = parameter->obstacle_pos[i][2];
    mk.header.stamp = ros::Time::now();
		mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
		mk.scale.x = r * 2;
    mk.scale.y = r * 2;
    mk.scale.z = 0.1;
    mk.color.r = 0;
    mk.color.g = 0;
    mk.color.b = 1;
    mk.color.a = 0.5;
    mk.lifetime = ros::Duration();
  	cr1_publisher.publish(mk);
		ros::spinOnce();
		cr1_rate.sleep();
  }
  //2.2 robots' initial poses
  mk.ns = "robot_initial_pos";
  mk.id = 0;
  for(int i = 0; i < parameter->N; i++)
  {
    mk.id++;
    float x = parameter->robot_pos[i][0];
		float y = parameter->robot_pos[i][1];
		float r = parameter->robot_radius;
		mk.pose.position.x = x;
		mk.pose.position.y = y;
		mk.pose.position.z = 0;
	  mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
		mk.scale.x = r * 2;
		mk.scale.y = r * 2;
		mk.scale.z = 0.1;
		mk.color.r = 0;
		mk.color.g = 1;
		mk.color.b = 0;
		mk.color.a = 1;
		mk.lifetime = ros::Duration();
		cr1_publisher.publish(mk);
		ros::spinOnce();
		cr1_rate.sleep();
	}
  //2.3 destination
  mk.ns = "destination";
  mk.id = 0;
  float x = parameter->target_center[0];
  float y = parameter->target_center[1];
	float r = parameter->target_radius;
	mk.pose.position.x = x;
  mk.pose.position.y = y;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.scale.x = r * 2;
  mk.scale.y = r * 2;
  mk.scale.z = 0.1;
  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 0;
  mk.color.a = 0.3;
  mk.lifetime = ros::Duration();
  cr1_publisher.publish(mk);
  ros::spinOnce();
  cr1_rate.sleep();

  //2.4 map boundary
  mk.ns = "boundary";
  mk.id = 0;
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.scale.x = 1;
  mk.scale.y = 1;
  mk.scale.z = 1;
  mk.color.r = 0;
  mk.color.g = 0;
  mk.color.b = 0;
  mk.color.a = 1;
  mk.lifetime = ros::Duration();

  geometry_msgs::Point p1, p2, p3, p4;
  p1.x = - (parameter->mapsize + mk.scale.x / 2);
  p1.y = - (parameter->mapsize + mk.scale.y / 2);
  p2.x = - (parameter->mapsize + mk.scale.x / 2);
  p2.y = (parameter->mapsize + mk.scale.y / 2);
  p3.x = (parameter->mapsize + mk.scale.x / 2);
  p3.y = (parameter->mapsize + mk.scale.y / 2);
  p4.x = (parameter->mapsize + mk.scale.x / 2);
  p4.y = -(parameter->mapsize + mk.scale.y / 2);

  mk.points.push_back(p1);
  mk.points.push_back(p2);
  mk.points.push_back(p3);
  mk.points.push_back(p4);
  mk.points.push_back(p1);
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;



  cr1_publisher.publish(mk);
  ros::spinOnce();
  cr1_rate.sleep();  

    



}

void handleEot(const custom_messages::R2C::ConstPtr& msg)
{
  int is_aided = 1;
  std::vector<float> time_array = msg->time_array;
  std::vector<int> sequence_array;
  std::vector<uint8_t> is_fixed;
  ROS_INFO("going into parsemap");
  ROS_INFO("came back from parsemap");
	printf("N = %d, M = %d\n", parameter->N, parameter->M);

  PARAM local_parameter;
  memcpy(&local_parameter, parameter, sizeof(PARAM));
  //3. Call the kernel code
  ROS_INFO("Right before test main parameter->N = %d, parameter->M = %d", parameter->N, parameter->M);
  return_struct = testmain(&local_parameter, is_aided, time_array, sequence_array, is_fixed);  


  custom_messages::C2R c2r;  

  for(int i = 0; i < return_struct.sequence_end_indices.size(); i++)
  {
    printf("sequence_end_indices for [%d] is %d\n", i,
      return_struct.sequence_end_indices[i]);
  }

  if(return_struct.is_valid_path == 0)
  	c2r.cost_of_path = 100000; //very high number
  else
    c2r.cost_of_path = return_struct.cost_of_path;
  c2r.is_valid_path = return_struct.is_valid_path;
  c2r.is_complete = return_struct.is_complete;
  c2r.sequence_string_array = return_struct.sequence_string_array;
  c2r.eot = 1; 
  c2r.map_number = map_sequence[MAP_NUM];
  ci_publisher.publish(c2r);
	ros::spinOnce();
 
	return;

}


void callBack(const custom_messages::R2C::ConstPtr& msg)
{ 
  //TODO:
  //1. Retrieve the data in R2C
  //2. Use that data and run testmain
  //3. Package the result from testmain in C2R
  //4. publish that data

  //5. FIRST, I will just see if message sending and recieving works. 
  
  
  ROS_INFO("backend_C_node callback called...\n");
  //1. Error check the given message
	int err = errorCheckR2C(msg);
  if(err) return;


	//1.5 (Newly added feature!!)
  if(msg->next == 1) {
		updateMap();
		return;
	}


  if(msg->eot == 1){
		handleEot(msg);
    return;
  }
 
  //2. Parse the given message
  int is_aided = (int) msg->is_aided;
  std::vector<float> time_array = msg->time_array;
  std::vector<int> sequence_array = msg->sequence_int_array;
  std::vector<uint8_t> is_fixed = msg->is_fixed;
  ROS_INFO("going into parsemap");
  ROS_INFO("came back from parsemap");
	printf("N = %d, M = %d\n", parameter->N, parameter->M);

  PARAM local_parameter;
  memcpy(&local_parameter, parameter, sizeof(PARAM));
  //3. Call the kernel code
  ROS_INFO("Right before test main parameter->N = %d, parameter->M = %d", parameter->N, parameter->M);
  return_struct = testmain(&local_parameter, is_aided, time_array, sequence_array, is_fixed);  

  ROS_INFO("After testmain parameter->N = %d, parameter->M = %d", parameter->N, parameter->M);
  //4. Publish a C2R reply
  publishC2R();
  ROS_INFO("Finished publishing C2R");
  ROS_INFO("AFter publishC2R, ->N = %d, ->M = %d", parameter->N, parameter->M);
  //5. Publish a visualization message
  publishMarkerArray();
	ROS_INFO("Finished publishing MarkerArray");
  ROS_INFO("AFter publishMarkerArray, ->N = %d, ->M = %d", parameter->N, parameter->M);
 
  //6. Publish a c2d message
  return;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "backend_C");
  ros::NodeHandle n;
  ROS_INFO("Starting backend_C node..\n");
  ci_publisher = n.advertise<custom_messages::C2R>("/hsi/C2R", 1000);
  cr_publisher = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	
	cr1_publisher = n.advertise<visualization_msgs::Marker>("map_related", 1);
	generateRandomMapSequence();

  parameter = new PARAM[1];
	updateMap();

  ros::Subscriber c_subscriber = n.subscribe<custom_messages::R2C>("/hsi/R2C", 1000, callBack);

  ros::spin();
  return 0;
}
