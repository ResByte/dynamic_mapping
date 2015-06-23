#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <deque>
#include <dynamic_mapping/Num.h>
#include <dynamic_mapping/hmm_srv.h>

const float res = 0.1;

std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded; 






void processCloud(const sensor_msgs::PointCloud2 msg)
{
	//********* Retirive and process raw pointcloud************
	std::cout<<"Recieved cloud"<<std::endl;
	std::cout<<"Create Octomap"<<std::endl;
	octomap::OcTree tree(res);
	std::cout<<"Load points "<<std::endl;
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(msg,cloud);
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	std::cout<<"Filter point clouds for NAN"<<std::endl;
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	octomap::Pointcloud oct_pc;
	octomap::point3d origin(0.0f,0.0f,0.0f);
	std::cout<<"Adding point cloud to octomap"<<std::endl;
	//octomap::point3d origin(0.0f,0.0f,0.0f);
	for(int i = 0;i<pcl_pc.points.size();i++){
		oct_pc.push_back((float) pcl_pc.points[i].x,(float) pcl_pc.points[i].y,(float) pcl_pc.points[i].z);
}
	tree.insertPointCloud(oct_pc,origin,-1,false,false);

	
	//*********** Remove the oldest data, update the data***************	
	cloud_seq_loaded.push_back(pcl_pc);
	std::cout<<cloud_seq_loaded.size()<<std::endl;
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
	
	}
	
	//*********** Process currently observerd and buffered data*********

	if(cloud_seq_loaded.size()==2){
		std::cout<< "Generating octomap"<<std::endl;
		for(int i = 0;i<pcl_pc.points.size();i++)
		{
			oct_pc.push_back((float) pcl_pc.points[i].x,(float) pcl_pc.points[i].y,(float) pcl_pc.points[i].z);			
		}
		octomap::Pointcloud prev_oct_pc;
		/*
			for(std::deque<pcl::PointCloud<pcl::PointXYZ> >::iterator it = cloud_seq_loaded.begin();it!=cloud_seq_loaded.end()-1;++it)
		
			std::cout<<"iterating"<<std::endl;	
			
		}*/
		pcl::PointCloud<pcl::PointXYZ> prev_pc = cloud_seq_loaded.front();
		for(int i =0;i<prev_pc.points.size();i++)
		{
			prev_oct_pc.push_back((float) prev_pc.points[i].x,(float) prev_pc.points[i].y,(float) prev_pc.points[i].z);
		}
		octomap::OcTree prev_tree(res);
		prev_tree.insertPointCloud(prev_oct_pc,origin,-1,false,false);
		
		//************** Publish the read octomap with nodes and their sizes***********
		int count = 0;
		int n_count = 0;
		for(octomap::OcTree::tree_iterator it = tree.begin_tree(),end=tree.end_tree(); it!= end; ++it)
		{	
		
			octomap::point3d curr_coord = it.getCoordinate();
			octomap::OcTreeNode* prev_node = prev_tree.search(curr_coord);
			octomap::OcTreeNode* curr_node = tree.search(curr_coord);
			
			//**********If the result is matched then perform the process*********
			if (prev_node!= NULL && curr_node!=NULL )
			{
				std::cout<<std::endl;
				std::cout<< prev_node->getOccupancy()<<std::endl;
				std::cout<< curr_node->getOccupancy()<<std::endl;
				//std::cout<< it->getValue()<<std::endl;
				ros::NodeHandle k;
				ros::Rate rate(2);
				//ros::Publisher pub = k.advertise<octomap_process::Num>("Hmm_data",1);
				ros::ServiceClient client = k.serviceClient<dynamic_mapping::hmm_srv>("hmm_data");
				//octomap_process::Num message;
				//message.num1 = curr_node->getOccupancy();
				//message.num2 = prev_node->getOccupancy();
				//pub.publish(message);
				//rate.sleep();
				dynamic_mapping::hmm_srv srv;
				srv.request.a = curr_node->getOccupancy();
				srv.request.b = prev_node->getOccupancy();
				if (client.call(srv))
				{
					std::cout<<srv.response.c<<std::endl;
				}
				else
				{
					ROS_ERROR("Failed to call service add_two_ints");
					
				}
				std::cout<<std::endl;
				count++;
			}else{
				n_count++;
				}	
			//std::cout<< curr_coord<<std::endl;
				
			//std::cout << "Node center: " << it.getCoordinate() << std::endl;
			//std::cout << "Node size: " << it.getSize() << std::endl;
			//std::cout << "Node value: " << it->getValue() << std::endl;
				

		}
		//********** print out the statistics ******************
		std::cout<<count<<std::endl;	
		std::cout<<n_count<<std::endl;	
				
	//**********If there are sufficient number of observations process them.********
	

	//tree.writeBinary("simple_tree.bt");
		
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;
	}
}

int main(int argc, char **argv){	
	std::cout<<std::endl;
	std::cout<<"initializing ROS node"<<std::endl;
	ros::init(argc,argv, "readCloud");
	ros::NodeHandle n;
	std::cout<<"subscribing to topic point_cloud"<<std::endl;
	//std::string topic =n.resolveName("point_cloud");
	uint32_t queue_size = 1;
	
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	//std::cout<<"Publishing OCtree as a msg"<<std::endl;
	//ros::Publisher oct_pub = n.advertise<octomap_msgs::Octomap>("oct_msg",1);
	//oct_pub.publish(map_msg);
	
	//if there are sufficient number of observations publish them.
 	
	ros::spin(); 
	return 0;
}
const float res = 0.1;

std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded; 
