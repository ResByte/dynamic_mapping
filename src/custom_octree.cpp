/*
 * custom_octree.cxx
 * 
 * Copyright 2015 Shibata-Lab <shibata-lab@shibatalab-X500H>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

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
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ctime>
using namespace octomap;
const float res = 0.1;
ros::Publisher octomap_pub ;

// Defining nodes for the octree, derives OcTreeNode
class MarkovOctreeNode : public OcTreeNode{
public:
  MarkovOctreeNode() :OcTreeNode() {
    count =0;
    prev_state = 0;
    prev_prob =(double)0.0;
  }
  //MarkovOctree(const MarkovOctree& rhs) : OcTreeNode(rhs){} 
  //children
  inline MarkovOctreeNode* getChild(unsigned int i ){
    return static_cast<MarkovOctreeNode*>(OcTreeNode::getChild(i));
  }
  inline const MarkovOctreeNode* getChild(unsigned int i ) const{
    return static_cast<const MarkovOctreeNode*>(OcTreeNode::getChild(i));
  }
  
  bool createChild(unsigned int i ){
    if(children == NULL ) allocChildren();
    children[i] = new MarkovOctreeNode();
    return true;
  }
  
  
  //return the values
  inline int getDuration() const { return count;}
  inline int getPrevState() const {return prev_state;}
  inline double prevProb() const{return prev_prob;}
  inline double currProb() const{return this->getOccupancy();}
  // modify the values
  inline void updateDuration(){this->count++;}// increment duration
  inline void resetDuration(){this->count = 1;} // reset the duration to 1 observation
  inline void setPrevState(int prev_state_val){this->prev_state = prev_state_val;} // set prev state to desired value
  inline void setPrevProb(double prev_prob_val){this->prev_prob = prev_prob_val;} // set previous prob values
  //update occupancy and timestamps of the inner nodes
  
  /*
    inline void updateOccupancyChildren(){
    setPrevProb(this->getOccupancy());
    setPrevState(1);
    this->setLogOdds(this->getMaxChildLogOdds());
    // TODO: add other updates
    }*/
protected:
  
  int count; // the number of observations in current state
  int prev_state; // previous state
  double prev_prob; /// previous occupancy prob
  
};


//tree definition
class DynamicGrid : public OccupancyOcTreeBase <MarkovOctreeNode> {  
public:
  // Default constructor to set the resolution
  DynamicGrid(double resolution) : OccupancyOcTreeBase<MarkovOctreeNode>(resolution){};
  // virtual constructor
  DynamicGrid* create() const {return new DynamicGrid(resolution);}
  std::string getTreeType() const{return "Dynamic Occupancy Grid";}
  
  // return timestamp of last update
  //virtual void updateNodeLogOdds(MarkovOctreeNode* node, const float& update) const;
  DynamicGrid operator =(const DynamicGrid& rhs) const{
    return rhs;
  }
};

std::deque< DynamicGrid> cloud_seq_loaded; 

// This function comes from the octomap_server pkg
std_msgs::ColorRGBA getColorByHeight(double h) {
  double range = 5.05;
  h = 1.0 - std::min(std::max(h/range, 0.0), 1.0);
  h *= 0.8;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)
  
  double s = 1.0;
  double v = 1.0;
  
  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;
  
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);
  
  switch (i) {
  case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
  case 1:
    color.r = n; color.g = v; color.b = m;
    break;
  case 2:
    color.r = m; color.g = v; color.b = n;
    break;
  case 3:
    color.r = m; color.g = n; color.b = v;
    break;
  case 4:
      color.r = n; color.g = m; color.b = v;
      break;
  case 5:
    color.r = v; color.g = m; color.b = n;
    break;
  default:
    color.r = 1; color.g = 0.5; color.b = 0.5;
    break;
  }
  
  return color;
}


void publishMapAsMarkers(DynamicGrid* octree_msg) {
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(octree_msg->getTreeDepth()+1);
  DynamicGrid::leaf_iterator it = octree_msg->begin_leafs();
  DynamicGrid::leaf_iterator end = octree_msg->end_leafs();
  // For leaf in leaves
  for (; it != end; ++it) {
    // If occupied
    if (octree_msg->isNodeOccupied(*it)) {
      // Get some info about the leaf
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      //std::cout<<x<<" "<<y<<" "<<z<<std::endl;
      
      size_t depth = it.getDepth();
      // Insert a point for the leaf's cube
      geometry_msgs::Point leaf_origin;
      leaf_origin.x = x;
      leaf_origin.y = y;
      leaf_origin.z = z;
      msg.markers[depth].points.push_back(leaf_origin);
      // Determine and set the leaf's color by height
      //std::cout<<"determining color by height"<<std::endl;
      msg.markers[depth].colors.push_back(getColorByHeight(leaf_origin.z));
      
    }
  }
  std::cout<<" Finish the marker array setup"<<std::endl;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  for (size_t i = 0; i < msg.markers.size(); ++i) {
    double size = octree_msg->getNodeSize(i);
    msg.markers[i].header.frame_id = "/odom";
    msg.markers[i].header.stamp = ros::Time::now();
    msg.markers[i].ns = "map";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    msg.markers[i].scale.x = size;
    msg.markers[i].scale.y = size;
    msg.markers[i].scale.z = size;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].color = color;
  }
  std::cout<<"Publish the marker array"<<std::endl;
  
  octomap_pub.publish(msg) ;
}


void processCloud(const sensor_msgs::PointCloud2 msg){
  ros::Time start_time = ros::Time::now();
  //********* Retirive and process raw pointcloud************
  // std::cout<<"Recieved cloud"<<std::endl;
  //std::cout<<"Create Octomap"<<std::endl;
  DynamicGrid curr_tree(res);
  //std::cout<<"Load points "<<std::endl;
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(msg,cloud);
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  pcl::fromPCLPointCloud2(cloud,pcl_pc);
  //std::cout<<"Filter point clouds for NAN"<<std::endl;
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
  octomap::Pointcloud oct_pc;
  octomap::point3d origin(0.0f,0.0f,0.0f);
  //std::cout<<"Adding point cloud to octomap"<<std::endl;
  //octomap::point3d origin(0.0f,0.0f,0.0f);
  for(int i = 0;i<pcl_pc.points.size();i++){
    oct_pc.push_back((float) pcl_pc.points[i].x,(float) pcl_pc.points[i].y,(float) pcl_pc.points[i].z);
  }
  //std::cout<< "inserting cloud to empty tree"<<std::endl;
  curr_tree.insertPointCloud(oct_pc,origin,-1,false,false);
  
  //std::cout<<"Size of dynamic grid : "<< curr_tree.size()<< std::endl;
  //*********** Remove the oldest data, update the data***************	
  cloud_seq_loaded.push_back(curr_tree);
  //std::cout<<"Size of cloud seq : "<<cloud_seq_loaded.size()<<std::endl;
  if(cloud_seq_loaded.size()>2){
    cloud_seq_loaded.pop_front();  
  }
  
  //publish the octomap 
  
  //std::cout<<"publishing octomap"<<std::endl;
  publishMapAsMarkers(&curr_tree);
  //std::cout<<"published"<<std::endl;  
  //*********** Process currently observerd and buffered data*********
  
  if(cloud_seq_loaded.size()==2){
    //std::cout<< "Generating octomap"<<std::endl;
    DynamicGrid prev_tree(res);
    prev_tree = cloud_seq_loaded.front();
    
    //************** Publish the read octomap with nodes and their sizes***********
    int count = 0;
    int n_count = 0;
    for(DynamicGrid::tree_iterator it = curr_tree.begin_tree(),end=curr_tree.end_tree(); it!= end; ++it)
      {	
	octomap::point3d curr_coord = it.getCoordinate();
	//std::cout << curr_coord<< std::endl;
	MarkovOctreeNode* prev_node = prev_tree.search(curr_coord);// search for the same node in previous 
	MarkovOctreeNode* curr_node = &*it;
        //std::cout<<it.isNodeOccupied()<<std::endl;
	
	//**********If the result is matched then perform the process*********
	if (prev_node!= NULL && curr_node!=NULL )
	  {
	    //std::cout<<std::endl;
	    //std::cout<< prev_node->getOccupancy()<<std::endl;
	    //std::cout<< curr_node->getOccupancy()<<std::endl;
	    //std::cout<< it->getValue()<<std::endl;
	    ros::NodeHandle k;
	    ros::Rate rate(2);
	    ros::ServiceClient client = k.serviceClient<dynamic_mapping::hmm_srv>("hmm_data");
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
      }
    //tree.writeBinary("simple_tree.bt");
    
    std::cout<<"finished"<<std::endl;
    std::cout<<std::endl;
  }
  ros::Duration delta_t = ros::Time::now() - start_time;
  std::cout<< "Time for update :"<< delta_t<<std::endl;
  
  
  // If want to save octomap,uncomment following
  //curr_tree.writeBinary("sampleTree.bt");
  
}

int main(int argc, char **argv){	
  std::cout<<std::endl;
  std::cout<<"initializing ROS node"<<std::endl;
  ros::init(argc,argv, "custom_octree");
  ros::NodeHandle n;
  uint32_t queue_size = 1;
  
  octomap_pub = n.advertise<visualization_msgs::MarkerArray>("/map_vis", 1);
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
  ros::spin(); 
  return 0;
}
