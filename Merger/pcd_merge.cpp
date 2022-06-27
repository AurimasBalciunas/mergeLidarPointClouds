#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


/*
void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& leftMsg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& rightMsg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& frontMsg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: ");
    int left_size = leftMsg -> width;
    int right_size = rightMsg -> width;
    int front_size = frontMsg -> width;

    Eigen::MatrixXd LeftPoint(left_size,4);
    Eigen::MatrixXd RightPoint(right_size,4);
    Eigen::MatrixXd FrontPoint(front_size,4);
    Eigen::VectorXd Intensity(left_size + right_size + front_size);
    Eigen::VectorXd left = Eigen::VectorXd::Constant(left_size,1);
    Eigen::VectorXd right = Eigen::VectorXd::Constant(right_size,1);
    Eigen::VectorXd front = Eigen::VectorXd::Constant(front_size,1);
    Eigen::MatrixXd combined(left_size + right_size + front_size,4);

    msg_to_pointcloud(leftMsg,LeftPoint);
    msg_to_pointcloud(rightMsg,RightPoint);
    msg_to_pointcloud(frontMsg,FrontPoint);

    Intensity << LeftPoint.col(3), RightPoint.col(3),FrontPoint.col(3);
    
    LeftPoint.block(0,3,left_size,1) = left;
    RightPoint.block(0,3,right_size,1) = right;
    FrontPoint.block(0,3,front_size,1) = front;

    LeftPoint = LeftPoint * left_global.transpose();
    RightPoint = RightPoint * right_global.transpose();
    FrontPoint = FrontPoint * front_global.transpose();


    combined << LeftPoint, FrontPoint, RightPoint;
    combined.block(0,3,combined.rows(),1) = Intensity; 
    saveXYZI(combined,frontMsg->header.stamp);
}

void saveXYZI(Eigen::MatrixXd& combined,auto stamp)
{
    uint32_t point_step = 16;
    size_t data_size = combined.rows();
    
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = stamp;
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.data.resize(std::max((size_t)1, (size_t)data_size) * point_step);
    msg.point_step = point_step;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = data_size;
    msg.is_bigendian = false;
    msg.is_dense = false;
    uint8_t *ptr = msg.data.data();
    
    for (size_t i = 0; i < data_size; i++)
    {
        *(reinterpret_cast<float*>(ptr + 0)) = combined(i,0);
        *(reinterpret_cast<float*>(ptr + 4)) = combined(i,1);
        *(reinterpret_cast<float*>(ptr + 8)) = combined(i,2);
        *(reinterpret_cast<float*>(ptr + 12)) = combined(i,3);
        ptr += point_step;
    }
    
    //publisher_->publish(msg);
    //TODO: save to bin here instead of publishing
}
*/


//Simple code for reading in .pcds and merging them into one .pcd
/*
int main()
{
	//Load in all three files
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLeft (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFront (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRight (new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../../split_results/1632410999719558958_left.pcd", *cloudLeft) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 1632410999719558958_left.pcd \n");
	     return (-1);
	}
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../../split_results/1632410999719558958_front.pcd", *cloudFront) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 1632410999719558958_front.pcd \n");
	     return (-1);
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../../split_results/1632410999719558958_right.pcd", *cloudRight) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 1632410999719558958_right.pcd \n");
	     return (-1);
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined(new pcl::PointCloud<pcl::PointXYZ>);
	*cloudCombined = (*cloudLeft) + (*cloudFront) + (*cloudRight);
	pcl::io::savePCDFile<pcl::PointXYZ>("combined.pcd", *cloudCombined); 
}
*/



//Another try: read in from a .bin and do the combination the way that adam did it
/*
#include <iostream>
#include <fstream>
#include <cmath>
#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <sys/types.h>
#include <typeinfo>
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

int main()
{
  std::ifstream file("../../split_results/1632410999719558958_front.bin", std::ios::in | std::ios::binary);
  if (!file) return EXIT_FAILURE;

  float item;
  while (file.read((char*)&item, 4))
  {
    std::cout << "[" << item;
    if (std::round(item) == item) std::cout << ".";
    std::cout  << "]\n";
  }

  //std::cout << item;
}
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <sys/types.h>
#include <typeinfo>
#include <pcl/conversions.h>
//#include "sensor_msgs/msg/point_cloud2.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


void readBin(std::string timestamp, std::string direction, pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{
	std::string infile = "../../split_results/" + timestamp + "_" + direction;
	//load point cloud
	std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
	if(!input.good())
	{
		std::cerr << "Could not read file: " << infile << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);


	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();

	std::cout << "Read KTTI point cloud with " << i << " points" << std::endl;
}


int main()
{	
	pcl::PointCloud<pcl::PointXYZI>::Ptr leftPoints (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr frontPoints (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr rightPoints (new pcl::PointCloud<pcl::PointXYZI>);
	readBin("1632410999719558958", "left", leftPoints);
	readBin("1632410999719558958", "front", frontPoints);
	readBin("1632410999719558958", "right", rightPoints);

	//sensor_msgs::msg::PointCloud2::ConstSharedPtr& leftMsg;
	pcl::PCLPointCloud2 leftPC2;
	pcl::toPCLPointCloud2(*leftPoints, leftPC2);

	//const sensor_msgs::msg::PointCloud2::ConstSharedPtr& leftMsg;
	//leftMsg = leftPC2;
	//use toROSMsg here

    /*pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    */



}