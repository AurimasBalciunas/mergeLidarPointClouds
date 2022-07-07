#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>
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
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


float left_shift[3] = {1.549, 0.267, 0.543};
float right_shift[3] = {1.549, -0.267, 0.543};
float front_shift[3] = {2.242, 0, 0.448};
float cog_shift[3] = {-1.3206, 0.030188, -0.23598};
Eigen::Matrix4d left_to_ram;
Eigen::Matrix4d right_to_ram;
Eigen::Matrix4d front_to_ram;
Eigen::Matrix4d ram_to_cog;
Eigen::Matrix4d left_global;
Eigen::Matrix4d right_global;
Eigen::Matrix4d front_global;

void pcd2bin (std::string &in_file, std::string &out_file)
{ 
   //Create a PointCloud value
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  //Open the PCD file
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (in_file, *cloud) == -1) 
  {
    PCL_ERROR ("Couldn't read in_file\n");
  }
  //Create & write .bin file
  std::ofstream bin_file(out_file.c_str(),std::ios::out|std::ios::binary|std::ios::app);
  if(!bin_file.good()) std::cout<<"Couldn't open "<<out_file<<std::endl;  

  //PCD 2 BIN 
  std::cout << "Converting "
            << in_file <<"  to  "<< out_file
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
  	bin_file.write((char*)&cloud->points[i].x,1*sizeof(float));
    bin_file.write((char*)&cloud->points[i].y,1*sizeof(float));
  	bin_file.write((char*)&cloud->points[i].z,1*sizeof(float));
    bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
  }
  	
  bin_file.close();
}

void saveXYZI(Eigen::MatrixXd& combined,auto stamp, std::string timestamp)
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
    std::string pcdFilePath = "./merged_pcds/" + timestamp + "_combined.pcd";
    pcl::io::savePCDFile(pcdFilePath, msg);
    
    std::string binFilePath = "./merged_bins/" + timestamp + "_combined.bin";
    pcd2bin(pcdFilePath, binFilePath);
}

float unpackFloat(const std::vector<unsigned char>& buf, int i) {
    uint32_t temp = 0;
    temp = ((buf[i+0]) |
            (buf[i+1] << 8) |
            (buf[i+2] << 16) |
            buf[i+3] << 24);
    return *((float *) &temp);
}

void msg_to_pointcloud(const sensor_msgs::msg::PointCloud2 Msg, Eigen::MatrixXd& Point){

    int size = Msg.width;
    int offset = 0;
    int pointstep = Msg.point_step;
    
    for (int j = 0; j < size; j++){

      Point(j,0) = unpackFloat(Msg.data,offset+0);
      Point(j,1) = unpackFloat(Msg.data,offset+4);
      Point(j,2) = unpackFloat(Msg.data,offset+8);
      Point(j,3) = unpackFloat(Msg.data,offset+12);
      offset += pointstep;
    }
}

void generate_transform(Eigen::Matrix4d& tranform, float vector[],float angle){

    tranform << cos(angle),-sin(angle),0,vector[0],
                sin(angle),cos(angle), 0,vector[1],
                0,0,1,vector[2],
                0,0,0,1;    
}

void combine_clouds(sensor_msgs::msg::PointCloud2 leftMsg, sensor_msgs::msg::PointCloud2 frontMsg, sensor_msgs::msg::PointCloud2 rightMsg, std::string timeStamp)
{

    generate_transform(left_to_ram,left_shift,2*M_PI/3);
    generate_transform(right_to_ram,right_shift,-2*M_PI/3);
    generate_transform(front_to_ram,front_shift,0);
    generate_transform(ram_to_cog,cog_shift,0);

    left_global = ram_to_cog * left_to_ram;
    right_global = ram_to_cog * right_to_ram;
    front_global = ram_to_cog * front_to_ram;

    int left_size = leftMsg.width;
    int right_size = rightMsg.width;
    int front_size = frontMsg.width;

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
    saveXYZI(combined,frontMsg.header.stamp, timeStamp);
}

inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main()
{    
    sensor_msgs::msg::PointCloud2 leftSensorPC2;
    sensor_msgs::msg::PointCloud2 frontSensorPC2;
    sensor_msgs::msg::PointCloud2 rightSensorPC2;

    //Looping through all files in the directory and merging them
    for (const auto & file : std::filesystem::directory_iterator("./split_pcds"))
    {
        std::string filePath = file.path();
        if(ends_with(filePath,"_front.pcd"))
        {
            std::string timeStamp= filePath.substr(13,19);

            std::string frontFilePath = "./split_pcds/" + timeStamp + "_front.pcd";
            std::string leftFilePath = "./split_pcds/" + timeStamp + "_left.pcd";
            std::string rightFilePath = "./split_pcds/" + timeStamp + "_right.pcd";

            if(!std::filesystem::exists(leftFilePath) || !std::filesystem::exists(rightFilePath))
                throw std::runtime_error(".pcd incorrectly matched up for timestamp " + timeStamp + ". Check to make sure python script ran correctly.");
            pcl::io::loadPCDFile(leftFilePath, leftSensorPC2);
            pcl::io::loadPCDFile(frontFilePath, frontSensorPC2);
            pcl::io::loadPCDFile(rightFilePath, rightSensorPC2);
            combine_clouds(leftSensorPC2, frontSensorPC2, rightSensorPC2, timeStamp);
        }
    }

    return 0;

}