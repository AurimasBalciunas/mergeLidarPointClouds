#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
    //cout<< 	cloud->points[i]<<endl;
  }
  	
  bin_file.close();
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
    std::string pcdFilePath = "test_pcd_trynew.pcd";
    pcl::io::savePCDFile(pcdFilePath, msg);
    std::cout << "made the pcd file" << std::endl;
    
    //pcl::PointCloud<pcl::PointXYZI> pcler;
    //pcl::moveFromROSMsg (msg, pcler);
    //std::string pcdFilePath = "test_pcd.pcd";
    //pcl::io::savePCDFileASCII(pcdFilePath, pcler);
    
    
    std::string binFilePath = "finalfinal.bin";
    pcd2bin(pcdFilePath, binFilePath);
    

}

void readBin(std::string timestamp, std::string direction, pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{
    std::string infile = "../split_results/" + timestamp + "_" + direction + ".bin";
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

void cloud_callback(sensor_msgs::msg::PointCloud2 leftMsg, sensor_msgs::msg::PointCloud2 frontMsg, sensor_msgs::msg::PointCloud2 rightMsg)
{

    generate_transform(left_to_ram,left_shift,2*M_PI/3);
    generate_transform(right_to_ram,right_shift,-2*M_PI/3);
    generate_transform(front_to_ram,front_shift,0);
    generate_transform(ram_to_cog,cog_shift,0);

    left_global = ram_to_cog * left_to_ram;
    right_global = ram_to_cog * right_to_ram;
    front_global = ram_to_cog * front_to_ram;

    //RCLCPP_INFO(this->get_logger(), "I heard: ");
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
    saveXYZI(combined,frontMsg.header.stamp);


    
}

int main()
{    
    sensor_msgs::msg::PointCloud2 leftSensorPC2;
    sensor_msgs::msg::PointCloud2 frontSensorPC2;
    sensor_msgs::msg::PointCloud2 rightSensorPC2;
    pcl::io::loadPCDFile("../split_results/1632410999719558958_left.pcd", leftSensorPC2);
    pcl::io::loadPCDFile("../split_results/1632410999719558958_front.pcd", frontSensorPC2);
    pcl::io::loadPCDFile("../split_results/1632410999719558958_right.pcd", rightSensorPC2);

    cloud_callback(leftSensorPC2, frontSensorPC2, rightSensorPC2);
    

    //int leftSize = leftSensorPC2.width;
    //Eigen::MatrixXd LeftPoint(leftSize,4);
    //Eigen::VectorXd left = Eigen::VectorXd::Constant(leftSize,1);
    //msg_to_pointcloud(leftSensorPC2,LeftPoint);

}