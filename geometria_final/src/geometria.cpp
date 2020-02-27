#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "klibrary.h"

class cloudHandler
{
  public:
    //constructor
    cloudHandler()
    {
      //subscriptores y publicadores para pcl
      pclRGB_sub=nh.subscribe("/kinect2/hd/points",1,&cloudHandler::cloudRGBCB,this);
      //publicadores XYZRGB
      pclRGB_pub=nh.advertise<sensor_msgs::PointCloud2>("pclRGB_recortada",1);
      pclRGB_pub2=nh.advertise<sensor_msgs::PointCloud2>("pclRGB_plano",1);
      pclRGB_pub3=nh.advertise<sensor_msgs::PointCloud2>("pclRGB_objetos",1);
      pclRGB_pub4=nh.advertise<sensor_msgs::PointCloud2>("pclRGB_proyeccion",1);
      pclRGB_pub5=nh.advertise<sensor_msgs::PointCloud2>("pclRGB_colorSeg",1);
      imageCloud=nh.advertise<sensor_msgs::Image>("imageCloud",1);
    }

    void cloudRGBCB(const sensor_msgs::PointCloud2 &input)
    {
      //creo una nube de tipo pcl
      pcl::PointCloud<pcl::PointXYZRGB> cloud; //cloud
      pcl::PointCloud<pcl::PointXYZRGB> plano;
      pcl::PointCloud<pcl::PointXYZRGB> objetos;
      //publicadores
      sensor_msgs::PointCloud2 cut,plane,obj;
      cut.header=input.header;
      plane.header=input.header;
      obj.header=input.header;

      //de la nube de ros a pcl
      pcl::fromROSMsg(input,cloud);

      std::cout<<"COMIENZA"<<std::endl;
      cloud=tlib::limpiarNube(cloud);
      std::cout<<"PROCESAR EL PLANO"<<std::endl;
      plano=klib::segmentacion(cloud,1);
      std::cout<<"PROCESAR LOS OBJETOS"<<std::endl;
      objetos=klib::segmentacion(cloud,2);

      //transformo de pcl a ros y publico
      pcl::toROSMsg(cloud,cut);
      pclRGB_pub.publish(cut);
      pcl::toROSMsg(plano,plane);
      pclRGB_pub2.publish(plane);
      pcl::toROSMsg(objetos,obj);
      pclRGB_pub3.publish(obj);
    }

  protected:
    ros::NodeHandle nh;
    //subscriptor a pcl
    ros::Subscriber pclRGB_sub;
    //publicadores pcl XYZRGB
    ros::Publisher pclRGB_pub,pclRGB_pub2,pclRGB_pub3,pclRGB_pub4,pclRGB_pub5,imageCloud;
};

class imageHandler
{
  public:
    //constructor
    imageHandler()
    {
      //subscriptores y publicadores para imagen RGB
      img_sub=nh.subscribe("/kinect2/hd/image_color_rect",1,&imageHandler::imageCB,this);
    }

    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
      //convert from ROS to OpenCV
      cv_bridge::CvImagePtr cv_ptr;

      try
      {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge Exception: %s",e.what());
      }
      std::cout<<"guardando imagen RGB"<<std::endl;
      cv::imwrite("imagenCasa.png",cv_ptr->image);
    }

  protected:
    ros::NodeHandle nh;
    //subscriptor a pcl
    ros::Subscriber img_sub;
};

main(int argc, char** argv)
{
	ros::init(argc,argv,"geometria");
	cloudHandler handler;
  //imageHandler handler2;
	ros::spin();
	return 0;
}
