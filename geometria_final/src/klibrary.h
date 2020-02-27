/*este es un header con todas las funciones que hasta ahorita necesito, las programe
para que pudieran resolver ciertas cosas en espefico buscando que fueran lo mas
genericas posibles*/
#ifndef KLIBRARY_H_
#define KLIBRARY_H_

//standar libs
#include <iostream>
#include <utility>
#include <string>
//PCL libs
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <sensor_msgs/Image.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
//OPENCV libs
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace klib
{
  pcl::PointCloud<pcl::PointXYZ> segmentacion (pcl::PointCloud<pcl::PointXYZ> nube, int regreso)
  {
    /*esta funcion recibe una nube de puntos y segmenta un plano con SAC_RANSAC
    si regreso es 1 regresa el plano, si es 2 regresa los objetos*/
    pcl::PointCloud<pcl::PointXYZ> plano;
    pcl::PointCloud<pcl::PointXYZ> objetos;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //segmento el plano con ransac
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02);//antes 0.01
    //filtro el plano segmentado con ransac
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    //segmento el plano
    seg.setInputCloud(nube.makeShared());
    seg.segment(*inliers,*coefficients);
    ext.setInputCloud(nube.makeShared());
    ext.setIndices(inliers);
    ext.setNegative(false);
    ext.filter(plano); //solo el plano
    ext.setNegative(true);
    ext.filter(objetos); //sin el plano

    switch (regreso)
    {
      case 1:
        return plano;
        break;
      case 2:
        return objetos;
        break;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> segmentacion (pcl::PointCloud<pcl::PointXYZRGB> nube, int regreso)
  {
    /*esta funcion recibe una nube de puntos y segmenta un plano con SAC_RANSAC
    si regreso es 1 regresa el plano, si es 2 regresa los objetos*/
    pcl::PointCloud<pcl::PointXYZRGB> plano;
    pcl::PointCloud<pcl::PointXYZRGB> objetos;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //segmento el plano con ransac
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    //filtro el plano segmentado con ransac
    pcl::ExtractIndices<pcl::PointXYZRGB> ext;
    //segmento el plano
    seg.setInputCloud(nube.makeShared());
    seg.segment(*inliers,*coefficients);
    ext.setInputCloud(nube.makeShared());
    ext.setIndices(inliers);
    ext.setNegative(false);
    ext.filter(plano); //solo el plano
    ext.setNegative(true);
    ext.filter(objetos); //sin el plano

    switch (regreso)
    {
      case 1:
        return plano;
        break;
      case 2:
        return objetos;
        break;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> proyeccion (pcl::PointCloud<pcl::PointXYZ> nube, int regreso)
  {
    /*esta funcion hace la proyeccion a un plano de la nube que se le pase como parametro
    proyecta el plano en el piso si es 1 y para el plano al frente es 2*/
    pcl::PointCloud<pcl::PointXYZ> proyectada;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    // Create a set of planar coefficients with X=Y=0,Z=1
    coefficients->values.resize (4);
    switch (regreso)
    {
      case 1:
        coefficients->values[0] = 0;
        coefficients->values[1] = 1.0;
        coefficients->values[2] = 0;
        coefficients->values[3] = 0;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (nube.makeShared());
        proj.setModelCoefficients(coefficients);
        proj.filter(proyectada);
        return proyectada;
        break;
      case 2:
        coefficients->values[0] = 0;
        coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (nube.makeShared());
        proj.setModelCoefficients(coefficients);
        proj.filter(proyectada);
        return proyectada;
        break;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> proyeccion (pcl::PointCloud<pcl::PointXYZRGB> nube, int regreso)
  {
    /*esta funcion hace la proyeccion a un plano de la nube que se le pase como parametro
    proyecta el plano en el piso si es 1 y para el plano al frente es 2*/
    pcl::PointCloud<pcl::PointXYZRGB> proyectada;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    // Create a set of planar coefficients with X=Y=0,Z=1
    coefficients->values.resize (4);
    switch (regreso)
    {
      case 1:
        coefficients->values[0] = 0;
        coefficients->values[1] = 1.0;
        coefficients->values[2] = 0;
        coefficients->values[3] = 0;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (nube.makeShared());
        proj.setModelCoefficients(coefficients);
        proj.filter(proyectada);
        return proyectada;
        break;
      case 2:
        coefficients->values[0] = 0;
        coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (nube.makeShared());
        proj.setModelCoefficients(coefficients);
        proj.filter(proyectada);
        return proyectada;
        break;
    }
  }

  Eigen::Vector4f centroide (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*esta funcion toma una nube y calcula el centroide geometrico y regresa
    un punto en forma de un Eigen::Vector4f*/
    Eigen::Vector4f centroide;
    pcl::compute3DCentroid(nube,centroide);
    return centroide;
  }

  Eigen::Vector4f centroide (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*esta funcion toma una nube y calcula el centroide geometrico y regresa
    un punto en forma de un Eigen::Vector4f*/
    Eigen::Vector4f centroide;
    pcl::compute3DCentroid(nube,centroide);
    return centroide;
  }

  void guardarNube (pcl::PointCloud<pcl::PointXYZ> nube, std::string nombre)
  {
    try
    {
      nube.points.resize(nube.width*nube.height);
      pcl::io::savePCDFileASCII (nombre, nube);
      std::cerr<<"Saved "<<nube.points.size ()<<" data points to file "<<nombre<<std::endl;
    }
    catch(...)
    {
      std::cout <<"no se pudo guardar la nube "<<nombre<<" por alguna razon" << std::endl;
    }
  }

  void guardarNube (pcl::PointCloud<pcl::PointXYZRGB> nube, std::string nombre)
  {
    try
    {
      nube.points.resize(nube.width*nube.height);
      pcl::io::savePCDFileASCII (nombre, nube);
      std::cerr<<"Saved "<<nube.points.size ()<<" data points to file "<<nombre<<std::endl;
    }
    catch(...)
    {
      std::cout <<"no se pudo guardar la nube "<<nombre<<" por alguna razon" << std::endl;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> eliminarExternos (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*esta funcion recibe una nube y elminar lo que este atras del centroide, esperando
    que los objetos se encuentren delante de estos mismos*/

    //saco el centroide
    Eigen::Vector4f centroid;
    centroid=centroide(nube);

    //busco los puntos mas alejados del centroide
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto en es mas atras del centroide almaceno su index
      if(nube.points[i].z<centroid[2])
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> extraccion;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(extraccion);

    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZRGB> eliminarExternos (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*esta funcion recibe una nube y elminar lo que este atras del centroide, esperando
    que los objetos se encuentren delante de estos mismos*/

    //saco el centroide
    Eigen::Vector4f centroid;
    centroid=centroide(nube);

    //busco los puntos mas alejados del centroide
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto en es mas atras del centroide almaceno su index
      if(nube.points[i].z<centroid[2])
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB> extraccion;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(extraccion);

    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZRGB> nubeColorCentroide (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*Esta funcion toma una nube que no sea RGB y la transforma a RGB, deja la nube
    en color rojo y el centroide lo pone en azul*/
    Eigen::Vector4f centroid=centroide(nube); //saco el centroide
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    pcl::PointXYZRGB center=pcl::PointXYZRGB(0,255,255);
    pcl::PointXYZRGB points=pcl::PointXYZRGB(255,0,0);
    center.x=centroid[0];
    center.y=centroid[1];
    center.z=centroid[2];
    colored_cloud.points.push_back(center);
    for (size_t i=0;i<nube.points.size();i++)
    {
      points.x=nube.points[i].x;
      points.y=nube.points[i].y;
      points.z=nube.points[i].z;
      colored_cloud.points.push_back(points);
    }
    colored_cloud.width=colored_cloud.points.size();
    colored_cloud.height=nube.height;
    colored_cloud.is_dense=true;
    colored_cloud.header=nube.header;

    return colored_cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB> nubeColor (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*Esta funcion toma una nube que no sea RGB y la transforma a RGB, deja la nube
    en color azul */
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;;
    pcl::PointXYZRGB points=pcl::PointXYZRGB(0,0,255);

    for (size_t i=0;i<nube.points.size();i++)
    {
      points.x=nube.points[i].x;
      points.y=nube.points[i].y;
      points.z=nube.points[i].z;
      colored_cloud.points.push_back(points);
    }
    colored_cloud.width=colored_cloud.points.size();
    colored_cloud.height=nube.height;
    colored_cloud.is_dense=true;
    colored_cloud.header=nube.header;

    return colored_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> downsampling (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /* esta funcion toma una nube de puntos y le hace un downsampling para tener
    menos puntos, se ingresa una nube XYZ y regresa una nube XYZ*/
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ> voxelCloud;
    vg.setInputCloud(nube.makeShared());
    vg.setLeafSize(0.01f,0.01f,0.01f);
    vg.filter(voxelCloud);

    return voxelCloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB> downsampling (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /* esta funcion toma una nube de puntos y le hace un downsampling para tener
    menos puntos, se ingresa una nube XYZRGB y regresa una nube XYZRGB*/
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB> voxelCloud;
    vg.setInputCloud(nube.makeShared());
    vg.setLeafSize(0.01f,0.01f,0.01f);
    vg.filter(voxelCloud);

    return voxelCloud;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ> > nubesVector (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*esta funcion toma una nube de puntos, y hace una extraccion de clusters mediante
    una distancia euclidian, necesita una nube de puntos de tipo XYZ y regresa un
    vector de nubes de tipo XYZ*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(nube.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); //2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(nube.makeShared());
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > vector_nubes;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it)
    {
      for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();++pit)
      {
        cloud.points.push_back(nube.points[*pit]);
      }
      cloud.width=cloud.points.size();
      cloud.height=nube.height;
      cloud.is_dense=true;
      cloud.header=nube.header;
      vector_nubes.push_back(cloud);
      cloud.clear();
    }
    return vector_nubes;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB> > nubesVector (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*esta funcion toma una nube de puntos, y hace una extraccion de clusters mediante
    una distancia euclidian, necesita una nube de puntos de tipo XYZRGB y regresa un
    vector de nubes de tipo XYZRGB*/
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(nube.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02); //2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(nube.makeShared());
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > vector_nubes;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it)
    {
      for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();++pit)
      {
        cloud.points.push_back(nube.points[*pit]);
      }
      cloud.width=cloud.points.size();
      cloud.height=nube.height;
      cloud.is_dense=true;
      cloud.header=nube.header;
      vector_nubes.push_back(cloud);
      cloud.clear();
    }
    return vector_nubes;
  }

  pcl::PointCloud<pcl::PointXYZ> convexHull (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*Esta funcion saca el convex Hull de una nube, recibe una nube XYZ y regresa
    una nube XYZ*/
    pcl::PointCloud<pcl::PointXYZ> cloud_hull;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(nube.makeShared());
    chull.reconstruct(cloud_hull);

    return cloud_hull;
  }

  pcl::PointCloud<pcl::PointXYZRGB> convexHull (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*Esta funcion saca el convex Hull de una nube, recibe una nube XYZRGB y regresa
    una nube XYZRGB*/
    pcl::PointCloud<pcl::PointXYZRGB> cloud_hull;
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud(nube.makeShared());
    chull.reconstruct(cloud_hull);

    return cloud_hull;
  }

  std::pair<pcl::PointXYZ,pcl::PointXYZ> minMaxPoints (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*Esta funcion calcula el minimo y maximo valor en coordenadas de una nube,
    recibe de entrada una nube de puntos y regresa un par de puntos, el primero es
    el minimo y el segundo es el maximo*/
    pcl::PointXYZ min,max;
    pcl::getMinMax3D(nube,min,max);
    return std::make_pair(min,max);
  }

  std::pair<pcl::PointXYZRGB,pcl::PointXYZRGB> minMaxPoints (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*Esta funcion calcula el minimo y maximo valor en coordenadas de una nube,
    recibe de entrada una nube de puntos y regresa un par de puntos, el primero es
    el minimo y el segundo es el maximo*/
    pcl::PointXYZRGB min,max;
    pcl::getMinMax3D(nube,min,max);
    return std::make_pair(min,max);
  }

  /*sensor_msgs::Image pointCloud2Image (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    //Esta funcion toma una nube de puntos y la convierte en una imagen de tipo
    //sensor_msgs::Image, ingresa una nube PointXYZ y sale una imagen sensor_msgs::Image
    sensor_msgs::Image imagen;
    sensor_msgs::PointCloud2 cloud;
    try
    {
      pcl::toROSMsg(nube,cloud);
      pcl::toROSMsg(cloud,imagen);
      return imagen;
    }
    catch(std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "<<e.what());
    }
  }*/

  /*sensor_msgs::Image pointCloud2Image (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    //Esta funcion toma una nube de puntos y la convierte en una imagen de tipo
    //sensor_msgs::Image, ingresa una nube PointXYZRGB y sale una imagen sensor_msgs::Image
    sensor_msgs::Image imagen;
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id="kinect2_rgb_optical_frame";
    imagen.header.frame_id="kinect2_rgb_optical_frame";
    try
    {
      pcl::toROSMsg(nube,cloud);
      pcl::toROSMsg(cloud,imagen);
      return imagen;
    }
    catch(std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "<<e.what());
    }
  }*/

  pcl::PointCloud<pcl::PointXYZRGB> eliminarExtremos (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*esta funcion recibe una nube y elminar lo que este fuera de la zona central,
    recibe una nube XYZRGB y sale una nuebe XYZRGB*/

    //saco el par min max de puntos
    std::pair<pcl::PointXYZRGB,pcl::PointXYZRGB> puntos=minMaxPoints(nube);

    //busco los puntos mas alejados del centroide
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto en es mas atras del centroide almaceno su index
      if(nube.points[i].x>(((puntos.second.x)/2)+(((puntos.second.x)/2)/2)) || nube.points[i].x<(((puntos.first.x)*2)+(((puntos.second.x)*2)/2)))
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB> extraccion;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(extraccion);

    return extraccion;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB> > colorSegment (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree=boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(nube.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,1.0);
    pass.filter(*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(nube.makeShared());
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(600);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > vector_nubes;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it=clusters.begin();it!=clusters.end();++it)
    {
      for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();++pit)
      {
        cloud.points.push_back(nube.points[*pit]);
      }
      cloud.width=cloud.points.size();
      cloud.height=nube.height;
      cloud.is_dense=true;
      cloud.header=nube.header;
      vector_nubes.push_back(cloud);
      cloud.clear();
    }
    return vector_nubes;
  }

  //
} //namespace klib

namespace tlib
{
  pcl::PointCloud<pcl::PointXYZ> cargarNubeXYZ (std::string nombre)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (nombre, cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
      exit(0);
    }
    std::cout<<"Loaded "<<cloud.width*cloud.height<<" from file "<<nombre<<std::endl;
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cargarNubeRGB (std::string nombre)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (nombre, cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
      exit(0);
    }
    std::cout<<"Loaded "<<cloud.width*cloud.height<<" from file "<<nombre<<std::endl;
    return cloud;
  }

  void visualizador(pcl::PointCloud<pcl::PointXYZ> nube)
  {
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(nube.makeShared());
    while (!viewer.wasStopped ())
    {
    }
  }

  void visualizador(pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(nube.makeShared());
    while (!viewer.wasStopped ())
    {
    }
  }

  pcl::PointCloud<pcl::PointXYZ> limpiarNube (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    //busco los puntos mas alejados del centroide
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::cout<<"recortando nubeP"<<std::endl;
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto en es mas atras del centroide almaceno su index
      if(nube.points[i].x>0.28 || nube.points[i].x<-0.65 || nube.points[i].y>0.46 || nube.points[i].y<-0.02)
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> extraccion;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(extraccion);
    //SAC_RANSAC
    //std::cout<<"quitando plano"<<std::endl;
    //extraccion=klib::segmentacion(extraccion,2);
    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZRGB> limpiarNube (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    //busco los puntos mas alejados del centroide
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::cout<<"recortando nube"<<std::endl;
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto en es mas atras del centroide almaceno su index
      //if(nube.points[i].x>0.45 || nube.points[i].x<-0.90 || nube.points[i].y>0.60 || nube.points[i].y<-0.40)
      if(nube.points[i].x>0.10 || nube.points[i].x<-0.30 || nube.points[i].y>0.20 || nube.points[i].y<-0.30)
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB> extraccion;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(extraccion);
    //SAC_RANSAC
    //std::cout<<"quitando plano"<<std::endl;
    //extraccion=klib::segmentacion(extraccion,2);
    /*//remuevo ruido Z
    pcl::PointIndices::Ptr inliersZ(new pcl::PointIndices());
    std::pair<pcl::PointXYZRGB,pcl::PointXYZRGB> p=klib::minMaxPoints(nube);
    float umbral=(p.second.z-p.first.z)/2;
    for(size_t i=0;i<extraccion.points.size();i++)
    {
      //si mi punto esta mas lejos del umbral lo almaceno su indice
      if(extraccion.points[i].z>umbral)
      {
        inliersZ->indices.push_back(i);
      }
    }
    std::cout<<"limpiando Z"<<std::endl;
    extract.setInputCloud(extraccion.makeShared());
    extract.setIndices(inliersZ);
    extract.setNegative(true);
    extract.filter(extraccion);*/
    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZ> eliminarZ (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    for(size_t i=0;i<nube.points.size();i++)
    {
      nube.points[i].z=0.0;
    }
    return nube;
  }

  pcl::PointCloud<pcl::PointXYZRGB> eliminarZ (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    for(size_t i=0;i<nube.points.size();i++)
    {
      nube.points[i].z=0.0;
    }
    return nube;
  }

  pcl::PointCloud<pcl::PointXYZ> filtrarRuido (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*esta funcion elimina todos los puntos posteriores a un umbral, con el fin
    tratar de eliminar ruido en las nubes, recib una nube tipo XYZ y regresa una
    nube tipo XYZ*/

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> extraccion;
    //remuevo ruido Z
    pcl::PointIndices::Ptr inliersZ(new pcl::PointIndices());
    std::pair<pcl::PointXYZ,pcl::PointXYZ> p=klib::minMaxPoints(nube);
    float umbral=(p.second.z-p.first.z)/2;
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto esta mas lejos del umbral lo almaceno su indice
      if(nube.points[i].z>(p.second.z-0.10))
      {
        inliersZ->indices.push_back(i);
      }
    }
    std::cout<<"limpiando Z"<<std::endl;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliersZ);
    extract.setNegative(true);
    extract.filter(extraccion);
    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZ> RemoverPlano (pcl::PointCloud<pcl::PointXYZ> nube)
  {
    /*esta funcion elimina todos los puntos posteriores a un umbral, con el fin
    tratar de eliminar ruido en las nubes, recib una nube tipo XYZ y regresa una
    nube tipo XYZ*/

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> extraccion;
    //remuevo ruido Z
    pcl::PointIndices::Ptr inliersZ(new pcl::PointIndices());
    std::pair<pcl::PointXYZ,pcl::PointXYZ> p=klib::minMaxPoints(nube);
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto esta mas lejos del umbral lo almaceno su indice
      if((nube.points[i].z>(p.first.z-0.02))&&(nube.points[i].z<(p.second.z-0.02)))
      {
        inliersZ->indices.push_back(i);
      }
    }
    std::cout<<"limpiando Z"<<std::endl;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliersZ);
    extract.setNegative(true);
    extract.filter(extraccion);
    return extraccion;
  }

  pcl::PointCloud<pcl::PointXYZRGB> RemoverPlano (pcl::PointCloud<pcl::PointXYZRGB> nube)
  {
    /*esta funcion elimina todos los puntos posteriores a un umbral, con el fin
    tratar de eliminar ruido en las nubes, recib una nube tipo XYZ y regresa una
    nube tipo XYZ*/

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB> extraccion;
    //remuevo ruido Z
    pcl::PointIndices::Ptr inliersZ(new pcl::PointIndices());
    std::pair<pcl::PointXYZRGB,pcl::PointXYZRGB> p=klib::minMaxPoints(nube);
    for(size_t i=0;i<nube.points.size();i++)
    {
      //si mi punto esta mas lejos del umbral lo almaceno su indice
      if(nube.points[i].z>(p.first.z+0.02))
      {
        inliersZ->indices.push_back(i);
      }
    }
    std::cout<<"limpiando Z"<<std::endl;
    extract.setInputCloud(nube.makeShared());
    extract.setIndices(inliersZ);
    extract.setNegative(true);
    extract.filter(extraccion);
    return extraccion;
  }
}//tlib

#endif
