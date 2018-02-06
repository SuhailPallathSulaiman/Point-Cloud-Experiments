#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
DIR *dir;
struct dirent *ent;
using pcl::visualization::PointCloudColorHandlerCustom;



boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2,pcl::PointCloud<pcl::PointXYZRGB>::Ptr final)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_h (cloud1, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud2, 255, 0, 0);
   viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, tgt_h, "sample cloud1", v1);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, src_h, "sample cloud2", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> fin_h (final, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB> (final, fin_h, "sample cloud3", v2);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, tgt_h, "sample cloud4", v2);
   // viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, src_h, "sample cloud4", v2);

  return (viewer);
}


int
main (int argc, char** argv)
{


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2original (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("Cup/cup0.pcd", *cloud1) == -1)
  {
    PCL_ERROR ("Couldn't read  \n");
    return (-1);
  }
  std::cout << "Loaded 1" ;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("Cup/cup1.pcd", *cloud2) == -1)
  {
    PCL_ERROR ("Couldn't read  \n");
    return (-1);
  }
  std::cout << "Loaded 2" ;

*cloud2original=*cloud2;

 // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

 //  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
 //  double theta = M_PI / 8;  // The angle of rotation in radians
 //  transformation_matrix (0, 0) = cos (theta);
 //  transformation_matrix (0, 1) = -sin (theta);
 //  transformation_matrix (1, 0) = sin (theta);
 //  transformation_matrix (1, 1) = cos (theta);

 //  // A translation on Z axis (0.4 meters)
 //  transformation_matrix (2, 3) = 0.4;

 //  // Display in terminal the transformation matrix
 //  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;


 //  // Executing the transformation
 //  pcl::transformPointCloud (*cloud1, *cloud2, transformation_matrix);



  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  // icp.setInputCloud(cloud2);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);

  // *cloud2=*Final;
for (int i = 0; i < 50; ++i)
{
  icp.setInputCloud(cloud2);
  icp.setInputTarget(cloud1);
  icp.align(*Final);
  *cloud2=*Final;

}
*Final=*Final+*cloud1;
  // icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);

  //   icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);

  //     icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);
  //     icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);
  //     icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);
  //     icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);
  //     icp.setInputCloud(Final);
  // icp.setInputTarget(cloud1);
  // icp.align(*Final);

  // *Final=*Final+*cloud1;
  // // pcl::concatenateFields (*Final, *cloud2, *Final);
  // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  // icp.getFitnessScore() << std::endl;

  // *Out=*Final+*Out;
   // pcl::io::savePCDFile("Cube/cube_t.pcd", *cloud2, true);

  viewer = viewportsVis(cloud1, cloud2original,Final);
  while (!viewer->wasStopped ())
  {

    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


