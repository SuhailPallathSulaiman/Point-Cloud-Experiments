#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
using pcl::visualization::PointCloudColorHandlerCustom;

    pcl::visualization::PCLVisualizer *p;
    //its left and right viewports
    int vp_1, vp_2;


void showCloudsLeft(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_h (cloud1, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud2, 255, 0, 0);
  p->addPointCloud (cloud1, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud2, src_h, "vp1_source", vp_1);

  // PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}

void showCloudsRight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_h (cloud1, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud2, 255, 0, 0);
  p->addPointCloud (cloud1, tgt_h, "target", vp_2);
  p->addPointCloud (cloud2, src_h, "source", vp_2);

  p->spin();
}



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
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  viewer->addPointCloud<pcl::PointXYZRGB> (final, tgt_h, "sample cloud3", v2);
   // viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, src_h, "sample cloud4", v2);

  return (viewer);
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("Cup/cup3.pcd", *cloud1) == -1)
  {
    PCL_ERROR ("Couldn't read  \n");
    return (-1);
  }
  std::cout << "Loaded " ;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("Cup/cup4.pcd", *cloud2) == -1)
  {
    PCL_ERROR ("Couldn't read  \n");
    return (-1);
  }
  std::cout << "Loaded 2" ;


  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud2);
  icp.setInputTarget(cloud1);
  icp.align(*Final);

  *cloud2=*Final;

  icp.setInputCloud(cloud2);
  icp.setInputTarget(cloud1);
  icp.align(*Final);

  *Final=*Final+*cloud1;
  // pcl::concatenateFields (*Final, *cloud2, *Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;

  pcl::io::savePCDFile("final.pcd", *Final, true);

  viewer = viewportsVis(cloud1, cloud2,Final);
  while (!viewer->wasStopped ())
  {

    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


