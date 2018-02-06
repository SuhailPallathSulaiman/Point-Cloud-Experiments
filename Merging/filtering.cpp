#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("pistol.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file pistol.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from pistol.pcd" << std::endl;


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("filtered.pcd", *cloud_filtered);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   // viewer_final->setBackgroundColor (255, 255, 255);

  // Coloring and visualizing target cloud (red).
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  // target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGB> (cloud_filtered);

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
