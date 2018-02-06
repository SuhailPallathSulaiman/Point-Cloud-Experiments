#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>


#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
DIR *dir;
struct dirent *ent;
size_t j ;
using pcl::visualization::PointCloudColorHandlerCustom;

struct PCD
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new pcl::PointCloud<pcl::PointXYZRGB>) {};
};


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
  //viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, tgt_h, "sample cloud4", v2);
   // viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, src_h, "sample cloud4", v2);

  return (viewer);
}

void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}
int
main (int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());

  // Create a PCLVisualizer object
  // p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  // p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  // p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2original (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  // Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

  // for (size_t i = 1; i < data.size (); ++i)
    // int j;
   Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity ();
   Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
  for (size_t j = 1; j < data.size (); ++j)
  {
    cloud1 = data[j-1].cloud;
    cloud2 = data[j].cloud;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    *cloud2original=*cloud2;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    for (int i = 0; i < 30; ++i)
    {
      icp.setInputCloud(cloud2);
      icp.setInputTarget(cloud1);
      icp.align(*Final);
      *cloud2=*Final;
      PCL_INFO ("Loop=%d\n",i);
      if (i==0)
      {
        pairTransform = icp.getFinalTransformation () ;
      }
      else{
       pairTransform = icp.getFinalTransformation () * pairTransform;
      }



    }
    *Final=*Final+*cloud1;
    if (j>1)
    {
      GlobalTransform=GlobalTransform*pairTransform;


      pcl::transformPointCloud (*Final, *temp, GlobalTransform);
      *Output=*Output+*temp;

    }
    else{
      GlobalTransform=pairTransform;
      *Output=*temp;
    }
    PCL_INFO ("j= %d .", j);




  // viewer = viewportsVis(cloud1, cloud2original,Final);
  // while (!viewer->wasStopped ())
  // {

  //   viewer->spinOnce (100);
  //   // boost::this_xthread::sleep (boost::posix_time::microseconds (100000));
  // }
}
      viewer = viewportsVis(cloud1, cloud2original,Output);
      viewer->spin();

}


