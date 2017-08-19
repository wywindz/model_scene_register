#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
  const std::string model_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//Brita_water_filter_pitcher_6cup_model.pcd";
  const std::string scene_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//Brita_water_filter_pitcher_6cup_scene.pcd";
  typedef pcl::PointXYZ pointT;
  typedef pcl::PointCloud<pcl::PointXYZ> pointCloudT;
  typedef pointCloudT::Ptr pointCloudPtr;
  typedef pcl::PointNormal normalT;
  typedef pcl::PointCloud<normalT> normalCloudT;
  typedef normalCloudT::Ptr normalCloudPtr;
  typedef pcl::FPFHSignature33 featureT;
  typedef pcl::PointCloud<featureT> featureCloudT;
  typedef featureCloudT::Ptr featureCloudPtr;

  //load model pcd file
  pointCloudPtr model(new pointCloudT);
  if(pcl::io::loadPCDFile(model_pcd_fileName,*model)==-1){
      std::cerr<<"ERROR: load model failed!"<<std::endl;
    }

  //load scene pcd file
  pointCloudPtr scene(new pointCloudT);
  if(pcl::io::loadPCDFile(scene_pcd_fileName,*scene)==-1){
      std::cerr<<"ERROR: load scenne failed!"<<std::endl;
    }

  std::cout<<"the size of model is : "<<model->points.size()<<std::endl;
  std::cout<<"the size of scene is : "<<scene->points.size()<<std::endl;

  //downsample
  pointCloudPtr sampled_model(new pointCloudT);
  pointCloudPtr sampled_scene(new pointCloudT);
  double leafsize=0.006;
  pcl::VoxelGrid<pointT> voxel;
  voxel.setLeafSize(leafsize,leafsize,leafsize);
  voxel.setInputCloud(model);
  voxel.filter(*sampled_model);
  voxel.setInputCloud(scene);
  voxel.filter(*sampled_scene);
  std::cout<<"the size of sampled model is : "<<sampled_model->points.size()<<std::endl;
  std::cout<<"the size of sampled scene is : "<<sampled_scene->points.size()<<std::endl;

  //features estimation
  normalCloudPtr model_normals(new normalCloudT);
  normalCloudPtr scene_normals(new normalCloudT);
  featureCloudPtr model_features(new featureCloudT);
  featureCloudPtr scene_features(new featureCloudT);

  pcl::search::KdTree<pointT>::Ptr tree(new pcl::search::KdTree<pointT>);
  pcl::NormalEstimationOMP<pointT,normalT> normal_est;
  normal_est.setInputCloud(sampled_model);
  normal_est.setRadiusSearch(0.06);
  normal_est.setSearchMethod(tree);
  normal_est.setSearchSurface(sampled_model);
  normal_est.compute(*model_normals);

  normal_est.setInputCloud(sampled_scene);
  normal_est.setSearchSurface(sampled_scene);
  normal_est.compute(*scene_normals);
  std::cout<<"the size of model normals is : "<<model_normals->points.size()<<std::endl;
  std::cout<<"the size of scene normals is : "<<scene_normals->points.size()<<std::endl;

  pcl::search::KdTree<normalT>::Ptr tree1(new pcl::search::KdTree<normalT>);
  pcl::FPFHEstimationOMP<normalT,normalT,featureT> feat_est;
  feat_est.setSearchMethod(tree1);
  feat_est.setRadiusSearch(0.01);
  feat_est.setInputCloud(model_normals);
  feat_est.setInputNormals(model_normals);
  feat_est.compute(*model_features);

  feat_est.setInputCloud(scene_normals);
  feat_est.setInputNormals(scene_normals);
  feat_est.compute(*scene_features);

  //sac_ia
  pointCloudPtr sac_aligned_model(new pointCloudT);
  pcl::SampleConsensusInitialAlignment<pointT,pointT,featureT> sac_ia;
  sac_ia.setInputSource(sampled_model);
  sac_ia.setSourceFeatures(model_features);
  sac_ia.setInputTarget(sampled_scene);
  sac_ia.setTargetFeatures(scene_features);
  sac_ia.setCorrespondenceRandomness(5);
  sac_ia.setMaxCorrespondenceDistance(0.01);
  sac_ia.setMaximumIterations(300);
  sac_ia.setNumberOfSamples(5);
  sac_ia.align(*sac_aligned_model);

  //icp
  pointCloudPtr icp_aligned_model(new pointCloudT);
  pcl::IterativeClosestPoint<pointT,pointT> icp;
  icp.setInputSource(sac_aligned_model);
  icp.setInputTarget(sampled_scene);
  icp.setMaxCorrespondenceDistance(0.01);
  icp.setMaximumIterations(500);
  icp.setUseReciprocalCorrespondences(true);
  icp.align(*icp_aligned_model);
  std::cout<<"the score of icp estimation is : "<<icp.getFitnessScore()<<std::endl;

  //output results
  pcl::visualization::PCLVisualizer viewer("registration results");
  pcl::visualization::PointCloudColorHandlerCustom<pointT> model_ch (sampled_model, 100, 100, 100);
  pcl::visualization::PointCloudColorHandlerCustom<pointT> scene_ch (sampled_scene, 250, 100, 100);
  pcl::visualization::PointCloudColorHandlerCustom<pointT> sac_model_ch (sac_aligned_model, 100, 100, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pointT> icp_model_ch (icp_aligned_model, 100, 255, 100);

  viewer.addPointCloud(sampled_model,model_ch,"model");
  viewer.addPointCloud(sampled_scene,scene_ch,"scene");
  viewer.addPointCloud(sac_aligned_model,sac_model_ch,"sac_model");
  viewer.addPointCloud(icp_aligned_model,icp_model_ch,"icp_model");
  viewer.spin();
}
