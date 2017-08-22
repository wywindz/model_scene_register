#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/time.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main()
{

  /**
    Note: when register cup, it seems that remove the plane part of the cup can get better results;
   */


///*
  const std::string model_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//model.pcd";
  const std::string scene_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//scene.pcd";//frame_20111220T111153.549117.pcd";
//*/

/*
  const std::string model_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//milk.pcd";
  const std::string scene_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//milk_cartoon_all_small_clorox.pcd";
*/
  /*
  const std::string model_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//chef.pcd";
  const std::string scene_pcd_fileName="//home//wangy//dev//3dvision_ws//projects//RAM_dataset"
                                       "//mydataset//rs1.pcd";
   */

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
  normalCloudPtr model(new normalCloudT);
  if(pcl::io::loadPCDFile<normalT>(model_pcd_fileName,*model)==-1){
      std::cerr<<"ERROR: load model failed!"<<std::endl;
    }

  //load scene pcd file
  normalCloudPtr scene(new normalCloudT);
  if(pcl::io::loadPCDFile<normalT>(scene_pcd_fileName,*scene)==-1){
      std::cerr<<"ERROR: load scenne failed!"<<std::endl;
    }

  std::cout<<"the size of model is : "<<model->points.size()<<std::endl;
  std::cout<<"the size of scene is : "<<scene->points.size()<<std::endl;

  double leafsize=0.005;
  //downsample
  pcl::VoxelGrid<normalT> voxel;
  voxel.setLeafSize(leafsize,leafsize,leafsize);
  voxel.setInputCloud(model);
  voxel.filter(*model);
  //voxel.setInputCloud(scene);
  //voxel.filter(*scene);
  std::cout<<"the size of sampled model is : "<<model->points.size()<<std::endl;
  std::cout<<"the size of sampled scene is : "<<scene->points.size()<<std::endl;

  //normal estimation
  normalCloudPtr model_normals(new normalCloudT);
  normalCloudPtr scene_normals(new normalCloudT);

  //pass through
  pcl::PassThrough<normalT> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0,1.8);
  pass.setInputCloud(scene);
  pass.filter(*scene);

  //plane segment
  pcl::SACSegmentation<normalT> plane_seg;
  pcl::ModelCoefficients::Ptr coeffPtr(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(pcl::SACMODEL_PLANE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setInputCloud(scene);
  plane_seg.setDistanceThreshold(0.008);
  plane_seg.segment(*inliers,*coeffPtr);
  if(inliers->indices.size()==0)
    {
      std::cout<<">>> INFO: no plane extracted"<<std::endl;
    }
  else
    std::cout<<">>> INFO: plane extracted, point size: "<<inliers->indices.size()<<std::endl;

  //extract plane and scene-without-plane
  pcl::ExtractIndices<normalT> extractor;
  extractor.setInputCloud(scene);
  extractor.setIndices(inliers);
  extractor.setNegative(true);
  extractor.filter(*scene);
  std::cout<<">>> INFO: scene extracted, point size: "<<scene->points.size()<<std::endl;
  //detection::pcd_viewer::showPCD<pointT>(sceneCloudPtr);

  //outliers removal
  pcl::StatisticalOutlierRemoval<normalT> sor;
  sor.setInputCloud (scene);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.1);
  sor.filter (*scene);
  std::cout<<">>> INFO: after outliers removal, scene point size: "<<scene->points.size()<<std::endl;

  //remove nans
  /*
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*model, *model, mapping);
  pcl::removeNaNNormalsFromPointCloud(*model, *model, mapping);
  pcl::removeNaNFromPointCloud(*scene, *scene, mapping);
  pcl::removeNaNNormalsFromPointCloud(*scene, *scene, mapping);
  */
  for(normalCloudT::iterator it=scene->begin();it<scene->end();++it){
      if(!pcl::isFinite(*it))
        scene->erase(it);
    }
  std::cout<<"the size of sampled scene after nans removal is : "<<scene->points.size()<<std::endl;

  pcl::NormalEstimationOMP<normalT,normalT> normal_est;
  pcl::search::KdTree<normalT>::Ptr tree (new pcl::search::KdTree<normalT> ());
  normal_est.setSearchMethod(tree);
  normal_est.setRadiusSearch(0.01);
  //normal_est.setInputCloud(model);
  //normal_est.compute(*model);

  normal_est.setInputCloud(scene);
  normal_est.compute(*scene);
  std::cout<<"the size of model normals is : "<<model->points.size()<<std::endl;
  std::cout<<"the size of scene normals is : "<<scene->points.size()<<std::endl;

  //pcl::visualization::PCLVisualizer vis("registratioin results");
  //vis.addPointCloud<normalT>(scene,"scene-after");
  //vis.spin();
  //return -1;


  //feature estimation
  featureCloudPtr model_features(new featureCloudT);
  featureCloudPtr scene_features(new featureCloudT);

  pcl::console::print_highlight ("Estimating features...\n");
  pcl::FPFHEstimationOMP<normalT,normalT,featureT> fest;
  fest.setRadiusSearch (0.015);
  fest.setInputCloud (model);
  fest.setInputNormals (model);
  fest.compute (*model_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);


  //sac_ia
  normalCloudPtr sac_aligned_model(new normalCloudT);
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<normalT,normalT,featureT> align;
  align.setInputSource (model);
  align.setSourceFeatures (model_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.05f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (0.01); // Inlier threshold
  align.setInlierFraction (0.05f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*sac_aligned_model);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");

  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }


  //icp
  normalCloudPtr icp_aligned_model(new normalCloudT);
  pcl::IterativeClosestPoint<normalT,normalT> icp;
  icp.setInputSource(sac_aligned_model);
  icp.setInputTarget(scene);
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setMaximumIterations(50000);
  icp.setUseReciprocalCorrespondences(true);
  icp.align(*icp_aligned_model);
  if(icp.hasConverged()){
      std::cout<<"the score of icp estimation is : "<<icp.getFitnessScore()<<std::endl;
      std::cout<<"is icp converged: "<<icp.hasConverged()<<std::endl;
      std::cout<<"icp transform matrix:"<<std::endl<<icp.getFinalTransformation()<<std::endl;
    }
  else{
      pcl::console::print_error("ICP Alignment stopped without convergence!\n");
    }

  //output results
  pcl::visualization::PCLVisualizer viewer("registration results");
  pcl::visualization::PointCloudColorHandlerCustom<normalT> model_ch (model, 100, 100, 100);
  pcl::visualization::PointCloudColorHandlerCustom<normalT> scene_ch (scene, 250, 100, 100);
  pcl::visualization::PointCloudColorHandlerCustom<normalT> sac_model_ch (sac_aligned_model, 100, 100, 255);
  pcl::visualization::PointCloudColorHandlerCustom<normalT> icp_model_ch (icp_aligned_model, 100, 255, 100);

  //viewer.addPointCloud(model,model_ch,"model");
  viewer.addPointCloud(scene,scene_ch,"scene");
  viewer.addPointCloud(sac_aligned_model,sac_model_ch,"sac_model");
  viewer.addPointCloud(icp_aligned_model,icp_model_ch,"icp_model");
  viewer.spin();

}
