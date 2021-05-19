#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_tests.h>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

//#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/common/centroid.h>


typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;

using namespace std;
using namespace pcl;

int
main (int argc, char** argv)
{
	  //Load the point cloud from pcd file
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("edge_cloud.pcd", *cloud) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	  }

	  //Compute centoid of point cloud
	  int num_pts = cloud->size();
	  float x = 0, y = 0, z = 0;
	  pcl::PointXYZ point;

	  for ( int i = 0; i < num_pts; i++ )
	  {
		x += cloud->points[i].x;
		y += cloud->points[i].y;
		z += cloud->points[i].z;
	  }

	  x = x / num_pts;
	  y = y / num_pts;
	  z = z / num_pts;

	  point.x = x;
	  point.y = y;
	  point.z = z;
	  

	  //Compute principal directions
	  Eigen::Vector4f pcaCentroid;
	  pcl::compute3DCentroid(*cloud, pcaCentroid);
	  Eigen::Matrix3f covariance;
	  computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  

	  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PCA<pcl::PointXYZ> pca;
	  pca.setInputCloud(cloud);
	  pca.project(*cloud, *cloudPCAprojection);
	  std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
	  std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;*/
	  //
	 
	  //Transform the original cloud to the origin where the principal components correspond to the axes.
	  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
	  //Get the minimum and maximum points of the transformed cloud.
	  pcl::PointXYZ minPoint, maxPoint;
	  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	  //Final transform
	  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	  // This viewer has 4 windows, but is only showing images in one of them as written here.
	  pcl::visualization::PCLVisualizer *visu;
	  visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
	  int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
	  visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
	  visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
	  visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
	  visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
	  visu->addPointCloud(cloud, ColorHandlerXYZ(cloud, 30, 144, 255), "bboxedCloud", mesh_vp_3);
	  visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
	  visu->spin();

	  return (0);
}
