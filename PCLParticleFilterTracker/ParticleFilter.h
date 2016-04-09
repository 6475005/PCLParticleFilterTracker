#ifndef __PARTICLE_FILTER_TRACKER_PCL_H__
#define __PARTICLE_FILTER_TRACKER_PCL_H__

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define NOMINMAX

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/distances.h>
#include <boost/format.hpp>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace pcl::tracking;

typedef pcl::PointXYZRGB				RefPointType;
typedef pcl::PointCloud<RefPointType>	RefCloud;

//typedef pcl::tracking::ParticleXYZRPY		ParticleType;
typedef pcl::tracking::ParticleXYZR		ParticleType;

int index = 0;

#define		MAP_SIZE	     400
#define		PIXEL_BY_METER   100.0



void output_points_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,char *filename)
{
	FILE *fp = fopen(filename,"w");
	for(int i=0;i<cloud->points.size();i++){
		fprintf(fp,"%lf %lf %lf \n",cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
	}
	fclose(fp);
}
void output_points_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,char *filename)
{
	FILE *fp = fopen(filename,"w");
	for(int i=0;i<cloud->points.size();i++){
		fprintf(fp,"%lf %lf %lf %d %d %d\n",
			cloud->points[i].x,cloud->points[i].y,cloud->points[i].z
			,cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
	}
	fclose(fp);
}
bool isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,std::vector<float> &coeff)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	std::vector<int> non_zero;
	non_zero.clear();

	for(int i=0;i<input_cloud->points.size();i++){
		if(input_cloud->points[i].z>0)
			non_zero.push_back(i);
	}

	pcl::copyPointCloud(*input_cloud,non_zero,*cloud);

	//ne.setSearchMethod (tree);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setRadiusSearch (0.03);
	ne.compute (*cloud_normals);
	
	const int seed = 0;
	const int num = cloud->points.size() * 0.1f;
	srand(seed);
	int count =0;

	//Vector3D coeff_vec = Vector3D(coeff[0],coeff[1],coeff[2]);
	//Eigen::Affine3f coeff_v = Eigen::Affine3f(coeff[0],coeff[1],coeff[2]); 
	Eigen::Vector3f coeff_v(coeff[0],coeff[1],coeff[2]);
	Eigen::Vector3f comp_v;

	for(int i=0;i<num;i++){
		int add_ind = rand()%cloud->points.size()+1;
		//Vector3D compared_vec = Vector3D(cloud_normals->points[add_ind].normal_x,cloud_normals->points[add_ind].normal_y,cloud_normals->points[add_ind].normal_z);
		comp_v = Eigen::Vector3f(cloud_normals->points[add_ind].normal_x,cloud_normals->points[add_ind].normal_y,cloud_normals->points[add_ind].normal_z);
		//float angle =  abs(VecAngle(coeff_vec,compared_vec));
		float angle =  abs( coeff_v.dot(comp_v) );
		if(angle>0.95)count++;
	}

	if(count>(num/2))
		return true;
	else
		return false;
}
void Euclidean_Clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, std::vector<pcl::PointIndices> &cluster_indices)
{
	const unsigned int minRegionSize = 100;
	const unsigned int maxRegionSize = 640*480;

	//ユークリッド距離で物体ごとのクラスタリング
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	tree->setInputCloud (input_cloud);

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (minRegionSize);
	ec.setMaxClusterSize (maxRegionSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud(input_cloud);
	ec.extract (cluster_indices);
}

void Plane_Detection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointIndices::Ptr indices,pcl::ModelCoefficients::Ptr coefficients,float thleshold)
{
	//平面検出のためのモデル
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (thleshold);

	seg.setInputCloud(input_cloud);
	seg.segment(*indices, *coefficients);
}
void Extractor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointIndices::Ptr indices,bool tf,pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
	//平面検出のためのモデル
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (input_cloud);
	extract.setIndices (indices);
	extract.setNegative (tf);
	extract.filter (*output_cloud);
}


void detectPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
				 Eigen::Vector4f  &out_coefficients,
				 pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud)
{
	//点群
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud       (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*input_cloud,*cloud);


	////平面検出のためのモデル
	pcl::PointCloud<pcl::PointXYZ>::Ptr			plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr			target_plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr					coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr						inliers (new pcl::PointIndices ());

	const int	plane_num			= 1;
	int			index_target_plane	= -1;//desk
	float		dist_nearest_plane	= 0.0f;

	pcl::ModelCoefficients::Ptr target_plane_coefficients (new pcl::ModelCoefficients ());


	//物体がのっていそうな平面を抽出
	for(int loop=0;loop<plane_num;loop++){
		Plane_Detection(cloud,inliers,coefficients,0.025);
		Extractor(cloud,inliers,false,plane_cloud);

		float dist = 0.0f;
		for(int i=0;i<plane_cloud->points.size();i++)
			dist += plane_cloud->points[i].z;
		dist /= (float)plane_cloud->points.size();

		bool Isplane = isPlane(plane_cloud,coefficients->values);
		if(Isplane){
			fprintf(stderr,"plane\n");
		}else{
			fprintf(stderr,"not plane\n");
		}

		if((dist < dist_nearest_plane || dist_nearest_plane==0) && Isplane ){
			dist_nearest_plane = dist;
			index_target_plane = loop+1;
			*target_plane_coefficients = *coefficients;
			pcl::copyPointCloud(*plane_cloud,*target_plane_cloud);
		}

		//検出した平面を点群から排除して，次の平面を探す
		Extractor(cloud,inliers,true,cloud);
		output_points_pcl(cloud,"cloud.txt");
		output_points_pcl(target_plane_cloud,"plane.txt");
	}

	if(index_target_plane == -1){
		return;//見つからなかった場合
	}

	//ユークリッド距離でクラスタリング　平面のゴミを消す
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_plane_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_in_plane_indices;
	Euclidean_Clustering(target_plane_cloud,cluster_in_plane_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_in_plane_indices.begin (); it != cluster_in_plane_indices.end (); ++it){
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			target_plane_cluster->points.push_back (target_plane_cloud->points[*pit]);
		}
		target_plane_cluster->width = target_plane_cluster->points.size ();
		target_plane_cluster->height = 1;
		target_plane_cluster->is_dense = true;
		break;
	}
	output_points_pcl(target_plane_cluster,"target_plane_cluster.txt");


	//平面の再推定
	pcl::PointIndices::Ptr tpc_inliner (new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr target_plane_coeff (new pcl::ModelCoefficients ());
	Plane_Detection(target_plane_cluster,inliers,coefficients,0.01);
	pcl::copyPointCloud(*target_plane_cluster,*inliers,*target_plane_cluster);


	//Convex Hull の領域を出す
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	//process projection 
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (target_plane_cluster);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	//target_plane_convexfull
	pcl::ConvexHull<pcl::PointXYZ> ch;
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_hull	(new pcl::PointCloud<pcl::PointXYZ>);	
	ch.setInputCloud(cloud_projected);
	ch.reconstruct(*c_hull);

	pcl::copyPointCloud(*c_hull,*out_cloud);

	//入力データの再コピー
	//pcl::copyPointCloud(*input_cloud,*cloud);

	//検出した平面で分ける
	//std::vector<int> inliers_target_plane;	 
	//for(int i=0;i<cloud->points.size();i++){
	//	const float thle = 0.015f;//1.5cm
	//	if(cloud->points[i].z>0.0f){
	//		if(!(abs(pointToPlaneDistanceSigned(cloud->points[i],target_plane_coeff->values[0],target_plane_coeff->values[1],target_plane_coeff->values[2],target_plane_coeff->values[3]))<thle)){
	//			inliers_target_plane.push_back(i);
	//		}
	//	}
	//}
	//pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers_target_plane, *cloud);
	//pcl::copyPointCloud<pcl::PointXYZ>(*index, inliers_target_plane, *index);

	//Convex Hullの中だけの点群にする
	//pcl::ExtractPolygonalPrismData<pcl::PointXYZ> epp;
	//pcl::PointIndices indices;
	//epp.setInputPlanarHull(c_hull);
	//epp.setHeightLimits(0.0,0.70);
	//epp.setInputCloud(cloud);
	//epp.segment(indices);	//平面上にある点のindexを獲得

	//pcl::copyPointCloud<pcl::PointXYZ>(*cloud, indices, *cloud);
	//pcl::copyPointCloud<pcl::PointXYZ>(*index, indices, *index);

	//output_points_pcl(cloud,"pc_w_p.txt");//平面の上の点群
	//pcl::copyValueString(*target_plane_coeff,*out_coefficients);

	//*out_coefficients = *target_plane_coeff;
	for(int i=0;i<4;i++)
		out_coefficients[i] = coefficients->values[i];
	
	return;

	/*
	//ロボットの点群を取り除く
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud);						//物体ｓの点群
	proj.setModelCoefficients (target_plane_coeff);	//射影する平面のcofficiensts
	proj.filter (*cloud_projected);					//射影した点群

	pcl::PointCloud<pcl::PointXYZ>::Ptr robotPos	(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr	robotPointIdx (new pcl::PointIndices());

	//ロボットの頭が一番上のほうにある
	float max_height = 0;
	for(int i=0;i<cloud->points.size();i++){
		if(cloud->points[i].y>max_height){
			max_height = cloud->points[i].y;
			naoPos[0] = cloud->points[i].x;
			naoPos[1] = cloud->points[i].y;
			naoPos[2] = cloud->points[i].z;
		}
	}

	robotPos->push_back(pcl::PointXYZ(naoPos[0],naoPos[1],naoPos[2]));
	robotPos->width = robotPos->points.size ();
	robotPos->height = 1;
	robotPos->is_dense = true;

	proj.setInputCloud (robotPos);	//ロボットの点
	proj.filter (*robotPos);		//平面に射影したロボットの点（位置）

	for(int i=0;i<cloud_projected->points.size();i++){
		double d = pcl::euclideanDistance(robotPos->points[0],cloud_projected->points[i]);
		if(d>0.30f){
			robotPointIdx->indices.push_back(i);
		}
	}

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *robotPointIdx, *cloud);
	pcl::copyPointCloud<pcl::PointXYZ>(*index, *robotPointIdx, *index);


	//pcl_function::output_points_pcl(cloud,"3ddata\\pc_w_p.txt");//平面の上の点群

	return;
	*/
}

void Test(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
		   Eigen::Vector4f	&input_coefficients,
	      pcl::PointCloud<pcl::PointXYZ>::Ptr &input_plane_cloud,
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud_rt,
		  pcl::PointCloud<pcl::PointXYZ>::Ptr &out_nao_cloud )
{
	//点群
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud       (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RT    (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_cluster    (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster    (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clouds;

	pcl::ModelCoefficients::Ptr plane_coefficient (new pcl::ModelCoefficients ());
	
	pcl::copyPointCloud(*input_cloud, *cloud);
	pcl::copyPointCloud(*input_plane_cloud, *plane_cloud);

	for(int i=0;i<4;i++)
		plane_coefficient->values.push_back(input_coefficients[i]);

	if(!plane_cloud->points.size()>0){
		return;
	}

	//Convex Hullの中だけの点群にする
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> epp;
	pcl::PointIndices indices;
	epp.setInputPlanarHull(plane_cloud);
	epp.setHeightLimits(0.015,0.70);
	epp.setInputCloud(cloud);
	epp.segment(indices);	//平面上にある点のindexを獲得

	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, indices, *cloud);
	
	//回転させる
	Eigen::Vector3f vec_plane_normal;
	Eigen::Vector3f vec_z(1.0,0.0,0.0);
	Eigen::Affine3f RTMat;

	for(int i=0;i<3;i++)
		vec_plane_normal[i] = input_coefficients[i] * (input_coefficients[1]>0 ? 1.0:-1.0) ;

	vec_z = vec_z.cross(vec_plane_normal);
	for(int i=0;i<3;i++)
		vec_z[i] = vec_z[i] * (vec_z[1]>0 ? 1.0:-1.0) ;
	

	pcl::getTransformationFromTwoUnitVectorsAndOrigin(vec_plane_normal,vec_z,Eigen::Vector3f(0.0f,0.0f,0.0f),RTMat);
	pcl::transformPointCloud(*cloud,*cloud_RT,RTMat);

	//距離でクラスタリング
	//std::vector<pcl::PointIndices> cluster_in_plane_indices;
	//Euclidean_Clustering(cloud_RT,cluster_in_plane_indices);
	
	//一番背の高いクラスタをNaoのクラスタとする．
	//その他は障害物(物体)のクラスタ
	//float height_max = 0.0f;
	//Eigen::Vector4f min_val,max_val;
	//
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_in_plane_indices.begin (); it != cluster_in_plane_indices.end (); ++it){
	//	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
	//		cloud_cluster->points.push_back (cloud_RT->points[*pit]);
	//	}
	//	cloud_cluster->width = cloud_cluster->points.size ();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	pcl::getMinMax3D (*cloud_cluster,min_val,max_val);
	//	if(max_val[1] > height_max || height_max == 0.0f){
	//		height_max = max_val[1];
	//		pcl::copyPointCloud(*cloud_cluster,*target_cloud_cluster);
	//	}

	//	object_clouds.push_back(cloud_cluster);
	//	cloud_cluster.reset (new pcl::PointCloud<pcl::PointXYZ>);
	//}

	//Naoのクラスタを射影して,点群の分布から向きを推定
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ProjectInliers<pcl::PointXYZ> proj;
	////process projection 
	//proj.setModelType (pcl::SACMODEL_PLANE);
	//proj.setInputCloud (target_cloud_cluster);
	//proj.setModelCoefficients ( plane_coefficient);
	//proj.filter (*cloud_projected);
	
	//エセプロジェクション
	//for(int i=0;i<target_cloud_cluster->points.size();i++){
	//	cloud_projected->points.push_back (target_cloud_cluster->points[i]);
	//	cloud_projected->points[i].y = 0.0f;
	//}
	//cloud_projected->width = cloud_projected->points.size ();
	//cloud_projected->height = 1;
	//cloud_projected->is_dense = true;
	
	pcl::copyPointCloud(*cloud_RT,*out_cloud_rt);
	//pcl::copyPointCloud(*cloud_projected,*out_nao_cloud);

	return;
}

CvPoint3D32f getClickedPoint(IplImage *DImg,CvPoint pt_clicked)
{
	CvPoint3D32f pt3d_clicked = cvPoint3D32f(0.0f, 0.0f, 0.0f);
	const float f = 290.0;//450
	const float cx = (float)DImg->width/2.0f;
	const float cy = (float)DImg->height/2.0f;

	for(int i=0;i<DImg->width;i++){
		for(int j=0;j<DImg->height;j++){
			unsigned int realdepth = 0;
			if(DImg->nChannels == 1 && DImg->depth == IPL_DEPTH_16U){
				unsigned char depth_lower = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 0];
				unsigned char depth_upper = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 1];
				realdepth = depth_upper << 5 |depth_lower >> 3;
			}else if(DImg->nChannels == 3 && DImg->depth == IPL_DEPTH_8U){
				unsigned char depth_lower = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 3 + 1];
				unsigned char depth_upper = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 3 + 2];
				realdepth = depth_upper << 8 | depth_lower;
			}
			if(realdepth>40000 || realdepth<400){
				continue;
			}else{
				if(i == pt_clicked.x && j == pt_clicked.y){
					pt3d_clicked.z =  (float)realdepth/1000.0f;
					pt3d_clicked.x =  (((float)(i))-cx) * pt3d_clicked.z/f;
					pt3d_clicked.y =  -1.0f *(((float)(j))-cy) * pt3d_clicked.z/f;
				}
			}
		}
	}
	return pt3d_clicked;
}

int KconvertPCLPoint(IplImage *DImg,pcl::PointCloud<RefPointType>::Ptr cloud,IplImage *color=NULL)
{
	const float f = 290.0;//450
	const float cx = (float)DImg->width/2.0f;
	const float cy = (float)DImg->height/2.0f;

	cloud->clear();
	for(int i=0;i<DImg->width;i++){
		for(int j=0;j<DImg->height;j++){
			unsigned int realdepth = 0;
			if(DImg->nChannels == 1 && DImg->depth == IPL_DEPTH_16U){
				unsigned char depth_lower = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 0];
				unsigned char depth_upper = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 1];
				realdepth = depth_upper << 5 |depth_lower >> 3;
			}else if(DImg->nChannels == 3 && DImg->depth == IPL_DEPTH_8U){
				unsigned char depth_lower = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 3 + 1];
				unsigned char depth_upper = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 3 + 2];
				realdepth = depth_upper << 8 | depth_lower;
			}
			if(realdepth>40000 || realdepth<400){
				continue;
			}else{
				RefPointType	temp;
				temp.z = (float)realdepth/1000.0f;
				temp.x =  (((float)(i))-cx) * temp.z/f;
				temp.y =  -1.0f *(((float)(j))-cy) * temp.z/f;
				if(color !=NULL){
					temp.r = color->imageData[color->widthStep*j+i*color->nChannels+2];
					temp.g = color->imageData[color->widthStep*j+i*color->nChannels+1];
					temp.b = color->imageData[color->widthStep*j+i*color->nChannels+0];
				}
				cloud->points.push_back(temp);
			}
		}
	}

	return 0;
}
void gridSampleApprox (const RefCloud::Ptr &cloud, RefCloud &result, double leaf_size = 0.01)
{
	pcl::ApproximateVoxelGrid<RefPointType> grid;
	grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud (cloud);
	grid.filter (result);
}
void cutOffCloud(const RefCloud::Ptr &cloud,const pcl::PointXYZ &point, RefCloud &result)
{
	
	float threshold = 1.0f;
	for(unsigned int i=0;i<cloud->points.size();i++){
		if(pcl::euclideanDistance(cloud->points[i],point) < threshold){
			result.points.push_back(cloud->points[i]);
		}
	}
	
	return;
}
int readModelData(RefCloud &result,char *filename = NULL){
	
	FILE *fp;
	if(filename == NULL){return -1;}
	if((fp = fopen(filename,"r") ) == NULL){
		return -1;
	}
	char buf[256];
	
	while(fgets(buf,256,fp)!=NULL){
		RefPointType point;
		int r,g,b;
		int num = sscanf(buf,"%f %f %f %d %d %d",&point.x,&point.y,&point.z,&r,&g,&b);
		if(num==6){
			point.r = (uint8_t)r;
			point.g = (uint8_t)g;
			point.b = (uint8_t)b;
		}
		result.push_back(point);
	}
	fclose(fp);
	return 0;
}
class PFT_Wrap{
public:
	PFT_Wrap(){
	}

	~PFT_Wrap(){
		end();
	}
	void set_model(RefCloud::Ptr &input_point){
		tracker->setReferenceCloud(input_point);
		tracker->setMinIndices (int (input_point->points.size ()) / 2);
	}

	void set_Trans( Eigen::Vector3f point){
		Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
		trans.translation ().matrix () = point;
		
		tracker->resetTracking();
		tracker->setTrans (trans);
	}

	void get_ResultPointsCloud(RefCloud::Ptr &output_point){
		Eigen::Affine3f transformation = tracker->toEigenMatrix (result);
		pcl::transformPointCloud<RefPointType> (*(tracker->getReferenceCloud ()), *output_point, transformation);
	}

	ParticleType track(const RefCloud::Ptr &input_point){
		if(input_point->points.size()<100){
			return result;
		}
		//ポイントをセット
		tracker->setInputCloud(input_point);
		//コンピュート
		tracker->compute();
		//結果を返す
		result = tracker->getResult();
		if (index == 5)
		{
			fprintf(stdout,"%f %f %f %f %f %f\n",result.x,result.y,result.z,result.roll,result.pitch,result.yaw);
			index = 0;
		}else{
			index ++;
		}
		return result;
	}

	void init(){
		tracker.reset((new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleType> (8)));
		//KLDSamplingのパラメータ設定
		tracker->setMaximumParticleNum (200);
		tracker->setDelta (0.98);
		tracker->setEpsilon (0.05);
		bin_size.x = 0.2f;
		bin_size.y = 0.2f;
		bin_size.z = 0.2f;
		bin_size.roll = 0.2f;
		bin_size.pitch = 0.2f;
		bin_size.yaw = 0.2f;
		tracker->setBinSize (bin_size);

		//パラメータ設定
		default_step_covariance.resize(6, 0.015 * 0.015);
    
		default_step_covariance[3] *= 10.0 ;
		default_step_covariance[4] *= 10.0 ;
		default_step_covariance[5] *= 10.0 ;
	
		initial_noise_covariance.resize(6, 0.0001 );
		default_initial_mean.resize(6, 0.0 );

		tracker->setTrans (Eigen::Affine3f::Identity ());
		tracker->setStepNoiseCovariance (default_step_covariance);
		tracker->setInitialNoiseCovariance (initial_noise_covariance);
		tracker->setInitialNoiseMean (default_initial_mean);
		tracker->setIterationNum (1);

		tracker->setParticleNum (150);
		tracker->setResampleLikelihoodThr(0.00);
		tracker->setUseNormal (false);

		//coherences設定
		coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr 
			(new ApproxNearestPairPointCloudCoherence<RefPointType> ());
		//distance
		distance_coherence = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
		coherence->addPointCoherence (distance_coherence);
	
		search.reset(new pcl::search::Octree<RefPointType> (0.01));
		coherence->setSearchMethod (search);
		coherence->setMaximumDistance (0.015);
		tracker->setCloudCoherence (coherence);
	}

	void end(){
		return;
	}


private:
	//Trackerの設定
	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleType> > tracker;
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence;
	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence;
	boost::shared_ptr<pcl::search::Octree<RefPointType> > search;
	ParticleType result;
	ParticleType bin_size;
	std::vector<double> default_step_covariance;
	std::vector<double> initial_noise_covariance;
	std::vector<double> default_initial_mean;
};

class PFT_Manager
{
public:
	PFT_Manager(){
		reference_cloud = pcl::PointCloud<RefPointType>::Ptr (new pcl::PointCloud<RefPointType>);
		readModelData(*reference_cloud,"nao_upbody.txt");
	}
	~PFT_Manager(){}
	
	void init(){
		tracker.init();
		tracker.set_model(reference_cloud);
		tracker.set_Trans(point_of_interest);
	}

	void setInitPose(cv::Point3f particle_pos){
		Eigen::Vector3f eigen_point = Eigen::Vector3f(particle_pos.x,particle_pos.y,particle_pos.z);
		tracker.init();
		tracker.set_model(reference_cloud);
		tracker.set_Trans(eigen_point);
	}

	bool setPlaneCoefficient(float *coefficient, int size){
		if(size !=4){return false;}
		for(int i=0;i<4;i++)
			plane_coefficients[i] = coefficient[i];
		return true;
	}

	bool getResult(ParticleType &r){
		r = result;
	}

	bool track(IplImage *depthImg, pcl::PointCloud<RefPointType>::Ptr	&result_cloud, IplImage *dcolorImg = NULL){

		if(depthImg == NULL){
			return false;
		}

		pcl::PointCloud<RefPointType>::Ptr	cloud			(new pcl::PointCloud<RefPointType>);
		pcl::PointCloud<RefPointType>::Ptr	sampled_cloud	(new pcl::PointCloud<RefPointType>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr	plane_cloud		(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PointCloud<RefPointType>::Ptr	cloud_RT		(new pcl::PointCloud<RefPointType>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr	nao_cloud		(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<RefPointType>::Ptr	nao_cloud_RT	(new pcl::PointCloud<RefPointType>);
	
	
		//距離画像を点群に変換
		KconvertPCLPoint(depthImg,cloud,dcolorImg);
		//グリッドサンプリング
		gridSampleApprox(cloud,*sampled_cloud,0.0075);

		//平面の情報を使って回転とかほげほげ
		Test(sampled_cloud,plane_coefficients,plane_cloud,cloud_RT,nao_cloud);

		//XYZRでPFT
		result = tracker.track(cloud_RT);
		tracker.get_ResultPointsCloud(nao_cloud_RT);

		pcl::copyPointCloud(*nao_cloud_RT,*result_cloud);
	}

	void RotateCloudUsingPlaneCoefficient(std::vector<std::pair<cv::Point3f,cv::Point3d>> &src_cloud, float *coefficient = NULL){
		
		//回転させる
		Eigen::Vector3f vec_plane_normal;
		Eigen::Vector3f vec_z(1.0,0.0,0.0);
		Eigen::Affine3f RTMat;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud       (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RT    (new pcl::PointCloud<pcl::PointXYZRGB>);


		for(int i=0;i<3;i++){
			if(coefficient != NULL)
				vec_plane_normal[i] = coefficient[i] * (coefficient[1]>0 ? 1.0:-1.0);
			else
				vec_plane_normal[i] = plane_coefficients[i] * (plane_coefficients[1]>0 ? 1.0:-1.0);
		}

		vec_z = vec_z.cross(vec_plane_normal);
		for(int i=0;i<3;i++)
			vec_z[i] = vec_z[i] * (vec_z[1]>0 ? 1.0:-1.0) ;
	

		pcl::getTransformationFromTwoUnitVectorsAndOrigin(vec_plane_normal,vec_z,Eigen::Vector3f(0.0f,0.0f,0.0f),RTMat);
		pcl::transformPointCloud(*cloud,*cloud_RT,RTMat);

		src_cloud.clear();
		for(int i=0;i<cloud_RT->points.size();i++){
			src_cloud.push_back(std::make_pair(cv::Point3f(cloud_RT->points[i].x,cloud_RT->points[i].y,cloud_RT->points[i].z),
											   cv::Point3d(cloud_RT->points[i].r,cloud_RT->points[i].g,cloud_RT->points[i].b)));
		}
	}

	void end(){
		
	}

private:
	PFT_Wrap	tracker;

	pcl::PointCloud<RefPointType>::Ptr		reference_cloud;

	Eigen::Vector3f							point_of_interest;
	Eigen::Vector4f							plane_coefficients;

	ParticleType result;
};



#endif