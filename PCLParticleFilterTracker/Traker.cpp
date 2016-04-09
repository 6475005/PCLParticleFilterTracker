#include <stdio.h>
#include<math.h>
#define NOMINMAX
#include "Kinect\KinectGeneral.h"
#include "ParticleFilter.h"
#include "Dissper3D\Dispper3D.h"
#include "NaoAPI.h"
#include "Sift.h"
#include "ColorHistMatch.h"

//XYZRでPFT？
ParticleType result;
//Navgate
bool NavgateState = false;
/***********************modify*****************/
CvPoint pt_clicked;
CvPoint3D32f pt3d_clicked = cvPoint3D32f(0.0f, 0.0f, 0.0f);

IplImage *depthImg  = cvCreateImage(cvSize(320,240),IPL_DEPTH_16U,1);
IplImage *colorImg  = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,4);

void onMouse(int event,int x,int y,int flags,void*);
/***********************modify*****************/
class MapBuilderForNaoTracker{
private:
	IplImage *Map;
	IplImage *naoIcon;
	IplImage *naoIconMask;

	cv::Point2f robotPos;
	float		robotAngle;

	void makeMask(){
		cvZero(naoIconMask);

		for(int i=0; i < naoIconMask->width; i++){
			for(int j=0; j < naoIconMask->height; j++){	
				if((uchar)naoIcon->imageData[naoIcon->widthStep*j + i*3 + 0] != 255){
					naoIconMask->imageData[naoIconMask->width*j+i] = 255;
				}else if((uchar)naoIcon->imageData[naoIcon->widthStep*j + i*3 + 1] != 255){
					naoIconMask->imageData[naoIconMask->width*j+i] = 255;
				}else if((uchar)naoIcon->imageData[naoIcon->widthStep*j + i*3 + 2] != 255){
					naoIconMask->imageData[naoIconMask->width*j+i] = 255;
				}else{
					naoIconMask->imageData[naoIconMask->width*j+i] = 0;
				}
			}
		}
	}

	cv::Point2i MapCoord2CvCoord(cv::Point2i a){
		cv::Point2i b;
		b.x = a.x * PIXEL_BY_METER + MAP_SIZE/2;
		b.y = MAP_SIZE - a.y * PIXEL_BY_METER;
		return b;
	}

public:
	MapBuilderForNaoTracker(){
		init();
	}

	~MapBuilderForNaoTracker(){
		end();
	}
	void init(){
		robotPos   = cv::Point2i(0,0);
		robotAngle = 0;

		Map = cvCreateImage(cvSize(MAP_SIZE,MAP_SIZE),IPL_DEPTH_8U,3);
		cvZero(Map);

		if((naoIcon = cvLoadImage("naoHead.png")) == NULL){
			fprintf(stderr,"cant read image\n");
		}
		if((naoIconMask = cvCreateImage(cvSize(naoIcon->width,naoIcon->height),IPL_DEPTH_8U,1))==NULL){
			fprintf(stderr,"cant create image\n");
		}else{
			makeMask();
		}
	}

	void end(){
		cvReleaseImage(&Map);
	}
	void getMask(IplImage* dst){
		if(dst != NULL)
			cvReleaseImage(&dst);
		dst = cvCloneImage(naoIconMask);
	}
	void getMap(IplImage* dst){
		IplImage *rot_naoIcon     = cvCloneImage (naoIcon);
		IplImage *rot_naoIconMask = cvCloneImage (naoIconMask);

		float m[6];
		CvMat M;

		cvZero(Map);
		m[0] = (float) (cos (robotAngle * CV_PI / 180.));
		m[1] = (float) (-sin (robotAngle * CV_PI / 180.));
		m[2] = naoIcon->width * 0.5;
		m[3] = -m[1];
		m[4] = m[0];
		m[5] = naoIcon->height * 0.5;
		cvInitMatHeader (&M, 2, 3, CV_32FC1, m, CV_AUTOSTEP);

		// 回転行列により，GetQuadrangleSubPixを用いて画像全体を回転させる
		cvGetQuadrangleSubPix (naoIcon, rot_naoIcon, &M);
		cvGetQuadrangleSubPix (naoIconMask, rot_naoIconMask, &M);

		cv::Point2i rPoint = MapCoord2CvCoord(robotPos);
		int harfIconWidth	= rot_naoIconMask->width/2;
		int harfIconHeight	= rot_naoIconMask->height/2;

		for(int i=0; i < rot_naoIconMask->width; i++){
			for(int j=0; j < rot_naoIconMask->height; j++){	
				if(((uchar)rot_naoIconMask->imageData[rot_naoIconMask->widthStep*j +i]) > 0){
					int x = rPoint.x +i -harfIconWidth;
					int y = rPoint.y +j -harfIconHeight;
					if(x>0 && x<MAP_SIZE && y>0 && y<MAP_SIZE ){
						Map->imageData[Map->widthStep*y+x*3+0] = rot_naoIcon->imageData[rot_naoIcon->widthStep*j+i*3+0];
						Map->imageData[Map->widthStep*y+x*3+1] = rot_naoIcon->imageData[rot_naoIcon->widthStep*j+i*3+1];
						Map->imageData[Map->widthStep*y+x*3+2] = rot_naoIcon->imageData[rot_naoIcon->widthStep*j+i*3+2];
					}
				}
			}
		}
		cvCopy(Map,dst);
	}

	void setRobotPos(cv::Point2f Pos,float deg){
		robotAngle  = deg;
		robotPos	= Pos;
	}

};

boolean isArrived(float x,float y, float z){
	if(fabs(z)<=0.1 && fabs(x)<=0.1){
			return true;
	}else{
			return false;
	}
}

DWORD WINAPI Fun2(LPVOID lpParamter)
{
	while(1) {
		if (NavgateState)
		{
			if(pt3d_clicked.x != 0.0f || pt3d_clicked.y != 0.0f || pt3d_clicked.z != 0.0f)
			{
				double angle;
				angle = atan((pt3d_clicked.z - result.z)/(pt3d_clicked.x - result.x));
				std::cout<<"Arrived!"<<angle<<std::endl;
				if(fabs(angle - result.pitch)>0.2f){
					if(angle > result.pitch){
						moveTo(0,0,0.2f);
						fprintf(stdout,"%f\n",result.pitch);
					}else{
						moveTo(0,0,-0.2f);
						fprintf(stdout,"%f\n",result.pitch);
					}
				}
			}
			/*
			if(fabs(result.pitch)>0.2f){
				if(result.pitch>0.2f){
					moveTo(0,0,-0.2f);
					fprintf(stdout,"%f\n",result.pitch);
				}
				if (result.pitch<-0.2f)
				{
					moveTo(0,0,0.2f);
					fprintf(stdout,"%f\n",result.pitch);
				}
			}else{
				if(pt3d_clicked.x != 0.0f || pt3d_clicked.y != 0.0f || pt3d_clicked.z != 0.0f){
					float x = pt3d_clicked.x-result.x;
					float y = pt3d_clicked.y-result.y;
					float z = pt3d_clicked.z-result.z;
					if(!isArrived(x,y,z)){
						if(fabs(z)>0.1f){
							moveTo(0.1f*z/fabs(z),0.1f*x/fabs(z),0);
						}else{
							moveTo(0.1f*z/fabs(x),0.1f*x/fabs(x),0);
						}
						std::cout<<"moveTo x:"<<0.1f*z/fabs(z)<<" y:"<<0.1f*x/fabs(z)<<std::endl;
					}else{
						std::cout<<"Arrived!"<<std::endl;
					}
				}
			}
			*/
		}
	}
}

DWORD WINAPI Fun(LPVOID lpParamter)
{    
	while(1) {
		if (NavgateState)
		{
			if(fabs(result.pitch)>0.2f){
				if(result.pitch>0.2f){
					moveTo(0,0,-0.2f);
					fprintf(stdout,"%f\n",result.pitch);
				}
				if (result.pitch<-0.2f)
				{
					moveTo(0,0,0.2f);
					fprintf(stdout,"%f\n",result.pitch);
				}
			}else{
				if(pt3d_clicked.x != 0.0f || pt3d_clicked.y != 0.0f || pt3d_clicked.z != 0.0f){
					float x = pt3d_clicked.x-result.x;
					float y = pt3d_clicked.y-result.y;
					float z = pt3d_clicked.z-result.z-0.15;
					if(!isArrived(x,y,z)){
						if(fabs(z)>0.1f){
							moveTo(0.1f*z/fabs(z),0.1f*x/fabs(z),0);
						}else{
							moveTo(0.1f*z/fabs(x),0.1f*x/fabs(x),0);
						}
						std::cout<<"moveTo x:"<<0.1f*z/fabs(z)<<" y:"<<0.1f*x/fabs(z)<<std::endl;
					}else{
						rest();
						std::cout<<"Arrived!"<<std::endl;
					}
				}
			}
		}
	}
}


int main(int argc,char *argv[]){
	HANDLE hThread = CreateThread(NULL, 0, Fun, NULL, 0, NULL);
	CloseHandle(hThread);
	//kinect
	KinectGeneral *kinect = new KinectGeneral(NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
	kinect->KinectInit();
	//画像

	IplImage *dcolorImg = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,3);
	//点群
	pcl::PointCloud<RefPointType>::Ptr		cloud       (new pcl::PointCloud<RefPointType>);
	pcl::PointCloud<RefPointType>::Ptr		sampled_cloud (new pcl::PointCloud<RefPointType>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr		plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<RefPointType>::Ptr reference_cloud (new pcl::PointCloud<RefPointType>);

	Eigen::Vector4f							plane_coefficients;
	Eigen::Vector3f							point_of_interest(0.0f,0.0f,1.0f);

	pcl::PointCloud<RefPointType>::Ptr		cloud_RT   (new pcl::PointCloud<RefPointType>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr		nao_cloud   (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<RefPointType>::Ptr		nao_cloud_RT   (new pcl::PointCloud<RefPointType>);

	//Tracker
	PFT_Wrap tracker;

	//MapDraw
	MapBuilderForNaoTracker mapbuilder;
	IplImage *map = cvCreateImage(cvSize(MAP_SIZE,MAP_SIZE),IPL_DEPTH_8U,3);
	mapbuilder.getMap(map);

	//表示モジュール
	Dispper3D dispper;

	//前処理
	readModelData(*reference_cloud,"nao_upbody.txt");

	tracker.init();
	tracker.set_model(reference_cloud);
	tracker.set_Trans(point_of_interest);
	//PFT_Manager tracker;

	//tracker.init();
	//tracker.setInitPose(cv::Point3f(0.0,0.0,0.6));

	cvNamedWindow("dcolorImg");
	cvNamedWindow("main");
	cvShowImage("main",map);

	cvSetMouseCallback( "dcolorImg", onMouse, 0 );
	//繰り返し処理
	while(true){
		//Kinect 
		kinect->CheckCameraInput(colorImg,depthImg);
		kinect->GetDepthColor(dcolorImg);

		int key = cvWaitKey(10);
		if(key == 'q'){
			break;
		}else if(key == 'p'){
			//平面検出
			detectPlane(cloud,plane_coefficients,plane_cloud);
			//float pc[4];
			//for(int i=0;i<4;i++)
			//	pc[i] = plane_coefficients[i];

			//tracker.setPlaneCoefficient(pc,sizeof(pc)/sizeof(float));
		}else if(key == 't'){
			NavgateState = true;
		}else if(key == 's'){
			NavgateState = false;
		}
		cv::Point3f particle_pos;
		if(dispper.getClick(particle_pos)){
			//tracker.setInitPose(particle_pos);

			Eigen::Vector3f eigen_point = Eigen::Vector3f(particle_pos.x,particle_pos.y,particle_pos.z);
			tracker.init();
			tracker.set_model(reference_cloud);
			tracker.set_Trans(eigen_point);

		}

		//距離画像を点群に変換
		KconvertPCLPoint(depthImg,cloud,dcolorImg);

		//グリッドサンプリング
		gridSampleApprox(cloud,*sampled_cloud,0.0075);

		//平面の情報を使って回転とかほげほげ
		Test(sampled_cloud,plane_coefficients,plane_cloud,cloud_RT,nao_cloud);

		//if(key == 'r'){
		if(cloud_RT->points.size()>0 ){
			result = tracker.track(cloud_RT);
			tracker.get_ResultPointsCloud(nao_cloud_RT);
		}

		//表示用に点群を変換
		Obj3D_Point scene_cloud,robot_cloud;
		scene_cloud.import(cloud_RT);
		//scene_cloud.import(sampled_cloud);
		robot_cloud.import(nao_cloud_RT,false);

		//マップに結果を投げる
		mapbuilder.setRobotPos(cv::Point2f( -result.x, result.z), result.pitch / CV_PI * 180.0);
		mapbuilder.getMap(map);

		//表示
		dispper.updateObj("scene",&scene_cloud);
		dispper.updateObj("robot",&robot_cloud);

		cvShowImage("main",map);

		/***********************modify*****************/
		//draw the 
		INT index = pt_clicked.y * dcolorImg->width + pt_clicked.x;
		//dcolorImg.imageData[index]
		cvCircle(dcolorImg,pt_clicked,2,cvScalar(0,0,255),-1,CV_AA,0);
		std::stringstream ss;
		ss<<"("<<pt3d_clicked.x<<","<<pt3d_clicked.y<<","<<pt3d_clicked.z<<")";
		const std::string& tmp = ss.str();
		const char* cstr = tmp.c_str();
		//show information
		CvFont font;  
		cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5f, 0.5f, 0, 1, 8);
		cvPutText(dcolorImg,cstr,cvPoint(0,20),&font,cvScalar(255));
		/***********************modify*****************/
		cvShowImage("dcolorImg",dcolorImg);
	}

	delete kinect;

	return 0;
}

bool available(CvPoint p1, CvPoint p2){
	float answer = sqrt(pow((p2.x-p1.x),2.0) + pow((p2.y-p1.y),2.0));
	if (answer < 40){
		return true;
	}else{
		return false;
	}
}

/***********************modify*****************/
static void onMouse( int event, int x, int y, int, void* )
{
	if( event != CV_EVENT_LBUTTONDOWN ){
		return;
	}else{
		cv::Mat colorImg2 = cv::cvarrToMat(colorImg);
		cv::imwrite( "demo2.jpg", colorImg2 );
		

		IplImage *src = cvLoadImage("demo2.jpg",1);
		IplImage *temp = cvLoadImage("demo1.png",1);
		CvPoint temppoint = colorHistMatch(src,temp);

		if(isAvailable){
			pt_clicked = temppoint;
		}else{
			CvPoint temppoint2 = sift();
			if(isAvailable2){
				if(available(temppoint,temppoint2))
				{
					pt_clicked = temppoint;
				}else{
					pt_clicked = temppoint2;
				}
			}else{
				pt_clicked = temppoint;
			}
		}

		pt3d_clicked = getClickedPoint(depthImg,pt_clicked);
		std::cout<<">> ("<<pt3d_clicked.x<<","<<pt3d_clicked.y<<","<<pt3d_clicked.z<<")"<<std::endl;
	}
}
/***********************modify*****************/