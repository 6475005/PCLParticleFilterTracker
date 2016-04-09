#include "KinectGeneral.h"

namespace kinect_common{
	void KConvertSetImg3Channel(IplImage *VImg,IplImage *DImg,IplImage *VideoImg,IplImage *DepthImg){	
		//Convert Color
		if(VImg!=NULL && VideoImg != NULL){
			cvCvtColor( VImg,VideoImg, CV_BGR2HSV );
			cvCvtColor( VideoImg,VideoImg, CV_HSV2BGR );
		}
		//Convert Depth
		if(DImg!=NULL && DepthImg != NULL){
			cvSetZero(DepthImg);
			for(int i=0;i<DImg->width;i++){
				for(int j=0;j<DImg->height;j++){
					unsigned char depth_lower = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 0];
					unsigned char depth_upper = (unsigned char)DImg->imageData[DImg->widthStep * j + i * 2 + 1];
					unsigned int realdepth = depth_upper << 5 |depth_lower >> 3;
			
					if(realdepth>40000 || realdepth<400){
						continue;
					}else{
						DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 0] = depth_lower & 0x0007;
						DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 1] = (realdepth & 0x000000ff);
						DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 2] = (realdepth & 0x0000ff00) >> 8;
					}
				}
			}
		}
	}
	void KConvertDepthImg1ChannelHue(IplImage *DImg,IplImage *DepthImg){	
		//Convert Depth
		cvSetZero(DepthImg);

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
					DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 0] = (unsigned char)(((double)realdepth-400)/3600.0 *180);
					DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 1] = (unsigned char)255;
					DepthImg->imageData[DepthImg->widthStep * j + i * 3 + 2] = (unsigned char)255;
				}
			}
		}
		cvCvtColor( DepthImg,DepthImg, CV_HSV2BGR );
	}
	void KSaveImgSet(IplImage *VImg,IplImage *DImg,IplImage *RImg){
		cvSaveImage("CapturedCameraImage.bmp",VImg);
		cvSaveImage("CapturedDepthImage.bmp",DImg);
		cvSaveImage("Camera2DepthImage.bmp",RImg);
	}
}

int KinectGeneral::KinectInit(){
	HRESULT hr = m_KinectSensor.Init(m_depthType, m_depthRes, m_bNearMode, m_bFallbackToDefault, m_colorType, m_colorRes, m_bSeatedSkeletonMode);
	if (SUCCEEDED(hr)){
		m_KinectSensorPresent = TRUE;
		State = Kinect_State_RUNNING;
		fprintf(stderr,"kinect general : open succeded.\n");
	}
	else{
		m_KinectSensorPresent = FALSE;
		State = Kinect_State_UNINITED;
		WCHAR errorText[MAX_PATH];
		ZeroMemory(errorText, sizeof(WCHAR) * MAX_PATH);
		fprintf(stderr,"kinect general : open failed.\n");
		return 1;
	}
	return 0;
}

void KinectGeneral::CheckCameraInput(IplImage* videoImg,IplImage* depthImg)
{
	if (m_KinectSensorPresent){
		m_KinectSensor.GetVideoBuffer(videoImg);
		m_KinectSensor.GetDepthBuffer(depthImg);
	}
}

void KinectGeneral::GetRefImage(IplImage* refImg)
{
 	if (m_KinectSensorPresent){
		m_KinectSensor.GetRefBuffer(refImg);
	}
}

void KinectGeneral::GetDepthColor(IplImage* retImg)
{
 	if (m_KinectSensorPresent){
		m_KinectSensor.GetDepthColor(retImg);
	}
}