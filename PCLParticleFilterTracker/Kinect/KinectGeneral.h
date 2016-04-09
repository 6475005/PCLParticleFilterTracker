#ifndef __KINECT_GENERAL__
#define __KINECT_GENERAL__

#pragma once

#define KINECT_TRACK_SEATED_MODE  TRUE
#define KINECT_TRACK_DEFAULT_MODE FALSE

#define KINECT_RANGE_NEAR_MODE  TRUE
#define KINECT_RANGE_DEFAULT_MODE FALSE

#define Kinect_State_UNINITED		0
#define Kinect_State_INITED			1
#define Kinect_State_RUNNING		2
#define Kinect_State_STOP			4

#include <opencv2/opencv.hpp>
#include "KinectSensor.h"


namespace kinect_common{
	void KConvertSetImg3Channel(IplImage *VImg,IplImage *DImg,IplImage *VideoImg,IplImage *DepthImg);
	void KConvertDepthImg1ChannelHue(IplImage *DImg,IplImage *DepthImg);
	void KSaveImgSet(IplImage *VImg,IplImage *DImg,IplImage *RImg);
}

class KinectGeneral{

	public:
		KinectGeneral(NUI_IMAGE_RESOLUTION color_reso = NUI_IMAGE_RESOLUTION_640x480,
					  NUI_IMAGE_RESOLUTION depth_reso = NUI_IMAGE_RESOLUTION_640x480,
					  bool range_setting = KINECT_RANGE_NEAR_MODE,
					  bool kinect_tracking_mode = KINECT_TRACK_SEATED_MODE){
			m_hWnd = NULL;
			m_ApplicationIsRunning = false;
			m_LastTrackSucceeded = false;
			m_XCenterFace = 0;
			m_YCenterFace = 0;
			m_DrawMask = TRUE;
			m_depthType = NUI_IMAGE_TYPE_DEPTH;
			m_depthRes = NUI_IMAGE_RESOLUTION_INVALID;
			m_bNearMode = range_setting;
			m_bFallbackToDefault = FALSE;
			m_colorType = NUI_IMAGE_TYPE_COLOR;
			m_colorRes = NUI_IMAGE_RESOLUTION_INVALID;

			//tmp
			m_depthType = NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX;
			m_depthRes  = depth_reso;
			m_colorType = NUI_IMAGE_TYPE_COLOR;
			m_colorRes  = color_reso;
			m_bNearMode = TRUE;

			m_bSeatedSkeletonMode = kinect_tracking_mode;
		};

		~KinectGeneral(){
			m_KinectSensor.Release();
		};

		int KinectInit();
		void CheckCameraInput(IplImage* videoImg,IplImage* depthImg);
		void GetRefImage(IplImage* refImg);
		void GetDepthColor(IplImage* depthcolorImg);
		
		CvPoint* GetSkeletonPos(int id){return this->m_KinectSensor.GetSkeletonPos(id);}

		bool Stop(){
			m_KinectSensor.Release();
			return true;
		}
		bool Start(){
			this->KinectInit();
			return true;
		}
		int TurnState(){
			if(State == Kinect_State_RUNNING){
				m_KinectSensor.Release();
				State = Kinect_State_STOP;
			}else if(State == Kinect_State_STOP){
				this->KinectInit();
				State = Kinect_State_RUNNING;
			}
			return State;
		}

	private:
		KinectSensor                m_KinectSensor;
		BOOL                        m_KinectSensorPresent;
		HANDLE						m_hEvKinectGeneralProcessStop;
		HWND                        m_hWnd;
		bool                        m_LastTrackSucceeded;
		bool                        m_ApplicationIsRunning;
		LPVOID                      m_CallBackParam;
		float                       m_XCenterFace;
		float                       m_YCenterFace;
		HANDLE                      m_hFaceTrackingThread;
		BOOL                        m_DrawMask;
		NUI_IMAGE_TYPE              m_depthType;
		NUI_IMAGE_RESOLUTION        m_depthRes;
		BOOL                        m_bNearMode;
		BOOL                        m_bFallbackToDefault;
		BOOL                        m_bSeatedSkeletonMode;
		NUI_IMAGE_TYPE              m_colorType;
		NUI_IMAGE_RESOLUTION        m_colorRes;
		int							State;
};

#endif