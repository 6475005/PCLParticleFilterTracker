//------------------------------------------------------------------------------
// <copyright file="KinectSensor.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
#ifndef __KINECT_HEADER__
#define __KINECT_HEADER__

#pragma once

#include <iostream>
#include <Windows.h>
#include <NuiApi.h>
#include <opencv2\opencv.hpp>

class KinectSensor
{
	public:
		KinectSensor();
		~KinectSensor();

		HRESULT Init(NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, BOOL bFallbackToDefault, NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes, BOOL bSeatedSkeletonMode);
		void Release();

		void   GetVideoBuffer(IplImage *dst){  cvCopy(m_VideoBuffer,dst); }
		void   GetDepthBuffer(IplImage *dst){  cvCopy(m_DepthBuffer,dst); }
		void   GetRefImgBuffer(IplImage *dst){ cvCopy(ref_DepthBuffer,dst); }

		bool        IsTracked(UINT skeletonId) { return(m_SkeletonTracked[skeletonId]);}
		void		GetColorPixelCoordinatesFromDepthPixel(int x,int y,int *cx,int *cy);
		CvPoint*	GetSkeletonPos(int id){
					if(this->IsTracked(id)){
						return this->body_points[id];
					}else{
						return NULL;
					}
				}
		void		GetRefBuffer(IplImage *img);
		void		GetDepthColor(IplImage *img);

	private:
		IplImage* m_VideoBuffer;
		IplImage* m_DepthBuffer;
		IplImage* ref_DepthBuffer;

		CvPoint pre_body_points[NUI_SKELETON_COUNT][20];
		CvPoint body_points[NUI_SKELETON_COUNT][20];
	
		cv::Point3f pre_3d_points[NUI_SKELETON_COUNT][20];
		cv::Point3f body_3d_points[NUI_SKELETON_COUNT][20];

		std::vector<std::vector<CvPoint>> tempPoints;
		
		bool        m_SkeletonTracked[NUI_SKELETON_COUNT];
	
		NUI_SKELETON_DATA *skeleton_data;
	
		//kinect
		INuiSensor*  m_pNuiSensor;
		HANDLE      m_hNextDepthFrameEvent;
		HANDLE      m_hNextVideoFrameEvent;
		HANDLE      m_hNextSkeletonEvent;
		HANDLE      m_pDepthStreamHandle;
		HANDLE      m_pVideoStreamHandle;
		HANDLE      m_pSkeletonHandle;
		HANDLE      m_hThNuiProcess;
		HANDLE      m_hEvNuiProcessStop;

		NUI_IMAGE_RESOLUTION color_reso,depth_reso;
		DWORD xc, yc;
		DWORD xd, yd;
		
		// for mapping depth to color
		USHORT*     m_depthD16;
		BYTE*       m_colorRGBX;
		LONG*       m_colorCoordinates;

		bool        m_bNuiInitialized; 
		int         m_FramesTotal;
		int         m_SkeletonTotal;
    
		static DWORD WINAPI ProcessThread(PVOID pParam);
		HRESULT GotVideoAlert();
		HRESULT GotDepthAlert();
		HRESULT GotSkeletonAlert();
};
#endif