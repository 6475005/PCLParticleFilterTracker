//------------------------------------------------------------------------------
// <copyright file="KinectSensor.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

//#include "StdAfx.h"
#include "KinectSensor.h"
#include <math.h>
KinectSensor::KinectSensor()
{
    m_hNextDepthFrameEvent = NULL;
    m_hNextVideoFrameEvent = NULL;
    m_hNextSkeletonEvent = NULL;
    m_pDepthStreamHandle = NULL;
    m_pVideoStreamHandle = NULL;
    m_hThNuiProcess=NULL;
    m_hEvNuiProcessStop=NULL;
    m_bNuiInitialized = false;
    m_FramesTotal = 0;
    m_SkeletonTotal = 0;
    m_VideoBuffer = NULL;
    m_DepthBuffer = NULL;
}

KinectSensor::~KinectSensor()
{
    Release();
}

HRESULT KinectSensor::Init(NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, BOOL bFallbackToDefault, NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes, BOOL bSeatedSkeletonMode)
{

    HRESULT hr = E_UNEXPECTED;

    Release(); // Deal with double initializations.

    //do not support NUI_IMAGE_TYPE_COLOR_RAW_YUV for now
    if(colorType != NUI_IMAGE_TYPE_COLOR && colorType != NUI_IMAGE_TYPE_COLOR_YUV
        || depthType != NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX && depthType != NUI_IMAGE_TYPE_DEPTH){
        return E_INVALIDARG;
    }

    //Setting Size
	this->color_reso = colorRes;
	this->depth_reso = depthRes;
    DWORD width = 0;
    DWORD height = 0;

    NuiImageResolutionToSize(colorRes, width, height);
	m_VideoBuffer = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 4);
	ref_DepthBuffer = cvCreateImage(cvSize(width,height),IPL_DEPTH_16U, 1);

	tempPoints.resize(width);
	for(int w=0;w<width;w++)
		tempPoints[w].resize(height);

    NuiImageResolutionToSize(depthRes, width, height);
	m_DepthBuffer = cvCreateImage(cvSize(width,height),IPL_DEPTH_16U, 1);
    
    m_FramesTotal = 0;
    m_SkeletonTotal = 0;

    for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
        m_SkeletonTracked[i] = false;
    }

	//first kinect setting
	INuiSensor * pNuiSensor = NULL;
	m_pNuiSensor = NULL;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr) ) { return hr; }
	// Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i){
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr)){
            continue;
        }
        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr){
            m_pNuiSensor = pNuiSensor;
            break;
        }
        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }
    if (NULL == m_pNuiSensor){return E_FAIL;}

    m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_hNextVideoFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_hNextSkeletonEvent   = CreateEvent(NULL, TRUE, FALSE, NULL);
    
	//Init Depth
    DWORD dwNuiInitDepthFlag = (depthType == NUI_IMAGE_TYPE_DEPTH)? NUI_INITIALIZE_FLAG_USES_DEPTH : NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
	// Initialize the Kinect and specify that we'll be using depth
    hr = m_pNuiSensor->NuiInitialize(dwNuiInitDepthFlag | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);
	if (FAILED(hr) ) { return hr; }
    m_bNuiInitialized = true;

	//set iamge size
	::NuiImageResolutionToSize( colorRes, xc, yc );
	::NuiImageResolutionToSize( depthRes, xd, yd );

	//Select mode default or near
	DWORD dwSkeletonFlags = NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
	if (bSeatedSkeletonMode){
		dwSkeletonFlags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
	}

	//Enable Skeleton
    hr =  m_pNuiSensor->NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, dwSkeletonFlags );
    if (FAILED(hr)){return hr;}
	
	//Setting Video Stream
    hr =  m_pNuiSensor->NuiImageStreamOpen(colorType,colorRes,0,2,m_hNextVideoFrameEvent,&m_pVideoStreamHandle );
    if (FAILED(hr)){return hr;}

	//Setting Depth Stream
    hr =  m_pNuiSensor->NuiImageStreamOpen(
        depthType,depthRes,(bNearMode)? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0,2,m_hNextDepthFrameEvent,&m_pDepthStreamHandle );
    if (FAILED(hr)){
        if(bNearMode && bFallbackToDefault){
            hr =  m_pNuiSensor->NuiImageStreamOpen(depthType,depthRes,0,2,
                m_hNextDepthFrameEvent,&m_pDepthStreamHandle );
        }

        if(FAILED(hr)){return hr;}
    }

	//init array
	m_depthD16 = new USHORT[xd*yd*2];
    m_colorCoordinates = new LONG[xc*yc*2];
  
    // Start the Nui processing thread
    m_hEvNuiProcessStop  = CreateEvent(NULL,TRUE,FALSE,NULL);
    m_hThNuiProcess		 = CreateThread(NULL,0,ProcessThread,this,0,NULL);

    return hr;
}

void KinectSensor::Release()
{
    // Stop the Nui processing thread
    if(m_hEvNuiProcessStop!=NULL)
    {
        // Signal the thread
        SetEvent(m_hEvNuiProcessStop);

        // Wait for thread to stop
        if(m_hThNuiProcess!=NULL)
        {
            WaitForSingleObject(m_hThNuiProcess,INFINITE);
            CloseHandle(m_hThNuiProcess);
            m_hThNuiProcess = NULL;
        }
        CloseHandle(m_hEvNuiProcessStop);
        m_hEvNuiProcessStop = NULL;
    }

	if (m_bNuiInitialized && NULL != m_pNuiSensor)
    {
        m_pNuiSensor->NuiShutdown();
        m_pNuiSensor->Release();
    }
    m_bNuiInitialized = false;
    if (m_hNextSkeletonEvent && m_hNextSkeletonEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextSkeletonEvent);
        m_hNextSkeletonEvent = NULL;
    }
    if (m_hNextDepthFrameEvent && m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
        m_hNextDepthFrameEvent = NULL;
    }
    if (m_hNextVideoFrameEvent && m_hNextVideoFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextVideoFrameEvent);
        m_hNextVideoFrameEvent = NULL;
    }
    if (m_VideoBuffer)
    {
		cvReleaseImage(&m_VideoBuffer);
        m_VideoBuffer = NULL;
    }
    if (m_DepthBuffer)
    {
		cvReleaseImage(&m_DepthBuffer);
        m_DepthBuffer = NULL;
    }
}

DWORD WINAPI KinectSensor::ProcessThread(LPVOID pParam)
{
    KinectSensor*  pthis=(KinectSensor *) pParam;
    HANDLE          hEvents[4];

    // Configure events to be listened on
    hEvents[0]=pthis->m_hEvNuiProcessStop;
    hEvents[1]=pthis->m_hNextDepthFrameEvent;
    hEvents[2]=pthis->m_hNextVideoFrameEvent;
    hEvents[3]=pthis->m_hNextSkeletonEvent;

	//disp setting
	IplImage *SvideoImg,*SdepthImg;
	SvideoImg = cvCreateImage(cvGetSize(pthis->m_VideoBuffer),pthis->m_VideoBuffer->depth,pthis->m_VideoBuffer->nChannels);
	SdepthImg = cvCreateImage(cvGetSize(pthis->m_DepthBuffer),pthis->m_DepthBuffer->depth,pthis->m_DepthBuffer->nChannels);
	char* windowName = "kinect_camera_image";
    char* window2Name = "kinect_depth_image";
	cvNamedWindow( windowName );
	cvNamedWindow( window2Name );
    // Main thread loop
    while (true)
    {
        // Wait for an event to be signaled
        WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);

        // If the stop event is set, stop looping and exit
        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->m_hEvNuiProcessStop, 0))
        {
            break;
        }

        // Process signal events
		if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->m_hNextVideoFrameEvent, 0))
        {
            pthis->GotVideoAlert();
        }
        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->m_hNextDepthFrameEvent, 0))
        {
            pthis->GotDepthAlert();
            pthis->m_FramesTotal++;
        }
        if (WAIT_OBJECT_0 == WaitForSingleObject(pthis->m_hNextSkeletonEvent, 0))
        {
            pthis->GotSkeletonAlert();
            pthis->m_SkeletonTotal++;
        }

		// draw joint circle
		for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ ){
			CvPoint *skeleton = pthis->body_points[i];
			if(skeleton != NULL){
				for(int j=0; j< NUI_SKELETON_POSITION_COUNT; j++)
					cvCircle( SvideoImg, skeleton[j], 9, CV_RGB ( 255,255,0 ),1,8, 0 );
			}else{
				//printf("skeleton %d not found.\n",i);
			}
		}

		cvCopy(pthis->m_VideoBuffer,SvideoImg);
		cvCopy(pthis->m_DepthBuffer,SdepthImg);

		//show images
		cvShowImage( windowName,  SvideoImg);
		cvShowImage( window2Name, SdepthImg);

		cvWaitKey(50);
		
    }

	cvDestroyWindow(windowName);
	cvDestroyWindow(window2Name);
	cvReleaseImage(&SvideoImg);
	cvReleaseImage(&SdepthImg);
	fprintf(stderr,"end kinect Sensor process.\n");

    return 0;
}

HRESULT KinectSensor::GotVideoAlert()
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 0, &imageFrame);
    if ( FAILED(hr) ) { return hr; }
  
    NUI_LOCKED_RECT LockedRect;
    hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
    if ( FAILED(hr) ) { return hr; }

    memcpy(m_VideoBuffer->imageData, LockedRect.pBits, LockedRect.size);
   
    hr = imageFrame.pFrameTexture->UnlockRect(0);
    if ( FAILED(hr) ) { return hr; };

    hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pVideoStreamHandle, &imageFrame);
    
    return hr;
}

HRESULT KinectSensor::GotDepthAlert()
{
	NUI_IMAGE_FRAME imageFrame;

    HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
    if ( FAILED(hr) ) { return hr; }
   
    NUI_LOCKED_RECT LockedRect;
    hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
    if ( FAILED(hr) ) { return hr; }

	memcpy(m_depthD16, LockedRect.pBits, LockedRect.size);
    memcpy(m_DepthBuffer->imageData, LockedRect.pBits, LockedRect.size);

    hr = imageFrame.pFrameTexture->UnlockRect(0);
    if ( FAILED(hr) ) { return hr; };

    hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	return hr;
}

void KinectSensor::GetColorPixelCoordinatesFromDepthPixel(int x,int y,int *cx,int *cy){
	LONG color_x, color_y;
	::NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(this->color_reso,this->depth_reso,NULL,(LONG)x,(LONG)y,0,&color_x,&color_y);
	*cx = color_x;
	*cy = color_y;
}

void KinectSensor::GetRefBuffer(IplImage *img){
	
	CvPoint topLeft, bottomRight;
	unsigned int idou_size = 3;
	int temp_sign[4] = {-1,-1,1,1};
	
	cvZero(this->ref_DepthBuffer);
	
	m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		this->color_reso,
       this->depth_reso,
	   this->xd * this->yd,
       m_depthD16,
       this->xd * this->yd*2,
       m_colorCoordinates);

	//Ese NN
	for(unsigned int i=0;i< this->xc;i++){
		for(unsigned int j=0;j< this->yc;j++){
			tempPoints[i][j].x = tempPoints[i][j].y = -1;
		}
	}

	for(int x=0;(unsigned int)x<xc;x++){
		for(int y=0;(unsigned int)y<yc;y++){
			LONG m_colorToDepthDivisor = xc/xd;
			int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * xd;
			//int depthIndex = x + y * xc;
			LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
            LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];

			tempPoints[colorInDepthX][colorInDepthY].x = x;
			tempPoints[colorInDepthX][colorInDepthY].y = y;

			//printf("%d%d->%d%d\n",x,y,colorInDepthX,colorInDepthY);

			if(x==0 && y==0){
				topLeft.x = colorInDepthX; topLeft.y = colorInDepthY;
			}else if(x== xd-1 && y==yd-1){
				bottomRight.x = colorInDepthX; bottomRight.y = colorInDepthY;
			}
		}
	}

	for(int x=0;(unsigned int)x<xc;x++){
		for(int y=0;(unsigned int)y<yc;y++){
			if(x<topLeft.x || x>bottomRight.x || y<topLeft.y || y>bottomRight.y){continue;}
			for(int idou=0;idou <= idou_size;idou++){
				bool breakflag = false;
				for(int x_idou=0;x_idou<=(idou/2);x_idou++){
					int loop_size = sizeof(temp_sign)/sizeof(int);
					for(int s=0;s<loop_size;s++){
						int ref_x = x + temp_sign[s%loop_size] * x_idou;
						int ref_y = y + temp_sign[(s+1)%loop_size] * (idou-x_idou);

						if(ref_x>topLeft.x && ref_x<bottomRight.x && ref_y>topLeft.y && ref_y<bottomRight.y){
							if(this->tempPoints[ref_x][ref_y].x != -1){
								this->ref_DepthBuffer->imageData[ref_DepthBuffer->widthStep*y + x*2] 
									= this->m_DepthBuffer->imageData[m_DepthBuffer->widthStep * tempPoints[ref_x][ref_y].y + tempPoints[ref_x][ref_y].x*2 +0];
								this->ref_DepthBuffer->imageData[ref_DepthBuffer->widthStep*y + x*2 +1] 
									= this->m_DepthBuffer->imageData[m_DepthBuffer->widthStep * tempPoints[ref_x][ref_y].y + tempPoints[ref_x][ref_y].x*2 +1];
								breakflag = true;
								break;
							}
						}
					}
					if(breakflag){break;}
				}
				if(breakflag){break;}
			}//end of find loop
		}
	}
	cvCopy(ref_DepthBuffer,img);
}

HRESULT KinectSensor::GotSkeletonAlert()
{
    NUI_SKELETON_FRAME SkeletonFrame = {0};

    HRESULT hr = NuiSkeletonGetNextFrame(0, &SkeletonFrame);
    if(FAILED(hr)){
        return hr;
    }

    for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ ){
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED ){
			m_SkeletonTracked[i] = true;
			for(int j=0;j< NUI_SKELETON_POSITION_COUNT;j++){
				float a,b;
				int c,d;
				NuiTransformSkeletonToDepthImage( SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &a, &b);
				this->body_points[i][j].x = (int)(a * xd) / 320;
				this->body_points[i][j].y = (int)(b * yd) / 240;
		
				this->GetColorPixelCoordinatesFromDepthPixel(body_points[i][j].x,body_points[i][j].y,&c,&d);
				this->body_points[i][j].x = c;
				this->body_points[i][j].y = d;

				NUI_SKELETON_DATA data = SkeletonFrame.SkeletonData[i];

				this->body_3d_points[i][j].x = data.SkeletonPositions[j].x;
				this->body_3d_points[i][j].y = data.SkeletonPositions[j].y;
				this->body_3d_points[i][j].z = data.SkeletonPositions[j].z;
			}
		}else if(SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_POSITION_ONLY){
				m_SkeletonTracked[i] = true;
				for(int j=0;j< NUI_SKELETON_POSITION_COUNT;j++){
				float a,b;
				int c,d;
				NuiTransformSkeletonToDepthImage( SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &a, &b);
				body_points[i][j].x = (int)(a * xd) / 320;
				body_points[i][j].y = (int)(b * yd) / 240;
				
				this->GetColorPixelCoordinatesFromDepthPixel(body_points[i][j].x,body_points[i][j].y,&c,&d);
				body_points[i][j].x = c;
				body_points[i][j].y = d;

				NUI_SKELETON_DATA data = SkeletonFrame.SkeletonData[i];

				this->body_3d_points[i][j].x = data.SkeletonPositions[j].x;
				this->body_3d_points[i][j].y = data.SkeletonPositions[j].y;
				this->body_3d_points[i][j].z = data.SkeletonPositions[j].z;
			}
		}else{
            m_SkeletonTracked[i] = false;
        }
    }
	return hr;
}

void KinectSensor::GetDepthColor(IplImage *img)
{
	
	if(img->width != this->xd || img->height != this->yd ){
		fprintf(stderr,"GetDepthColor: size is not correct");
		return;
	}

	IplImage *color = cvCreateImage(cvSize(xc,yc),IPL_DEPTH_8U,3);
	
	//EnterCriticalSection(&video_cs);
	cvCvtColor( this->m_VideoBuffer,color, CV_BGR2HSV );
	//LeaveCriticalSection(&video_cs);
	cvCvtColor( color,color, CV_HSV2BGR );

	m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		this->color_reso,
		this->depth_reso,
		this->xd * this->yd,
		m_depthD16,
		this->xd * this->yd*2,
		m_colorCoordinates);

	for(int x=0;(unsigned int)x<xd;x++){
		for(int y=0;(unsigned int)y<yd;y++){
			int depthIndex = x + y * xd;
			LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
            LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];
			
			if(colorInDepthX<0 || colorInDepthX > this->xc){continue;}
			if(colorInDepthY<0 || colorInDepthY > this->yc){continue;}

			//printf("%d %d\n",colorInDepthX,colorInDepthY);
			img->imageData[img->widthStep*y+x*3+0] 
				= color->imageData[color->widthStep*colorInDepthY+colorInDepthX*3+0];
			img->imageData[img->widthStep*y+x*3+1] 
				= color->imageData[color->widthStep*colorInDepthY+colorInDepthX*3+1];
			img->imageData[img->widthStep*y+x*3+2] 
				= color->imageData[color->widthStep*colorInDepthY+colorInDepthX*3+2];
		}
	}

	cvReleaseImage(&color);

	return;
}