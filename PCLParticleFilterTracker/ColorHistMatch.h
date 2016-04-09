#include <iostream>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
using namespace std;

int GIRDSIZE = 20;
int a2[100][100];
int max_x2,max_y2;
int sum = 0;

CvHistogram* GetHist2D_HS(IplImage* src,int h_bins, int s_bins, float h_r = 180, float s_r = 255)
{
	//首先需要做好把 图像转换为 HSV空间的准备
	IplImage* hsv = cvCreateImage( cvGetSize(src), 8, 3 );
	IplImage* h_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };

	int hist_size[] = {h_bins, s_bins};
	/** H 分量的变化范围 */
	float h_ranges[] = { 0, h_r }; 
	/** S 分量的变化范围*/
	float s_ranges[] = { 0, s_r };
	float* ranges[] = { h_ranges, s_ranges };

	/** 输入图像转换到HSV颜色空间 */
	cvCvtColor( src, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );

	/** 创建直方图，二维, 每个维度上均分 */
	CvHistogram * hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	/** 根据H,S两个平面数据统计直方图 */
	cvCalcHist( planes, hist, 0, 0 );
	//cvNormalizeHist(hist, 1.0);

	return hist;
}

void BackProject2D(IplImage* src,	IplImage* back_project, CvHistogram* hist)
{
	IplImage* hsv = cvCreateImage( cvGetSize(src), 8, 3 );
	IplImage* h_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };

	/** 输入图像转换到HSV颜色空间 */
	cvCvtColor( src, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );

	//cvNormalizeHist(hist,1.0);
	/** 计算BackProject **/
	cvCalcBackProject(planes,back_project,hist);
}

void max_gird(){
	int temp = 0;
	for(int i=0; i < 100 ; i++){
		for(int j=0; j < 100; j++){
			if(a2[i][j] > temp){
				temp = a2[i][j];
				max_x2 = i;
				max_y2 = j;
			}
		}
	}
	max_x2 = max_x2 + 0.5;
	max_y2 = max_y2 + 0.5; 
}

void reset(){
	for(int i=0; i < 100 ; i++){
		for(int j=0; j < 100; j++){
			a2[i][j] = 0;
		}
	}
}

CvPoint colorFilter(IplImage *inputImage)
{  
	if (sum != 0)
		sum = 0;
	CvPoint pt_clicked;
	int x = inputImage->height;
	int y = inputImage->width;
	for(int i = 0; i < x; i++)
	{
		uchar* ptr = (uchar*) (inputImage->imageData + i * inputImage->widthStep);
		for( int j=0; j < y; j++ )
		{
			int color = (int)ptr[j];
			if (color < 250){
				((uchar *)(inputImage->imageData + i*inputImage->widthStep))[j]=0;
			}else{
				sum++;
				for(int k=0; k < x; k++){
					if(i >= k*GIRDSIZE && i <=  (k+1) * GIRDSIZE){
						for(int n=0; n < y; n++){
							if(j >= n * GIRDSIZE && j <=  (n+1) * GIRDSIZE)
								a2[k][n]++;
						}
					}
				}
			}
		}
	}
	max_gird();
	cout<<endl;
	cout<<"sum->"<<sum<<endl;
	cout<<max_x2 * GIRDSIZE<<","<<max_y2 * GIRDSIZE<<"->"<<a2[max_x2][max_y2]<<endl;

	IplImage* aaa = cvCreateImage(cvSize(inputImage->width,inputImage->height),8,1);
	for(int i = 0; i < x; ++i)
	{
		uchar* ptr = (uchar*) (aaa->imageData + i * aaa->widthStep);
		for( int j=0; j < y; j++ )
		{
			((uchar *)(aaa->imageData + i*aaa->widthStep))[j]=0;
		}
	}
	((uchar *)(aaa->imageData + (max_x2)* GIRDSIZE *aaa->widthStep))[(max_y2)* GIRDSIZE]=255;

	cvNamedWindow("filter2");  
	cvShowImage("filter2", aaa);

	cvNamedWindow("filter");  
	cvShowImage("filter", inputImage);

	pt_clicked.x = max_y2 * GIRDSIZE/2;
	pt_clicked.y = max_x2 * GIRDSIZE/2;

	return pt_clicked;
}

bool isAvailable(){
	if (sum <= 1000 && a2[max_x2][max_y2]/sum >= 0.2)
	{
		return true;
	}else{
		return false;
	}
}

CvPoint colorHistMatch(IplImage *src, IplImage *templ)
{
	reset();
	CvPoint pt_clicked;
	IplImage* back_project = cvCreateImage(cvSize(src->width,src->height),8,1);

	//需要求解模板图像的直方图
	int h_bins = 60;
	int s_bins = 40;
	int scale  = 10;
	CvHistogram* hist = GetHist2D_HS(templ,h_bins,s_bins);

	BackProject2D(src,back_project,hist);
	cvNamedWindow("Back_Projection",1);
	cvShowImage("Back_Projection", back_project);
	pt_clicked = colorFilter(back_project);

	return pt_clicked;
}