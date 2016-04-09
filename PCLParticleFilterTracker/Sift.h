#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

//#define USE_SURF
int GIRDSIZE2 = 20;
int a[100][100];
int max_x,max_y;
cv::Mat colorImg2;
int sum2 = 0;

void gird_model(cv::Point2f kpt){
	int x = (int)colorImg2.cols/GIRDSIZE2;
	int y = (int)colorImg2.rows/GIRDSIZE2;
	for(int i=0; i < x; i++){
		if(kpt.x >= i*GIRDSIZE2 && kpt.x <=  (i+1) * GIRDSIZE2){
			for(int j=0; j < y; j++){
				if(kpt.y >= j * GIRDSIZE2 && kpt.y <=  (j+1) * GIRDSIZE2){
					a[i][j]++;
				}
			}
		}
	}
}

void max_gird1(){
	int temp = 0;
	for(int i=0; i < 100 ; i++){
		for(int j=0; j < 100; j++){
			if(a[i][j] > temp){
				temp = a[i][j];
				max_x = i;
				max_y = j;
			}
		}
	}
}

void reset1(){
	for(int i=0; i < 100 ; i++){
		for(int j=0; j < 100; j++){
				a[i][j] = 0;
		}
	}
}

bool isAvailable2(){
	if (sum2 >50 && a[max_x][max_y]/sum2 >= 0.4)
	{
		return true;
	}else{
		return false;
	}
}

CvPoint sift()
{
	reset1();
	CvPoint pt_clicked;
	//画像読み込み
	cv::Mat colorImg1 = cv::imread("demo3.png");
	colorImg2 = cv::imread("demo2.jpg");

	if(colorImg1.empty() || colorImg2.empty()){
		std::cout << "No Image" << std::endl;
	}

	//特徴点抽出用のグレー画像用意
	cv::Mat grayImg1, grayImg2;
	cv::cvtColor(colorImg1, grayImg1, CV_BGR2GRAY);
	cv::normalize(grayImg1, grayImg1, 0, 255, cv::NORM_MINMAX);
	cv::cvtColor(colorImg2, grayImg2, CV_BGR2GRAY);
	cv::normalize(grayImg2, grayImg2, 0, 255, cv::NORM_MINMAX);

#ifdef USE_SURF
	//SURF
	cv::SurfFeatureDetector detector(1000);
	cv::SurfDescriptorExtractor extractor;
#else
	//SIFT
	cv::SiftFeatureDetector detector;
	cv::SiftDescriptorExtractor extractor;    
#endif

	//画像から特徴点を検出
	std::vector<cv::KeyPoint> keypoints1;
	detector.detect(grayImg1, keypoints1);
	std::vector<cv::KeyPoint> keypoints2;
	detector.detect(grayImg2, keypoints2);

	//画像の特徴点における特徴量を抽出
	cv::Mat descriptors1;    
	extractor.compute(grayImg1, keypoints1, descriptors1);    
	cv::Mat descriptors2;    
	extractor.compute(grayImg2, keypoints2, descriptors2);

	//特徴点の対応付け
	std::vector<cv::DMatch> matches;
	cv::BruteForceMatcher<cv::L2<float> > matcher;
	matcher.match(descriptors1, descriptors2, matches);

	sum2 = matches.size();
	//std::vector<cv::Point2f> queryPoints(matches.size());    
	std::vector<cv::Point2f> trainPoints(matches.size());
	for (size_t i = 0; i < matches.size(); i++)    
	{    
		//queryPoints[i] = keypoints1[matches[i].queryIdx].pt;    
		trainPoints[i] = keypoints2[matches[i].trainIdx].pt;    
	}

	for(int i=0;i<trainPoints.size();i++){
		gird_model(trainPoints[i]);
	}
	max_gird1();
	for(int i=0;i<trainPoints.size();i++){
		//if(trainPoints[i].y >= max_y * GIRDSIZE2 && trainPoints[i].y <=  (max_y+1) * GIRDSIZE2){
		//	if(trainPoints[i].x >= max_x * GIRDSIZE2 && trainPoints[i].x <=  (max_x+1) * GIRDSIZE2){
				cv::circle(colorImg2,trainPoints[i],keypoints2[i].size,CV_RGB(255,0,0));
				cv::line(colorImg2,trainPoints[i],trainPoints[i],CV_RGB(255,0,0));
		//	}
		//}
	}

	std::cout <<"-------------"<< std::endl;
	std::cout <<"("<<max_x<<","<<max_y<<")"<<a[max_x][max_y]<< std::endl;
	std::cout <<"("<<(max_x+1) * GIRDSIZE2<<","<<(max_y+1) * GIRDSIZE2<<")"<< std::endl;

	cv::namedWindow("sift_result2", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("sift_result2", colorImg2);

	pt_clicked.x = (max_x+1) * GIRDSIZE2/2;
	pt_clicked.y = (max_y+1) * GIRDSIZE2/2;

	return pt_clicked;
}