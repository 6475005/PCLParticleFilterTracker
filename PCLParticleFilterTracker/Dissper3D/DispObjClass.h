#ifndef __DISP_OBJ_CLASS__
#define __DISP_OBJ_CLASS__

#define LINE  0
#define POINT 1
#define FACE  2

int hsv2rgb( cv::Point3f *hsv,  cv::Point3f *rgb ) {
   float S, H, V, F, M, N, K;
   int   I;
   
   S = hsv->y;  /* Saturation */
   H = hsv->x;  /* Hue */
   V = hsv->z;  /* value or brightness */
   
   if ( S == 0.0 ) {
      rgb->x = V;
      rgb->y = V;
      rgb->z = V;
   } else {
      /* 
       * Determine levels of primary colours. 
       */
      if (H >= 1.0) {
         H = 0.0;
      } else {
         H = H * 6;
      } /* end if */
      I = (int) H;   /* should be in the range 0..5 */
      F = H - I;     /* fractional part */

      M = V * (1 - S);
      N = V * (1 - S * F);
      K = V * (1 - S * (1 - F));

      if (I == 0) { rgb->x = V; rgb->y = K; rgb->z = M; }
      if (I == 1) { rgb->x = N; rgb->y = V; rgb->z = M; }
      if (I == 2) { rgb->x = M; rgb->y = V; rgb->z = K; }
      if (I == 3) { rgb->x = M; rgb->y = N; rgb->z = V; }
      if (I == 4) { rgb->x = K; rgb->y = M; rgb->z = V; }
      if (I == 5) { rgb->x = V; rgb->y = M; rgb->z = N; }
   } /* end if */

   return 0;
}

struct Obj3D{
	int datatype; 
	float const_hue;
	bool color;
	std::vector<std::pair<cv::Point3f,cv::Point3f>> PointCloud;
	cv::Point3f CenterOfPointCloud;
};

void initObj3D(Obj3D &temp){
	temp.color = false;
	temp.datatype = -1;
	temp.CenterOfPointCloud = cv::Point3f(0.0f,0.0f,0.0f);
	temp.const_hue = -1.0;
	temp.PointCloud.clear();
}

class Obj3D_Disp{
	//ポイントクラウド表示とメッシュ表示用の仮想クラス
	//描画の仕方とかは使う人が決める

	friend class Dispper3D;

public :
	Obj3D_Disp(){
		init();
	}
	~Obj3D_Disp(){
		PointCloud.clear();
	}

	void init(void){
		const_hue	= 1.0;
		color		= false;
		CenterOfPointCloud	= cv::Point3f(0.0f,0.0f,0.0f);
		RotationAngles		= cv::Point3f(0.0f,0.0f,0.0f);
		TranslationVector	= cv::Point3f(0.0f,0.0f,0.0f);
		PointCloud.clear();
	}
	void setPointCloud(std::vector<std::pair<cv::Point3f,cv::Point3f>> _PointCloud){
		PointCloud = _PointCloud;
	}
	void update(const Obj3D_Disp* p){
		PointCloud.resize(p->PointCloud.size());
		for(unsigned int i=0;i<PointCloud.size();i++){
			PointCloud[i] = p->PointCloud[i];
		}
		CenterOfPointCloud = p->CenterOfPointCloud;
	}
	void setPose(const cv::Point3f _TranslationVector,cv::Point3f _RotationAngles){
		TranslationVector = _TranslationVector;
		RotationAngles = _RotationAngles;
		return;
	}

	cv::Point3f	getCenterPoint(){return CenterOfPointCloud;}

	//デザインする関数////////////////////
	virtual void draw(void) = 0;
	virtual Obj3D_Disp* clone() = 0;
	//////////////////////////////////////

protected :
	float												const_hue;
	bool												color;

	std::vector<std::pair<cv::Point3f,cv::Point3f>>		PointCloud;
	std::vector<cv::Point3f>							FacePointList;

	cv::Point3f											CenterOfPointCloud;
	cv::Point3f											RotationAngles;
	cv::Point3f											TranslationVector;
	//テクスチャへのポインタをついかする
};

class Obj3D_Point : public Obj3D_Disp{

protected:
	Obj3D_Point(const Obj3D_Point* that){
		this->PointCloud	= that->PointCloud;
		this->color			= that->color;
	}

public:
	Obj3D_Point():Obj3D_Disp(){}

	Obj3D_Disp* clone(){
		return new Obj3D_Point(this);
	}

	void draw(){
		cv::Point3f rgb,hsv;
		hsv = cv::Point3f(const_hue,1.0f,1.0f);
		hsv2rgb(&hsv,&rgb);

		for(unsigned int i=0;i< PointCloud.size();i++){
			if(color){
				glColor4f(
					PointCloud[i].second.x,
					PointCloud[i].second.y,
					PointCloud[i].second.z,
					1.0f);
			}else{
				glColor4f(rgb.x,rgb.y,rgb.z,1.0f);
			}
			glBegin(GL_POINTS);
			glVertex3f(PointCloud[i].first.x,PointCloud[i].first.y,PointCloud[i].first.z);
			glEnd();
		}
	}

	void import(std::vector<std::pair<cv::Point3f, cv::Point3d>> input){
		this->color = true;
		for(unsigned int i=0; i< input.size() ;i++){
			float r = ((float)input[i].second.x)/255.0f;
			float g = ((float)input[i].second.y)/255.0f;
			float b = ((float)input[i].second.z)/255.0f;
			this->PointCloud.push_back(std::make_pair( input[i].first , cv::Point3f(r,g,b) ));
		}
	}

	#ifdef PCL_POINT_CLOUD_H_
		void import(pcl::PointCloud< pcl::PointXYZRGB >::Ptr &input,bool bool_color = true){
			this->color = bool_color;
			for(unsigned int i=0; i< input->points.size() ;i++){
				float r = ((float)input->points[i].r)/255.0f;
				float g = ((float)input->points[i].g)/255.0f;
				float b = ((float)input->points[i].b)/255.0f;
				this->PointCloud.push_back(std::make_pair( 
					cv::Point3f(input->points[i].x,input->points[i].y,input->points[i].z),cv::Point3f(r,g,b)));
			}
		}
		void import(pcl::PointCloud< pcl::PointXYZ >::Ptr &input){
			this->color = false;
			for(unsigned int i=0; i< input->points.size() ;i++){		
				this->PointCloud.push_back(std::make_pair(cv::Point3f(input->points[i].x,input->points[i].y,input->points[i].z),cv::Point3f(1.0f,1.0f,1.0f)));
			}
		}
	#endif

};


void KconvertObj3D(IplImage *DImg, Obj3D &obj, IplImage *color = NULL, IplImage *mask = NULL ){

	if(DImg == NULL){return;}
	initObj3D(obj);

	const float f = 290.0;
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
				float Z = (float)realdepth/1000.0f;
				float X =  (((float)(i))-cx) * Z/f;
				float Y =  -1.0f *(((float)(j))-cy) * Z/f;
				if(color == NULL){
					obj.PointCloud.push_back(std::pair<cv::Point3f,cv::Point3f>
						(cv::Point3f(X,Y,Z),cv::Point3f(0,0,0)));
				}else{
					obj.color = true;
					float r = (unsigned char)color->imageData[color->widthStep * j + i*3+2];
					float g = (unsigned char)color->imageData[color->widthStep * j + i*3+1];
					float b = (unsigned char)color->imageData[color->widthStep * j + i*3+0];
					obj.PointCloud.push_back(std::pair<cv::Point3f,cv::Point3f>
						(cv::Point3f(X,Y,Z),cv::Point3f(r/255.0f,g/255.0f,b/255.0f)));
				}
			}
		}
	}
	//calc irigin
	cv::Point3d origin = cv::Point3f(0.0,0.0,0.0);
	unsigned int sample_num = ((100) > (obj.PointCloud.size()) ? (obj.PointCloud.size()):(100));

	for(unsigned int i=0;i<sample_num; i++){
		int index = rand() % obj.PointCloud.size();
		origin += (cv::Point3d) obj.PointCloud[index].first;
	}
	
	origin.x = origin.x / (double)sample_num;
	origin.y = origin.y / (double)sample_num;
	origin.z = origin.z / (double)sample_num;

	obj.CenterOfPointCloud = origin;

	return;
}

#endif