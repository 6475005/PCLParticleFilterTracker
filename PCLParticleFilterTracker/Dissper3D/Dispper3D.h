#ifndef __DISPPER3D_H__ 
#define __DISPPER3D_H__ 

#include <stdio.h>
#include <stdlib.h>
#include <opencv2\opencv.hpp>
#include <windows.h>
#include <vector>
#include "freeglut.h"

#include "DispObjClass.h"

//#include "glut.h"

//�ޥ�����״�B
struct MouseState{
	int state;
	int button;
	int option;
	int x;
	int y;
	bool mouseFlag1,mouseFlag2,mouseFlag3;
	int xStart,yStart;
};
//���`�ܩ`�ɤ�״�B
struct KeyState{
	unsigned char key;
	int SpKey;
	int x;
	int y;
};
//������״�B
struct ViewState{
	float dis;
	float twist;
	float elevation;
	float azimuth;
	float  xlook;
	float  ylook;
};
//����״�B
struct WindowState{
	int	w,h,wh;
};

//draw axis
void drawAxis(){
	glBegin(GL_LINES);
	glColor4f(1.0f,0.0f,0.0f,1.0f);
	glVertex3f(0.0,0.0,0.0);
	glVertex3f(1.0,0.0,0.0);

	glColor4f(0.0f,1.0f,0.0f,1.0f);
	glVertex3f(0.0,0.0,0.0);
	glVertex3f(0.0,1.0,0.0);
	
	glColor4f(0.0f,0.0f,1.0f,1.0f);
	glVertex3f(0.0,0.0,0.0);
	glVertex3f(0.0,0.0,1.0);
	
	glEnd();
	return;
}

class Dispper3D{
	private:
		//State
		static MouseState	m_Mouse;	//�ޥ�����״�B
		static KeyState		m_KeyState;//���`��״�B
		static ViewState	m_ViewState;//ҕ���״�B
		static WindowState  m_WinState;//����״�B

		//data
		static cv::Point3f origin, clicked_point;
		//static std::multimap<std::string, Obj3D> ObjData;
		static std::multimap<std::string, Obj3D_Disp *> ObjData;
		
		//thread params
		HANDLE							m_hThDispperProcess;
		HANDLE							m_hEvDispperProcessStop;
		static CRITICAL_SECTION			cs;

		// GLϵ�Υ����v��
		static void initViewState(){
			m_ViewState.dis   = 0.0f;
			m_ViewState.twist = 0.0f;
			m_ViewState.elevation = 0.0f;
			m_ViewState.azimuth = 0.0f;
			m_ViewState.xlook = 0.0f;
			m_ViewState.ylook = 0.0f;
		}
			
		static int SELECT_HITS(int hits,GLuint *buf){
			/*�ҥå��������å�*/
			if(hits<=0){
				for(int i = 0;i<3;i++){
					//FLAG[i] = false;
				}
				return -1;
			}

			int name = buf[3];
			int depth = buf[1];

			for(int loop = 1; loop < hits;loop++){
				//�����ǰ�Τ�Τ�Ҋ�Ĥ��ä��Ȥ�
				if(buf[loop*4+1] < GLuint(depth)){
					name = buf[loop*4+3];
					depth = buf[loop*4+1];
				}
			}

			//FLAG[name-1] = true;
			
			return 1;
		}

		static int pick_up(int x, int y)
		{
			/*�ӥ�`�ݩ`��ȡ��*/
			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);

			/*���쥯�����Хåե�����*/
			#define BUFSIZE 256
			GLuint selectBuf[BUFSIZE];
			glSelectBuffer(BUFSIZE, selectBuf);

			/*�����`��`�ɉ��*/
			glRenderMode(GL_SELECT);

			glInitNames();

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			gluPickMatrix(x,viewport[3]-y,5.0,5.0, viewport);

			gluPerspective(45.0,(double)viewport[2]/(double)viewport[3],0.01,30.0);

			glMatrixMode( GL_MODELVIEW );
			glLoadIdentity();

			polarview();

			drow();

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glMatrixMode( GL_MODELVIEW );
			
			int hits;
			hits = glRenderMode(GL_RENDER);

			int n = SELECT_HITS(hits, selectBuf);

			return n;
		}

		static void myMouseFunc(int button,int state,int x,int y){
		if(button == GLUT_LEFT_BUTTON &&state == GLUT_DOWN)//��ܥ���
		{
			m_Mouse.xStart = x;
			m_Mouse.yStart = y;
			m_Mouse.mouseFlag1 = GL_TRUE;

			if(pick_up(x,y)>0){//�ԥå����åׄI��
				GLdouble model[16], proj[16];
				GLint view[4];
				GLfloat z;
				GLdouble ox, oy, oz;

				glGetDoublev(GL_MODELVIEW_MATRIX, model);
				glGetDoublev(GL_PROJECTION_MATRIX, proj);
				glGetIntegerv(GL_VIEWPORT, view);

				glReadPixels(x, m_WinState.wh - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
				gluUnProject(x, m_WinState.wh - y, z, model, proj, view, &ox, &oy, &oz);
				clicked_point.x = (float) ox;
				clicked_point.y = (float) oy;
				clicked_point.z = (float) oz;
				m_Mouse.mouseFlag1 = GL_FALSE;
			}
		}
		else
		{
			m_Mouse.mouseFlag1 = GL_FALSE;
		}

		if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)//�ҥܥ���
		{
			m_Mouse.xStart = x;	
			m_Mouse.yStart = y;
			m_Mouse.mouseFlag2 = GL_TRUE;
		}
		else
		{
			m_Mouse.mouseFlag2 = GL_FALSE;
		}

		if(button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)//�Хܥ���
		{
			m_Mouse.xStart = x;	
			m_Mouse.yStart = y;
			m_Mouse.mouseFlag3 = GL_TRUE;
		}
		else
		{
			m_Mouse.mouseFlag3 = GL_FALSE;
		}	
	}

		static void myMouseMotion(int x,int y){
			int xdis,ydis;
			double a = -0.2,b = -0.01,c = 0.001; //�m��

			if(m_Mouse.mouseFlag1 == GL_FALSE && m_Mouse.mouseFlag2 == GL_FALSE && m_Mouse.mouseFlag3 == GL_FALSE)
				return;
	
			if(m_Mouse.mouseFlag1 == GL_TRUE){
				xdis = x - m_Mouse.xStart;
				ydis = y - m_Mouse.yStart;

				m_ViewState.elevation += (float)((double)ydis * a);
				m_ViewState.azimuth += (float)((double)xdis * a);
			}

			if(m_Mouse.mouseFlag2 == GL_TRUE){
				ydis = y - m_Mouse.yStart;

				m_ViewState.dis += (float)((double)ydis * b);
			}

			if(m_Mouse.mouseFlag3 == GL_TRUE){
				xdis = x - m_Mouse.xStart;
				ydis = y - m_Mouse.yStart;

				m_ViewState.ylook += (float)((double)ydis * c);
				m_ViewState.xlook += (float)((double)xdis * -c);
			}
	
			m_Mouse.xStart = x;
			m_Mouse.yStart = y;

			glutPostRedisplay();
		}
	
		static void polarview(void){
			
			glTranslatef(-m_ViewState.xlook,-m_ViewState.ylook,0);
			//��ǥ�ӥ�`��Q
			glTranslatef(0.0,0.0,m_ViewState.dis);
			glRotatef(-m_ViewState.twist,0.0,0.0,1.0);
			glRotatef(-m_ViewState.elevation,1.0,0.0,0.0);
			glRotatef(-m_ViewState.azimuth,0.0,1.0,0.0);
			//�������ˉ�Q
			glTranslatef(-origin.x,-origin.y,-origin.z);				
		}

		static void drow(void){
			//draw axis
			glPushMatrix();
			drawAxis();
			glPopMatrix();

			glEnable(GL_DEPTH_TEST);

			//draw objects
			EnterCriticalSection(&cs);
			std::multimap<std::string,Obj3D_Disp*>::iterator ite = ObjData.begin();
			while(ite != ObjData.end()){
				ite->second->draw();
				ite++;
			}
			LeaveCriticalSection(&cs);			
		}

		static void display(void){
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
			glMatrixMode(GL_MODELVIEW);	
			glLoadIdentity();
			polarview();
			//�ɥ�`
			drow();
			glDisable(GL_DEPTH_TEST);
			glutSwapBuffers();
		}

		static void resize(int w, int h){
			m_WinState.wh = h;
			//glViewport(0, 0, w, w);
			glViewport(0, 0, w, h);
			glLoadIdentity();
			glOrtho(-w / 200.0, w / 200.0, -h / 200.0, h / 200.0, -1.0, 1.0);
		}

		static void myKey(unsigned char key, int x, int y){
			//std::multimap<std::string,Obj3D>::iterator ite = ObjData.begin();
			std::multimap<std::string,Obj3D_Disp*>::iterator ite = ObjData.begin();
			
			int objNum = ObjData.size();
			static int index = 0;
			
			switch (key){
				case 'c':
					//��ܞ���Ĥ����ˤ�䤨��
					index = (++index) % (objNum+1);
					for(int i=1;i<index;i++){ite++;}
					if(index == 0){
						origin = cv::Point3f(0.0f,0.0f,0.0f);
					}else{
						origin = ite->second->getCenterPoint();
					}
					initViewState();
					break; 
			}
		}

		static void idle(void){
			glutPostRedisplay();
		}

		void myInit(char *progname){
			const int width  = 500;
			const int height = 500;
			float aspect = (float)width/(float)height;

			glutInitWindowPosition(0,0);
			glutInitWindowSize(width,height);
			glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH |GLUT_DOUBLE);//���Хåե�
			glutCreateWindow(progname);

			glClearColor(0.0,0.0,0.0,1.0);
			glShadeModel(GL_SMOOTH);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(45.0,aspect,0.01,1000.0);
			glMatrixMode(GL_MODELVIEW);
			polarview();

			glEnable(GL_LIGHT0);

			glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
		}

		static DWORD WINAPI ProcessThread(PVOID pParam);

	public:
		Dispper3D(){
			// Init params
			initViewState();

			m_Mouse.mouseFlag1 = GL_FALSE;
			m_Mouse.mouseFlag2 = GL_FALSE;
			m_Mouse.mouseFlag3 = GL_FALSE;

			origin			= cv::Point3f(0.0f,0.0f,0.0f);
			clicked_point	= cv::Point3f(0.0f,0.0f,0.0f);

			// Start the Dispper3D processing thread
			InitializeCriticalSection(&cs);
			m_hEvDispperProcessStop = CreateEvent(NULL,TRUE,FALSE,NULL);
			m_hThDispperProcess = CreateThread(NULL,0,ProcessThread,this,0,NULL);
		};

		~Dispper3D(){
			//LeaveCriticalSection(&cs);
			
			glutLeaveMainLoop();
			WaitForSingleObject(m_hThDispperProcess,INFINITE);
			DeleteCriticalSection(&cs);
			std::multimap<std::string, Obj3D_Disp*>::iterator ite = this->ObjData.begin();
			while(ite != this->ObjData.end()){
				delete ite->second;
				ite++;
			}
			this->ObjData.clear();
		 };

		//�����֥������Ȳ���
		int		newObj(std::string str,Obj3D_Disp *Obj);
		int		delObj(std::string str);
		int		delAll(void);
		int		updateObj(std::string str,Obj3D_Disp *Obj);
		bool	getClick(cv::Point3f &point);
		
};

CRITICAL_SECTION Dispper3D::cs;
cv::Point3f Dispper3D::origin,Dispper3D::clicked_point;

MouseState Dispper3D::m_Mouse;
KeyState   Dispper3D::m_KeyState;
ViewState  Dispper3D::m_ViewState;
WindowState  Dispper3D::m_WinState;
std::multimap<std::string, Obj3D_Disp*> Dispper3D::ObjData;


int Dispper3D::newObj(std::string str,Obj3D_Disp *Obj){
	EnterCriticalSection(&cs);
	Obj3D_Disp *Obj_ = Obj->clone();
	this->ObjData.insert(std::pair<std::string,Obj3D_Disp*>(str,Obj_));
	LeaveCriticalSection(&cs);
	return 0;
}

int Dispper3D::delObj(std::string str){

	EnterCriticalSection(&cs);
	std::multimap<std::string, Obj3D_Disp*>::iterator ite = this->ObjData.find(str);
	if(ite == this->ObjData.end()){
		fprintf(stderr,"cant find %s image!\n",str.c_str());
		LeaveCriticalSection(&cs);
		return -1;
	}else{
		//��ńI��
		delete ite->second;
		this->ObjData.erase(ite);
		LeaveCriticalSection(&cs);
	}
	return 0;
}
int Dispper3D::delAll(){

	EnterCriticalSection(&cs);
	std::multimap<std::string, Obj3D_Disp*>::iterator ite = this->ObjData.begin();
	while(ite != this->ObjData.end()){
	//��ńI��
		delete ite->second;
		this->ObjData.erase(ite++);
	}
	LeaveCriticalSection(&cs);
	return 0;
}
int Dispper3D::updateObj(std::string str, Obj3D_Disp *Obj)
{
	std::multimap<std::string,Obj3D_Disp*>::iterator ite;
	ite = this->ObjData.find(str);
	if(ite == this->ObjData.end()){
		newObj(str,Obj);
		return 0;
	}

	EnterCriticalSection(&cs);
	ite->second->update(Obj);
	LeaveCriticalSection(&cs);

	return 0;
};

bool Dispper3D::getClick(cv::Point3f &dst)
{
	cv::Point3f i_point(0.0f,0.0f,0.0f);
	if(i_point == clicked_point){
		return false;
	}else{
		dst = clicked_point;
		clicked_point = i_point;
		fprintf(stdout,"click %f %f %f\n",dst.x,dst.y,dst.z);
		return true;
	}
}

DWORD WINAPI Dispper3D::ProcessThread(LPVOID pParam)
{
	Dispper3D *pthis = (Dispper3D *) pParam;
	int  argc   = 1;
    char *argv[] = {"MyApp", NULL};

	// Main thread loop
    glutInit(&argc,argv);
	pthis->myInit("Dispper3D");
	glutDisplayFunc(pthis->display);
	glutReshapeFunc(pthis->resize);
	glutMouseFunc(pthis->myMouseFunc);
	glutMotionFunc(pthis->myMouseMotion);
	glutKeyboardFunc(pthis->myKey);
	glutIdleFunc(pthis->idle);

	glutMainLoop();

    return 0;
}

#endif