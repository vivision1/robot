// http://ahux.narod.ru/olderfiles/1/OSG3_Cookbook.pdf https://titanwolf.org/Network/Articles/Article?AID=306e6b2e-3e45-4cc2-8f86-3e674ff557c3#gsc.tab=0
//math collision detection two cubes https://gamedev.stackexchange.com/questions/60505/how-to-check-for-cube-collisions

#ifndef _WIN32
#include <windows.h>
// Use discrete GPU by default.
extern "C" {
	// http://developer.download.nvidia.com/devzone/devcenter/gamegraphics/files/OptimusRenderingPolicies.pdf
	__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;

	// http://developer.amd.com/community/blog/2015/10/02/amd-enduro-system-for-developers/
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif
#define GLEW_STATIC
// #include <GL/glew.h> 
// #include <GL/gl.h>
// #include <GL/glu.h>
// #include <GL/glcorearb.h>
// #include <GL/glext.h>  
// #include <GL/GLwDrawA.h>
// #include <GL/GLwDrawAP.h>
// #include <GL/glxext.h>
// #include <GL/glx.h>


#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <functional>
#include <unordered_map> 
#include <algorithm>
// #include <boost/thread/thread.hpp>
// #include <boost/thread/mutex.hpp> 
#include <boost/algorithm/string/replace.hpp>
// #include <boost/algorithm/string.hpp>
// #include "fl_editor.hpp"
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Text_Editor.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Multiline_Input.H>
#include <FL/Fl_Scroll.H>
#include <Fl/Fl_Value_Slider.H>
#include <FL/Fl_Toggle_Button.H>
#include <FL/Fl_Double_Window.H>
// #include <FL/Fl_Browser.H>
#include <FL/Fl_Button.H> 
#include <FL/Fl_Choice.H>
#include <FL/Fl_Input_Choice.H>
// #include <FL/Fl_Tabs.H>
// #include <glm/glm.hpp>
// #include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtc/type_ptr.hpp>

// #include <condition_variable>
#include <thread>
#include <mutex>

#include "filesystem.hpp"
#include "regular.hpp"
#include "threads.hpp"
#include "timef.hpp"
#include "arrayf.hpp"
#include "stringf.hpp" 


struct flocvs;
flocvs* flocv;


vector<vector<Fl_Button*>> btp;
struct vix{int index;float angle; };	
vector<vector<vix*>> vixs;

#if 1 //servo i2c
// https://community.element14.com/products/devtools/single-board-computers/b/blog/posts/maaxboard-setup-for-servo-pwm-control-with-python permissions
// https://www.jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/
// https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
// https://forums.raspberrypi.com/viewtopic.php?t=260395

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value
int pmap ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}
#endif

#if 1 //stepper carril
// https://www.instructables.com/Raspberry-Pi-Python-and-a-TB6600-Stepper-Motor-Dri/

#endif


#if 1 //Gpio
// https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
// https://github.com/jetsonhacks/JHPWMDriver/blob/master/src/JHPWMPCA9685.h
// https://www.jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/
// https://forums.raspberrypi.com/viewtopic.php?t=327125
// https://crish4cks.net/a-simple-c-library-to-drive-a-stepper-motor-using-the-raspberry-pi/
#if __linux__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

 /****************************************************************
 * Constants
 ****************************************************************/
 
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
 
	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}
 
	if (out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}
 
	write(fd, edge, strlen(edge) + 1); 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

 
int gpio_fd_close(int fd)
{
	return close(fd);
}
 
int gpio(int argc, char **argv, char **envp)
{
	struct pollfd fdset[2];
	int nfds = 2;
	int gpio_fd, timeout, rc;
	char *buf[MAX_BUF];
	unsigned int gpio;
	int len;



	if (argc < 2) {
		printf("Usage: gpio-int <gpio-pin>\n\n");
		printf("Waits for a change in the GPIO pin voltage level or input on stdin\n");
		exit(-1);
	}

	gpio = atoi(argv[1]);

	gpio_export(gpio);
	gpio_set_dir(gpio, 0);
	gpio_set_edge(gpio, "rising");
	gpio_fd = gpio_fd_open(gpio);

	timeout = POLL_TIMEOUT;
 
	while (1) {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = STDIN_FILENO;
		fdset[0].events = POLLIN;
      
		fdset[1].fd = gpio_fd;
		fdset[1].events = POLLPRI;

		rc = poll(fdset, nfds, timeout);      

		if (rc < 0) {
			printf("\npoll() failed!\n");
			return -1;
		}
      
		if (rc == 0) {
			printf(".");
		}
            
		if (fdset[1].revents & POLLPRI) {
			lseek(fdset[1].fd, 0, SEEK_SET);
			len = read(fdset[1].fd, buf, MAX_BUF);
			printf("\npoll() GPIO %d interrupt occurred\n", gpio);
		}

		if (fdset[0].revents & POLLIN) {
			(void)read(fdset[0].fd, buf, 1);
			printf("\npoll() stdin read 0x%2.2X\n", atoi(buf[0]));
		}

		fflush(stdout);
	}

	gpio_fd_close(gpio_fd);
	return 0;
}
#endif
#endif



//osg
#if 1
#include <osg/io_utils>
#include <osgViewer/Viewer>
// #include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/OrbitManipulator>
#include <osgDB/ReadFile>
#include <osg/AlphaFunc>
#include <osg/BlendFunc>
#include <osg/Depth>
// #include <osg/Geode>
#include <osg/Material>
#include <osg/LOD>
#include <osg/Math>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/Projection>
// #include <osg/ShapeDrawable>
// #include <osg/Texture2D>
// #include <osg/TextureBuffer>
// #include <osg/Image>
// #include <osg/Texture2DArray>
// #include <osg/Multisample> 
#include <osg/LineWidth>  
// #include <osg/Camera>
#include <osg/PositionAttitudeTransform>
#include <osg/ComputeBoundsVisitor>
using namespace osg;

struct Fl_Double_Windowc : public Fl_Double_Window{
	Fl_Double_Windowc(int x, int y, int w, int h, const char *label=0): Fl_Double_Window(x, y, w, h, label){ 
	}
    ~Fl_Double_Windowc() {}	  
 
	virtual int handle(int event){
		// cot(event);
		// cot(FL_KEYDOWN);
				// cot( Fl::event_key());
	// cot(Fl::get_key);
				return Fl_Double_Window::handle(event);
	}
};

struct AdapterWidget : public Fl_Gl_Window{
	AdapterWidget(int x, int y, int w, int h, const char *label=0): Fl_Gl_Window(x, y, w, h, label){
		_gw = new osgViewer::GraphicsWindowEmbedded(x,y,w,h);
	}
    ~AdapterWidget() {}	
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
    osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }
    const osgViewer::GraphicsWindow* getGraphicsWindow() const { return _gw.get(); }
	void resize(int x, int y, int w, int h){
		_gw->getEventQueue()->windowResize(x, y, w, h );
		_gw->resized(x,y,w,h);
		Fl_Gl_Window::resize(x,y,w,h);
	}
	virtual int handle(int event){
		// cot(event);
		// cot(FL_KEYDOWN);
				// cot( Fl::event_key());
	// cot(Fl::get_key);
				// return Fl_Gl_Window::handle(event);
		if(event==3)Fl::focus(this);
		switch(event){
			case FL_PUSH:
				_gw->getEventQueue()->mouseButtonPress(Fl::event_x(), Fl::event_y(), Fl::event_button());
				return 1;
			case FL_MOVE:
			case FL_DRAG:
				_gw->getEventQueue()->mouseMotion(Fl::event_x(), Fl::event_y());
				return 1;
			case FL_RELEASE:
				_gw->getEventQueue()->mouseButtonRelease(Fl::event_x(), Fl::event_y(), Fl::event_button());
				return 1;
			case FL_KEYDOWN:
				// _gw->getEventQueue()->keyPress((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
				return 1;
			case FL_KEYUP:
				// _gw->getEventQueue()->keyRelease((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
				return 1;
			default:
				// pass other events to the base class
				return Fl_Gl_Window::handle(event);
		}
	};
};
struct ViewerFLTK : public osgViewer::Viewer, public AdapterWidget{
	static void Timer_CB(void *userdata) {
        ViewerFLTK *pb = (ViewerFLTK*)userdata; 
        pb->redraw(); 
        Fl::repeat_timeout(1.0/24, Timer_CB, userdata);
    }
    ViewerFLTK(int x, int y, int w, int h, const char *label=0):
            AdapterWidget(x,y,w,h,label) {
				Fl::add_timeout(1.0/24.0, Timer_CB, (void*)this);
                getCamera()->setViewport(new osg::Viewport(0,0,w,h));
                getCamera()->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(w)/static_cast<double>(h), 1.0f, 10000.0f);
                getCamera()->setGraphicsContext(getGraphicsWindow());
                getCamera()->setDrawBuffer(GL_BACK);
                getCamera()->setReadBuffer(GL_BACK);
                setThreadingModel(osgViewer::Viewer::SingleThreaded);
            }
        void draw() { 
			frame();
		}
	// virtual int handle(int event){
		// cot(event);
		// cot(FL_KEYDOWN);
				// cot( Fl::event_key());
				// return Fl_Gl_Window::handle(event);
	// }

};

#include <sqlite3.h>
sqlite3* sql3;
Fl_Double_Window* win;
Fl_Double_Window* flt; 
Fl_Double_Window* flbd; 
ViewerFLTK* osggl;
unordered_map<string,osg::ref_ptr<osg::Node>> stlgroup;
osg::Group* group = new osg::Group(); 
struct osgdr;
vector<osgdr*> ve;
osgdr* carril;


void settranparency(Node* model,bool val=1){
	osg::StateSet* state2 = model->getOrCreateStateSet();
	state2->clear();	
	osg::ref_ptr<osg::Material> mat2 = new osg::Material;
	mat2->setAlpha(osg::Material::FRONT_AND_BACK, 0.5); //Making alpha channel
	state2->setAttributeAndModes( mat2.get() ,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	if(!val)return;
	osg::BlendFunc* bf = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA,osg::BlendFunc::ONE_MINUS_DST_COLOR );
	state2->setAttributeAndModes(bf);  
	model->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
	model->getStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
}



void setalpha(Node* model,float val=0.9){
// https://groups.google.com/g/osg-users/c/APGaJ_5icx8
	osg::StateSet* state2 = model->getOrCreateStateSet(); //Creating material
	osg::ref_ptr<osg::Material> mat2 = new osg::Material;

	mat2->setAlpha(osg::Material::FRONT_AND_BACK, 0.5); //Making alpha channel
	state2->setAttributeAndModes( mat2.get() ,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	
	osg::BlendFunc* bf = new                        //Blending
	osg::BlendFunc(osg::BlendFunc::SRC_ALPHA,
	osg::BlendFunc::ONE_MINUS_DST_COLOR );
	state2->setAttributeAndModes(bf);  
	model->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
	model->getStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN ); 
	
	
	state2->clear();
	state2->setAttributeAndModes( mat2.get() ,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	
}
#endif

osgGA::TrackballManipulator* tmr;
typedef osg::Vec3f vec3f; 
typedef osg::Vec3f vec3; 
#define Pi 3.141592653589793238462643383279502884L
#define Pi180 0.01745329251

bool posa_debug=0;
bool dbg_force=0; 
vvfloat posapool;
struct posv{ vfloat p; vec3 topoint; float x; float y; float z; int posa_counter=0; bool cancel=0; };
vector<posv> pool;
std::mutex posa_mtx;
mutex posa_counter_mtx;
mutex posa_erase_mtx;
				

float distance_two_points(vec3* point1,vec3* point2){
	// cot(*point2);
	return sqrt( pow(point2->x()-point1->x(),2) +pow(point2->y()-point1->y(),2) +pow(point2->z()-point1->z(),2)    );	
}

vector<vec3> segments_3d(vec3 p1,vec3 p2, int segments=10){
	vector<vec3> res(segments);
	float xx=p2.x()-p1.x();
	float yy=p2.y()-p1.y();
	float zz=p2.z()-p1.z();
	float m;
	lop(i,0,segments){
		m=i/(float)segments;
		res[i].x()=p1.x()+xx*m;
		res[i].y()=p1.y()+yy*m;
		res[i].z()=p1.z()+zz*m;
	}
	return res;
}

void bound_box();
void dbg_pos();
void copy_points_k();
void arm_len_fill();
void movz_ik(float z);
struct osgdr{
	int index=0;
	vec3* offset=0;
	float angle=0;
	float anglestart=0;
	float anglemax=0;
	float anglemin=0;
	int dir=1;
	int rotatedir=1;
	float angleik=0;
	float arm_len=0;
	bool moving=0;
	vec3 *axisbegin;
	vec3 *axisend;
	vec3 axis;
	vec3 *axisbeginik;
	vec3 *axisendik;
	vector<osg::ref_ptr<osg::Node>> nodes;
	vstring nodesstr;
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> pointsik = new osg::Vec3Array;;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	osg::ref_ptr<osg::Geometry> geometry= new osg::Geometry; 
    MatrixTransform* transform = new osg::MatrixTransform;
	DrawArrays* drw=new osg::DrawArrays;
	// osg::ComputeBoundsVisitor* cbv=new osg::ComputeBoundsVisitor;
	osgdr(Group* group){	
		group->addChild( transform); 
		group->addChild( geometry);
	}
	void newdr(vec3 _axisbegin=vec3(30,0,0),vec3 _axisend=vec3(0,0,0) ){
		index=ve.size()-1;
		// cot(index);
		points->push_back(_axisbegin);
		points->push_back(_axisend);
		axisbegin=&points[0][0];
		axisend=&points[0][1];
		color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
		// color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
		geometry ->setVertexArray(points.get());
		geometry ->setColorArray(color.get());
		geometry ->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		// geometry ->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		// geometry ->setColorBinding(osg::Geometry::BIND_OVERALL);
		geometry ->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0, points->size()));
		osg::LineWidth* linew = new osg::LineWidth(5);
		geometry->getOrCreateStateSet()->setAttributeAndModes(linew);
		// transform->addChild(geometry);
		
		lop(i,0,nodesstr.size()){
			nodes.push_back(osgDB::readRefNodeFile(nodesstr[i]));
			settranparency(nodes[i].get(),0);
			transform->addChild(nodes[i].get());
			// transform->accept(*cbv);
		}
		// arm_len_fill();
		
		//pointsik init
		// pointsik = new osg::Vec3Array;
		// pointsik->push_back(points[0][0]);
		// pointsik->push_back(points[0][1]); 
		// axisbeginik=&pointsik[0][0];
		// axisendik=&pointsik[0][1];
		
		// dbg_pos();
		// cot(*axisbegin);
		// cot(*axisbeginik);
	}
	
	void gooffset(){
		if(offset==0)offset=new vec3(0,0,0);
		
	}
	

	
	void rotate_pos(float newangle){	 
		float nangle=angle-newangle; 
		rotate(nangle*-1);
	}
	void rotate_posk(float newangle){	
		float nangle=angleik-newangle; 
		rotateik(nangle*-1);
	}
	//ve[index]-> == this
	//posa
	void rotatetoposition(float newangle, posv* pov){	
		thread th([&](float newangle, posv* pov ){
			float precision=1;
			if(moving==1){
				// moving=0;
			}
			mtxlock(index+1);
			moving=1;
			if(pov->cancel)moving=0;
			// cot(angle);
			// cot(newangle);
			// if(newangle>anglemax)return;
			mtxlock(9);
			float nangle=newangle-angle;
			// cot(index);
			// cot(newangle);
			// cot(angle);
			// cot(nangle);
			mtxunlock(9);
			// cot(anglemax);
			// rotate( nangle);
			if(nangle>0){
				float dir=1*precision;
				for(;;){
					if(moving==0)break;
					if(angle>anglemax)break;
					if(nangle<=0.1)break;
					rotate( dir);
					// cot(nangle);
					// cot(angle);
					nangle-=dir;
					sleepms(20);
				
				}
			}
			if(nangle<0){
				float dir=-1*precision;
				for(;;){
					if(moving==0)break;
					if(angle<anglemin)break;
					if(nangle>=-0.1)break;
					rotate( dir);
					nangle-=dir;
					sleepms(20);
				
				}
			}
			moving=0;
			mtxunlock(index+1);
				posa_counter_mtx.lock();
				pov->posa_counter--;
				// cot(posa_counter);
				posa_counter_mtx.unlock();
		},newangle,pov);
		th.detach( ); 
		
	
	} 
	void rotate( float _angle ){
	// cot(_angle);
		if(_angle==0)return;
		mtxlock(0);
				
		// cot(index);
		// cot(angle);
		// lop(i,0,ve.size()) cout<<i<<" "<<ve[i]->angle<<"  ";cout<<endl;
		// cot(axisb.length());
		// if(axisb.length()==0)
		vec3 axisb=*axisbegin; 
		// angle+=_angle;
		// cot(angle);
		 		
		osg::Matrix Tr;
		Tr.makeTranslate( axisb.x(),axisb.y(),axisb.z() );
		osg::Matrix T; 
		T.makeTranslate( -axisb.x(),-axisb.y(),-axisb.z() ); 
		 
		lop(i,0,index) {		
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*-ve[i]->angle, ve[i]->axis ); 
			lop(j,0,points[0].size())points[0][j] = points[0][j]   * T * Ra * Tr   ; 
			transform->setMatrix(transform->getMatrix()   * T * Ra * Tr ); 
		}
		osg::Matrix R; 
		R.makeRotate( Pi180*_angle*rotatedir, axis ); 	 
		lop(j,0,points[0].size())points[0][j] = points[0][j] * T * R * Tr  ; 
		transform->setMatrix(transform->getMatrix()   * T  *  R  * Tr   ); 
		 
		for(int i=index-1;i>=0;i--)	{ 
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*ve[i]->angle, ve[i]->axis ); 
			lop(j,0,points[0].size())points[0][j] = points[0][j]    * T * Ra * Tr   ; 
			transform->setMatrix(transform->getMatrix()   * T * Ra * Tr    );
		} 
		drw->dirty();
		drw->set(GL_LINES,0, points->size());
		geometry ->setPrimitiveSet(0,drw); 
		
		//todas as posteriores teem que rodar tambem 
		lop(i,index+1,ve.size()-0){
			// cot(ve[i]->nodesstr);
				 
			lop(jj,0,index)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*-ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * R * Tr   );
			lop(j,0,ve[i]->points[0].size()){ 
				ve[i]->points[0][j] = ve[i]->points[0][j]   * T * R * Tr   ;  
			}
			
			for(int jj=index-1;jj>=0;jj--)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->drw->dirty();
			ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
			ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
		}

		// transform->accept(*cbv);
		angle+=_angle;
		
		//modulo float
		angle = angle - int( angle/360.0 )*360.0 ;
		
		
		
		//acende luz no fk 
		int ang10=angle/10*10;
		lop(i,0,vixs[index].size()){
			if((int)vixs[index][i]->angle==ang10){ 
				lop(j,0,vixs[index].size())if(btp[index][j]->color()!=FL_GRAY){
					btp[index][j]->color(FL_GRAY );
					btp[index][j]->redraw();				
				}
				btp[index][i]->color(FL_GREEN);
				btp[index][i]->redraw();
			}
		}
		dbg_pos();
		// bound_box();
		mtxunlock(0);
	};
	void rotateik( float _angle ){  
 
		 
		
		vec3 axisb=*axisbeginik;  
		 		
		osg::Matrix Tr;
		Tr.makeTranslate( axisb.x(),axisb.y(),axisb.z() );
		osg::Matrix T; 
		T.makeTranslate( -axisb.x(),-axisb.y(),-axisb.z() ); 
		
		lop(i,0,index){		
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*-ve[i]->angleik, ve[i]->axis ); 
			lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j]   * T * Ra * Tr   ; 
			// transform->setMatrix(transform->getMatrix()   * T * Ra * Tr ); 
		}
		osg::Matrix R; 
		R.makeRotate( Pi180*_angle*rotatedir, axis ); 	 
		lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j] * T * R * Tr  ; 
		// transform->setMatrix(transform->getMatrix()   * T * R * Tr   );
		 
		for(int i=index-1;i>=0;i--)	{ 
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*ve[i]->angleik, ve[i]->axis ); 
			lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j]    * T * Ra * Tr   ; 
			// transform->setMatrix(transform->getMatrix()   * T * Ra * Tr    );
		} 
  
		 
		//todas as posteriores teem que rodar tambem 
		lop(i,index+1,ve.size()-0){
			// cot(ve[i]->nodesstr);
				 
			lop(jj,0,index)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*-ve[jj]->angleik, ve[jj]->axis ); 
				lop(j,0,ve[i]->pointsik[0].size())ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * Ra * Tr   ; 
				// ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			// ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * R * Tr   );
			lop(j,0,ve[i]->pointsik[0].size()){ 
				ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * R * Tr   ;  
			}
			
			for(int jj=index-1;jj>=0;jj--)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*ve[jj]->angleik, ve[jj]->axis ); 
				lop(j,0,ve[i]->pointsik[0].size())ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * Ra * Tr   ; 
				// ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			// ve[i]->drw->dirty();
			// ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
			// ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
		}

		angleik+=_angle;
 
	}; 

	//returns angles to given point
	posv posik(vec3 topoint,float x=0,float y=0,float z=1000){
		copy_points_k();
		// dbg_force=1;
		// dbg_pos();
		// posa_debug=0;
		float precision=1;
		int sz=ve.size(); 
		vfloat angles(sz);
		sz+=1; ///eixo z, depois implementar o x e o y
		// vec3 axisb=*axisbegin;
		#define cdist distance_two_points(&topoint,axisendik)
		 
		// cot(axisb);
		// cot(arm_len);
		// lop(i,0,ve.size())cout<<ve[i]->arm_len<<" ";cout<<endl;
		// cot(*axisendik)
		// cot(*axisend) 
		float ld=cdist;
		vfloat cdir(sz);
		lop(i,0,sz)cdir[i]=precision;  //aqui o z tem a mesma precisao k a rotaçao
		for(int wi=0;wi<1000;wi++){
			for(int vi=sz-2;vi>=0;vi--){ //-2 é o z //o ultimo n tem angulo
				if(vi<sz-1)if( ve[vi]->angleik >  ve[vi]->anglemax || ve[vi]->angleik <  ve[vi]->anglemin ){ 
					cdir[vi]*=-1;
					ve[vi]->rotateik(cdir[vi]*50);
				}
				ve[vi]->rotateik(cdir[vi]);
				if(ld<=cdist){
					cdir[vi]*=-1;
					ve[vi]->rotateik(cdir[vi]); 
				}  
				// cot(ve[vi]->angle);
				angles[vi]=ve[vi]->angleik;
				// cot(vi)
				// pausa
				// cot(angles);
			}
			movz_ik(cdir[sz-1]);
			if(ld<=cdist){
				cdir[sz-1]*=-1;
				movz_ik(cdir[sz-1]); 
			}  
			
			ld=cdist;
			// if(wi%100==0) cot(ld);
			// cot(cdir);
			// cot(angles);
			if(wi%3500==0){
				lop(i,0,sz)cdir[i]=precision;
			}
		}
		// cot(ld);
		// cot(angles);
		// cot(*axisendik)
		// cot(*axisend)
		posv res;
		res.p=angles;
		// res.z=1000;
		res.z=ve[0]->axisbeginik->z();
		// posv r(angles,{},0,0,0);
		return res;
	}
};

void arm_len_fill(){
	lop(i,0,ve.size()-1){
		ve[i]->arm_len=distance_two_points(ve[i]->axisbegin,ve[i+1]->axisbegin);
		cot(ve[i]->arm_len);
	}
}
void dbg_pos(){
	if(!posa_debug && dbg_force==0) return;
	dbg_force=0;
	// arm_len_fill();
	// lop(i,0,ve.size()) cout<<i<<" "<<ve[i]->angle<<"  ";cout<<endl; 
	cout<<"posa: "; lop(i,0,ve.size()) cout<<i<<","<<ve[i]->angle<<"  ";cout<<endl;		
	cout<<"posa( "; lop(i,0,ve.size()-1) cout<<ve[i]->angle<<" , "; cout<<ve.back()->angle<<" )"; cout<<endl;		
	cout<<"\t"<<"axisbx\taxisby\taxisbz\t"<<"axisex\taxisey\taxisez\t"<<endl;
	lop(i,0,ve.size()){
		cout<<"idx"<<i<<"\t"<<(int)(ve[i]->axisbegin->x())<<"\t"<<(int)(ve[i]->axisbegin->y())<<"\t"<<(int)(ve[i]->axisbegin->z())<<"\t"<<(int)(ve[i]->axisend->x())<<"\t"<<(int)(ve[i]->axisend->y())<<"\t"<<(int)(ve[i]->axisend->z())<<endl;;
	} 
	cot(distance_two_points(ve[1]->axisbegin,ve[3]->axisbegin) );
		cout<<"posak( "; lop(i,0,ve.size()-1) cout<<ve[i]->angleik<<" , "; cout<<ve.back()->angleik<<" )"; cout<<endl;
		cout<<"\t"<<"axisbx\taxisby\taxisbz\t"<<"axisex\taxisey\taxisez\t"<<"axisbxk\taxisbyk\taxisbzk\t"<<"axisexk\taxiseyk\taxisezk\t"<<endl;
		lop(i,0,ve.size()){
			cout<<"idx"<<i<<"\t"<<(int)(ve[i]->axisbegin->x())<<"\t"<<(int)(ve[i]->axisbegin->y())<<"\t"<<(int)(ve[i]->axisbegin->z())<<"\t"<<(int)(ve[i]->axisend->x())<<"\t"<<(int)(ve[i]->axisend->y())<<"\t"<<(int)(ve[i]->axisend->z())<<"\t"<<(int)(ve[i]->axisbeginik->x())<<"\t"<<(int)(ve[i]->axisbeginik->y())<<"\t"<<(int)(ve[i]->axisbeginik->z())<<"\t"<<(int)(ve[i]->axisendik->x())<<"\t"<<(int)(ve[i]->axisendik->y())<<"\t"<<(int)(ve[i]->axisendik->z())<<endl;;
		} 
		cout<<"Z "<<ve[0]->axisbegin->z()<<endl;
		cout<<"posaik( ";   cout<<std::round(ve[3]->axisend->x())<<" , "<<std::round(ve[3]->axisend->y())<<" , "<<std::round(ve[3]->axisend->z())<<" )"; cout<<endl;
}
//tem de copiar tambem o angulo
void copy_points_k(){ 
	lop(i,0,ve.size()){ 
		ve[i]->pointsik->clear();
		ve[i]->pointsik->push_back(ve[i]->points[0][0]);
		ve[i]->pointsik->push_back(ve[i]->points[0][1]); 
		ve[i]->axisbeginik=&ve[i]->pointsik[0][0];
		ve[i]->axisendik=&ve[i]->pointsik[0][1]; 
		ve[i]->angleik=ve[i]->angle;
	}
}
void pos_k(vfloat angles){ 
	lop(i,0,angles.size()){ 
		ve[i]->rotate_posk(angles[i]);
	}
	
}

void goffset(vec3* offset){
	osg::Matrix Trf;
	Trf.makeTranslate( offset->x(),offset->y(),offset->z() );
	lop(i,0,ve.size()){
		ve[i]->transform->setMatrix(ve[i]->transform->getMatrix() *  Trf );		
		lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]  * Trf  ; //points axisbegin actualiza
		ve[i]->drw->dirty();
		ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
		ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
	}
	//carril
	// carril->transform->setMatrix(carril->transform->getMatrix() *  Trf );	
	
	copy_points_k();
}

void movz_ik(float z){
	osg::Matrix Trf;
	Trf.makeTranslate( 0,0,z );	
	lop(i,0,ve.size())	
		lop(j,0,ve[i]->pointsik[0].size())ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]  * Trf  ;

}
void movetoposz(float z, posv* pov){
	thread th([](float z, posv* pov){
		float currz=(float)((*ve[0]->axisbegin).z());
		float rz=z-currz;
		float speed=2;
		// cot (currz);
		// cot(rz);
		float newz=rz;
		int dir=1;
		if(z<=currz){dir=-1;   }
		for(;;){
			mtxlock(0);
			// cot(dir);
			// cot(newz);
			if(pov->cancel){mtxunlock(0);break;}
			if(dir==1 && newz<=0){mtxunlock(0);break;}
			if(dir==-1 && newz>=0){mtxunlock(0);break;}
			if(dir==1)newz-=speed;else newz+=speed;
			osg::Matrix Trf;
			Trf.makeTranslate( 0,0,speed*dir );	
			lop(i,0,ve.size()){	
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix() *  Trf );	
				lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]  * Trf  ; 
				ve[i]->drw->dirty();
				ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
				ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
			}
			mtxunlock(0);
			sleepms(5);
				// cot("PC");
		}
				
				// posa_counter_mtx.lock();
				// cot(pov->posa_counter);
				pov->posa_counter--;
				// posa_counter_mtx.unlock();
		// dbg_pos();
		
	},z,pov);
	th.detach();
}
void geraeixos(Group* group){ 
	
	// vec3* offset=new vec3(0,0,0);
	//robot offset
	vec3* offset=new vec3(610-110,272,-300);
	
	
	carril=new osgdr(group);
	carril->nodesstr.push_back("stl/robot_nema23.stl"); 
	carril->offset=offset;
	carril->newdr(vec3(110,250,-20),vec3(110,250,200));
	
	
	int idx;
	idx=0;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_corpo.stl");
	ve[idx]->nodesstr.push_back("stl/robot_balde.stl");
	ve[idx]->nodesstr.push_back("stl/robot_servospt70.stl"); 
	ve[idx]->axis=vec3(0,1,0);
	ve[idx]->anglemax=190;
	ve[idx]->anglemin=-130;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(110,250,-20),vec3(110,250,200));
		
	idx=1;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_servospt70_1.stl");
	ve[idx]->nodesstr.push_back("stl/robot_armj1.stl");
	ve[idx]->axis=vec3(1,0,0);
	ve[idx]->anglemin=-20;
	ve[idx]->anglemax=200;
	// ve[idx]->rotatedir=-1;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(0,250-20,-42.0),vec3(0,250+50,-42.0));
		
	// idx=2;
	// ve.resize(idx+1);
	// ve[idx]=new osgdr(group); 
	// ve[idx]->nodesstr.push_back("stl/robot_armj1_1.stl"); 
	// ve[idx]->axis=vec3(0,0,1); 
	// ve[idx]->anglemax=160;
	// ve[idx]->anglemin=-160;
	// ve[idx]->offset=offset;
	// ve[idx]->newdr(vec3(-55,228,0),vec3(-50,228,0));
	
	idx=2;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_armj2.stl"); 
	ve[idx]->axis=vec3(0,1,0); 
	ve[idx]->anglemin=0;
	ve[idx]->anglemax=90;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(-66.61,271,-42.19),vec3(-150,271,-42.19));
	
	idx=3;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_armj2_1.stl"); 
	ve[idx]->axis=vec3(0,1,0);
	ve[idx]->anglemin=0;
	ve[idx]->anglemax=170;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(-300,281,-42.19),vec3(-600,281,-42.19));
 
	goffset(offset);
	
 
}
osg::ref_ptr<osg::Node> maquete;
osg::ref_ptr<osg::Node> ucs_icon;
vector<osg::ref_ptr<osg::Node>> cube10(8);
bool toggletranspbool=0;
void toggletransp(){
	toggletranspbool=!toggletranspbool;
	lop(i,0,ve.size()){
		lop(j,0,ve[i]->nodes.size()){
			settranparency(ve[i]->nodes[j],toggletranspbool);
		}
	}

} 

void bound_box(){
	//https://stackoverflow.com/questions/36830660/how-to-get-aabb-bounding-box-from-matrixtransform-node-in-openscenegraph https://groups.google.com/g/osgworks-users/c/K4px3UQXOew https://gamedev.stackexchange.com/questions/60505/how-to-check-for-cube-collisions
	// osg::ComputeBoundsVisitor* cbv=new osg::ComputeBoundsVisitor;
	// ve[0]->transform->accept(*cbv); 
	
	osg::ComputeBoundsVisitor cbv;
	ve[0]->transform->accept(cbv);
	osg::BoundingBox bb = cbv.getBoundingBox(); // in local coords.
	
	osg::ComputeBoundsVisitor cbv3;
	ve[3]->transform->accept(cbv3);
	osg::BoundingBox bb3 = cbv3.getBoundingBox(); // in local coords.
	
	// osg::BoundingBox bb = ve[0]->cbv->getBoundingBox();
	// osg::BoundingBox bb3 = ve[3]->cbv->getBoundingBox();
	 
// osg::Matrix localToWorld = osg::computeLocalToWorld( ve[0]->transform->getParentalNodePaths().front());// node->getParent(0)->getParentalNodePaths()[0] );
// osg::Matrix localToWorld = osg::computeLocalToWorld( ve[0]-> nodes[0]->getParent(0)->getParentalNodePaths()[0] );
 // for ( unsigned int i=0; i<8; ++i ) bb.expandBy( bb.corner(i) * localToWorld );
 
// osg::Matrix localToWorld3 = osg::computeLocalToWorld( ve[3]->transform->getParentalNodePaths().front());// node->getParent(0)->getParentalNodePaths()[0] );
// osg::Matrix localToWorld3 = osg::computeLocalToWorld( ve[3]->nodes[0]->getParent(0)->getParentalNodePaths()[0] );
 // for ( unsigned int i=0; i<8; ++i ) bb3.expandBy( bb3.corner(i) * localToWorld );
 
 
	bool intersects=bb.intersects(bb3);
	cot(intersects);
}


void loadstl(Group* group){
	
    maquete = osgDB::readRefNodeFile("stl/maquete.stl");
	settranparency(maquete.get(),1);
	group->addChild(maquete.get());
	
	
    ucs_icon = osgDB::readRefNodeFile("stl/3DUCSICON2.stl");
	settranparency(ucs_icon.get(),1);
	group->addChild(ucs_icon.get());
	
	// int plat_x=8, plat_y=4, plat_z=37, plat_zblock=4;
	int plat_x=4, plat_y=1, plat_z=2, plat_zblock=4;
	vector<vec3*> bdcpos(8);
	lop(i,0,cube10.size()){
		cube10[i] = osgDB::readRefNodeFile("stl/cube10.stl");
		settranparency(cube10[i].get(),1);
		MatrixTransform* trfm=new MatrixTransform;
		group->addChild(trfm);
		trfm->addChild(cube10[i].get());
		int x=i%4;
		int z=0;
		if(i>3)z=1;
		bdcpos[i]=new vec3(150+300*x,150,-150+-300*z);
		cot(*bdcpos[i]);
		osg::Matrix Trf;
		Trf.makeTranslate( bdcpos[i]->x(),bdcpos[i]->y(),bdcpos[i]->z() );
		trfm->setMatrix(trfm->getMatrix() *  Trf );	
	}

    // osg::ref_ptr<osg::Node> estrutura = osgDB::readRefNodeFile("Assembly1.stl");
	// settranparency(estrutura.get(),1);
	// group->addChild(estrutura.get());
	
};


//topoint faz override ao p
//vai ao posapool p=angles
//1000 é = ao anterior no p e  no z, x=0 e y=0 é =
// o topoint faz override ao p
void posa(vfloat p, vec3 topoint=vec3(0,0,0),int speedtype=0, float x=0, float y=0, float z=1000){
	// int sz=p.size();
	// posapool.push_back(p);
	cot("P");
	pool.push_back(posv{p,topoint,0,0,z,0});
	// posa_erase_mtx.unlock();
	// cot(posapool.size());
	// if(posapool.size()>1)return ;
	cot(pool.size());
	// cot(posa_counter);
	if(pool.size()>1)return ;
	std::thread thm([](float z ){
		while(pool.size()!=0){ 
			posa_mtx.lock(); 
			
	// posa_erase_mtx.lock();
			posv* pov=&pool[0];
	// posa_erase_mtx.unlock();
	
			if(pov->cancel){ 
				// posa_erase_mtx.lock();
				pool.erase(pool.begin()); 
				// posa_erase_mtx.unlock();
				posa_mtx.unlock();
				continue;
			}
	
			// posv* pov=new posv;
			// pov->p=pool[0].p;
			// pov->topoint=pool[0].topoint;
			// pov->x=pool[0].x;
			// pov->y=pool[0].y;
			// pov->z=pool[0].z;
			z=pov->z;
			cout<<"MOVE START"<<endl;
			//here can be more complete with all axes topoint 0, we assume robot arm never goes 0,0,0
			if(pov->topoint.x()!=0){ 
				cout<<"TO POINT"<<pov->topoint<<endl;
				// cot(pov->topoint);
				posv pv=ve[3]->posik(pov->topoint);
				pov->p=pv.p;
				
				z=pv.z;
				
				// sz=pov->p.size();
			}
			
			pov->posa_counter=pov->p.size(); 
			// cot(pov->posa_counter);
			// cot(*ve[3]->axisend); //aparece valor anterior ao movimento 
			cot(pov->p);
			lop(i,0,pov->p.size()){ 
				float newangle= pov->p[i];
  				if(newangle>=999){pov->posa_counter--; continue;}
				ve[i]->rotatetoposition(newangle,pov); 
	// cot(pool.size()); 
			}	
			
			if(z<1000){
				// cout<<"MOVZ "<<z<<endl;
			// cot(pov->posa_counter);
				pov->posa_counter+=1;
				movetoposz(z,pov);
			// cot(z);
			}
			while(pov->posa_counter>0){
				// cot(pov->posa_counter);
				sleepms(100);
			} 
			// cot(*ve[3]->axisend);
			// delete pov;
			posa_erase_mtx.lock();
			pool.erase(pool.begin());
			// delete pov;
			posa_erase_mtx.unlock();
			posa_mtx.unlock();
			// cot("unlock"); 
		}
	},z);
	thm.detach( );
}
void posi(int index,float angle){
	vfloat angles(ve.size());
	posa_erase_mtx.lock();
	// cot(pool.back().p);
	lop(i,0,ve.size())angles[i]=ve[i]->angle;
	for(int j=pool.size()-1;j>=0;j--)
		if(pool[j].p.size()>0){
			lop(i,0,pool[j].p.size())angles[i]=pool[j].p[i]; 
			break;
		}
	posa_erase_mtx.unlock();
	angles[index]=angle;
	// cout<<"ii"<<index<<endl;
	// cot(index);
	// cot(posapool.size());
	// cot(angles);
	posa(angles);
	
}

void posik(vec3 vv){
	// copy_points_k();
	// dbg_force=1;
	// dbg_pos();
	// cot("copied ik");
	// pos_k(vfloat{0,20,20,70});
	
	// vfloat angles=ve[3]->posik(vv);
	// posa(angles);
	posa({},vv);
	return;
	// lop(i,0,ve.size()){
		// ve[i]->rotate_posk(angles[i]);
		// ve[i]->rotatetoposition(angles[i]);
	// }
	// cot(angles);
}

void movz(float z){
	posa({},{},0,0,0,z);
}
//Rotas
#if 1

void reset(){
	movz(150);
	posa({-90 , 140 , 80 , 120} );
}


void loop(){
	
	
	
}
#endif

//Fann
#if 1
#include "comb.hpp"
#include "fann.h"
#define mathNumscale(num,factorx,min,max,midletarget)({float _mnsfd = (factorx)/2.0-(midletarget); float _mnsfactor = (float)(factorx) / ((max) - (min));( ((num) - (min)) * _mnsfactor - _mnsfd);})
#define mathNumdescale(num,factorx,min,max,midletarget)({float _mndsfd = (factorx)/2.0-(midletarget); float _mndsfactor = (float)(factorx) / ((max) - (min));( ((num) + _mndsfd) / _mndsfactor + (min) );})
// float scale(float min,float max
void fann_init(){
	int sz=ve.size();
	
	float xmin=0;
	float xmax=600;
	float ymin=120 , ymax=150;
	float zmin=-600 , zmax=0;
	
	const unsigned int num_input = 3;
    const unsigned int num_output = sz;
    const unsigned int num_layers = 3;
    const unsigned int num_neurons_hidden = 3;
	
	float factorx=2;
	float midletarget=0;
	
	vint r( sz );
	
	lop(i,0,sz){
		float ms=mathNumscale(1,factorx,ve[i]->anglemin,ve[i]->anglemax,midletarget);
		cot(ms);
		float msn=mathNumdescale(ms,factorx,ve[i]->anglemin,ve[i]->anglemax,midletarget);
		cot(msn);
		r[i]= ( ve[i]->anglemax - ve[i]->anglemin ) /10 +1;
		
		cot(r[i]);
	} 
	combR cb(r);
	cot(cb.range);
	vint cbres;
	
	cot(cb.toComb(0));
	cot(cb.toComb(cb.range-1));
	// return;
	
	stringstream strm;
	stringstream strmnscale;
	int count=0;
	vec3* axis=ve[3]->axisend;
	posa_debug=0;
	lop(i,0,cb.range){
		cbres=cb.toComb(i);
		// cot(cbres);
		lop(j,0,cb.k){
			cbres[j]*=10;
			cbres[j]+=ve[j]->anglemin;
			ve[j]->rotate_posk(cbres[j]);
		}
		// cot(cbres);
		// cot(*axis);
		if(axis->x()<xmin)continue;
		if(axis->x()>xmax)continue;
		if(axis->y()<ymin)continue;
		if(axis->y()>ymax)continue;
		if(axis->z()<zmin)continue;
		if(axis->z()>zmax)continue;
		strmnscale<<axis->x()<<" "<<axis->y()<<" "<<axis->z()<<endl;;
		float msx=mathNumscale( axis->x() ,factorx ,xmin,xmax,midletarget);
		strm<<msx<<" ";
		float msy=mathNumscale( axis->y() ,factorx,ymin,ymax,midletarget);
		strm<<msy<<" ";
		float msz=mathNumscale( axis->z() ,factorx,zmin,zmax,midletarget);
		strm<<msz<<endl;
		
		lop(j,0,sz){
			float ms=mathNumscale( cbres[j] ,factorx,ve[j]->anglemin,ve[j]->anglemax,midletarget);
			strm<<ms<<" ";
			strmnscale<<cbres[j]<<" ";
			// cot(ms);
		}
		count++;
		strm<<endl;
		strmnscale<<endl;
		// pausa
	}
	stringstream str;
	str<<count<<" "<<num_input<<" "<<num_output<<endl<<strm.str();
	ofstream ostrm("fann.txt");
	ostrm<<str.str();
	ofstream ostrmns("fann_noscale.txt");
	ostrmns<<strmnscale.str();
}
void train(){	
	int sz=ve.size();
	// float xmin=0;
	// float xmax=1200;
	// float ymin=120 , ymax=670;
	// float zmin=-600 , zmax=670;
	const unsigned int num_input = 3;
    const unsigned int num_output = sz;
    const unsigned int num_layers = 3;
    const unsigned int num_neurons_hidden = 10;
	const float desired_error = (const float) 0.009;
    const unsigned int max_epochs = 500000;
    const unsigned int epochs_between_reports = 100;
	// struct fann *ann = fann_create_standard(num_layers, num_input,num_neurons_hidden, num_output);
	struct fann *ann = fann_create_from_file("fann.net");

    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

	// fann_set_training_algorithm(ann, FANN_TRAIN_BATCH);
	
    fann_train_on_file(ann, "fann.txt", max_epochs,
        epochs_between_reports, desired_error);

    fann_save(ann, "fann.net");

    fann_destroy(ann);
	
	
}
void fann_calc(){
	fann_type *calc_out;
    fann_type input[3];
	struct fann *ann = fann_create_from_file("fann.net");
	input[0] = 0.0337729;
    input[1] = -0.212766;
    input[2] = -0.469913;
    calc_out = fann_run(ann, input);
	
	printf("xor test (%f,%f) -> %f\n", input[0], input[1], calc_out[3]);

    fann_destroy(ann);
}

#endif

//Lua
#if 1
// https://www.lua.org/manual/5.2/pt/manual.html
#ifdef WIN32 
#include <lua.hpp>
	#else
#include <lua5.3/lua.hpp> 
#endif
lua_State* lua_init();
int ltable(lua_State* L){ 
	int t = lua_type(L, 1); 
	if(t == LUA_TTABLE) {
        int len = lua_rawlen(L, 1); 
		lua_pushvalue(L, 1);
		lua_pushnil (L);
		while (lua_next (L, -2) != 0) { 
			cot( lua_tonumber(L,-1) ); 
			lua_pop (L, 1);
		}
 
	}  
	cot( lua_tonumber(L,2) ); 
	return 1;
}
int add(lua_State* L){ 
	float n1=lua_tonumber(L,1);
	float n2=lua_tonumber(L,2);
	lua_pushnumber(L,n1+n2);
	return 1;
}
int movz(lua_State* L){
	float n1=lua_tonumber(L,1);
	// movetoposz( n1 );;
	posa({},{},0,0,0,n1);
	return 1;
}
int posadebug(lua_State* L){
	posa_debug=lua_tonumber(L,1); 
	return 1;
}
int cancel_all(lua_State* L){
	cot(0);
	lop(i,0,pool.size()){
		pool[i].cancel=1;
	}
	return 1;
}
int posa(lua_State* L){
	cot(lua_gettop(L));
	int sz=lua_gettop(L); 
	vfloat p(sz);
	lop(i,0,sz){  
		if(lua_tostring (L,i+1)==NULL)
			p[i]=1000;
		else
			p[i]=lua_tonumber(L,i+1 );
	}
	posa(p);
	return 1;
}
int posi(lua_State* L){
	// cot(lua_gettop(L));
	int sz=lua_gettop(L); 
	vfloat p(sz);
	lop(i,0,sz)p[i]=lua_tonumber(L,i+1);

	posi(p[0],p[1]);
	return 1;
}
int pik2(lua_State* L){
	// cot(lua_gettop(L));
	int sz=lua_gettop(L); 
	vfloat p(sz);
	lop(i,0,sz)p[i]=lua_tonumber(L,i+1);

	// pik2(vec3(p[0],p[1],p[2]));
	
	
	return 1;
}
int posaik(lua_State* L){
	// cot(lua_gettop(L));
	int sz=lua_gettop(L); 
	vfloat p(sz);
	lop(i,0,sz)p[i]=lua_tonumber(L,i+1);

	posik(vec3(p[0],p[1],p[2]));
	
	
	return 1;
}
int func(lua_State* L){
	int id=lua_tonumber(L,1);
	sqlite3_stmt* st;
    sqlite3_prepare_v2(sql3, rprintf("select run from tabRobot where id='%d'",id),-1, &st, NULL);
	cot(id);
	while(sqlite3_step(st)== SQLITE_ROW){ 
	cot(sqlite3_column_text(st,0));
		lua_State* L=lua_init();
		luaL_loadstring(L, (const char*)sqlite3_column_text(st,0) );
		lua_pcall(L, 0, 0, 0);		
	} 
	sqlite3_finalize(st); 	
	return 1;
}
int view(lua_State* L){
	cot(88);
	// cot(lua_gettop(L));
	int sz=lua_gettop(L);
	if(sz<9)return 1;
	
	vfloat vf(sz);
	lop(i,0,sz){ ;
		vf[i]= lua_tonumber(L,i+1);   
	}
	Vec3d eye( vf[0] , vf[1] , vf[2]  );
	Vec3d center( vf[3] , vf[4] , vf[5]  );
	Vec3d up( vf[6] , vf[7] , vf[8]  ); 
	cot(center);
	tmr->setAutoComputeHomePosition(true);
	tmr->setHomePosition( eye, center, up );
	// tm->home(0.0);
	// tm->setPivot(Vec3f(0,0,0));
	osggl->setCameraManipulator(tmr);
	
	return 1;
}
lua_State* lua_init(){	
	lua_State* L=luaL_newstate();
	luaL_openlibs(L); 
	
	lua_pushcfunction(L,  add );
	lua_setglobal(L,"add");
	
	lua_pushcfunction(L,  movz );
	lua_setglobal(L,"movz");
	
	lua_pushcfunction(L,  func );
	lua_setglobal(L,"func");
	
	lua_pushcfunction(L,  posa );
	lua_setglobal(L,"posa");
	
	lua_pushcfunction(L,  posi );
	lua_setglobal(L,"posi");
	
	lua_pushcfunction(L,  pik2 );
	lua_setglobal(L,"pik2");
	
	lua_pushcfunction(L,  posaik );
	lua_setglobal(L,"posaik");
	
	lua_pushcfunction(L,  view );
	lua_setglobal(L,"view");
	
	lua_pushcfunction(L,  cancel_all );
	lua_setglobal(L,"cancel_all");
	
	lua_pushcfunction(L,  posadebug );
	lua_setglobal(L,"posadebug");
	
	lua_pushcfunction(L,  ltable );
	lua_setglobal(L,"ltable");
	
	return L;
}
void lua_str(string str){	
	ifstream input( "frobot.lua" );
	stringstream str1;
	str1<<input.rdbuf();
	str=str1.str()+"\n"+str;
	// cot(str);
	lua_State* L=lua_init();
	luaL_loadstring(L, str.c_str());
	lua_pcall(L, 0, 0, 0);
	
	
	lua_close(L);
}
void lua(){
	
	lua_State* L=lua_init();
	
	cout<<"LUA FILE"<<endl;
	luaL_dofile(L, "frobot.lua");
	
	// lua_getglobal(L,"x");
	// cout<<lua_tonumber(L,-1)<<endl;
	// lua_getglobal(L,"xx");
	// cout<<lua_tonumber(L,-1)<<endl;
	
	lua_close(L);
	
	// lua_pushstring(L, "nick");
	
	// lua_pushstring(L, "nick");         // push a string on the stack
	// lua_setglobal(L, "name");          // set the string to the global 'name'
	
}
void fparse_str(string text, int linepos){ 
	vector<string> strs=split(text,"\n");
	// cot(strs.size());
	// cot(linepos);
	if(linepos<0)return;
	cot(strs[linepos]);
	lua_str(strs[linepos]);	
}
#endif

//VIEW
#if 1 
void getview(){
	#define mathRound(n,d) ({float _pow10=pow(10,d); floorf((n) * _pow10 + 0.5) / _pow10;})
	osg::Vec3 eye, center, up;
	osggl->getCamera()->getViewMatrixAsLookAt( eye, center, up );
	cot(eye);
	cot(center);
	cot(up); 
	cout<<"view1: "<<mathRound(eye.x(),1)<<" "<<mathRound(eye.y(),1)<<" "<<mathRound(eye.z(),1)<<" "<<mathRound(center.x(),1)<<" "<<mathRound(center.y(),1)<<" "<<mathRound(center.z(),1)<<" "<<mathRound(up.x(),1)<<" "<<mathRound(up.y(),1)<<" "<<mathRound(up.z(),1)<<" "<<endl;
	cout<<"view( "<<mathRound(eye.x(),1)<<" , "<<mathRound(eye.y(),1)<<" , "<<mathRound(eye.z(),1)<<" , "<<mathRound(center.x(),1)<<" , "<<mathRound(center.y(),1)<<" , "<<mathRound(center.z(),1)<<" , "<<mathRound(up.x(),1)<<" , "<<mathRound(up.y(),1)<<" , "<<mathRound(up.z(),1)<<" )"<<endl;
}

#endif

//FlEditor
#if 1
struct FlEditor:Fl_Text_Editor{
	Fl_Text_Buffer  *texto = 0;
	// string fname="frobot.txt";
	string fname="frobot.lua";
	//tipo=0 editor do lua
	//tipo=1 editor do sqlite
	int tipo=0;
	FlEditor(int x,int y,int w, int h) : Fl_Text_Editor(x,y,w, h) {
		Fl::set_fonts();	
		texto = new Fl_Text_Buffer;
		texto->tab_distance(2);
		buffer(texto);
		//textfont(FL_HELVETICA);
		wrap_mode(Fl_Text_Editor::WRAP_AT_PIXEL, 0);
		linenumber_width(17);
		linenumber_size(9);
		textsize(12);
		// if(tipo==0)load(); 
	}
	int handle(int e){ 
		int ret=Fl_Text_Editor::handle(e);
		// if(e==FL_KEYDOWN &&  Fl::event_state() ==FL_CTRL && Fl::event_key()==102) find_cb();
		if(e==FL_KEYDOWN){
			if(tipo==0)save();
			//altgr
			if(!Fl::event_shift()  && (Fl::event_key()==65027 || Fl::event_key()==65514)){//altgr
				int new_pos = insert_position();
				int line = texto->count_lines(0, new_pos);
				// int ls = texto->line_start(new_pos);
				// cot(line);
				// cot(ls);
				fparse_str(texto->text(),line);
				kf_down(0,this);
			}  
			if(Fl::event_shift()  && (Fl::event_key()==65027 || Fl::event_key()==65514) ){
				//run all text
				lua_str(texto->text());
			}
			// cot(Fl::event_key());
		}
		if(e==FL_PUSH ){
			// cot(texto->selection_text()); 
		}
		return ret;
	}
	void load(){
		string line;
		stringstream lines;
		ifstream myfile (fname);
		if (myfile.is_open()){
			while ( getline (myfile,line) ){
				lines<< line << '\n';
			}
			myfile.close();
			string t=lines.str();
			boost::replace_all(t, "\r", "");
			texto->text(t.c_str());
		}
		else cout << "Unable to open file "<<fname<<endl; 
	}
	void save(){
		string t=texto->text();
		boost::replace_all(t, "\r", ""); 
		ofstream o(fname);
		o<<t;
	}


};

FlEditor* fle;
#endif

int Break=0;
bool makvisible=1;
bool robotvisible=1;
//OPENCV
#if 1
#include "Fl_ViewerCV.h"
Fl_ViewerCV* flcv=0;
 
struct flocvs{
	cv::VideoCapture cap;
	cv::Mat* dst;
	cv::Mat frame;
	flocvs(){ 
		if(!cap.open(0))// open the default camera, use something different from 0 otherwise; Check VideoCapture documentation.
			return ; 
		// cap.set(CV_CAP_PROP_FRAME_WIDTH , 352);
		// cap.set(CV_CAP_PROP_FRAME_HEIGHT , 288);
		dst=new cv::Mat;
		Fl::add_timeout(1.0/24.0, Timer_CB, (void*)this);
	}
	void draw(){ 
		if(Break)return;
		cap >> frame;
		if( frame.empty() ) return; 
		cv::flip(frame, *dst, 1); 
		flcv->SetImage(dst); 
 
	} 
	static void Timer_CB(void *userdata) { 
		if(Break)return;
		flocv->draw();
        Fl::repeat_timeout(1.0/12, Timer_CB, userdata);
    } 
	void close(){
		cap.release();
	}
};
#endif

#if 1 //SQLITE3 && FLTK
 
struct Fl_Scroll_p:Fl_Scroll{ 
	vector<float> ch;
	vector<float> cw;
	vector<float> cx;
	vector<float> cy;
	Fl_Scroll_p(int X,int Y,int W, int H) : Fl_Scroll(X,Y,W,H) {  
	};
	void resize (int X, int Y, int W, int H){ 
		Fl_Widget::resize(X,Y,W,H);
 		int dx = X-x(), dy = Y-y();
		int dw = W-w(), dh = H-h();
		// fix_scrollbar_order(); 
		float wf=1.35;
		float hf=1.3; 
		//preserve initial values
		bool fillp=0;
		if(ch.size()==0){
			ch=vector<float>(children());
			cw=vector<float>(children());
			cx=vector<float>(children());
			cy=vector<float>(children());
			fillp=1;
		} 
		Fl_Widget*const* a = array();
		// for (int i=0;i<children();i++) {
		for (int i=children()-2; i--;) {
			Fl_Widget* o = *a++; 
			if(fillp){
				ch[i]=o->h();
				cw[i]=o->w();
				cx[i]=o->x();
				cy[i]=o->y();
			}
			// o->resize(o->x()+dx, o->y()+dy,o->w()*wf,o->h()*hf);
			o->resize(cx[i]+dx, cy[i]+dy,cw[i]*wf,ch[i]*hf); 
		} 
		Fl_Widget::resize(X,Y,W,H); // resize _before_ moving children around  
	}	
};


vector<Fl_Input*> flml_id;
vector<Fl_Input*> flml_time;
vector<Fl_Input*> flml_desc;
vector<FlEditor*> flml_run; 
Fl_Double_Window* flscrolldiv;  
Fl_Scroll* flscroll; 
sqlite3_stmt* st;
int idx=0;
void fill_input(int idx,bool newr=0);

void input_callback(Fl_Widget *, void* v){
	int vo=*((int*)&v);
	cot(vo);
	cot(flscroll->w());
	int id=atoi(flml_id[vo]->value());
	cot(id);
	cot(idx);
	if(idx==vo){//insert
        sqlite3_exec(sql3, rprintf("insert into tabRobot (time) values('%s')","") ,NULL,NULL,NULL);
		flml_id[vo]->value(to_string(sqlite3_last_insert_rowid(sql3)).c_str());
		idx++;
		int yp= (flscroll->yposition ());
		flscroll->scroll_to(0,120); //resolve bug do fltk
		fill_input(idx+0,1);
		flscroll->scroll_to(0,yp);
	}else{//update
        sqlite3_exec(sql3, rprintf("update tabRobot set time='%s',desc='%s',run='%s' where id='%s'",flml_time[vo]->value(),flml_desc[vo]->value(),flml_run[vo]->texto->text(),flml_id[vo]->value()   ) ,NULL,NULL,NULL);
	
		
	}
}
void fill_input(int idx,bool newr){	
	int hh=18;
	int hh1=90;
	flml_id.push_back(new Fl_Input(0,idx*hh1,30,hh));
	flml_id.back()->type(FL_INPUT_READONLY);
	if(newr==0)flml_id.back()->value( (const char*)sqlite3_column_text(st,0) );
	flml_time.push_back(new Fl_Input(30,idx*hh1,40,hh));
	if(newr==0){
		string str_time=(const char*)sqlite3_column_text(st,1);	
		if(str_time.size()>5)str_time=str_time.substr(0, str_time.size()-3);
		flml_time.back()->value( str_time.c_str() );	
	}
	flml_time.back()->callback(input_callback,(void*)idx);
	flml_desc.push_back(new Fl_Input(70,idx*hh1,50,hh));
	if(newr==0)flml_desc.back()->value( (const char*)sqlite3_column_text(st,2) );
	flml_desc.back()->callback(input_callback,(void*)idx);
	flml_run.push_back(new FlEditor(0,idx*hh1+hh,206,hh1-hh));
	flml_run.back()->tipo=1;
	if(newr==0)flml_run.back()->texto->text( (const char*)sqlite3_column_text(st,3) );
	flml_run.back()->callback(input_callback,(void*)idx);
	flml_run.back()->when(FL_WHEN_CHANGED);
	// flml_run.back()->resizable(flt);
	// flml_run.back()->resize(flml_run.back()->x(),flml_run.back()->y(),flscroll->w(),flml_run.back()->h());
	flscroll->add(flml_id.back());
	flscroll->add(flml_time.back());
	flscroll->add(flml_desc.back());
	flscroll->add(flml_run.back());
	flscroll->redraw(); 
	
}
 int sql_count(string sql){
	sql="SELECT count(*) from ("+sql+")";
    sqlite3_prepare_v2(sql3, sql.c_str(),-1, &st, NULL);
	int count=0;
	while(sqlite3_step(st)== SQLITE_ROW){ 
		count=atoi( (const char*)sqlite3_column_text(st,0) );
	}
	sqlite3_finalize(st);  
	return count;
}
Fl_Button* bt14; 
Fl_Input_Choice* search_in;
void choice_cb(Fl_Widget *w, void *userdata) {
	lop(i,0,flml_id.size()){
		Fl::delete_widget (flml_id[i]);
		Fl::delete_widget (flml_time[i]);
		Fl::delete_widget (flml_desc[i]);
		Fl::delete_widget (flml_run[i]);
	}
	flml_id.clear();
	flml_time.clear();
	flml_run.clear();
	flml_desc.clear();
	string txt=(search_in->input()->value());
	// int exit = 0;
    // exit = sqlite3_open("frobot.sqlite", &sql3);
	string sql="select * from tabRobot where desc like '%"+txt+"%'";  
    sqlite3_prepare_v2(sql3, sql.c_str(),-1, &st, NULL);
	idx=0;
	while(sqlite3_step(st)== SQLITE_ROW){  
		fill_input(idx);
		idx++;
	}
	fill_input(idx,1); //new
	threadDetach([]{sleepms(50);cot(flscroll->yposition ());flscroll->scroll_to(0,0);flscroll->redraw(); flscroll->damage(1);}); 
	flscroll->scroll_to (0, 0);
	flscroll->redraw();
	sqlite3_finalize(st);
}
// http://localhost/vivision/phpliteadmin.php?database=..%2Frobot%5Crobot.sqlite
void sql3_init(){
	Fl_Button* bt13=new Fl_Button(0,  90, 30, 30,"luaTx");
	bt14=new Fl_Button(30,  90, 30, 30,"bdAll");
	Fl_Button* bt15=new Fl_Button(60,  90, 30, 30,"bdRun");	
	bt13->down_color(FL_GREEN );
	bt14->down_color(FL_GREEN );
	bt15->down_color(FL_GREEN );
	bt13->type(FL_RADIO_BUTTON);
	bt14->type(FL_RADIO_BUTTON);
	bt15->type(FL_RADIO_BUTTON);
	bt13->callback([](Fl_Widget *, void* v){  
		fle->show();
		flscroll->hide();
	});
	bt14->callback([](Fl_Widget *, void* v){  
		fle->hide();
		flscroll->show(); 
	});	 		 
	
	// Fl_Button* in=new Fl_Button(0,120,100,30,"Test");
	search_in=new Fl_Input_Choice(0,120,160,30,"Test");
    search_in->callback(choice_cb, 0);
	// in->align(FL_ALIGN_LEFT);
    search_in->menubutton()->add("one");
    search_in->menubutton()->add("two");
    search_in->menubutton()->add("three");
    // in.menuvalue(0);
    // Fl_Button onoff(0,150,200,28,"Activate/Deactivate");
    // onoff.callback(buttcb, (void*)&in);
	
	// return;
	flscroll=new Fl_Scroll(0,150,160,480-150); 
	
    int exit = 0;
    exit = sqlite3_open("frobot.sqlite", &sql3);
	string sql="select * from tabRobot"; 
	// resize_flscroll(sql);
    sqlite3_prepare_v2(sql3, sql.c_str(),-1, &st, NULL);
	idx=0;
	while(sqlite3_step(st)== SQLITE_ROW){ 
		fill_input(idx);
		idx++;
	}
	// 
	fill_input(idx,1); //new
	sqlite3_finalize(st);  
	
	// flscroll->resizable(flt);
	flscroll->end();  
	// lop(i,1,150)	fill_input(idx+i,1);
	// flml_run.back()->do_callback();
	// flml_id.back()->hide();
}
#endif

#if 1 //TIMER
Fl_Input* time_input;
Fl_Toggle_Button* time_input_btn;

void time_f(){	
	string time_input_str= (time_input->value());
	int hour=atoi( time_input_str.substr(0, 2).c_str() ); 
	int minute=atoi( time_input_str.substr(3, 2).c_str() );
	
	cot(minute);
	sqlite3_stmt* st;
	string m="0"+to_string(minute);
	if(m.size()>2)m=m.substr(1,2);
	string h=to_string(hour)+":"+ m;
	// cot(rprintf("select run from tabRobot where time='%s'",h.c_str()));
    sqlite3_prepare_v2(sql3, rprintf("select id,run from tabRobot where time='%s'",h.c_str()),-1, &st, NULL); 
	while(sqlite3_step(st)== SQLITE_ROW){ 
	cot(sqlite3_column_text(st,0));
		// cot((const char*)sqlite3_column_text(st,0) );
		
	posa_mtx.lock();
		lua_State* L=lua_init();
		luaL_loadstring(L, (const char*)sqlite3_column_text(st,1) );
		lua_pcall(L, 0, 0, 0);
	posa_mtx.unlock();
	} 
	sqlite3_finalize(st);
}

void interval(){ 
	time_input_btn->value(1);
	time_input_btn->down_color(FL_RED ); 
	time_input->value("0:00");
	threadDetach([]{
		for(;;){
			if(time_input_btn->value()==0){ //loop off
				time_t timer;
				time(&timer);
				struct tm* tms=localtime(&timer);
				time_input->value( (to_string(tms->tm_hour)+":"+(tms->tm_min<10?"0":"")+to_string(tms->tm_min)).c_str()); 
				time_f();
			}
			sleepms(1000);
		}	
	});
}
#endif


void test(){
	int servoMin = 500 ;
	int servoMax = 2500 ;
	cout<<pmap(160,0,320,servoMin, servoMax);
}

#if 1 //WMOVING
vbool wmoving(4);
osg::Vec3 fix_center_bug;
void wmove(){	
	// return;
   // cot(wmoving);
	osg::Vec3 eye, center, up;
	osggl->getCamera()->getViewMatrixAsLookAt( eye, center, up,50 ); 
	center=fix_center_bug;
	// tmr->setAutoComputeHomePosition(0);
	if(wmoving[0]){
		center.z()+=40; 
		eye.z()+=40; 
	}
	if(wmoving[1]){
		center.z()-=40; 
		eye.z()-=40; 
	}
	if(wmoving[2]){
		center.x()-=40; 
		eye.x()-=40; 
	}
	if(wmoving[3]){
		center.x()+=40; 
		eye.x()+=40; 
	}
	tmr->setHomePosition( eye, center, up );
	// tm->home(0.0);
	// tm->setPivot(Vec3f(0,0,0));
	osggl->setCameraManipulator(tmr);
	fix_center_bug=center;
}
int OnKeyPress(int Key, Fl_Window *MyWindow) { 
	if(Fl::focus()!=osggl)return Fl::handle_(Key, MyWindow);
	if(Key == FL_KEYDOWN) {
		// cot( Fl::event_key());
		int dkey=Fl::event_key();
		if(dkey==119)wmoving[0]=1;//w
		if(dkey==115)wmoving[1]=1;//s
		if(dkey==97)wmoving[2]=1;//a
		if(dkey==100)wmoving[3]=1;//d
		wmove();		 
      // cot( Fl::event_text());
		osggl->_gw->getEventQueue()->keyPress((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
 
   }
   if(Key == FL_KEYUP) {
		int dkey=Fl::event_key();
		if(dkey==119)wmoving[0]=0;//w
		if(dkey==115)wmoving[1]=0;//s
		if(dkey==97)wmoving[2]=0;//a
		if(dkey==100)wmoving[3]=0;//d
		wmove();
		osggl->_gw->getEventQueue()->keyRelease((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
      // cot( Fl::event_text());
 
   }
   return Fl::handle_(Key, MyWindow);
}
#endif
int main(){   
	// test(); return 0;
// https://www.fltk.org/doc-1.3/opengl.html#opengl_drawing
// https://osg-users.openscenegraph.narkive.com/HiVHDrXM/change-camera-position
// https://dis.dankook.ac.kr/lectures/med08/wp-content/uploads/sites/35/1/1321343577.pdf 
    

	int w=800+300;
	int h=480;
    Fl::scheme("plastic");	
	// win=new Fl_Double_Window(0,0,w,h,"frobot");   
	win=new Fl_Double_Windowc(0,0,w,h,"frobot");  
	flt=new Fl_Double_Window(0,   0, 160, 480);
	Fl_Button* bt1=new Fl_Button(0,  0, 30, 30,"Alpha");
	Fl_Button* bt2=new Fl_Button(30,  0, 30, 30,"rota");
	Fl_Button* bt3=new Fl_Button(60,  0, 30, 30,"gview");
	Fl_Button* bt4=new Fl_Button(90,  0, 30, 30,"MV");
	Fl_Button* bt5=new Fl_Button(120,  0, 30, 30,"fit");
	Fl_Button* bt6=new Fl_Button(0,  30, 30, 30,"ik");
	Fl_Button* bt7=new Fl_Button(30,  30, 30, 30,"lua");
	Fl_Button* bt8=new Fl_Button(60,  30, 30, 30,"Lrun");
	Fl_Button* bt9=new Fl_Button(90,  30, 30, 30,"rvis");
	Fl_Button* bt10=new Fl_Button(120,  30, 30, 30,"ucs");
	Fl_Button* bt11=new Fl_Button(0,  60, 30, 30,"dbg");
	Fl_Button* bt12=new Fl_Button(30,  60, 30, 30,"mak");
	time_input=new Fl_Input(60,  60, 30, 30 );
	time_input_btn=new Fl_Toggle_Button(90,  60, 30, 30,"hdbg");
	Fl_Button* time_f_btn=new Fl_Button(120,  60, 30, 30,"Tdbg");
	

	
	// Fl_Input_Choice in(0,150,100,28,"Test");
    // in.menubutton()->add("one");
    // in.menubutton()->add("two");
    // in.menubutton()->add("three");
	
	sql3_init();
	// win->show(); Fl::run(); return 0;
	
	fle=new FlEditor(0,150,160,400-150);
	fle->load();
	flt->resizable(fle);   
	flt->end();
	flcv=new Fl_ViewerCV(800,0,300,300);
	osggl=new ViewerFLTK(160,  0, 800-160, 480-10); 
	Fl::event_dispatch(OnKeyPress);
	osgViewer::StatsHandler *sh = new osgViewer::StatsHandler;
	sh->setKeyEventTogglesOnScreenStats('p');
    osggl->addEventHandler(sh);
    // osggl->addEventHandler(new osgViewer::StatsHandler); 
   
	tmr=new osgGA::TrackballManipulator;
	
	// fparse(fle->texto->text(),9);
 
	
	// flocv=new flocvs;
	 
	// setview();
 
	loadstl(group);

	geraeixos(group);
	osggl->setSceneData(group);
	
	
	
	//botoes dos angulos
	Fl_Double_Window* flpos=new Fl_Double_Window(800, 310, 300, 480-300);   
	// Fl_Double_Windowc* flpos=new Fl_Double_Windowc(800, 310, 300, 480-300);   
	Fl_Box* flposbox=new Fl_Box(0, 0, 300, 480-300);   
	// cot(flpos->w());
	// vector<vector<Fl_Button*>> btp(ve.size());
	// struct vix{int index;float angle; };	
	// vector<vector<vix*>> vixs(> btp(ve.size())
	btp=vector<vector<Fl_Button*>> (ve.size()); 
	vixs=vector<vector<vix*>> (ve.size());
		// cot(ve.size());
	lop(vei,0,ve.size()){
		// cot(button_width);
		// string bs=
		float btn=ve[vei]->anglemin/10.0*10;
		int btnsz=( ve[vei]->anglemax/10.0 ) - btn/10+1;
		// cot(btnsz);
		// cot(btn);
		// cot(ve[vei]->anglemax/10.0*10);
		btp[vei]=vector<Fl_Button*>(btnsz);
		vixs[vei]=vector<vix*>(btnsz);
		float button_width=flpos->w()/(float)btp[vei].size();
		lop(i,0,btp[vei].size()){ 
			const char* button_val=to_string(i).c_str();
			btp[vei][i]=new Fl_Button(i*button_width,20*vei,button_width,20); 
				// btp[vei][i]->color(FL_GREEN);
			// btp[i]->copy_label(button_val);
			btp[vei][i]->copy_label(to_string((int)abs(btn/10)).c_str() );
		// cot(to_string((int)abs(btn/10)));
			vix* vsend=new vix({vei,btn});
			vixs[vei][i]=vsend;
			btp[vei][i]->callback([](Fl_Widget *, void* v){ 
				vix* vv=(vix*)v; 
				// cot(vv->index);
				// cot(vv->angle);
				posv p;
				ve[vv->index]->rotatetoposition(vv->angle,&p);
				// ve[vv->index]->rotate_pos(vv->angle);
			},(void*)vsend);
			btn+=10;
		}
	}
	flpos->resizable(flposbox);
	flpos->end();
   
   

	// osg::Matrix mat = osg::computeWorldToLocal(loadedModel.get());        
// std::cout << "X: " << mat.getTrans().x() << std::endl;      
// std::cout << "X: " << mat.getTrans().y() << std::endl;      
// std::cout << "X: " << mat.getTrans().z() << std::endl;
// std::cout << "Rot X: " << mat.getRotate().x() << std::endl;
// std::cout << "Scale X: " << mat.getScale().x() << std::endl;

	
	bt1->callback([](Fl_Widget *, void* v){ 	  
		toggletransp();	  
	});
	
	bt2->callback([](Fl_Widget *, void* v){ 
		// flgl->resize(flgl->x(),flgl->y(),200,200); 
			// ve[1]->rotate( 10);
			// cot(*ve[1]->axisbegin ); 
			// cot(*ve[1]->axisend );  		
		threadDetach([]{
			lop(i,0,1900000000){
				sleepms(10); 
				if(Break)return;
				// ve[0]->rotate( 1);
				if(ve[0]->angle>180)ve[0]->dir=-1;
				if(ve[0]->angle<-140)ve[0]->dir=1;
				ve[0]->rotate( 1*ve[0]->dir);
				// cot(ve[0]->angle);
				
				
				if(ve[1]->angle>90)ve[1]->dir=-1;
				if(ve[1]->angle<-90)ve[1]->dir=1;
				ve[1]->rotate( 2*ve[1]->dir); 
				// cot(*ve[0]->axisend );
			}
		});
	}); 
			
	bt3->callback([](Fl_Widget *, void* v){  
		// ve[0]->rotate(Vec3f(0,0,1),45); 
		getview();
		// Break=1;
		
			// ve[0]->rotate( 10); 
			// cot(*ve[1]->axisbegin ); 
			// cot(*ve[1]->axisend );  
	});	
	
	bt4->callback([](Fl_Widget *, void* v){ 	
		makvisible = !makvisible;
        maquete->setNodeMask(makvisible ? 0xffffffff : 0x0);	
	});
	
	bt5->callback([](Fl_Widget *, void* v){ 
		ViewerFLTK* view=osggl;
		if ( view->getCamera() ){
			// http://ahux.narod.ru/olderfiles/1/OSG3_Cookbook.pdf
			double _distance=-1; float _offsetX=0, _offsetY=0;
			osg::Vec3d eye, center, up;
			view->getCamera()->getViewMatrixAsLookAt( eye, center,
			up );

			osg::Vec3d lookDir = center - eye; lookDir.normalize();
			osg::Vec3d side = lookDir ^ up; side.normalize();

			const osg::BoundingSphere& bs = view->getSceneData()->getBound();
			if ( _distance<0.0 ) _distance = bs.radius() * 3.0;
			center = bs.center();
			center -= (side * _offsetX + up * _offsetY) * 0.1;
			fix_center_bug=center;
			tmr->setHomePosition( center-lookDir*_distance, center, up );
			osggl->setCameraManipulator(tmr);
			getview();
		}	
	});
	
	bt6->callback([](Fl_Widget *, void* v){ 	
		// ve[ve.size()-1]->rotate_posk(10);	
		dbg_pos();
		ve[3]->posik(vec3(150,150,-150));
	});
	
	bt7->callback([](Fl_Widget *, void* v){ 	
		if(fle->fname=="frobot.lua")
			fle->fname="frobot_f.lua";
		else
			fle->fname="frobot.lua";
		fle->load();
	});
	
	bt8->callback([](Fl_Widget *, void* v){ 	
		lua();	
	});
	
	bt9->callback([](Fl_Widget *, void* v){ 	
		robotvisible = !robotvisible;
		lop(i,0,ve.size()) lop(j,0,ve[i]->nodes.size())
			ve[i]->nodes[j]->setNodeMask(robotvisible ? 0xffffffff : 0x0);	
	});

	bt10->callback([](Fl_Widget *, void* v){ 	 
        ucs_icon->setNodeMask( (ucs_icon->getNodeMask()==0x0) ? 0xffffffff : 0x0);	
	});
	
	bt11->down_color(FL_RED ); 
	bt11->type(FL_TOGGLE_BUTTON);
	bt11->callback([](Fl_Widget *, void* v){ 
		posa_debug=!posa_debug;
		dbg_pos();	
	});
	
	
	bt12->callback([](Fl_Widget *, void* v){ 	 
        maquete->setNodeMask( (maquete->getNodeMask()==0x0) ? 0xffffffff : 0x0);	
	});
	
	time_f_btn->callback([](Fl_Widget *, void* v){ 	 
        time_f();	
	});
	
	
	bt5->do_callback();
	// fix_center_bug.x()=500;
	// fix_center_bug.y()=500; 
	// cot(fix_center_bug.x());
	// wmove();
	
	interval(); //perform tasks of the robot
	
	fle->hide();
	flscroll->show();   
	threadDetach([]{sleepms(1200);cot(flscroll->yposition ());flscroll->scroll_to(0,0);flscroll->redraw(); flscroll->damage(1);}); 
	bt14->setonly(); 
	
	// fann_init();
	// train();
	// fann_calc();
	
	Fl::scheme("gtk+"); 
	#if 1
	((Fl_Widget*)win)->callback([](Fl_Widget *widget, void* v){ 
	    Fl_Window *window = (Fl_Window *)widget;		
		if (Fl::event()==FL_SHORTCUT && Fl::event_key()==FL_Escape) 
			return; // ignore Escape
		Break=1;
		if(flocv)flocv->close();
		sleepms(50);
		// threadDetach([]{
			// sleepms(500);
			// exit(0);
		// });
			// sleepms(100);
			exit(0); 
	}); 
	#endif 
	win->clear_visible_focus(); 	 
	win->color(0x7AB0CfFF);
	win->resizable(win);
	
	int X,Y,W,H;
	Fl::screen_work_area(X,Y,W,H,0,0);	
	win->resize(X,Y,W*.8,H*.8);
	
	win->position(Fl::w()/2-win->w()/2,0);
	win->show();   


	osggl->make_current();
#ifdef GLEW
  	glewExperimental = GL_TRUE;
	glewInit();
	//Print info of GPU and supported OpenGL version
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("OpenGL version supported %s\n", glGetString(GL_VERSION));
#ifdef GL_SHADING_LANGUAGE_VERSION
	printf("Supported GLSL version is %s.\n", (char *)glGetString(GL_SHADING_LANGUAGE_VERSION));
#endif
    printf("Using GLEW version %s.\n", glewGetString(GLEW_VERSION));
	printf("------------------------------\n");  
#endif
	Fl::run();
}
//To do
//acabar o pik2
//adjust to 40' container