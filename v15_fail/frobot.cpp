// http://ahux.narod.ru/olderfiles/1/OSG3_Cookbook.pdf https://titanwolf.org/Network/Articles/Article?AID=306e6b2e-3e45-4cc2-8f86-3e674ff557c3#gsc.tab=0
//math collision detection two cubes https://gamedev.stackexchange.com/questions/60505/how-to-check-for-cube-collisions

#ifdef _WIN32not
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
#include <FL/Fl_Double_Window.H>
// #include <FL/Fl_Browser.H>
#include <FL/Fl_Button.H> 
#include <FL/Fl_Choice.H>
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
				_gw->getEventQueue()->keyPress((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
				return 1;
			case FL_KEYUP:
				_gw->getEventQueue()->keyRelease((osgGA::GUIEventAdapter::KeySymbol)Fl::event_key());
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

};


Fl_Double_Window* flt; 
ViewerFLTK* osggl;
unordered_map<string,osg::ref_ptr<osg::Node>> stlgroup;
osg::Group* group = new osg::Group(); 
struct osgdr;
vector<osgdr*> ve;


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

bool posa_debug=1;
int posa_counter=0;
vvint posapool;
std::mutex posa_mtx;
mutex posa_counter_mtx;
				
typedef osg::Vec3f vec3f; 
typedef osg::Vec3f vec3; 
#define Pi 3.141592653589793238462643383279502884L
#define Pi180 0.01745329251

float distance_two_points(vec3* point1,vec3* point2){
	// cot(*point2);
	return sqrt( pow(point2->x()-point1->x(),2) +pow(point2->y()-point1->y(),2) +pow(point2->z()-point1->z(),2)    );	
}
void bound_box();
void dbg_pos();
void arm_len_fill();
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
	osg::ref_ptr<osg::Vec3Array> pointsik;
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
		cot(index);
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
		arm_len_fill();
	}
	
	void gooffset(){
		if(offset==0)offset=new vec3(0,0,0);
		
	}
	

	
	void rotate_posk(float newangle){	
			float nangle=angle-newangle; 
			rotateik(nangle*-1);
	
	}
	//ve[index]-> == this
	//posa
	void rotatetoposition(float newangle){	
		thread th([&](float newangle ){
			if(moving==1){
				// moving=0;
			}
			mtxlock(index+1);
			moving=1;
			// cot(angle);
			// cot(newangle);
			// if(newangle>anglemax)return;
			float nangle=newangle-angle;
			// cot(anglemax);
			// rotate( nangle);
			if(nangle>0){
				float dir=1;
				for(;;){
					if(moving==0)break;
					if(angle>anglemax)break;
					if(nangle<=0)break;
					rotate( dir);
					// cot(nangle);
					// cot(angle);
					nangle-=dir;
					sleepms(20);
				
				}
			}
			if(nangle<0){
				float dir=-1;
				for(;;){
					if(moving==0)break;
					if(angle<anglemin)break;
					if(nangle>=0)break;
					rotate( dir);
					nangle-=dir;
					sleepms(20);
				
				}
			}
			moving=0;
			mtxunlock(index+1);
				posa_counter_mtx.lock();
				posa_counter--;
				// cot(posa_counter);
				posa_counter_mtx.unlock();
		},newangle);
		th.detach( ); 
		
	
	} 
	void rotate( float _angle ){
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
		
		//copy points
		lop(i,0,ve.size()){
			
			ve[i]->pointsik = new osg::Vec3Array;
			ve[i]->pointsik->push_back(ve[i]->points[0][0]);
			ve[i]->pointsik->push_back(ve[i]->points[0][1]); 
			ve[i]->axisbeginik=&ve[i]->pointsik[0][0];
			ve[i]->axisendik=&ve[i]->pointsik[0][1]; 
		}
		 
		
		vec3 axisb=*axisbegin; 
		// angle+=_angle;
		// cot(angle);
		 		
		osg::Matrix Tr;
		Tr.makeTranslate( axisb.x(),axisb.y(),axisb.z() );
		osg::Matrix T; 
		T.makeTranslate( -axisb.x(),-axisb.y(),-axisb.z() ); 
		
		lop(i,0,index){		
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*-ve[i]->angle, ve[i]->axis ); 
			lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j]   * T * Ra * Tr   ; 
			// transform->setMatrix(transform->getMatrix()   * T * Ra * Tr ); 
		}
		osg::Matrix R; 
		R.makeRotate( Pi180*_angle*rotatedir, axis ); 	 
		lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j] * T * R * Tr  ; 
		// transform->setMatrix(transform->getMatrix()   * T * R * Tr   );
		 
		for(int i=index-1;i>=0;i--)	{ 
			osg::Matrix Ra; 
			Ra.makeRotate( Pi180*ve[i]->angle, ve[i]->axis ); 
			lop(j,0,pointsik[0].size())pointsik[0][j] = pointsik[0][j]    * T * Ra * Tr   ; 
			// transform->setMatrix(transform->getMatrix()   * T * Ra * Tr    );
		} 
  
		 
		//todas as posteriores teem que rodar tambem 
		lop(i,index+1,ve.size()-0){
			// cot(ve[i]->nodesstr);
				 
			lop(jj,0,index)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*-ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->pointsik[0].size())ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * R * Tr   );
			lop(j,0,ve[i]->pointsik[0].size()){ 
				ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * R * Tr   ;  
			}
			
			for(int jj=index-1;jj>=0;jj--)	{		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->pointsik[0].size())ve[i]->pointsik[0][j] = ve[i]->pointsik[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->drw->dirty();
			ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
			ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
		}

		// angle+=_angle;
		 
		
		
 
		
		// lop(i,0,ve.size()) cout<<i<<" "<<ve[i]->angle<<"  ";cout<<endl; 
		cout<<"posa: "; lop(i,0,ve.size()) cout<<i<<","<<ve[i]->angle<<"  ";cout<<endl; 
		cout<<"\t"<<"axisbx\taxisby\taxisbz\t"<<"axisex\taxisey\taxisez\t"<<"axisbxk\taxisbyk\taxisbzk\t"<<"axisexk\taxiseyk\taxisezk\t"<<endl;
		lop(i,0,ve.size()){
			cout<<"idx"<<i<<"\t"<<(int)(ve[i]->axisbegin->x())<<"\t"<<(int)(ve[i]->axisbegin->y())<<"\t"<<(int)(ve[i]->axisbegin->z())<<"\t"<<(int)(ve[i]->axisend->x())<<"\t"<<(int)(ve[i]->axisend->y())<<"\t"<<(int)(ve[i]->axisend->z())<<"\t"<<(int)(ve[i]->axisbeginik->x())<<"\t"<<(int)(ve[i]->axisbeginik->y())<<"\t"<<(int)(ve[i]->axisbeginik->z())<<"\t"<<(int)(ve[i]->axisendik->x())<<"\t"<<(int)(ve[i]->axisendik->y())<<"\t"<<(int)(ve[i]->axisendik->z())<<endl;;
		} 
	}; 

	//returns angles to given point
	vector<float> posik(vec3 topoint){
		vfloat angles(ve.size());
		vec3 axisb=*axisbegin;
		#define Calc_Distance(a,b) distance_two_points(axisb
	
	}
};

void arm_len_fill(){
	lop(i,0,ve.size()-1){
		ve[i]->arm_len=distance_two_points(ve[i]->axisbegin,ve[i+1]->axisbegin);
		cot(ve[i]->arm_len);
	}
}
void dbg_pos(){
	if(!posa_debug) return;
	// arm_len_fill();
	// lop(i,0,ve.size()) cout<<i<<" "<<ve[i]->angle<<"  ";cout<<endl; 
	cout<<"posa: "; lop(i,0,ve.size()) cout<<i<<","<<ve[i]->angle<<"  ";cout<<endl;		
	cout<<"posa( "; lop(i,0,ve.size()-1) cout<<ve[i]->angle<<" , "; cout<<ve.back()->angle<<" )"; cout<<endl;		
	cout<<"\t"<<"axisbx\taxisby\taxisbz\t"<<"axisex\taxisey\taxisez\t"<<endl;
	lop(i,0,ve.size()){
		cout<<"idx"<<i<<"\t"<<(int)(ve[i]->axisbegin->x())<<"\t"<<(int)(ve[i]->axisbegin->y())<<"\t"<<(int)(ve[i]->axisbegin->z())<<"\t"<<(int)(ve[i]->axisend->x())<<"\t"<<(int)(ve[i]->axisend->y())<<"\t"<<(int)(ve[i]->axisend->z())<<endl;;
	}
	cot(distance_two_points(ve[1]->axisbegin,ve[3]->axisbegin) );
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
}

void movetoposz(float z){
	thread th([&](float z){
		float currz=(float)((*ve[0]->axisbegin).z());
		float rz=z-currz;
		float speed=2;
		cot (currz);
		cot(rz);
		float newz=rz;
		int dir=1;
		if(z<=currz){dir=-1;   }
		for(;;){
			// cot(dir);
			// cot(newz);
			if(dir==1 && newz<=0)break;
			if(dir==-1 && newz>=0)break;
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
			sleepms(5);
		}
		dbg_pos();
	},z);
	th.detach();
}
void geraeixos(Group* group){ 
	
	// vec3* offset=new vec3(0,0,0);
	//robot offset
	vec3* offset=new vec3(610-110,272,-300);
	
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
	ve[idx]->anglemax=100;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(-42.19,271,-42.19),vec3(-150,271,-42.19));
	
	idx=3;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_armj2_1.stl"); 
	ve[idx]->axis=vec3(0,1,0);
	ve[idx]->anglemin=-50;
	ve[idx]->anglemax=170;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(-300,281,-42.19),vec3(-600,281,-42.19));
 
	goffset(offset);
	
 
}
osg::ref_ptr<osg::Node> maquete;
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
	
	
    osg::ref_ptr<osg::Node> ucs_icon = osgDB::readRefNodeFile("stl/3DUCSICON2.stl");
	settranparency(ucs_icon.get(),1);
	group->addChild(ucs_icon.get());
	
	
	
    // osg::ref_ptr<osg::Node> estrutura = osgDB::readRefNodeFile("Assembly1.stl");
	// settranparency(estrutura.get(),1);
	// group->addChild(estrutura.get());
	
};
 
//Lua
#if 1
// https://www.lua.org/manual/5.2/pt/manual.html
#ifdef WIN32 
#include <lua.hpp>
	#else
#include <lua5.3/lua.hpp> 
#endif
int add(lua_State* L){
	float n1=lua_tonumber(L,1);
	float n2=lua_tonumber(L,2);
	lua_pushnumber(L,n1+n2);
	return 1;
}
int movz(lua_State* L){
	float n1=lua_tonumber(L,1);
	movetoposz( n1 );;
	return 1;
}
int posadebug(lua_State* L){
	posa_debug=lua_tonumber(L,1); 
	return 1;
}
int posa(lua_State* L){
	cot(lua_gettop(L));
	int sz=lua_gettop(L); 
	vint p(sz);
	lop(i,0,sz)p[i]=lua_tonumber(L,i+1);

	posapool.push_back(p);
	if(posapool.size()>1)return 1;
	std::thread thm([](int sz ){
		while(posapool.size()!=0){ 
			posa_mtx.lock(); 
			posa_counter=posapool[0].size(); 
			cot(posapool[0]);
			lop(i,0,sz){ ;  
				float newangle= posapool[0][i];  				
					ve[i]->rotatetoposition(newangle);  
			}		
			while(posa_counter!=0){
				// cot(posa_counter);
				sleepms(100);
			} 
			posapool.erase(posapool.begin());
			posa_mtx.unlock();
			// cot("unlock"); 
		}
	},sz);
	thm.detach( );
	return 1;
}
int view(lua_State* L){
	cot(lua_gettop(L));
	int sz=lua_gettop(L);
	if(sz<9)return 1;
	
	vfloat vf(sz);
	lop(i,0,sz){ ;
		vf[i]= lua_tonumber(L,i+1);   
	}
	Vec3d eye( vf[0] , vf[1] , vf[2]  );
	Vec3d center( vf[3] , vf[4] , vf[5]  );
	Vec3d up( vf[6] , vf[7] , vf[8]  ); 
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
	
	lua_pushcfunction(L,  posa );
	lua_setglobal(L,"posa");
	
	lua_pushcfunction(L,  view );
	lua_setglobal(L,"view");
	
	lua_pushcfunction(L,  posadebug );
	lua_setglobal(L,"posadebug");
	
	return L;
}
void lua_str(string str){	
	lua_State* L=lua_init();
	luaL_loadstring(L, str.c_str());
	lua_pcall(L, 0, 0, 0);
	
	
	lua_close(L);
}
void lua(){
	
	lua_State* L=lua_init();
	
	luaL_dofile(L, "test.lua");
	
	lua_getglobal(L,"x");
	cout<<lua_tonumber(L,-1)<<endl;
	lua_getglobal(L,"xx");
	cout<<lua_tonumber(L,-1)<<endl;
	
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
	string fname="test.lua";
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
		load(); 
	}
	int handle(int e){ 
		int ret=Fl_Text_Editor::handle(e);
		// if(e==FL_KEYDOWN &&  Fl::event_state() ==FL_CTRL && Fl::event_key()==102) find_cb();
		if(e==FL_KEYDOWN){
			save();
			if(Fl::event_key()==65027 || Fl::event_key()==65514){//altgr
				int new_pos = insert_position();
				int line = texto->count_lines(0, new_pos);
				// int ls = texto->line_start(new_pos);
				// cot(line);
				// cot(ls);
				fparse_str(texto->text(),line);
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
int main(){   
// https://www.fltk.org/doc-1.3/opengl.html#opengl_drawing
// https://osg-users.openscenegraph.narkive.com/HiVHDrXM/change-camera-position
// https://dis.dankook.ac.kr/lectures/med08/wp-content/uploads/sites/35/1/1321343577.pdf 
    

	int w=800+300;
	int h=480;
	Fl_Double_Window* win=new Fl_Double_Window(0,0,w,h,"frobot");   
	
	flt=new Fl_Double_Window(0,  20, 150, 400);
	Fl_Button* bt1=new Fl_Button(0,  0, 30, 30,"Alpha");
	Fl_Button* bt2=new Fl_Button(30,  0, 30, 30,"rota");
	Fl_Button* bt3=new Fl_Button(60,  0, 30, 30,"gview");
	Fl_Button* bt4=new Fl_Button(90,  0, 30, 30,"MV");
	Fl_Button* bt5=new Fl_Button(120,  0, 30, 30,"fit");
	Fl_Button* bt6=new Fl_Button(0,  30, 30, 30,"ik");
	Fl_Button* bt7=new Fl_Button(30,  30, 30, 30,"lua");
	Fl_Button* bt8=new Fl_Button(60,  30, 30, 30,"Lrun");
	Fl_Button* bt9=new Fl_Button(90,  30, 30, 30,"rvis");
   
	fle=new FlEditor(0,60,150,400-60);
	flt->resizable(fle);   
	flt->end();
	flcv=new Fl_ViewerCV(800,0,300,300);
	osggl=new ViewerFLTK(160,  0, 800-160, 480-10); 
    osggl->addEventHandler(new osgViewer::StatsHandler);
   
// Fl::run();return 1;
   
	tmr=new osgGA::TrackballManipulator;
	
	// fparse(fle->texto->text(),9);
 
	
	// flocv=new flocvs;
	
	// displaycam();
	// setview();
 
	loadstl(group);

	geraeixos(group);
	osggl->setSceneData(group);
	
	
	
	//botoes dos angulos
	Fl_Double_Window* flpos=new Fl_Double_Window(800, 310, 300, 480-300);   
	Fl_Box* flposbox=new Fl_Box(0, 0, 300, 480-300);   
	cot(flpos->w());
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
		cot(btnsz);
		cot(btn);
		cot(ve[vei]->anglemax/10.0*10);
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
				cot(vv->index);
				cot(vv->angle);
				ve[vv->index]->rotatetoposition(vv->angle);
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
			tmr->setHomePosition( center-lookDir*_distance, center, up );
			osggl->setCameraManipulator(tmr);
			getview();
		}	
	});
	
	bt6->callback([](Fl_Widget *, void* v){ 	
		ve[ve.size()-1]->rotate_posk(10);	
	});
	
	bt7->callback([](Fl_Widget *, void* v){ 	
		fle->fname="test.lua";
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
	
	
	bt5->do_callback();
	
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
	
	// win->position(Fl::w()/2-win->w()/2,0);
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