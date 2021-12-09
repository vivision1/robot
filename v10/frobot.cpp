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
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp> 
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


#include "filesystem.hpp"
#include "regular.hpp"
#include "threads.hpp"
#include "timef.hpp"
#include "arrayf.hpp"
#include "stringf.hpp" 

struct flocvs;
flocvs* flocv;

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

typedef osg::Vec3f vec3f; 
typedef osg::Vec3f vec3; 
#define Pi 3.141592653589793238462643383279502884L
#define Pi180 0.01745329251
struct osgdr{
	int index=0;
	vec3* offset=0;
	float angle=0;
	float anglestart=0;
	float anglemax=0;
	float anglemin=0;
	int dir=1;
	bool moving=0;
	vec3 *axisbegin;
	vec3 *axisend;
	vec3 axis;
	vector<osg::ref_ptr<osg::Node>> nodes;
	vstring nodesstr;
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	osg::ref_ptr<osg::Geometry> geometry= new osg::Geometry; 
    MatrixTransform* transform = new osg::MatrixTransform;
	DrawArrays* drw=new osg::DrawArrays;
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
		geometry ->setVertexArray(points.get());
		geometry ->setColorArray(color.get());
		geometry ->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		geometry ->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0, points->size()));
		osg::LineWidth* linew = new osg::LineWidth(5);
		geometry->getOrCreateStateSet()->setAttributeAndModes(linew);
		// transform->addChild(geometry);
		
		lop(i,0,nodesstr.size()){
			nodes.push_back(osgDB::readRefNodeFile(nodesstr[i]));
			settranparency(nodes[i].get(),0);
			transform->addChild(nodes[i].get());
		}
	}
	
	void gooffset(){
		if(offset==0)offset=new vec3(0,0,0);
		
	}
	//ve[index]-> == this
	//posa
	void rotatetoposition(float newangle){	
		boost::thread th([&](float newangle ){
			if(moving==1){
				moving=0;
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
		},newangle);
		th.detach( ); 
		
	
	} 
	void rotate( float _angle ){
		if(_angle==0)return;
		mtxlock(0);
		cot(index);
		cot(angle);
		// cot(axisb.length());
		// if(axisb.length()==0)
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
			lop(j,0,points[0].size())points[0][j] = points[0][j]   * T * Ra * Tr   ; 
			transform->setMatrix(transform->getMatrix()   * T * Ra * Tr ); 
		}
					 
		osg::Matrix R; 
		R.makeRotate( Pi180*_angle, axis );
		lop(j,0,points[0].size())points[0][j] = points[0][j] * T * R * Tr  ; 
		transform->setMatrix(transform->getMatrix()   * T * R * Tr   );
		
		lop(i,0,index){ 
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
				
			lop(jj,0,index){		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*-ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * R * Tr   );
			lop(j,0,ve[i]->points[0].size()){ 
				ve[i]->points[0][j] = ve[i]->points[0][j]   * T * R * Tr   ;  
			}
			
			lop(jj,0,index){		
				osg::Matrix Ra; 
				Ra.makeRotate( Pi180*ve[jj]->angle, ve[jj]->axis ); 
				lop(j,0,ve[i]->points[0].size())ve[i]->points[0][j] = ve[i]->points[0][j]   * T * Ra * Tr   ; 
				ve[i]->transform->setMatrix(ve[i]->transform->getMatrix()   * T * Ra * Tr ); 
			}
			
			ve[i]->drw->dirty();
			ve[i]->drw->set(GL_LINES,0, ve[i]->points->size());
			ve[i]->geometry ->setPrimitiveSet(0,ve[i]->drw); 
		}

		angle+=_angle;
		
		//modulo float
		angle = angle - int( angle/360.0 )*360.0 ;
		mtxunlock(0);
	};

};
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
	boost::thread th([&](float z){
		float currz=(float)((*ve[0]->axisbegin).z());
		float rz=z-currz;
		float speed=2;
		cot (currz);
		cot(rz);
		float newz=rz;
		int dir=1;
		if(z<=currz){dir=-1;   }
		for(;;){
			cot(dir);
			cot(newz);
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
	// ve[idx]->nodesstr.push_back("stl/robot_balde.stl");
	// ve[idx]->nodesstr.push_back("stl/robot_servospt70.stl"); 
	ve[idx]->axis=vec3(0,1,0);
	ve[idx]->anglemax=190;
	ve[idx]->anglemin=-130;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(110,250,-20),vec3(110,250,0));
		
	idx=1;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	//////// ve[idx]->nodesstr.push_back("stl/robot_servospt70_1.stl");
	ve[idx]->nodesstr.push_back("stl/robot_armj1.stl");
	ve[idx]->axis=vec3(1,0,0);
	ve[idx]->anglemax=160;
	ve[idx]->anglemin=-160;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(0,250-20,-42.0),vec3(0,250+5,-42.0));
		
	idx=2;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_armj1_1.stl"); 
	ve[idx]->axis=vec3(0,0,1);
	ve[idx]->anglemax=160;
	ve[idx]->anglemin=-160;
	ve[idx]->offset=offset;
	ve[idx]->newdr(vec3(-55,228,0),vec3(-50,228,0));
		
	// idx=3;
	// ve.resize(idx+1);
	// ve[idx]=new osgdr(group); 
	// ve[idx]->nodesstr.push_back("stl/robot_armj1_2.stl"); 
	// ve[idx]->axis=vec3(0,0,1);
	// ve[idx]->anglemax=160;
	// ve[idx]->anglemin=-160;
	// ve[idx]->offset=offset;
	// ve[idx]->newdr(vec3(0,250-20,-42.0),vec3(0,250+5,-42.0));
 
	goffset(offset);
	
 
}

bool toggletranspbool=0;
void toggletransp(){
	toggletranspbool=!toggletranspbool;
	lop(i,0,ve.size()){
		lop(j,0,ve[i]->nodes.size()){
			settranparency(ve[i]->nodes[j],toggletranspbool);
		}
	}

} 
void loadstl(Group* group){
	
    osg::ref_ptr<osg::Node> maquete = osgDB::readRefNodeFile("stl/maquete.stl");
	settranparency(maquete.get(),1);
	group->addChild(maquete.get());
	
    // osg::ref_ptr<osg::Node> estrutura = osgDB::readRefNodeFile("Assembly1.stl");
	// settranparency(estrutura.get(),1);
	// group->addChild(estrutura.get());
	
};
// _node->setNodeMask(visible ? 0xffffffff : 0x0);

//VIEW
#if 1
void getview(){
	osg::Vec3 eye, center, up;
	osggl->getCamera()->getViewMatrixAsLookAt( eye, center, up );
	cot(eye);
	cot(center);
	cot(up);
}

#endif

osgGA::TrackballManipulator* tmr;
vvstring fparsenew; 
vvstring fparseold;
void fparse(string text, int linepos){ 
#define ifc(view) if(((std::find(runidx.begin(), runidx.end(), idx) != runidx.end()) && fparsenew[idx][0]==view) || (idx==currline && fparsenew[idx][0]==view)) 
// cot(text); 
	int currline=-1;
	vint runidx;
	vector<string> strs=split(text,"\n");
	// cot(strs);
	fparsenew=vvstring(0);
	lop(i,0,strs.size()){
		vstring stspl=split(strs[i],": ");
		if(stspl.size()>=2){ 
			fparsenew.push_back(vstring{stspl[0],stspl[1]});
			runidx.push_back(fparsenew.size()-1);
			if(linepos==i)currline=(fparsenew.size()-1); 
		}
	}  
	if(fparseold.size()==0)fparseold=fparsenew;
	
	lop(i,0,fparsenew.size()){
		lop(j,0,fparseold.size())
			if(fparseold[j][0]==fparsenew[i][0] && fparseold[j][1]==fparsenew[i][1]){
				lop(ri,0,runidx.size())if(runidx[ri]==i)runidx.erase(runidx.begin() + ri);
			}
	}
	
	for ( int idx=0;idx<fparsenew.size();idx++ ){  
		ifc("view1"){ 
			vstring s=split(fparsenew[idx][1]," ",0); 
			// cot(s);
			// osgGA::TrackballManipulator* tm=new osgGA::TrackballManipulator;
			Vec3d eye( atof(s[0].c_str()) ,atof(s[1].c_str()) ,atof(s[2].c_str())  );
			Vec3d center( atof(s[3].c_str()) ,atof(s[4].c_str()),atof(s[5].c_str()) );
			Vec3d up( atof(s[6].c_str()) ,atof(s[7].c_str()),atof(s[8].c_str()) );
			tmr->setAutoComputeHomePosition(true);
			tmr->setHomePosition( eye, center, up );
			// tm->home(0.0);
			// tm->setPivot(Vec3f(0,0,0));
			osggl->setCameraManipulator(tmr);
		} 
		ifc("posa"){
			string fp=fparsenew[idx][1];
			boost::replace_all(fp,",  ",",");
			boost::replace_all(fp,", ",",");
			boost::replace_all(fp,"  ,",",");
			boost::replace_all(fp," ,",","); 
			vstring s=split(fp," ",0);
			vint vindex;
			vfloat vrangle;
			lop(i,0,s.size()){
				vstring vv=split(s[i],",",0);
				vindex.push_back( atoi(vv[0].c_str()) );
				vrangle.push_back( atoi(vv[1].c_str()) );
			}
			lop(i,0,s.size()){
				int vi=vindex[i];
				float rangle= (vrangle[i]);  
				ve[vi]->rotatetoposition(rangle); 
			}
			
		}
		ifc("posad"){
			string fp=fparsenew[idx][1];
			boost::replace_all(fp,",  ",",");
			boost::replace_all(fp,", ",",");
			boost::replace_all(fp,"  ,",",");
			boost::replace_all(fp," ,",","); 
			vstring s=split(fp," ",0);
			vint vindex;
			vfloat vrangle;
			lop(i,0,s.size()){
				vstring vv=split(s[i],",",0);
				vindex.push_back( atoi(vv[0].c_str()) );
				vrangle.push_back( atoi(vv[1].c_str()) );
			}
			lop(i,0,s.size()){
				int vi=vindex[i];
				float rangle= (vrangle[i]);  
				ve[vi]->rotate(rangle); 
			}
			
		}
		ifc("movz"){
			
			vstring s=split(fparsenew[idx][1]," ",0);  
			movetoposz( atof(s[0].c_str()) );
			
		}
		
	}
	fparseold=fparsenew;
	
	
}

//FlEditor
#if 1
struct FlEditor:Fl_Text_Editor{
	Fl_Text_Buffer  *texto = 0;
	string fname="frobot.txt";
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
		fparse(texto->text(),-1);
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
				fparse(texto->text(),line);
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
#endif

int Break=0;

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
	Fl_Button* bt3=new Fl_Button(60,  0, 30, 30);
	FlEditor* fle=new FlEditor(0,60,150,400-60);
	flt->resizable(fle);   
	flt->end();
	flcv=new Fl_ViewerCV(800,0,300,300);
	osggl=new ViewerFLTK(160,  0, 800-160, 480-10); 
    osggl->addEventHandler(new osgViewer::StatsHandler);
   
	tmr=new osgGA::TrackballManipulator;
	
	fparse(fle->texto->text(),9);
 
	
	// flocv=new flocvs;
	
	// displaycam();
	// setview();
 
	loadstl(group);

	geraeixos(group);
	osggl->setSceneData(group);

	// osg::Matrix mat = osg::computeWorldToLocal(loadedModel.get());        
// std::cout << "X: " << mat.getTrans().x() << std::endl;      
// std::cout << "X: " << mat.getTrans().y() << std::endl;      
// std::cout << "X: " << mat.getTrans().z() << std::endl;
// std::cout << "Rot X: " << mat.getRotate().x() << std::endl;
// std::cout << "Scale X: " << mat.getScale().x() << std::endl;
	
	bt3->callback([](Fl_Widget *, void* v){  
		// ve[0]->rotate(Vec3f(0,0,1),45); 
		getview();
		Break=1;
		
			ve[0]->rotate( 10); 
			cot(*ve[1]->axisbegin ); 
			cot(*ve[1]->axisend );  
	});	
	
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