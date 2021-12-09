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

void setalpha(Node* model,float val=0.9){
// https://groups.google.com/g/osg-users/c/APGaJ_5icx8
	osg::StateSet* state2 = model->getOrCreateStateSet(); //Creating material
	osg::ref_ptr<osg::Material> mat2 = new osg::Material;

	mat2->setAlpha(osg::Material::FRONT_AND_BACK, val); //Making alpha channel
	state2->setAttributeAndModes( mat2.get() ,
	osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	// osg::BlendFunc* bf = new                        //Blending
	// osg::BlendFunc(osg::BlendFunc::SRC_ALPHA,
	// osg::BlendFunc::ONE_MINUS_DST_COLOR );
	// state2->setAttributeAndModes(bf);  
	// model->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
	// model->getStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
}
#endif

   
 
Fl_Double_Window* flt; 
ViewerFLTK* osggl;
unordered_map<string,osg::ref_ptr<osg::Node>> stlgroup;
osg::Group* group = new osg::Group(); 
struct osgdr;
vector<osgdr*> ve;
struct osgdr{
	int index=0;
	float angle=0;
	float anglemax=0;
	float anglemin=0;
	Vec3f *axisbegin;
	Vec3f *axisend;
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
	void newdr(Vec3f _axisbegin=Vec3f(30,0,0),Vec3f _axisend=Vec3f(0,0,0) ){
		index=ve.size()-1;
		cot(index);
		points->push_back(_axisbegin);
		points->push_back(_axisend);
		axisbegin=&points[0][0];
		axisend=&points[0][1];
		color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
		geometry ->setVertexArray(points.get());
		geometry ->setColorArray(color.get());
		geometry ->setColorBinding(osg::Geometry::BIND_OVERALL);
		geometry ->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0, points->size()));
		osg::LineWidth* linew = new osg::LineWidth(15);
		geometry->getOrCreateStateSet()->setAttributeAndModes(linew);
		// transform->addChild(geometry);
		
		lop(i,0,nodesstr.size()){
			nodes.push_back(osgDB::readRefNodeFile(nodesstr[i]));
			setalpha(nodes[i].get(),0.5);
			transform->addChild(nodes[i].get());
		}
	}
	void rotate(Vec3f axis,float angle){
		osg::Matrix Tr;
		Tr.makeTranslate( axisbegin->x(),axisbegin->y(),axisbegin->z() );
		osg::Matrix T; 
		T.makeTranslate( -axisbegin->x(),-axisbegin->y(),-axisbegin->z() ); 
		osg::Matrix R; 
		R.makeRotate( PI/180*angle, axis ); 
		lop(i,0,points[0].size()){ 
			points[0][i] = points[0][i] * T * R * Tr;  
		}
		drw->dirty();
		drw->set(GL_LINES,0, points->size());
		geometry ->setPrimitiveSet(0,drw); 
		
		// mt->addChild( loadedModel.get()  );
		transform->setMatrix(transform->getMatrix() * T * R * Tr );
		
		//todas as anteriores teem que rodar tambem
	};
};
 
 
void geraeixos(Group* group){
	// e1=new osgdr(Vec3f(30,0,0),Vec3f(0,0,0));
	// group->addChild(e1->transform); 
	 
	// e1->rotate(Vec3f(0,0,1),90);
	// cot(e1->points[0][1]);
	
	// ve.push_back(new osgdr(Vec3f(30,0,0),Vec3f(0,0,0)) );
 
	int idx;
	idx=0;
	ve.resize(idx+1);
	ve[idx]=new osgdr(group); 
	ve[idx]->nodesstr.push_back("stl/robot_corpo.stl");
	ve[idx]->nodesstr.push_back("stl/robot_balde.stl");
	ve[idx]->nodesstr.push_back("stl/robot_servospt70.stl");
	ve[idx]->nodesstr.push_back("stl/robot_servospt70_1.stl");
	ve[idx]->newdr(Vec3f(110,250,-20),Vec3f(110,250,0));
		
 
	threadDetach([&]{
		lop(i,0,1900000000){
			sleepms(100); 
			ve[0]->rotate(Vec3f(0,1,0),2);  
			// cot(*ve[0]->axisbegin ); 
			cot(*ve[0]->axisend ); 
			// e1->rotate(Vec3f(0,0,1),1); 
			// cot((*e1->axisbegin).x()); 
			// cot((*e1->axisend).x()); 
		}
	});
}


osg::ref_ptr<osg::Node> loadedModel2;
void loadstl(Group* group){
#if 0
    osg::ref_ptr<osg::Node> loadedModel1 = osgDB::readRefNodeFile("cube.stl");
        // viewerWindow.setSceneData(loadedModel1.get());
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFile("cube1.stl");
	stlgroup["cube.stl"]=loadedModel;
	setalpha(loadedModel);
	


	setalpha(loadedModel1);
		 

	osg::Matrix T;
	T.makeTranslate( 0,10,0 );
	osg::Matrix R;
		  Vec3f axis(0, 0, 1);
	R.makeRotate( 0.8, axis );
	 
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->addChild( loadedModel.get()  );
	mt->setMatrix(mt->getMatrix() * T );
	
    // osg::Group* group = new osg::Group();
	group->addChild(loadedModel1.get());
	group->addChild(mt.get());
 
 
    // loadedModel2 = osgDB::readRefNodeFile("Assembly1.stl");
	// setalpha(loadedModel2);
	// group->addChild(loadedModel2.get());
    // loadedModel2 = osgDB::readRefNodeFile("Assembly4.1.stl");
	// setalpha(loadedModel2);
	// group->addChild(loadedModel2.get());


    loadedModel2 = osgDB::readRefNodeFile("stl/robot_corpo.stl");
	setalpha(loadedModel2);
	group->addChild(loadedModel2.get());
    osg::ref_ptr<osg::Node> loadedModel3 = osgDB::readRefNodeFile("stl/robot_balde.stl");
	setalpha(loadedModel3);
	group->addChild(loadedModel3.get());
#endif
};
// _node->setNodeMask(visible ? 0xffffffff : 0x0);

//SETVIEW
#if 1
void getview(){
	osg::Vec3 eye, center, up;
	osggl->getCamera()->getViewMatrixAsLookAt( eye, center, up );
	cot(eye);
	cot(center);
	cot(up);
}
void setview(){
	//http://www.sm.luth.se/csee/courses/smm/009/JavaOSG/doc/openscenegraph/osgGA/NodeTrackerManipulator.html
	osgGA::NodeTrackerManipulator * tm = new osgGA::NodeTrackerManipulator ;
	tm->setTrackerMode ( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION) ;
	tm->setRotationMode ( osgGA::NodeTrackerManipulator::ELEVATION_AZIM ) ;
	tm->setAutoComputeHomePosition(true); 
	// tm->setMinimumDistance(100); 
	// tm->setDistance(300); 
	// tm->setElevation(100);
	Vec3d eye( 0.0, 0.0, 350.0 );
	Vec3d center( 0.0, 0.0, 0.0 );
	Vec3d up( 0.0, 1.0, 0.0 );
	tm->setHomePosition( eye, center, up );
	osggl->setCameraManipulator(tm);
	return;
	// osgGA::NodeTrackerManipulator * tm = new osgGA::NodeTrackerManipulator ;
	// tm->setTrackerMode ( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION) ;
	// tm->setRotationMode ( osgGA::NodeTrackerManipulator::ELEVATION_AZIM ) ;
	osg::PositionAttitudeTransform* followerOffset= new PositionAttitudeTransform();
	followerOffset->setPosition(osg::Vec3(0.0,0.0,10.0));
	followerOffset->setAttitude(osg::Quat(PI/180*( -90.0),osg::Vec3(1,0,0)));
	group->addChild(followerOffset);
	tm->setTrackNode(followerOffset);
	// tm->setHomePosition( eye, center, up );
    osggl->setCameraManipulator(tm);
}

void setviewangle(float cameraAngleX,float cameraAngleY,float cameraPositionX, float cameraPositionY, float cameraPositionZ){
#define piover180 0.0174532925
	osgGA::NodeTrackerManipulator * tm = new osgGA::NodeTrackerManipulator ;
	tm->setTrackerMode ( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION) ;
	tm->setRotationMode ( osgGA::NodeTrackerManipulator::ELEVATION_AZIM ) ;
	tm->setAutoComputeHomePosition(true);

	float pointX, pointY, pointZ, temp;
	cot(cameraPositionZ);
	pointY = -sin(cameraAngleX*piover180)+cameraPositionY;
	temp = cos(cameraAngleX*piover180);
	pointX = sin(cameraAngleY*piover180)*temp+cameraPositionX;
	pointZ = cos(cameraAngleY*piover180)*temp+cameraPositionZ;

	Vec3d eye( cameraPositionX, cameraPositionY, cameraPositionZ );
	Vec3d center( pointX, pointY, pointZ );
	Vec3d up( 0.0, 1.0, 0.0 ); 
	tm->setHomePosition( eye, center, up );
	osggl->setCameraManipulator(tm);
}
#endif

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
			osgGA::TrackballManipulator* tm=new osgGA::TrackballManipulator;
			Vec3d eye( atof(s[0].c_str()) ,atof(s[1].c_str()) ,atof(s[2].c_str())  );
			Vec3d center( atof(s[3].c_str()) ,atof(s[4].c_str()),atof(s[5].c_str()) );
			Vec3d up( atof(s[6].c_str()) ,atof(s[7].c_str()),atof(s[8].c_str()) );
			tm->setAutoComputeHomePosition(true);
			tm->setHomePosition( eye, center, up );
			// tm->home(0.0);
			// tm->setPivot(Vec3f(0,0,0));
			osggl->setCameraManipulator(tm);
		}
		ifc("viewangle"){
			vstring s=split(fparsenew[idx][1]," ",0);
			setviewangle(atof(s[0].c_str()) ,atof(s[1].c_str()) ,atof(s[2].c_str()),atof(s[3].c_str()) ,atof(s[4].c_str()) );
		}
		ifc("viewnt"){
			vstring s=split(fparsenew[idx][1]," ",0);
			// cout<<atof(s[0].c_str())<<" "<<atof(s[1].c_str())<<" "<<atof(s[2].c_str())<<" "<<atof(s[3].c_str())<<" "<<atof(s[4].c_str())<<endl;
			osgGA::NodeTrackerManipulator * tm = new osgGA::NodeTrackerManipulator ;
			tm->setTrackerMode ( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION) ;
			tm->setRotationMode ( osgGA::NodeTrackerManipulator::ELEVATION_AZIM ) ;
			osg::PositionAttitudeTransform* followerOffset= new PositionAttitudeTransform();
			followerOffset->setPosition(osg::Vec3(atof(s[0].c_str()) ,atof(s[1].c_str()) ,atof(s[2].c_str())));
			followerOffset->setAttitude(osg::Quat(PI/180*( atof(s[3].c_str()) ),osg::Vec3(atof(s[4].c_str()),atof(s[5].c_str()),atof(s[6].c_str()) )));
			group->addChild(followerOffset);
			tm->setTrackNode(followerOffset); 
			osggl->setCameraManipulator(tm);
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
Fl_ViewerCV* flcv;

void displaycam(){
	threadDetach([&]{
	// return 0;
	cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
	int c=0;
		  cv::Mat* dst=new cv::Mat;
    for(;;){
		cot(c++);
		cot(Break);
          cv::Mat frame;
          cap >> frame;
			cv::flip(frame, *dst, 1);
          if( frame.empty() ) break; // end of video stream
          // cv::imshow("this is you, smile! :)", *dst);
			// flcv->SetImage(&dst);
			flcv->SetImage(dst);
			flcv->UpdateView();
		  // sleepms(100);
          // cv::imshow("this is you, smile! :)", frame);
          if( waitKey(10) == 27 || Break==1){ 
			cap.release();
			cv::destroyAllWindows();
            break;
        }
    }
	cap.release();
	return 0;
	});
}
struct flocvs{
	cv::VideoCapture cap;
	cv::Mat* dst;
	flocvs(){ 
		if(!cap.open(0))// open the default camera, use something different from 0 otherwise; Check VideoCapture documentation.
			return ; 
		dst=new cv::Mat;
		Fl::add_timeout(1.0/24.0, Timer_CB, (void*)this);
	}
	void draw(){ 
		  cv::Mat frame;
		  cap >> frame;
			cv::flip(frame, *dst, 1);
		  if( frame.empty() ) return;  
			flcv->SetImage(dst); 
 
	} 
	static void Timer_CB(void *userdata) { 
		flocv->draw();
        Fl::repeat_timeout(1.0/20, Timer_CB, userdata);
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
	Fl_Button* bt1=new Fl_Button(0,  0, 30, 30);
	Fl_Button* bt2=new Fl_Button(30,  0, 30, 30);
	Fl_Button* bt3=new Fl_Button(60,  0, 30, 30);
	FlEditor* fle=new FlEditor(0,60,150,400-60);
	flt->resizable(fle);   
	flt->end();
	flcv=new Fl_ViewerCV(800,0,300,300);
	osggl=new ViewerFLTK(160,  0, 800-160, 480-10); 
    osggl->addEventHandler(new osgViewer::StatsHandler);
 
    // osggl->setCameraManipulator(new osgGA::TrackballManipulator);
    // osggl->setCameraManipulator(new osgGA::OrbitManipulator);
	
    osgGA::OrbitManipulator* tm=new osgGA::OrbitManipulator;
	osg::Vec3d eye( 100.0, 100.0, 350.0 );
	osg::Vec3d center( 40.0, 40.0, 0.0 );
	osg::Vec3d up( 0.0, 1.0, 0.0 );
	tm->setHomePosition( eye, center, up );
	osggl->setCameraManipulator(tm);
	
	flocv=new flocvs;
	
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
	});	
	
	bt1->callback([](Fl_Widget *, void* v){ 		 
		osggl->setCameraManipulator(0);
		osg::Vec3d eye( 0.0, 0.0, 50.0 );
		osg::Vec3d center( 0.0, 0.0, 0.0 );
		osg::Vec3d up( 0.0, 1.0, 0.0 );
		osggl->getCamera()->setViewMatrixAsLookAt( eye, center, up );
	});
	bt2->callback([](Fl_Widget *, void* v){ 
		// flgl->resize(flgl->x(),flgl->y(),200,200);
		setview(); 		
	}); 
	
	// Fl::scheme("gtk+");
	// ((Fl_Widget*)win)->callback([](Fl_Widget *, void* v){ 
		// if ( (Fl::event()==10)){ 
			// exit(0);
		// } 
	// });
	#if 1
	((Fl_Widget*)win)->callback([](Fl_Widget *widget, void* v){ 
	    Fl_Window *window = (Fl_Window *)widget;		
		if (Fl::event()==FL_SHORTCUT && Fl::event_key()==FL_Escape) 
			return; // ignore Escape
		threadDetach([]{
			Break=1;
			sleepms(500);
			// window->hide();
		});
		flocv->close();
			exit(0);
		return;
	}); 
	#endif 
	win->clear_visible_focus(); 	 
	win->color(0x7AB0CfFF);
	win->resizable(win);  
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