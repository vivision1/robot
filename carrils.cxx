#include <vector>

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

#include "regular.hpp"
using namespace std;
using namespace osg;

typedef osg::Vec3f vec3f; 
typedef osg::Vec3f vec3; 
#define Pi 3.141592653589793238462643383279502884L
#define Pi180 0.01745329251

struct posv{ vfloat p; vec3 topoint; float x; float y; float z; int posa_counter=0;  };

struct carrils{
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
	carrils(Group* group){	
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
			for(int vi=sz-2;vi>=0;vi--){ //-2 é o z
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
			if(wi%500==0){
				lop(i,0,sz)cdir[i]=precision;
			}
		}
		// cot(ld);
		// cot(angles);
		// cot(*axisendik)
		// cot(*axisend)
		posv res;
		res.p=angles;
		res.z=ve[0]->axisbeginik->z();
		// posv r(angles,{},0,0,0);
		return res;
	}
};
