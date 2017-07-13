#include "refine_holes.h"
#include <OGF/devfab/common/defs.h>
#include <iostream>
#include <OGF/cells/graph/graph_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_attributes.h>

#include <OGF/cells/map/ext_map_editor.h>
#include <OGF/cells/map_algos/gluer.h>

#include "../meanvalue2d/meanvalue2d.h"

using namespace OGF;
using namespace std;

namespace devfab {

    RefineHolesTriangle::RefineHolesTriangle(OGF::Map* mp)
	    :m_map(mp){

	    vertexIndex.bind(m_map, INDEX);

	    if(added_facets.is_defined(m_map, "addedfacets")){
		added_facets.bind(m_map, "addedfacets");
	    } else {
		added_facets.bind(m_map, "addedfacets");
		initialize_facets_attribute();
	    }
	}

    RefineHolesTriangle::ERROR2 RefineHolesTriangle::fill_hole(OGF::Map::Vertex* v){
	// STEP one: triangulate

	if(!v)
	    return VERTEX_NULL;

	if(!v->is_on_border())
	    return VERTEX_NOT_ON_BDRY;
	
	std::vector<Map::Vertex*> border;
	
	Map::Halfedge* e = v->halfedge();
	while(!e->is_border()) {
	    e = e->next()->opposite();
	}	
     	
	triangulateio out;
	memset(&out, 0, sizeof(triangulateio));

	refine(e, border, out);
	
	// STEP two: change the mesh  

	int count_original_border = border.size();

	// how many points have been added
	for(int i = count_original_border;i<out.numberofpoints;++i)
	    border.push_back(NULL);
	
	ogf_assert(out.numberofcorners == 3);

	ERROR2 ret = refine_mesh(border, out.pointlist, out.numberoftriangles, out.trianglelist);
	

	// glue edges
	glue();

	// update the 3d positionso the vertices
	update3d(count_original_border, border);

	// have yet to cleanup out stuff that was alocated by triangle

	// check to see if the map is still valid
	ogf_assert(m_map->is_valid());

	return ret;
	
    }// end of function


    void RefineHolesTriangle::initialize_facets_attribute(){
	FOR_EACH_FACET(Map, m_map, it){
	    added_facets[it] = 1;
	}

    }

    void RefineHolesTriangle::aquire_border(Map::Halfedge* e, std::vector<Map::Vertex*>& border,std::vector<double>& border_points ){

	Map::Halfedge* tmp = e;
	do {
	    border.push_back(tmp->vertex());
	    border_points.push_back(tmp->tex_coord()[0]);
	    border_points.push_back(tmp->tex_coord()[1]);
	    tmp = tmp->next();
	    ogf_assert(tmp->is_border());
	}while(tmp!=e);	
    }

    void RefineHolesTriangle::refine(Map::Halfedge* e, std::vector<Map::Vertex*>& border, triangulateio& out){
	
	
	/// first step, aquire the entire border
	
	std::vector<double> border_points;

	aquire_border(e, border, border_points);	

	// call the triangle code
	triangulateio in;

	in.numberofpoints = border.size();
	in.pointlist = &border_points.front();

	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofpointattributes=0;

	in.trianglelist = NULL;
	in.triangleattributelist = NULL;
	in.trianglearealist = NULL;
	in.numberoftriangles = 0;
	in.numberofcorners = 0;
	in.numberoftriangleattributes = 0;

	in.numberofsegments =  border.size();
	std::vector<int> segmentlist;
	segmentlist.resize(2*in.numberofsegments);
	for(int i=0;i<in.numberofsegments;++i)
	    for(int j=0;j<2;++j)
		segmentlist[2*i+j] = (i+j)%in.numberofsegments;
	in.segmentlist = &segmentlist.front();

    
	in.segmentmarkerlist = NULL;
	in.holelist = NULL;
	in.numberofholes = 0;
	in.regionlist = NULL;
	in.numberofregions = 0;	
    

	// call triangulate
	// p - reads a polygonal file
	// q - add vertices
	// Y - does not add vertices on border
	// z - zero based indexing
	// Q - quiet mode
	// a - area constraint
	triangulate("pqYzQ", &in, &out, NULL);

    }


    RefineHolesTriangle::ERROR2 RefineHolesTriangle::refine_mesh(
	std::vector<Map::Vertex*>&border, 
	double* points, 
	int ntriangles, 
	int* triangles){

	bool skiped = true;
	std::vector<bool> processed_triangles;
	processed_triangles.resize(ntriangles);
	for(int i=0;i<ntriangles;++i)
	    processed_triangles[i] = false;

	while(skiped){
	    skiped = false;	    
	    for(int i=0;i<ntriangles;++i){
		
		//if(i==4)
		//  return OK;

		if(processed_triangles[i])
		    continue;
		

		cout<<"Iteration: "<<i<<endl;

		// the 3 vertices
		int index[3];
		
                //how many old vertices do we have....
		int oldv = 0;
		for(int j=0;j<3;++j){
		    index[j] = triangles[3*i+j];
		    if(border[index[j]]!=0)
			oldv+=1;
		}

		ogf_assert(oldv>=0 && oldv<=3);

                // case 1 -> all 3 vertices are old
		if(oldv==3){

		    cout<<"case 1"<<endl;

		    Map::Vertex* v[3];		   
		    for(int j=0;j<3;++j)
			v[j] = border[index[j]];

		    Map::Halfedge* he[3];
		    int perm[3];
		    
		    if(PrepareAddFace(v, he, perm)){		    
			AddFace(he, perm);
		    } else {
			cout<<"skiped"<<endl;
			skiped = true;
			continue;
		    }			    		    
		} else {
		    // case 2 -> 2 vertices are old
		    if(oldv==2){
			cout<<"case 2"<<endl;

			Map::Vertex* v[3];

			for(int j=0;j<3;++j)
			    v[j] = border[index[j%3]];
			
			int offset = 0;
			for(;offset<3;++offset)
			    if(v[offset]!=NULL && v[(offset+1)%3]!=NULL)
				break;
			ogf_assert(offset<3);
			
			for(int j=0;j<3;++j)
			    v[j] = border[index[(offset+j)%3]];

			if(AddVertexFace(v, points[2*index[(offset+2)%3]], points[2*index[(offset+2)%3]+1])){
			// update the border array
			border[index[(offset+2)%3]] = v[2];
			} else {
			    cout<<"skiped"<<endl;
			    skiped = true;
			    continue;
			}

			
		    } else {
			// case 3 -> 0 or 1 vertices are old in which case we pass
			cout<<"case 3"<<endl;
			skiped = true;
			continue;
		    }// else		
		}// else
		
		processed_triangles[i] = true;
	    }// for
	}// while	

	return OK;
    }// end function


    bool RefineHolesTriangle::PrepareAddFace(Map::Vertex* v[3], Map::Halfedge* he[3], int rperm[3]){

	for(int i=0;i<3;++i)
	    cout<<"Vertex "<<vertexIndex[v[i]]<<endl;

	// they should all be on the border and they should be consecutive
	ogf_assert(v[0]->is_on_border());
	ogf_assert(v[1]->is_on_border());
	ogf_assert(v[2]->is_on_border());

	for(int i=0;i<3;++i){
	    he[i] = v[i]->halfedge();
	    while(!he[i]->is_border()) {
		he[i] = he[i]->next()->opposite();		    
	    }// while
	    ogf_assert(he[i]->vertex()==v[i]);
	}// for
	    
	int perm[6][3] = {{0, 1, 2}, 
			  {0, 2, 1}, 
			  {1, 0, 2}, 
			  {1, 2, 0}, 
			  {2, 0, 1}, 
			  {2, 1, 0}};

	int pi=0;
	for(;pi<6;++pi){
	    if(he[perm[pi][0]]->next()==he[perm[pi][1]] &&
	       he[perm[pi][1]]->next()==he[perm[pi][2]])
		break;
	}
	
	if(pi<6){	     
	    for(int i=0;i<3;++i)
		rperm[i] = perm[pi][i];
	    return true;
	} 

	return false;
    }

    void RefineHolesTriangle::AddFace(Map::Halfedge* he[3], int perm[3]){		
	Map::Halfedge* nh = nil ;
	ExtMapEditor editor(m_map) ;

	nh = editor.add_facet_to_border(he[perm[0]], he[perm[2]]);
	if(nh != nil) {
	    editor.compute_normals_around_facet(nh->facet()) ;

	    // I add this facet as "added"
	    added_facets[nh->facet()] = 0;
	}
    }// add face
	
    
    // preconditions: v[2] - is the unknown border vertex
    bool RefineHolesTriangle::AddVertexFace(Map::Vertex* v[3], double px, double py){

	ogf_assert(v[0]!=NULL && v[1]!=NULL && v[2]==NULL);

	for(int i=0;i<2;++i)
	    cout<<"Vertex "<<vertexIndex[v[i]]<<endl;

	ogf_assert(v[0]->is_on_border() && v[1]->is_on_border());
	

	Map::Halfedge* he[2];
	for(int i=0;i<2;++i){
	    he[i] = v[i]->halfedge();
	    while(!he[i]->is_border()) {
		he[i] = he[i]->next()->opposite();		    
	    }// while
	}// for

	int offset = 0;

	if(he[0]->next()==he[1]){
	    offset = 1;
	} else 
	    if(he[1]->next()==he[0]){
		offset = 0;
	    } else {
		return false;
	    }


	Map::Halfedge* h = he[offset]-> prev() ;
	Map::Halfedge* g = h-> next() ;
	ExtMapEditor editor(m_map) ;
	Map::Halfedge* added = editor.add_vertex_and_facet_to_border(h, g) ;
	v[2] = (h-> next()->vertex()) ;
	OGF::Point2d p(px, py);
	v[2]->halfedge()->set_tex_coord(p) ;		

	vertexIndex[v[2]] = -1;

	// I add this facet as "added"
	if(added != nil) 
	    added_facets[added->facet()] = 0;

	return true;
    }


    void RefineHolesTriangle::update3d(int count, std::vector<Map::Vertex*>&border){

	if(count==(int)border.size()){
	    cout<<"No points added!"<<endl;
	    return;
	}

	MeanValue2d interpolant(m_map);
	for(int i=0;i<count;++i)
	    interpolant.basis().push_back(border[i]);
	
	for(int i = count;i<(int)border.size();++i){
	    std::vector<double> weights;
	    bool ret = interpolant.computeWeights2d(border[i], weights);
	    ogf_assert(ret);
	    OGF::Point3d P = interpolant.blendweights3d(weights);
	    border[i]->point() = P;
	}// for
    }

    void RefineHolesTriangle::glue(){
	RecursiveGluer gluer ;
        gluer.set_tolerance(1e-2) ;
        gluer.set_map(m_map) ;
        gluer.apply() ;
    }


    bool RefineHolesTriangle::isCriticalVertex3d(OGF::Map::Vertex* v){
	int count = 0;
	Map::Halfedge* e = v->halfedge();
	Map::Halfedge* temp = e;

	do {
	    if(temp->is_border_edge())
		count+=1;
	    temp = temp->next_around_vertex();
	} while(temp!=e);
	
	ogf_assert(count!=1);
	
	if(count>2)
	    return true;

	return false;
	   
    }

    bool RefineHolesTriangle::isCriticalVertex2d(OGF::Map::Vertex* v){
	return true;
	   
    }

} // namespace
