#ifndef _DEVFAB_TRIANGLE_WRAP_ALG_H
#define _DEVFAB_TRIANGLE_WRAP_ALG_H

#define REAL double

extern "C" {
#include "triangle.h"
}

#include <OGF/cells/map/map.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/cells/graph/graph_attributes.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/graph/graph_editor.h>
#include <OGF/cells/map/geometry.h>

#include <vector>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/geometry.h>
#include "../common/defs.h"

namespace devfab {

    class RefineHolesTriangle {
    public:
	

	static bool isCriticalVertex3d(OGF::Map::Vertex* v);
	static bool isCriticalVertex2d(OGF::Map::Vertex* v);

	enum ERROR2 {
	    OK = 0, 
	    VERTEX_NOT_ON_BDRY = 1, 
	    VERTEX_NULL = 2
	};

	// constructor
	RefineHolesTriangle(OGF::Map* mp);

	// main function
	ERROR2 fill_hole(OGF::Map::Vertex* v);

    private:
	
	OGF::Map* m_map;
	
	OGF::MapVertexAttribute<int> vertexIndex;
       	

	// helper functions
	void refine(OGF::Map::Halfedge*, std::vector<OGF::Map::Vertex*>&, triangulateio& out);
	void aquire_border(OGF::Map::Halfedge* e, std::vector<OGF::Map::Vertex*>& border,std::vector<double>& border_points );
	
	ERROR2 refine_mesh(std::vector<OGF::Map::Vertex*>&border, double* points, int ntriangles, int* triangles);
	
	// constructing new faces
	bool PrepareAddFace(OGF::Map::Vertex* v[3], OGF::Map::Halfedge* he[3], int rperm[3]);
	void AddFace(OGF::Map::Halfedge* v[3], int perm[3]);
	
	bool AddVertexFace(OGF::Map::Vertex* v[3], double px, double py);

	void glue();

	void update3d(int count, std::vector<OGF::Map::Vertex*>&border);

	// added: to keep track of the new added faces -> 0 new added facet; 1 - otherwise
	OGF::MapFacetAttribute<int> added_facets;
	void initialize_facets_attribute();

};

}// namespace

#endif
