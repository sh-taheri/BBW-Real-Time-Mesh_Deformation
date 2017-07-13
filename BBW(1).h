#ifndef BBW_H
#define BBW_H

#define REAL double
#define VOID void
#define ANSI_DECLARATORS



extern "C" {
#include "OGF/cells/third_party/triangle.h"
}

#include <OGF/cells/map/map.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/cells/graph/graph_attributes.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/graph/graph_editor.h>
#include <OGF/cells/map/geometry.h>






#include <vector>
//#include <OGF/cells/map/map_attributes.h>
//#include <OGF/cells/map/geometry.h>
//#include <OGF/cells/third_party/triangle.h>

using namespace std;
using namespace OGF;



        class BBW{

        public:
        
        
        void create_triangle(vector<double> point_list, triangulateio& out);
        bool create_mesh(int npoints, double* points, int ntriangles,
                         int* triangles, 
		         Map* ret);
		       
	bool save_out(const triangulateio& out,char* name); 
	void Laplacian(OGF::Map *map,double** laplacian_matirx);
	void Find_Weights(int n,int m,double* weights,double** Q0,OGF::Map* map);
	void Multiply_matrices(int size,double** matrix1 , double** matrix2, double** out);

		         
        


        };
        
extern BBW P;


#endif
