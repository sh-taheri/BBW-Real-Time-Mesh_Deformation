
#include <OGF/quick_start/ARAP/BBW.h>
#include <OGF/cells/map/map_builder.h>
#include <mosek.h>



//# define NUMCON      //number of constraints			
//# define NUMVAR      //number of variables			
//# define NUMANZ      //number of non-zero in A			
//# define NUMQNZ      //number of non-zero in Q			


using namespace std;
using namespace OGF;

BBW P;
 
    
        void BBW::create_triangle(vector<double> point_list, triangulateio& out){
         
        triangulateio in;  
        in.numberofpoints = (point_list.size())/2;
        in.pointlist = & point_list.front();
        
        //for (int i=0 ; i<point_list.size(); i++)
        //std::cout<<in.pointlist[i]<<"***"<<endl;
	
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofpointattributes=0;
	in.trianglelist = NULL;
	in.triangleattributelist = NULL;
	in.trianglearealist = NULL;
	in.numberoftriangles = 0;
	in.numberofcorners = 0;
	in.numberoftriangleattributes = 0;

	in.numberofsegments =  (point_list.size())/2;
	std::vector<int> segmentlist;
	segmentlist.resize(2*in.numberofsegments);
	for(int i=0;i<in.numberofsegments;++i)
	    for(int j=0;j<2;++j)
		segmentlist[2*i+j] = (i+j)%in.numberofsegments;
;
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
	triangulate("pqVza0.05", &in, &out, NULL);
         }
         
         
         
        bool BBW::create_mesh(int npoints, double* points, int ntriangles,
                         int* triangles, 
		         Map* ret){
		          
		        MapBuilder mb(ret);
			mb.begin_surface();
			
			for(int i=0;i<npoints;++i){

				Point3d P(points[2*i], points[2*i+1], 0);
				Point2d p(P[0], P[1]);

				mb.add_vertex(P);
				mb.add_tex_vertex(p);
			}

			for(int i=0;i<ntriangles;++i){
				mb.begin_facet();
				for(int j=0;j<3;++j){
					mb.add_vertex_to_facet(triangles[3*i+j]);
					mb.set_corner_tex_vertex(triangles[3*i+j]);
				}
				
				mb.end_facet();      
			}

			mb.end_surface();
			ogf_assert(ret->is_valid());
			return true;

		         }
		         
		         
		         
    
    
    	bool BBW::save_out(const triangulateio& out,char* name){
		ofstream f(name);
		if(!f){
			cout<<"Unable to open "<<name<<endl;
			return false;
		}

		for(int i=0;i<out.numberofpoints;++i){
			Point3d P(out.pointlist[2*i], out.pointlist[2*i+1], 0);
			f<<"v "<<P[0]<<" "<<P[1]<<" "<<P[2]<<endl;
		}
		for(int i=0;i<out.numberofpoints;++i){
			Point3d P(out.pointlist[2*i], out.pointlist[2*i+1], 0);
			f<<"vt "<<P[0]<<" "<<P[1]<<" "<<P[2]<<endl;
		}
		for(int i=0;i<out.numberoftriangles;++i){
			int index[3] = {
				out.trianglelist[3*i]+1, 
				out.trianglelist[3*i+1]+1,
				out.trianglelist[3*i+2]+1
			};

			f<<"f "<<index[0]<<"/"<<index[0]<<" "<<index[1]<<"/"<<index[1]<<" "<<index[2]<<"/"<<index[2]<<endl;
		}

		return true;
	}
	
	
	
	
	
	
	
	
	
        void BBW::Laplacian(OGF::Map *map,double** laplacian_matrix){
        
        
         MapVertexAttribute<int> attrib_name(map, "INDEX");
         FOR_EACH_VERTEX(Map, map,it){ 
         
         Point3d VPoint = it -> point();
         int D = it->degree();
         double W[D];
         
         double gammaji;
         double gammaij;
         double beta;
         double alpha;
         
         Map::Halfedge* halfedge1 = it->halfedge();
         Map::Halfedge* halfedge0=halfedge1;
                
         double SumWij = 0;
         for(int i=0;i<D;i++)
         {
   	   Map::Halfedge* halfedge2 = halfedge0 ->opposite();
   	   Map::Vertex* vertex2 = halfedge2->vertex();
   	   Point3d VPoint2 = vertex2 -> point();           
          
          Map::Vertex* A;
          Map::Vertex* B;
          /////////////////////////////////////////////////////
          Map::Halfedge* halfedge3 = halfedge0 -> next_around_vertex();
          halfedge3 = halfedge3 -> opposite();
          A = halfedge3 -> vertex();
          Point3d VPointA = A -> point();
          gammaji = Geom::angle(VPoint - VPointA , VPoint2 - VPointA);
          beta = Geom::angle(VPointA - VPoint , VPoint2 - VPoint);
          /////////////////////////////////////////////////////
          Map::Halfedge* halfedge4 = halfedge0 -> prev_around_vertex();
          halfedge4 = halfedge4 -> opposite();
          B = halfedge4 -> vertex();
          Point3d VPointB = B -> point();
          alpha = Geom::angle(VPointB - VPoint , VPoint2 - VPoint);
          gammaij = Geom::angle(VPoint - VPointB , VPoint2 - VPointB);
          //// Cotan //////////////////////////////////////////
           W[i] = (cos(gammaij))/(sin(gammaij)) + (cos(gammaji))/(sin(gammaji));  
           SumWij = SumWij + W[i];  
           halfedge0 = halfedge0 ->next_around_vertex();
   	 } 
   	 
   	 int index = attrib_name[it]; 
         laplacian_matrix[index][index]=1; 
         
   	 
   	  for(int i=0;i<D;i++)
         {
   	   Map::Halfedge* halfedge2 = halfedge1 ->opposite();
   	   Map::Vertex* vertex2 = halfedge2->vertex();
   	   
   	   laplacian_matrix[index][attrib_name[vertex2]]= -(W[i] /SumWij); 
   	   
           halfedge1 = halfedge1 ->next_around_vertex();}

         }

        }
        
        
        
        void BBW::Find_Weights(int n,int m,double* weights,double** Q0,OGF::Map* map){
        
       
        /// Using MOSEK library for finding the weights
        
        int NUMVAR = n*m; // number of variables.
        int NUMCON = n; // number of constraints.
        
        
        /// In this problem c is set to zero.
        double c[m*n];        
        for (int i=0;i<m*n;i++)
        c[i] = 0;
        
        
        // Constraints lower and upper bounds
        MSKboundkeye bkc[n];
        double blc[n];
        double buc[n];
        
        for (int i=0;i<n;i++){
        bkc[i] = MSK_BK_FX;
        blc[i] = 1;
        buc[i] = 1;}
        
        
        
        /// Variables lower and upper bounds
        MSKboundkeye bkx[n*m];
        double blx[n*m];
        double bux[n*m];
        
                
        MapVertexAttribute<int> attrib_name(map,"INDEX"); 
        MapVertexAttribute<int> is_handle(map,"HANDLE");  
        
             for (int i=0; i<m*n ; i++){
             bkx[i] = MSK_BK_RA;
             blx[i] = 0.0 ;
             bux[i] = 1.0 ;}
        
        
        ///  Applying constraint (3)
        FOR_EACH_VERTEX(Map, map, it){ 
        int index = attrib_name[it];
        
        if ( ! is_handle[it] ){
            
             bkx[(is_handle[it]-1)*n + index] = MSK_BK_FX;
             blx[(is_handle[it]-1)*n + index] = 1.0 ;
             bux[(is_handle[it]-1)*n + index] = 1.0 ;}
             
             
             
       for (int i=1; i <=m ;i++)
             if ( i != is_handle[it]){
           
             bkx[(i-1)*n + index] = MSK_BK_FX;
             blx[(i-1)*n + index] = 0.0 ;
             bux[(i-1)*n + index] = 0.0 ;}
                        

             }     
        ////////////////////////////    
             

             
             
                 
             
       /// Constraints matrix sparse representation
       
       // considering the partition of unity
       
        MSKlidxt  ptrb[m*n]; 
        MSKlidxt  ptre[m*n];
        MSKidxt   asub[m*n];
        double    aval[m*n];
        
        for (int i=0; i< m*n ;i++){
        
        ptrb[i] = i;
        ptre[i] = i+1;
        asub[i] = i%n;
        aval[i] = 1;
        }
       
       
       /// Q matrix sparse representation ///////////////////////////////////////
       
        vector<int>    q0subi;
        vector<int>    q0subj;
        vector<double> q0val;
       

       /// Calculating the sparse representation of Qi (using the lower triangle)

       for (int i=0;i<n;i++)
          for (int j=0; j<i ;j++)
          
          if ( Q0[i][j] !=0)    
             {
             
             q0subi.push_back(i);
             q0subj.push_back(j);
             q0val.push_back(Q0[i][j]);             
             }
          
          int  q0_size = q0val.size(); // q0_size = # non-zero elements in Q0
          int NUMQNZ = m*q0_size;      // NUMQNZ = # non-zero elements in Q
          
         
       /// Calculating the sparse representaion of Q
       
        MSKlidxt  qsubi[NUMQNZ];
        MSKlidxt  qsubj[NUMQNZ];
        double    qval[NUMQNZ];
        
        for (int i=0;i< NUMQNZ ; i++){
           
           int index1 = i%q0_size;
           int index2 = i/q0_size;

           qsubi[i] = q0subi[index1] + n*index2;
           qsubj[i] = q0subj[index1] + n*index2;
           qval [i] = q0val[index1];
            
           }
           
       /////////////////////////////////////////////////////////////////////////////
        MSKidxt j;
        double xx[NUMVAR];
        

        MSKenv_t env;
        MSKtask_t task;
        MSKrescodee r;
        
        
        
        // Create the mosek environment . 
        r = MSK_makeenv(&env , NULL);
        
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 1"<<endl; 


        /// Initialize the environment . 
        r = MSK_initenv ( env );
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 2"<<endl;
        
        /// Create the optimization task . 
        r = MSK_maketask(env , NUMCON , NUMVAR ,& task); 
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 3"<<endl;
        
        
        r = MSK_inputdata ( task ,
        NUMCON , NUMVAR ,
        NUMCON , NUMVAR ,
        c ,0.0 ,
        ptrb ,
        ptre ,
        asub ,
        aval ,
        bkc ,
        blc ,
        buc ,
        bkx ,
        blx ,
        bux );
        
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 4"<<endl;
 
        /// Input the Q for the objective . 
        r = MSK_putqobj ( task , NUMQNZ , qsubi , qsubj , qval ); 
        
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 5"<<endl;
       
        if ( r == MSK_RES_OK )
        {
        MSK_getsolutionslice(task ,
        MSK_SOL_ITR ,
        MSK_SOL_ITEM_XX ,
        0,
        NUMVAR ,
        xx );             
        }
        
        if(r == MSK_RES_OK)
        std::cout<<"****** I'm here 6"<<endl;
        
        
        if ( r == MSK_RES_OK ){
        std::cout<<"****** I'm here 7"<<endl;
        printf(" Primal solution \ n ");
        for( j =0; j < NUMVAR ; ++ j )
        printf(" x [% d ]: % e \ n " ,j , xx [ j ]);}
        
       
        
        
        
        

        }
        
        
        
        
        void BBW::Multiply_matrices(int size,double** matrix1 , double** matrix2, double** out){
        
        
        for (int i=0;i<size ; i++)
            for (int j=0; j<size ; j++)
            
            {
                double sum = 0;
                for (int l=0 ; l<size ; l++)
                sum = sum + matrix1[i][l] * matrix2[l][j];
                
                out[i][j] = sum;
            
            }
        
        
        
        }
        
        
        
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

