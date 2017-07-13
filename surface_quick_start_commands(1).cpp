/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */

#include <OGF/quick_start/commands/surface_quick_start_commands.h>
#include <OGF/quick_start/ARAP/ARAP.h>
#include <OGF/quick_start/ARAP/BBW.h>
#include <OGF/quick_start/ARAP/Registration.h>
#include <OGF/scene_graph/types/scene_graph.h>
#include <math.h>
#include <mosek.h>



#define NUMCON 1   /* Number of constraints.             */
#define NUMVAR 3   /* Number of variables.               */
#define NUMANZ 3   /* Number of non-zeros in A.          */
#define NUMQNZ 4   /* Number of non-zeros in Q.          */





#define PI 3.14159265359

  namespace OGF {

      int k;
   
   /*
    void SurfaceQuickStartCommands::command_example(
        double my_value,
        int my_integer,
        const Color& my_color,
        bool my_flag,
        RobotType my_robot,
        const std::string& my_string
    ) {
        Surface* s = surface() ;

        // Insert your code here

        // Redraws current object.
       s->update() ;
    }
    
    

    void SurfaceQuickStartCommands::algorithm_under_test(
        double arg1, double arg2, double arg3,
        double arg4, double arg5, double arg6
    ) {
        Surface* s = surface() ;

        // Insert your code here

        // Redraws current object.
        s->update() ;
    }
    */
    
     
    
    //int SurfaceQuickStartCommands::index_vertices(){
                                
   //}
   
   
    void SurfaceQuickStartCommands::Rotation(bool x_axis,bool y_axis,bool z_axis, float angle){
    
    
    float R[9];
    float teta = (angle*PI)/180.f;  
    
    if (x_axis){
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;
    R[3] = 0;
    R[4] = cos(teta);
    R[5] = -sin(teta);
    R[6] = 0;
    R[7] = sin(teta);
    R[8] = cos(teta);
    
    }
    
    if (y_axis){
    R[0] = cos(teta);
    R[1] = 0;
    R[2] = sin(teta);
    R[3] = 0;
    R[4] = 1;
    R[5] = 0;
    R[6] = -sin(teta);
    R[7] = 0;
    R[8] = cos(teta);}
    
    if (z_axis){
    R[0] = cos(teta);
    R[1] = -sin(teta);
    R[2] = 0;
    R[3] = sin(teta);
    R[4] = cos(teta);
    R[5] = 0;
    R[6] = 0;
    R[7] = 0;
    R[8] = 1;}
    
    FOR_EACH_VERTEX(Map, surface(), it){
    Point3d point; 
     point.x = R[0]*(it->point().x) + R[1]*(it->point().y) + R[2]*(it->point().z);   
     point.y = R[3]*(it->point().x) + R[4]*(it->point().y) + R[5]*(it->point().z);  
     point.z = R[6]*(it->point().x) + R[7]*(it->point().y) + R[8]*(it->point().z);      
     it->point() = point;   
    }
    
    surface()->update();
       
    }
    
    
    void SurfaceQuickStartCommands::ICP(int iter,bool acc,float threshold){
    
    Surface* s11 = surface();
    Surface* s22 = (Surface*)nextInScene(surface());
    
    B.Reg_ICP(s11,s22, iter, acc,threshold);
   

    s11->update();
    s22 ->update();
    }
  
    
    void SurfaceQuickStartCommands::LaplacianPrecompute(){
   
    MapVertexAttribute<int> attrib_name(surface(), "INDEX"); 
    k=0;	                                                 FOR_EACH_VERTEX(Map, surface(), it){                    
    attrib_name[it]=k++;}  
    A.Laplacian_precompute(surface());  
             
   }
   
   
   void SurfaceQuickStartCommands::Tan_Weights(){
   A.weight_type=1;
   }
   
   void SurfaceQuickStartCommands::Cot_Weights(){
   A.weight_type=0;   
   }
   
   
  void SurfaceQuickStartCommands::Uniform_Weights(){
   A.weight_type=2;
   }
   
   void SurfaceQuickStartCommands::Triangulation(){
   
        // In this command I have assumed that the input surface
        // is a border.
   
        ///// Reading the input border ///// 
        vector<double> point_list;	                                                     FOR_EACH_VERTEX(Map, surface(), it){   
         double x = it->point().x;
         double y = it->point().y;                 
          point_list.push_back(x) ;
          point_list.push_back(y);}
        ///// Triangulation //////////////
        triangulateio out;
        memset(&out, 0, sizeof(triangulateio));        
        P.create_triangle(point_list,out);
        P.save_out(out,"triangulated_mesh.obj");
        
        /////// Creating a mesh ///////
        /*
        int npoints = out.numberofpoints;
        double* points = out.pointlist;
        int ntriangles = out.numberoftriangles;
        int *triangles = out.trianglelist;
        		
        Map*  mp = 0;
        mp = new Map();        
        bool ret = P. create_mesh(npoints, points,ntriangles,triangles,mp);
        if(!ret)
	cout<<"Something is wrong in triangle code!"<<endl;
	*/

   }
   
   
   void SurfaceQuickStartCommands::BoundedBiharmonic(){
   

        /// Assigning index to all the vertices
        MapVertexAttribute<int> attrib_name(surface(),"INDEX"); 
        MapVertexAttribute<int> is_handle(surface(),"HANDLE"); 
        int num_ver=0;	
        int num_handles = 0;
                                                             FOR_EACH_VERTEX(Map, surface(), it){                    
        attrib_name[it]=num_ver++;
                 
        MapVertexLock is_anchor(surface());
        if(is_anchor[it]){
        is_handle[it] = num_handles +1;
        num_handles++;}
        
        else 
        is_handle[it] = 0;
        
        }
        
        
        std::cout<<"Handles : "<<num_handles<<endl;
        
        int row = num_ver;
        int col = num_ver;  
        
        /// Calculate Laplacian Matrix//////////////////////
        
        double **laplacian_matrix;
        laplacian_matrix = new double *[row]; 
        for (int count = 0 ; count < row ; count++)
             laplacian_matrix[count] = new double[col];
         
            
        P.Laplacian(surface(),laplacian_matrix);
        
        ///// Print Laplacian Matrix 
        for (int i=0; i<num_ver ; i++)
        {
            for (int j = 0 ; j<num_ver ; j++)
            std::cout<<"    "<<laplacian_matrix[i][j];
            
            std::cout<<endl;}

        
        /// Calculate Qi Matrix ///////////////////////////
        
        double **Qi;
        Qi = new double *[row]; 
        for (int count = 0 ; count < row ; count++)
             Qi[count] = new double[col];
         
        for (int i=0; i<num_ver ; i++)
              for (int j= 0; j<num_ver ;j++)
              Qi[i][j] = 0;     
  
             
        // I assume that for now Mass matrix M is identity. --> Qi = L*I*L = L*L
        
        P.Multiply_matrices(num_ver,laplacian_matrix ,laplacian_matrix , Qi);
 
        ///// Print Qi Matrix 
        for (int i=0; i<num_ver ; i++)
        {
            for (int j = 0 ; j<num_ver ; j++)
            std::cout<<"    "<<Qi[i][j];
            
            std::cout<<endl;}

        
        
        //////////////////////////////////////////////////
        
        int n = num_ver;      // number of vertices
        int m = num_handles;  // number of handles  
         
        double* weights = new double[n*m];       // we put the weights(Wij) in an 1  by n*m vector
        
        for (int i = 0;i<n*m ; i++)
           weights[i] = 0;
        

        //P.Find_Weights(n,m,weights,laplacian_matrix ,surface());
        
        
        
        
        
        //////////////////////////////////////////////////////// copying code //////////////////////////////////////////
        
  MSKrescodee  r;
  
  double       c[]    = {0.0,-1.0,0.0};
    
  MSKboundkeye bkc[]  = {MSK_BK_LO};
  double       blc[]  = {1.0};
  double       buc[]  = {+MSK_INFINITY}; 
    
  MSKboundkeye bkx[]  = {MSK_BK_LO,
                         MSK_BK_LO,
                         MSK_BK_LO};
  double       blx[]  = {0.0,
                         0.0,
                         0.0};
  double       bux[]  = {+MSK_INFINITY,
                         +MSK_INFINITY,
                         +MSK_INFINITY};
  
  MSKint32t    aptrb[] = {0, 1, 2 },
               aptre[] = {1, 2, 3},
               asub[] = { 0,   0,   0};
  double       aval[] = { 1.0, 1.0, 1.0};
  MSKint32t    qsubi[NUMQNZ],
               qsubj[NUMQNZ];
  double       qval[NUMQNZ];
  
  MSKint32t    j,i;
  double       xx[NUMVAR];
  MSKenv_t     env;
  MSKtask_t    task;

  /* Create the mosek environment. */
  r = MSK_makeenv(&env,NULL);

  if ( r==MSK_RES_OK )
  {   
    /* Create the optimization task. */
    r = MSK_maketask(env,NUMCON,NUMVAR,&task);

    if ( r==MSK_RES_OK )
    {
      //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

    /* Append 'NUMCON' empty constraints.
     The constraints will initially have no bounds. */
    if ( r == MSK_RES_OK )
      r = MSK_appendcons(task,NUMCON); 

    /* Append 'NUMVAR' variables.
     The variables will initially be fixed at zero (x=0). */
    if ( r == MSK_RES_OK )
      r = MSK_appendvars(task,NUMVAR);

    /* Optionally add a constant term to the objective. */
    if ( r ==MSK_RES_OK )
      r = MSK_putcfix(task,0.0);
    for(j=0; j<NUMVAR && r == MSK_RES_OK; ++j)
    {
      /* Set the linear term c_j in the objective.*/  
      if(r == MSK_RES_OK)
        r = MSK_putcj(task,j,c[j]);

      /* Set the bounds on variable j.
       blx[j] <= x_j <= bux[j] */
      if(r == MSK_RES_OK)
        r = MSK_putvarbound(task,
                            j,           /* Index of variable.*/
                            bkx[j],      /* Bound key.*/
                            blx[j],      /* Numerical value of lower bound.*/
                            bux[j]);     /* Numerical value of upper bound.*/

      /* Input column j of A */   
      if(r == MSK_RES_OK)
        r = MSK_putacol(task,
                        j,                 /* Variable (column) index.*/
                        aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
                        asub+aptrb[j],     /* Pointer to row indexes of column j.*/
                        aval+aptrb[j]);    /* Pointer to Values of column j.*/
      
    }

    /* Set the bounds on constraints.
       for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
    for(i=0; i<NUMCON && r==MSK_RES_OK; ++i)
      r = MSK_putconbound(task,
                          i,           /* Index of constraint.*/
                          bkc[i],      /* Bound key.*/
                          blc[i],      /* Numerical value of lower bound.*/
                          buc[i]);     /* Numerical value of upper bound.*/

      if ( r==MSK_RES_OK )
      {
        /*
         * The lower triangular part of the Q^o
         * matrix in the objective is specified.
         */

        qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = 2.0;
        qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = 0.2;
        qsubi[2] = 2;   qsubj[2] = 0;  qval[2] = -1.0;
        qsubi[3] = 2;   qsubj[3] = 2;  qval[3] = 2.0;

        /* Input the Q^o for the objective. */

        r = MSK_putqobj(task,NUMQNZ,qsubi,qsubj,qval);
      }

      if ( r==MSK_RES_OK )
      {
         /*
         * The lower triangular part of the Q^0
         * matrix in the first constraint is specified.
         This corresponds to adding the term
         - x_1^2 - x_2^2 - 0.1 x_3^2 + 0.2 x_1 x_3
         */
        
        qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = -2.0;
        qsubi[1] = 1;   qsubj[1] = 1;  qval[1] = -2.0;
        qsubi[2] = 2;   qsubj[2] = 2;  qval[2] = -0.2;
        qsubi[3] = 2;   qsubj[3] = 0;  qval[3] = 0.2;

        /* Put Q^0 in constraint with index 0. */

         r = MSK_putqconk(task,
                          0,
                          4,
                          qsubi, 
                          qsubj, 
                          qval); 
      }

      if ( r==MSK_RES_OK )
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);      

      if ( r==MSK_RES_OK )
      {
        MSKrescodee trmcode;
        
        /* Run optimizer */
        r = MSK_optimizetrm(task,&trmcode);

        /* Print a summary containing information
           about the solution for debugging purposes*/
        MSK_solutionsummary (task,MSK_STREAM_LOG);
        
        if ( r==MSK_RES_OK )
        {
          MSKsolstae solsta;
          int j;
          
          MSK_getsolsta (task,MSK_SOL_ITR,&solsta);
          
          switch(solsta)
          {
            case MSK_SOL_STA_OPTIMAL:   
            case MSK_SOL_STA_NEAR_OPTIMAL:
              MSK_getxx(task,
                       MSK_SOL_ITR,    /* Request the interior solution. */
                       xx);
              
              printf("Optimal primal solution\n");
              for(j=0; j<NUMVAR; ++j)
                printf("x[%d]: %e\n",j,xx[j]);
              
              break;
            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER:
            case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
            case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:  
              printf("Primal or dual infeasibility certificate found.\n");
              break;
              
            case MSK_SOL_STA_UNKNOWN:
              printf("The status of the solution could not be determined.\n");
              break;
            default:
              printf("Other solution status.");
              break;
          }
        }
        else
        {
          printf("Error while optimizing.\n");
        }
      }
    
      if (r != MSK_RES_OK)
      {
        /* In case of an error print error code and description. */      
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];
        
        printf("An error occurred while optimizing.\n");     
        MSK_getcodedesc (r,
                         symname,
                         desc);
        printf("Error %s - '%s'\n",symname,desc);
      }
    }
 
    MSK_deletetask(&task);
  }
  MSK_deleteenv(&env);

        
        //////////////////////////////////////////////////////// copying code //////////////////////////////////////////
        
        
        
  
        
        }
        

         

   
   

	Map* SurfaceQuickStartCommands::nextInScene(Surface* s){
		Map* ret = 0;
		int increment = 1;
		SceneGraph* scene_graph = s->scene_graph();
		scene_graph->disable_signals();

		for(int i=0; i<scene_graph->nb_children(); i++)
		{
			int succ = (i+increment+scene_graph->nb_children())%scene_graph->nb_children();
			scene_graph->set_current_object(scene_graph->ith_child(i)->name(),false);
			if(s==scene_graph->current()){
				scene_graph->set_current_object(scene_graph->ith_child(succ)->name(),false);
				ret	=	dynamic_cast<Map*>(scene_graph->current());
				scene_graph->set_current_object(scene_graph->ith_child(i)->name(),false);
				break;
			}
		}
		scene_graph->enable_signals();
		return	ret;
	}
   
   
   
   
   

     } 

    


