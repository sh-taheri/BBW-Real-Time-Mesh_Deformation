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

#include <OGF/image/types/image_library.h>

 
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
    P.BBW_flag = false;
       
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
   
   void SurfaceQuickStartCommands::Triangulation(float area){
   
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
        P.create_triangle(area,point_list,out);
        P.save_out(out,"triangulated_mesh.obj");
        
        ///////// Creating a mesh ////////
        
        int npoints = out.numberofpoints;
        double* points = out.pointlist;
        int ntriangles = out.numberoftriangles;
        int *triangles = out.trianglelist;
        
        ////// Updating the surface //////		
        surface()->clear();        
        bool ret = P. create_mesh(npoints, points,ntriangles,triangles,surface());
        if(!ret)
	cout<<"Something is wrong in triangle code!"<<endl;
	
        surface()->compute_normals();
        surface()->update();

	

   }
   
   
   void SurfaceQuickStartCommands::BoundedBiharmonic(){
   
   
        /// Test loading image ////
        //Image* image ;// load_image("OGF/wooy.png") ;
        //image = load_image("OGF/wooy.png");
        
        P.BBW_flag = true;
   
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
        
        
        std::cout<<"Number of handles : "<<num_handles<<endl;
        cout<<endl;
        
        int row = num_ver;
        int col = num_ver;  
        
        /// Calculate Laplacian Matrix//////////////////////
        
        double **laplacian_matrix;
        laplacian_matrix = new double *[row]; 
        for (int count = 0 ; count < row ; count++)
             laplacian_matrix[count] = new double[col];
             
        for (int i=0; i<num_ver ; i++)
              for (int j= 0; j<num_ver ;j++)
              laplacian_matrix[i][j] = 0;    
             
         
            
        P.Laplacian(surface(),laplacian_matrix); // I used uniform weights for now.
        
        ///// Print Laplacian Matrix 
        /*
        for (int i=0; i<num_ver ; i++)
        {
            for (int j = 0 ; j<num_ver ; j++)
            std::cout<<"    "<<laplacian_matrix[i][j];
            
            std::cout<<endl;}
            
          */  

        
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
        //for (int i=0; i<num_ver ; i++)
        //{
            //for (int j = 0 ; j<num_ver ; j++)
            //std::cout<<"    "<<Qi[i][j];
            //std::cout<<endl;}
            //std::cout<<endl;
            

        //////////////////////////////////////////////////
        P.n = num_ver;      // number of vertices
        P.m = num_handles;  // number of handles  
         


        P.Find_Weights(Qi,surface());
        

       ///// Printing the weights for each vertex 
       
       for (int i=0 ; i<P.n ; i++){     
           std::cout<<"Vertex ("<<i<<") : ";
           for (int j=0; j<P.m;j++)
                std::cout<<"W"<<j+1<<" : "<<P.BBWeights[i][j]<<" , ";   
           std::cout<<endl;}  
           
                               
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
     
     
     
     extern bool deformation_flag;

    


