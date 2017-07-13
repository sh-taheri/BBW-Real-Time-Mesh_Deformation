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

#include <OGF/quick_start/tools/surface_quick_start_tool.h>
#include <OGF/surface/tools/surface_picker.h>
#include <OGF/quick_start/ARAP/ARAP.h>
#include <OGF/quick_start/ARAP/Registration.h>
#include <OGF/math/geometry/matrix_vector.h>
#include <OGF/quick_start/ARAP/BBW.h>



// see OGF/surface/tools/surface_facet_tools.cpp for examples



namespace OGF {
     
   // Map::Vertex* constraint;
   //Map::Vertex* v;
   Map::Vertex* c;
   int flag=0;
    
    SurfaceQuickStartTool::SurfaceQuickStartTool(
        ToolsManager* manager
    ) : SurfaceTool(manager) {
    }

    void SurfaceQuickStartTool::grab(const RayPick& p_ndc) {
    
        MapVertexAttribute<int> attrib_name(surface(),"INDEX");
        MapVertexAttribute<int> is_handle(surface(),"HANDLE");
        MapVertexAttribute<int> vertex_weight(surface(),"WEIGHT");
        
        //Map::Vertex* v = picker()->pick_vertex(p_ndc) ;
         c = picker()->pick_vertex(p_ndc) ;
        if(c == nil) {
           flag = 0;
	   status("did not pick any vertex") ;
        } else {
	   status("picked a vertex") ;
	   flag = 1;
	   
        }
        
        MapVertexLock is_anchor(surface());
        if(is_anchor[c] & flag==1 & P.BBW_flag==1){
        FOR_EACH_VERTEX(Map,surface(), it){
        int index = attrib_name[it];
        int index_handle = is_handle[c];
        vertex_weight[it] = P.BBWeights[index][index_handle-1];}
        
        }
        
           
        
    }
    
    void SurfaceQuickStartTool::drag(const RayPick& p_ndc) {
    
        MapVertexAttribute<int> attrib_name(surface(),"INDEX");
        MapVertexAttribute<int> is_handle(surface(),"HANDLE");
        MapVertexAttribute<int> vertex_weight(surface(),"WEIGHT");

        vec3 new_pos = picker()->drag_point3d(p_ndc);
        MapVertexLock is_anchor(surface());
        
        
        if(is_anchor[c] & flag==1)
        {
        
         
        if (P.BBW_flag == false){ 
        
        //// As Rigid As Possible Deformation
        
        c->set_point(new_pos) ;
        A.SVD(surface());
        
        FOR_EACH_VERTEX(Map,surface(), it){
        if ( attrib_name[c] != attrib_name[it] ){
        
        /// For 2D 
        it->point().x = A.pxnew[attrib_name[it]];
        it->point().y = A.pynew[attrib_name[it]];}
        
        // For 3D
        //it->set_point(vec3(A.pxnew[attrib_name[it]],
                         // A.pynew[attrib_name[it]],
                          //A.pznew[attrib_name[it]]));} 
        } 
         
         }              
        ///////////////////////////////////////    
        
        if (P.BBW_flag == true){ 
 
        //// Bounded Biharmonic Weights Deformation
        
        int index_handle = is_handle[c];
        
        c->set_point(new_pos) ;
       
        P.handles_pxnew[index_handle-1] = c->point().x;
        P.handles_pynew[index_handle-1] = c->point().y;
        
        P.Deformation(surface());
               
        P.handles_pxold[index_handle-1] = P.handles_pxnew[index_handle-1];
        P.handles_pyold[index_handle-1] = P.handles_pynew[index_handle-1];
        
        FOR_EACH_VERTEX(Map,surface(), it){
        int index = attrib_name[it];
        vertex_weight[it] = P.BBWeights[index][index_handle-1];
        it->point().x = P.pxnew[index];
        it->point().y = P.pynew[index];
        
        P.pxold[index] = P.pxnew[index];
        P.pyold[index] = P.pynew[index];  
         }
          
        }    
        //////////////////////////////////////////                
                          
       surface()->update(); 
        
       }
       

    
    }
    
    
    
    void SurfaceQuickStartTool::release(const RayPick& p_ndc) {
    }

    void SurfaceQuickStartTool::reset() {
        SurfaceTool::reset() ;
    }
}



