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
 
#ifndef __OGF_QUICK_START_SURFACE_QS_TOOL__
#define __OGF_QUICK_START_SURFACE_QS_TOOL__

#include <OGF/quick_start/common/common.h>
#include <OGF/surface/common/common.h>
#include <OGF/surface/grob/surface.h>
#include <OGF/surface/tools/surface_tool.h>



namespace OGF {


    // specifies in which box this tool will be added 
    gom_attribute(category,"QuickStart") ;
    //an icon can be specified for this tool
    //(this example corresponds to GRAPHITE_ROOT/lib/icons/my_icon.xpm)
    // gom_attribute(icon,"my_icon") ;
    // specifies the help bubble associated with this tool 
    gom_attribute(help,"quick start tool") ;
    // the message is displayed in the status bar when this
    // tool is selected 
    gom_attribute(message,"insert your message here") ;

    gom_class QUICK_START_API SurfaceQuickStartTool : public SurfaceTool {
    public:
        SurfaceQuickStartTool(ToolsManager* parent) ;
        virtual void grab(const RayPick& p_ndc) ;
        virtual void drag(const RayPick& p_ndc) ;
        virtual void release(const RayPick& p_ndc) ;
        virtual void reset() ;
    } ;    

}

#endif

