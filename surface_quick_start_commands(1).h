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
 

#ifndef __OGF_QUICK_START_COMMANDS_S_QUICK_START_COMM__
#define __OGF_QUICK_START_COMMANDS_S_QUICK_START_COMM__

#include <OGF/quick_start/common/common.h>
#include <OGF/surface/commands/surface_commands.h>
#include <OGF/surface/grob/surface.h>
#include <OGF/cells/map/map_attributes.h>	
#include <vector>
#include <OGF/quick_start/LSSolver/ls_solver.hpp>
//#include <OGF/quick_start/ARAP/ARAP.h>



namespace OGF {

    // Insert your commands in this file.
    // Two commands are provided:
    // * algorithm_under_test is a place-holder,
    //  you can use it directly to call your code.
    // * command_example shows different type of
    //  arguments that can be used.
    // Feal free to experiment, change the arguments,
    // add new commands (see instructions below).
    //
    // Basics
    // ------
    //
    //  Graphite commands are implemented by classes.
    //  The AutoGUI subsystem automatically generates
    // menus and dialog boxes for each member function
    // of a command class.
    //
    //  AutoGUI uses "GOM meta-information", i.e. a 
    // representation of classes, member functions and 
    // their arguments. This meta_information is located
    // in one automatically generated C++ source file
    // per package. Each time you modify the interface
    // of the classes that should have meta-information
    // (i.e. commands, tools and shaders), you need to
    // regenerate this file, by 'touching' the file
    // quick_start/common/gom_adapters_quick_start.gom
    // (this '.gom' file is there just to trigger the
    // generation of the meta-information)

    //  Advanced
    //  --------
    //  In addition, any C++ class that has a << and >> operator can 
    //  be used as an argument to gom_slots.
    //  The default widget is a TextField. This can be changed by
    //  binding a widget type to an argument type in AutoGUI,
    //  this should be done in the surface_quick_start_common.cpp
    //  file:
    //  AutoGUI::instance()->declare_widget("MyClass", "ThumbWheel") ;
    
    enum RobotType {
        r2d2    = 0,
        z6po    = 1,
        krag    = 2,
        malla   = 3,
        G7zark7  = 4,
        G1nonos1 = 5,
        nono    = 6
    } ;

    gom_class QUICK_START_API SurfaceQuickStartCommands : public SurfaceCommands {
    public:
        SurfaceQuickStartCommands() { }
        static Map* nextInScene(Surface* s);

    gom_slots:
    /*
        void command_example(
            double my_value,
            int my_integer,
            const Color& my_color,
            bool my_flag,
            RobotType my_robot,
            const std::string& my_string
        ) ;

        void algorithm_under_test(
            double arg1 = 0, double arg2 = 0, double arg3 = 0,
            double arg4 = 0, double arg5 = 0, double arg6 = 0
        ) ;
        
        */
        
        //int index_vertices();
        void ICP(int Iteration,bool Acceleration,float Threshold);
        void Rotation(bool x_axis, bool y_axis, bool z_axis,float angle);
        void LaplacianPrecompute();
        void Cot_Weights();
        void Tan_Weights();
        void Uniform_Weights();
        void BoundedBiharmonic(); // Compute weights at bind time
        void Triangulation();

    } ;

}

#endif
