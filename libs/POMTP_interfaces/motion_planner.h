/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <skeleton.h>

class MotionPlanner
{
public:
    typedef std::shared_ptr< MotionPlanner > ptr;

public:
    virtual void setKin( const std::string & kinDescription, const arr& q = NoArr ) = 0; // specify start kinematics
    virtual void solveAndInform( const MotionPlanningParameters &, Policy &, bool watchResult = false, bool saveVideo = false ) = 0;
    virtual void display( const Policy &, double sec = 30 ) {};
};
