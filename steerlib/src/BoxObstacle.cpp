//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/// @file BoxObstacle.cpp
/// @brief Implements the BoxObstacle class.

#include "obstacles/BoxObstacle.h"
#include "util/DrawLib.h"

using namespace SteerLib;
using namespace Util;

void BoxObstacle::draw() {
#ifdef ENABLE_GUI
	DrawLib::glColor(Util::gBlue);
	DrawLib::drawBoxWireFrame(_bounds.xmin,_bounds.xmax,_bounds.ymin,_bounds.ymax,_bounds.zmin,_bounds.zmax);
	DrawLib::glColor(Color(1.0f, 1.0f, 1.0f));
	DrawLib::drawBox(_bounds.xmin,_bounds.xmax,_bounds.ymin,_bounds.ymax,_bounds.zmin,_bounds.zmax);
#endif // ifdef ENABLE_GUI
}
