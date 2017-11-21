// Copyright (C) Geom Software e.U, Bernhard Kornberger, Graz/Austria
//
// This file is part of the Fade2D library. The student license is free
// of charge and covers personal non-commercial research. Licensees
// holding a commercial license may use this file in accordance with
// the Commercial License Agreement.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are
// not clear to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at



#pragma once
#include "Point2.h"
#include "Segment2.h"
#include "Edge2.h"
#include <vector>

#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif


/** \defgroup freeFunctions Free Functions
 *
 *  @{
 */

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get normal vector
 *
* Returns the normalized normal vector of a triangle made of the three
* input points
*/
CLASS_DECLSPEC
Vector2 getNormalVector(const Point2& p0,const Point2& p1,const Point2& p2);
#endif


/** \brief Get directed edges
 *
* The directed edges of \p vT are returned, i.e. edges with two adjacent
* triangles are contained twice with different orientations.
*/
CLASS_DECLSPEC
void getDirectedEdges(std::vector<Triangle2*>& vT,std::vector<Edge2>& vDirectedEdgesOut);

/** \brief Get undirected edges
 *
* A unique set of edges of \p vT is returned.
*/
CLASS_DECLSPEC
void getUndirectedEdges(std::vector<Triangle2*>& vT,std::vector<Edge2>& vUndirectedEdgesOut);



/** \brief Fade2D version string
 *
 *
* This method returns a version string
*/
CLASS_DECLSPEC
std::string getFade2DVersion();
/** \brief Get the major version number
*/
CLASS_DECLSPEC
int getMajorVersionNumber();
/** \brief Get the minor version number
*/
CLASS_DECLSPEC
int getMinorVersionNumber();
/** \brief Get the revision version number
*/
CLASS_DECLSPEC
int getRevisionNumber();
/** \brief Check if a RELEASE or a DEBUG version is used.
*/
CLASS_DECLSPEC
bool isRelease();
/** \brief Get Borders
 *
 * Computes the border of the triangles in \p vT
 *
 * \param [in] vT are the input triangles
 * \param [out] vBorderSegmentsOut is used to return all edges of triangles
 * in vT which have only one adjacent triangle
*/
CLASS_DECLSPEC
void getBorders(const std::vector<Triangle2*>& vT,std::vector<Segment2>& vBorderSegmentsOut);
/** \brief Sort a vector of Segments
 *
 * The segments in vRing are re-aligned and sorted such that subsequent
 * segments join at the endpoints.
*/
CLASS_DECLSPEC
bool sortRing(std::vector<Segment2>& vRing);

/** \brief Get the orientation of three points
 *
 * This function returns the \e exact orientation of the points p0,p1,p2.
 * Possible values are \n
 * ORIENTATION2_COLLINEAR if p0,p1,p2 are located on a line, \n
 * ORIENTATION2_CCW if p0,p1,p2 are counterclockwise oriented \n
 * ORIENTATION2_CW if p0,p1,p2 are clockwise oriented \n
 *
 * Not thread-safe but a bit faster than the thread-safe version
*/

CLASS_DECLSPEC
Orientation2 getOrientation2(const Point2* p0,const Point2* p1,const Point2* p2);
/** \brief Get Orientation2 (MT)
 *
 * This function returns the \e exact orientation of the points p0,p1,p2.
 * Possible values are \n
 * ORIENTATION2_COLLINEAR if p0,p1,p2 are located on a line, \n
 * ORIENTATION2_CCW if p0,p1,p2 are counterclockwise oriented \n
 * ORIENTATION2_CW if p0,p1,p2 are clockwise oriented \n
 *
 * This version is thread-safe.
*/

CLASS_DECLSPEC
Orientation2 getOrientation2_mt(const Point2* p0,const Point2* p1,const Point2* p2);

/** \brief Get human readable Orientation2 string
 *
 * Thought for debugging
*/
CLASS_DECLSPEC
std::string getString(const Orientation2 ori);


// License type
CLASS_DECLSPEC
void setLic(
	const std::string& l1,
	const std::string& l2,
	const std::string& dt,
	const std::string& s1,
	const std::string& s2_
	);
class Lic;
Lic* getLic();


/** \brief Read (x y) points
 *
 * Reads points from an ASCII file. Expected file format: Two
 * coordinates (x y) per line, whitespace separated.
 *
 * \cond SECTION_FADE25D
 * The z coordinate is set to 0.
 * \endcond
*/
CLASS_DECLSPEC
bool readXY(const char* filename,std::vector<Point2>& vPointsOut);

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Read (x y z) points
 *
 * Reads points from an ASCII file. Expected file format: Three
 * coordinates (x y z) per line, whitespace separated.
*/
CLASS_DECLSPEC
bool readXYZ(const char* filename,std::vector<Point2>& vPointsOut);

/** \brief Write points to an ASCII file
 *
 * Writes points to an ASCII file, three coordinates (x y z) per line,
 * whitespace separated.
*/
CLASS_DECLSPEC
bool writePoints(const char* filename,const std::vector<Point2*>& vPointsIn);

/** \brief Write points to an ASCII file
 *
 * Writes points to an ASCII file, three coordinates (x y z) per line,
 * whitespace separated.
*/
CLASS_DECLSPEC
bool writePoints(const char* filename,const std::vector<Point2>& vPointsIn);
#endif






/** @}*/
} // NAMESPACE
