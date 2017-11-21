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

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class Triangle2;
class Point2;

class Edge2
{
public:
	Edge2();
	Edge2(const Edge2& e_);
	Edge2(Triangle2* pT,int oppIdx_);
	~Edge2();

	// Combinatorial operator<
	bool operator<(const Edge2& e) const
	{
		if(pT<e.pT) return true;
		if(pT>e.pT) return false;
		if(oppIdx<e.oppIdx) return true;
		return false;
	}

	bool operator==(const Edge2& e) const
	{
		return(pT==e.pT && oppIdx==e.oppIdx);
	}

	bool operator!=(const Edge2& e) const
	{
		return((pT!=e.pT || oppIdx!=e.oppIdx));
	}

	void getPoints(Point2*& p1,Point2*& p2) const;

	double getLength2D() const;
	double getLength25D() const;


	void getTriangles(Triangle2*& pT0,Triangle2*& pT1,int& idx0,int& idx1) const;


	friend std::ostream &operator<<(std::ostream &stream, const Edge2& e);

protected:
	Triangle2* pT;
	int oppIdx;
};

struct Func_ltUndirected
{
	bool operator()(const Edge2& eA,const Edge2& eB) const
	{
		Point2 *pA1,*pA2,*pB1,*pB2;
		eA.getPoints(pA1,pA2);
		eB.getPoints(pB1,pB2);
		if(pA1>pA2) std::swap(pA1,pA2);
		if(pB1>pB2) std::swap(pB1,pB2);

		if(pA1<pB1) return true;
		if(pA1>pB1) return false;
		if(pA2<pB2) return true;
		if(pA2>pB2) return false;
		return false;
	}
};


struct Func_ltEdge2D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength2D()<e1.getLength2D()) return true;
			else return false;
	}
};

struct Func_gtEdge2D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength2D()>e1.getLength2D()) return true;
			else return false;
	}
};



#if GEOM_PSEUDO3D==GEOM_TRUE
struct Func_ltEdge25D
{
	bool operator()(const Edge2& e0,const Edge2& e1) const
	{
		if(e0.getLength25D()<e1.getLength25D()) return true;
			else return false;
	}
};
#endif




} // (namespace)
