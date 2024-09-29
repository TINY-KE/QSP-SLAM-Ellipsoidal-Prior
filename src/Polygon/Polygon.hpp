/*
*	This file is dependent on github open-source. All rights are perserved by original authors.
* 	Web: https://github.com/abreheret/polygon-intersection
*/
// Update: Add the API for EllipsoidSLAM.

#ifndef __EllipsoidSLAM_POLYGON_HPP__
#define __EllipsoidSLAM_POLYGON_HPP__

#include <cxcore.h>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{
	
float distPoint( cv::Point p1, cv::Point p2 ) ;
float distPoint(CvPoint2D32f p1,CvPoint2D32f p2) ;
bool segementIntersection(cv::Point p0_seg0,cv::Point p1_seg0,cv::Point p0_seg1,cv::Point p1_seg1,cv::Point * intersection) ;
bool segementIntersection(CvPoint2D32f p0_seg0,CvPoint2D32f p1_seg0,CvPoint2D32f p0_seg1,CvPoint2D32f p1_seg1,CvPoint2D32f * intersection) ;

bool pointInPolygon(cv::Point p,const cv::Point * points,int n) ;
bool pointInPolygon(CvPoint2D32f p,const CvPoint2D32f * points,int n) ;


#define MAX_POINT_POLYGON 64
struct Polygon {
	cv::Point pt[MAX_POINT_POLYGON];
	int n;

	Polygon(int n_ = 0 ) { assert(n_>= 0 && n_ < MAX_POINT_POLYGON); n = n_;}
	virtual ~Polygon() {}

	void clear() { n = 0; }
	void add(const cv::Point &p) {if(n < MAX_POINT_POLYGON) pt[n++] = p;}
	void push_back(const cv::Point &p) {add(p);}
	int size() const { return n;}
	cv::Point getCenter() const ;
	const cv::Point & operator[] (int index) const { assert(index >= 0 && index < n); return pt[index];}
	cv::Point& operator[] (int index) { assert(index >= 0 && index < n); return pt[index]; }
	void pointsOrdered() ;
	float area() const ;
	bool pointIsInPolygon(cv::Point p) const ;
};


void intersectPolygon( const cv::Point * poly0, int n0,const cv::Point * poly1,int n1, Polygon & inter ) ;
void intersectPolygon( const Polygon & poly0, const Polygon & poly1, Polygon & inter ) ;
void intersectPolygonSHPC(const Polygon * sub,const Polygon* clip,Polygon* res) ;
void intersectPolygonSHPC(const Polygon & sub,const Polygon& clip,Polygon& res) ;

}

#endif //