/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#define PI 3.141592653589793238462643383279f

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}
float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}


// FactorialN: Started learning Assignment 3 at this time.
inline float calcTent(const float &a){
    return (a<=0.5?sqrt(2*a)-1:1-sqrt(2*(1-a)));
}
Point2f Warp::squareToTent(const Point2f &sample) {
    return Point2f(calcTent(sample.x()),calcTent(sample.y()));
}


inline float calcTentPdf(const float &a){
    return (a<-1||a>1)?0:1-fabs(a);
}
float Warp::squareToTentPdf(const Point2f &p) {
    return calcTentPdf(p.x())*calcTentPdf(p.y());
}


Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r=sqrt(sample.x()),phi=sample.y()*2.0f*PI;
    return Point2f(r*cos(phi),r*sin(phi));
}
inline float squDiskPdf(const double &a){return a*a;}
float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (squDiskPdf(p.x())+squDiskPdf(p.y())>1?0.0f:1.0f/PI);
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float r=sqrt(1-squDiskPdf(2*sample.x()-1)),phi=2*PI*sample.y();
    return Vector3f(r*cos(phi),r*sin(phi),2*sample.x()-1);
}
float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return fabs(squDiskPdf(v.x())+squDiskPdf(v.y())+squDiskPdf(v.z())-1)<1e-5?1.0f/(4.0f*PI):0.0f;
}


Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float r=sqrt(1-squDiskPdf(2*sample.x()-1)),phi=2*PI*sample.y();
    return Vector3f(r*cos(phi),r*sin(phi),abs(2*sample.x()-1));
}
float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return fabs(squDiskPdf(v.x())+squDiskPdf(v.y())+squDiskPdf(v.z())-1)<1e-5&&v.z()>0?1.0f/(2.0f*PI):0.0f;
}


Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float r=sqrt(sample.x()),phi=sample.y()*2.0f*PI;
    return Vector3f(r*cos(phi),r*sin(phi),sqrt(1-r*r));
}
float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if(fabs(squDiskPdf(v.x())+squDiskPdf(v.y())+squDiskPdf(v.z())-1)>=1e-5||v.z()<=0)return 0.0f;
    return v.z()/PI;
}


Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float cth, sth;
    if (sample.y() == 1.0f)cth = 0.0f;
    else {
        float t1 = log(1 - sample.y()) * alpha * alpha;
        t1 = 1 - t1;
        cth = sqrt(1.0f / t1);
    }
    float phi = 2.0f * M_PI * sample.x();
    sth = sqrt(1 - cth * cth);
    return Vector3f(sth * cos(phi), sth * sin(phi), cth);
}
float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if ((m.z() > 0) && (m.norm() <= 1)) {
        float theta = acos(m.z());
        float d = 0.5f / M_PI;
        float t1 = tan(theta) / alpha;
        float t2 = cos(theta);
        d *= 2.0f * exp(-t1 * t1);
        d /= alpha * alpha * t2 * t2 * t2;
        return d;
    }
    else return 0.0f;
}

NORI_NAMESPACE_END
