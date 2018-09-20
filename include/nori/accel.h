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

#pragma once

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */

/*
* FactorialN: This is a class discribing the node of the Octree, the triangles and the sons are needed here.
*/
class OctreeNode{
public:
    OctreeNode(){n = 1;tri=new uint32_t[1];tri[0]=1;}
    OctreeNode(const OctreeNode &t){
        //cerr<<"Copy construct: "<<t.n<<endl;
        for(uint32_t i = 0; i < 8; i++)son[i] = t.son[i];
        tri = new uint32_t[t.n];
        for(uint32_t i = 0; i < t.n; i++)tri[i] = t.tri[i];
        n = t.n;
        bbox = t.bbox;
    }
    OctreeNode(uint32_t *a, const uint32_t &m){
        for(uint32_t i = 0; i < 8; i++)son[i] = 0;
        tri = new uint32_t[m];
        for(uint32_t i = 0; i < m; i++)tri[i] = a[i];
        n = m;
    }
    OctreeNode(BoundingBox3f &m_bbox){
        for(uint32_t i = 0; i < 8; i++)son[i] = 0;
        bbox = m_bbox;
        n = 0;
    }
    ~OctreeNode(){
        
    }
    uint32_t son[8];
    BoundingBox3f bbox;
    uint32_t *tri;
    uint32_t n;
};
class Octree{
public: 
    Octree(Mesh *m_mesh, BoundingBox3f &m_bbox){tot=0;
        uint32_t k = m_mesh->getTriangleCount();
        tri = new uint32_t[k];
        for(uint32_t i=0; i<k; i++)tri[i]=i;
        plist = new OctreeNode[3*k];
        build(root, 0, m_mesh, m_mesh->getBoundingBox(), tri, k, 1);
        cerr<<"built "<<tot<<endl;
    }
    void build(uint32_t &rt, uint32_t fa, Mesh *m_mesh, BoundingBox3f m_bbox, uint32_t *a, uint32_t n, uint32_t depth){
        if(n <= 10 || depth > 50){
            rt = ++tot;
            plist[tot]=OctreeNode(a, n);
            return;
        }
        rt = ++tot;
        plist[tot]=OctreeNode(m_bbox);
        Point3f cen = plist[rt].bbox.getCenter();
        uint32_t *c = new uint32_t[n];
        bool *usd = new bool[n];

        Point3f min = m_bbox.min;
        Point3f max = m_bbox.max;
        Point3f ExtendingBox[8];
        ExtendingBox[0]=Point3f(min.x(), min.y(), min.z());
        ExtendingBox[1]=Point3f(max.x(), min.y(), min.z());
        ExtendingBox[2]=Point3f(min.x(), max.y(), min.z());
        ExtendingBox[3]=Point3f(max.x(), max.y(), min.z());
        ExtendingBox[4]=Point3f(min.x(), min.y(), max.z());
        ExtendingBox[5]=Point3f(max.x(), min.y(), max.z());
        ExtendingBox[6]=Point3f(min.x(), max.y(), max.z());
        ExtendingBox[7]=Point3f(max.x(), max.y(), max.z());

        for(uint32_t i = 0; i < n; i++) usd[i] = 0;
        for(uint32_t i = 0; i < 8; i++){
            BoundingBox3f result(cen);
            result.expandBy(ExtendingBox[i]);
            c[0]=0;
            for(uint32_t idx = 0; idx < n; idx++){
                if(usd[idx])continue;
                BoundingBox3f bdb = m_mesh->getBoundingBox(a[idx]);
                if(result.overlaps(bdb)){
                    c[0]++;
                    c[c[0]] = a[idx];
                    usd[idx] = 1;
                }
            }
            if(c[0]){
                uint32_t *b = new uint32_t[c[0]];
                for(uint32_t idx = 1; idx <= c[0]; idx++) b[idx-1] = c[idx];
                build(plist[rt].son[i], rt, m_mesh, result, b, c[0], depth + 1);
            }
        }
    }
    bool Intersect(uint32_t rt, Ray3f &ray, Intersection &its, bool shadowRay, Mesh *m_mesh, uint32_t &f, bool &foundIntersection){
        std::pair<float,uint32_t>srt[10];
        uint32_t inTot=0;
        for(uint32_t i = 0; i < 8; i++){
            if(plist[rt].son[i] != 0){
                float nearT, farT;
                bool tg = plist[plist[rt].son[i]].bbox.rayIntersect(ray,nearT,farT);
                if(tg)srt[inTot++]=std::pair<float,uint32_t>(nearT,i);
            }
        }
        std::sort(srt,srt+inTot);
        for(uint32_t i = 0; i < inTot; i++)
            if(plist[rt].son[srt[i].second] != 0){
                if(ray.maxt < srt[i].first)break;
                bool tg = Intersect(plist[rt].son[srt[i].second], ray, its, shadowRay, m_mesh, f, foundIntersection);
                if(tg)return true;
            }
        if(plist[rt].n){
            for(uint32_t idx = 0; idx < plist[rt].n; idx++){
                float u, v, t;
                if (m_mesh->rayIntersect(plist[rt].tri[idx], ray, u, v, t)) {
                    if (shadowRay)
                        return true;
                    ray.maxt = its.t = t;
                    its.uv = Point2f(u, v);
                    its.mesh = m_mesh;
                    f = plist[rt].tri[idx];
                    foundIntersection = true;
                }
            }
        }
        return false;
    }
    bool rayIntersect(Ray3f &ray, Intersection &its, bool shadowRay, Mesh *m_mesh, uint32_t &f, bool &foundIntersection){
        bool tg=Intersect(root, ray, its, shadowRay, m_mesh, f, foundIntersection);
        return tg;
    }
    ~Octree(){
        
    }
private:
    OctreeNode *plist;
    uint32_t tot;
    uint32_t root;
    uint32_t *tri;
};

class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();

    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

private:
    Mesh         *m_mesh = nullptr; ///< Mesh (only a single one for now)
    BoundingBox3f m_bbox; ///< Bounding box of the entire scene
    Octree *m_ocTree; // The Octree of the Scene
};

NORI_NAMESPACE_END
