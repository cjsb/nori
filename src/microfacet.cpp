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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    float Dc(const Vector3f &wh) const {
        float cosh = Frame::cosTheta(wh);
        float sinh = Frame::sinTheta(wh);
        return exp(- sinh * sinh / cosh / cosh / m_alpha / m_alpha)/ cosh / cosh / cosh / m_alpha / m_alpha;
    }

    float Gc(const Vector3f &wv,const Vector3f &wh) const {
        float cosv = Frame::cosTheta(wv);
        float sinv = Frame::sinTheta(wv);
        float b = cosv / (m_alpha * sinv);
        float secp = b < 1.6 ? (3.535f * b + 2.181f * b * b) / (1 + 2.276f * b + 2.577f * b * b) : 1.0f;
        float firp = wv.dot(wh) / cosv <= 0 ? 0 : 1;
        return firp * secp;
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
        
        Vector3f wh = (bRec.wi + bRec.wo);
        wh.normalize();
        Color3f fr = Color3f(1.0f) * m_ks * Dc(wh);
        fr *= fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        fr *= Gc(bRec.wi, wh) * Gc(bRec.wo, wh);
        fr /= 4.0f * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) * Frame::cosTheta(wh);
        fr += m_kd / M_PI;
        return fr;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0.0f
            || Frame::cosTheta(bRec.wo) <= 0.0f)
            return 0.0f;
    	
        Vector3f wh = (bRec.wi + bRec.wo);
        wh.normalize();
        float fr = m_ks * Dc(wh);
        fr *= Frame::cosTheta(wh);
        fr /= 4.0f * wh.dot(bRec.wo);
        fr += (1 - m_ks) * Frame::cosTheta(bRec.wo) / M_PI;
        return fr;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        if(Frame::cosTheta(bRec.wi) <= 0.0f) return Color3f(0.0f);
        bRec.eta = 1.0f;
        bRec.measure = ESolidAngle;
        if(drand48() < m_ks){
            Point2f sp = _sample;
            Vector3f wn = Warp::squareToBeckmann(sp, m_alpha);
            bRec.wo = 2 * wn * bRec.wi.dot(wn) - bRec.wi;
        }
        else{
            Point2f sp = _sample;
            bRec.wo = Warp::squareToCosineHemisphere(sp);
        }
        if(Frame::cosTheta(bRec.wi) <= 0.0f) return Color3f(0.0f);
        float PDF = pdf(bRec);
        if(PDF > 0)return eval(bRec) * Frame::cosTheta(bRec.wo) / PDF;
        else return Color3f(0.0f);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
