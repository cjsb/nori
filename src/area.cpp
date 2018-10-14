//
// Created by factorialn on 2018/10/02.
//

#include <nori/mesh.h>
#include <nori/emitter.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter{
public:
    AreaLight(const PropertyList &props){
        Emit = props.getColor("radiance");
    }

    Color3f getEmit() const {return Emit;}

    void activate(){}

    void activate(MatrixXf &V, MatrixXf &N, MatrixXu &F, uint32_t ns);

    float surfaceArea(uint32_t index) const;

    void samplePosition(Point2f sample, Point3f &p, Normal3f &n);

    std::string toString() const{
        return "AreaLight[]";
    }
private:
    DiscretePDF *dpdf = nullptr;
    Color3f Emit;
    MatrixXf      m_V;                   ///< Vertex positions
    MatrixXf      m_N;                   ///< Vertex normals
    MatrixXu      m_F;                   ///< Faces
    uint32_t n;
};

void AreaLight::activate(MatrixXf &V, MatrixXf &N, MatrixXu &F, uint32_t ns) {
        m_V = V;
        m_N = N;
        m_F = F;
        n = ns;
        dpdf = new DiscretePDF(n);
        dpdf->clear();
        for(uint32_t i = 0; i < n; i++)dpdf->append(surfaceArea(i));
        dpdf->normalize();
}

float AreaLight::surfaceArea(uint32_t index) const {
        uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
        const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);
        return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

void AreaLight::samplePosition(Point2f sample, Point3f &p, Normal3f &n){
        uint32_t index = dpdf->sampleReuse(sample.x());
        uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
        const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);
        float alpha = 1 - sqrt(1 - sample.x()), beta = sample.y() * sqrt(1 - sample.x()), theta = 1 - alpha - beta;
        p = alpha * p0 + beta * p1 + theta * p2;
        n = Vector3f((p1 - p0).cross(p2 - p0));
        n /= n.norm();
}

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END