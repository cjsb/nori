//
// Created by factorialn on 2018/9/17.
//
#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
#define PI 3.141592653589793238462643383279f

    class SimpleIntegrator : public Integrator {
    public:
        SimpleIntegrator(const PropertyList &props) {
            /* No parameters this time */
            lightPoint = props.getPoint("position");
            EmiC = props.getColor("energy");
        }

        /*FactorialN did this in PA3.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            Normal3f n = its.shFrame.n;
            Vector3f x = its.p;
            Vector3f d = lightPoint - x;
            float lenD=sqrt(d.dot(d));
            d/=lenD;
            float cosTHETA = n.dot(d);
            float tg = !scene->rayIntersect(Ray3f(x,d,1e-4,lenD-1e-4));
            
            Color3f Lt = EmiC/(4*PI*PI)*fmax(0.0f,cosTHETA)/(lenD*lenD)*tg;
            return Lt;
        }

        std::string toString() const {
            return "SimpleIntegrator[]";
        }
    private:
        Point3f lightPoint;
        Color3f EmiC;
    };
    NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END