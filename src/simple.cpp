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
            lightPoint=props.getPoint("position");
            EmiC=props.getColor("energy");
        }

        /*FactorialN did this in PA1.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            /* Return the component-wise absolute
               value of the shading normal as a color */
            Normal3f n = its.shFrame.n.cwiseAbs();
            Vector3f x = ray.o+ray.d*its.t;
            Vector3f d = lightPoint - x;
            float lenD=sqrt(d.x()*d.x()+d.y()*d.y()+d.z()*d.z());
            float lenN=sqrt(n.x()*n.x()+n.y()*n.y()+n.z()*n.z());
            float cosTHETA = (d.x()*n.x()+d.y()*n.y()+d.z()*n.z())/(lenD*lenN);
            float tg = 1;
            if(scene->rayIntersect(Ray3f(x,d,1e-5,1.0f-1e-5)))tg=0;
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