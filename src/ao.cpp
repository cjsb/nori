//
// Created by factorialn on 2018/9/17.
//
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
#define PI 3.141592653589793238462643383279f

    class AoIntegrator : public Integrator {
    public:
        AoIntegrator(const PropertyList &props) {
            /* No parameters this time */
        }

        /*FactorialN did this in PA3.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            Normal3f n = its.shFrame.n;
            Vector3f x = its.p;
            Color3f Lt = Color3f(0.0f,0.0f,0.0f);
            uint32_t tms = 1;
            for(uint32_t k=0;k<tms;k++){
                Vector3f dx = generaTor->squareToCosineHemisphere(Point2f(drand48(),drand48()));
                Vector3f d = its.toWorld(dx);
                float tg = 1.0/tms;
                if(scene->rayIntersect(Ray3f(x,d,1e-4,20)))tg=0;
                Lt += Color3f(0.95f,0.95f,0.95f)*tg;
            }
            return Lt;
        }

        std::string toString() const {
            return "AoIntegrator[]";
        }
    private:
        Warp *generaTor = new Warp();
    };
    NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END