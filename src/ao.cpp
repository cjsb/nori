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

            Normal3f n = its.shFrame.n.cwiseAbs();
            Vector3f x = ray.o+ray.d*its.t;
            Color3f Lt = Color3f(0.0f,0.0f,0.0f);
            for(int k=0;k<2;k++){
                Vector3f dx = generaTor->squareToCosineHemisphere(Point2f(drand48(),drand48()));
                Frame localFrame = Frame(n);
                Vector3f d = localFrame.toWorld(dx);
                float tg = 1;
                if(scene->rayIntersect(Ray3f(x,d)))tg=0;
                Lt += Color3f(1.0f,1.0f,1.0f)*tg/3.0f;
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