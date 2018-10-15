//
// Created by factorialn on 2018/9/17.
//
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN
#define PI 3.141592653589793238462643383279f

    class EmsIntegrator : public Integrator {
    public:
        EmsIntegrator(const PropertyList &props) {
            /* No parameters this time */
        }

        /*FactorialN did this in PA4.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, bool diff) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            const BSDF *bsdf = its.mesh->getBSDF();
            const Emitter *emitter = its.mesh->getEmitter();

            std::vector<Mesh*> meshes = scene->getMeshes();
            std::vector<int> meshIndex;
            meshIndex.clear();
            DiscretePDF dpdf;
            dpdf.clear();
            for(uint32_t i = 0; i < meshes.size(); i++){
                if(meshes[i]->getEmitter() != nullptr){
                    dpdf.append(1.0f);
                    meshIndex.push_back(i);
                }
            }
            dpdf.normalize();
            Color3f res = Color3f(0.0f);
            BSDFQueryRecord bRec(Vector3f(0.0f, 0.0f, 0.0f));
            if (emitter != nullptr){
                res = emitter->getEmit();
                //if(diff) res *= 0.01f;
                bRec = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
            }
            else{
                float emitterPDF;
                Emitter *chsdEmitter = meshes[meshIndex[dpdf.sample(drand48(), emitterPDF)]]->getEmitter();
                Point3f p;
                Normal3f n;
                chsdEmitter->samplePosition(Point2f(drand48(), drand48()), p, n);
                Vector3f d = p - its.p;
                float dis = fmax(d.norm(), 1e-4);
                Vector3f dn = d.normalized();
            
                bRec = BSDFQueryRecord(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(dn), ESolidAngle);
            }
            if(drand48() < 0.95){
                Color3f xcolor = bsdf->sample(bRec, Point2f(drand48(), drand48()));
                Color3f reLix = Li(scene, sampler, Ray3f(its.p, its.shFrame.toWorld(bRec.wo)), diff||bsdf->isDiffuse());
                res += reLix * xcolor / 0.95;
            }
            return res;
        }

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            return Li(scene, sampler, ray, false);
        }

        std::string toString() const {
            return "EmsIntegrator[]";
        }
    private:
        Warp *generaTor = new Warp();
    };
    NORI_REGISTER_CLASS(EmsIntegrator, "path_ems");
NORI_NAMESPACE_END