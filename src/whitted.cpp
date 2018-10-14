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

    class WhittedIntegrator : public Integrator {
    public:
        WhittedIntegrator(const PropertyList &props) {
            /* No parameters this time */
        }

        /*FactorialN did this in PA4.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            const BSDF *bsdf = its.mesh->getBSDF();
            const Emitter *emitter = its.mesh->getEmitter();
            if (emitter != nullptr){
                return emitter->getEmit();
            }

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
            float emitterPDF;
            Emitter *chsdEmitter = meshes[meshIndex[dpdf.sample(drand48(), emitterPDF)]]->getEmitter();
            Point3f p;
            Normal3f n;
            chsdEmitter->samplePosition(Point2f(drand48(), drand48()), p, n);
            Vector3f d = p - its.p;
            float dis = d.norm();
            Vector3f dn = d / dis;
            Normal3f nn = its.shFrame.n;
            float cosTHETA = nn.dot(dn);
            float tg = !scene->rayIntersect(Ray3f(its.p, dn, 1e-4, dis-1e-4));
            Color3f Lt = chsdEmitter->getEmit() * (4 * PI * PI) * fmax(0.0f, cosTHETA) / (dis * dis) * tg;
            
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(dn), ESolidAngle);
            Color3f dcolor = bsdf->eval(bRec);
            Color3f res = Lt * dcolor;

            return res;
        }

        std::string toString() const {
            return "WhittedIntegrator[]";
        }
    private:
        Warp *generaTor = new Warp();
    };
    NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END