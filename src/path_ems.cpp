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
                return emitter->getEmit() / its.t / its.t;
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
            Vector3f dn = d.normalized();
            Normal3f nn = its.shFrame.n;
            float tg = !scene->rayIntersect(Ray3f(its.p, dn, 1e-4, dis-1e-4));
            Color3f Lt = chsdEmitter->getEmit() * fmax(0.0f, -n.dot(dn)) * fabs(nn.dot(dn)) / (dis * dis) * tg / emitterPDF / chsdEmitter->Pdf();
            
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(dn), ESolidAngle);
            Color3f dcolor = bsdf->eval(bRec);
            Color3f res = Lt * dcolor;

            if(drand48() < 0.95){
                if(!bsdf->isDiffuse()){
                    Color3f xcolor = bsdf->sample(bRec, Point2f(drand48(), drand48()));
                    Color3f reLix = Li(scene, sampler, Ray3f(its.p, its.shFrame.toWorld(bRec.wo)));
                    res += reLix * xcolor / 0.95;
                }
            }
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