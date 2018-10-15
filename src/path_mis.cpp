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

    class MisIntegrator : public Integrator {
    public:
        MisIntegrator(const PropertyList &props) {
            /* No parameters this time */
        }

        /*FactorialN did this in PA4.*/
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, bool diff, float pb) const {
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
            float lightSum = dpdf.normalize();
            Color3f res = Color3f(0.0f);
            float pl;
            BSDFQueryRecord bRec(Vector3f(0.0f, 0.0f, 0.0f));
            if (emitter != nullptr){
                res = emitter->getEmit();
                if(diff){
                    pl = 1.0 / emitter->Pdf() / lightSum * its.t * its.t;
                    res *= pb / (pb + pl) ;
                }
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
                Normal3f nn = its.shFrame.n;
                float tg = !scene->rayIntersect(Ray3f(its.p, dn, 1e-5, dis-1e-5));
                Color3f Lt = chsdEmitter->getEmit() * fmax(0.0f, -n.dot(dn)) * fabs(nn.dot(dn)) / (dis * dis) * tg / emitterPDF / chsdEmitter->Pdf();
            
                bRec = BSDFQueryRecord(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(dn), ESolidAngle);
                Color3f dcolor = bsdf->eval(bRec);
                pl = fmax(1e-5, chsdEmitter->Pdf() * emitterPDF * dis * dis / (-n.dot(dn)));
                pb = bsdf->pdf(bRec);
                res = Lt * dcolor * pl / (pb + pl);
            }
            if(drand48() < 0.95){
                Color3f xcolor = bsdf->sample(bRec, Point2f(drand48(), drand48()));
                Color3f reLix = Li(scene, sampler, Ray3f(its.p, its.shFrame.toWorld(bRec.wo)), bsdf->isDiffuse(), bsdf->pdf(bRec));
                res += reLix * xcolor / 0.95;
            }
            return res;
        }

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            return Li(scene, sampler, ray, false, 0.0f);
        }

        std::string toString() const {
            return "MisIntegrator[]";
        }
    private:
        Warp *generaTor = new Warp();
    };
    NORI_REGISTER_CLASS(MisIntegrator, "path_mis");
NORI_NAMESPACE_END