# Summary of Nori
This is FactorialN's learning of Nori.

## PA 1 Learning to Use Nori

In this section I spent a very long time finding the correct version of CMake, and finally found that CMake 3.11.4 works. It is the first time of me to compile a big project with CMake, and successfully get the Normal render of the bunny.

## PA 2 Use Octree to Accelerate Intersection

There's two tricks that works on finding mesh:
1. use Octree with bounding box
2. visit the sons in the order of distance

It's because Intersection is the main time consumer in Nori, which appears in every iteration.
The biggest problem in this part is that I fail to use C++ pointers correctly, which cause me to waste a lot of time debugging.

I modified accel.h and added serveral method to accelerate intersection, and it's now in the file myaccel.h.

## PA 3 Possibility Generator and simple Light Shading

In this part, I implemented serveral possibility generator and possibility density functions. It is used for sampling. Here's some of the possiblity functions:
1. UniformSquare: $p(x,y)=(x,y \in [0,1])$, and $\vec p=\vec sp$
2. Tent: $p(x,y)=p(x)p(y),p(x)=max(0,1-|x|)$, and $\vec p=inv(\vec sp),inv(y)=y<0.5?1-\sqrt{2y}:\sqrt{2y}-1$
3. UniformDisk:
4. UniformSphere:
5. UniformHemiSphere:
6. CosineHemiSphere:
7. Beckmann:
This part is purely the application of mathematics, and my biggest problem is I have problem calculating multiple integral. The only sample method used here is integral inverse method.
Every thind I modified in this part lies in warp.cpp, and can be visualize in warptest.

And then I implemented two method two deal with a single light source or a light box lighting. 
1. In the single light source case, the equation is:
$L(x) = \frac{\Phi}{4\pi^2} * \frac{max(0,cos \theta)}{||x-p||^2}*Visibility(x,p)$,
there's only one possibility so we don't sample, we just calculate the radiance.
2. In the light box case, the equation is:
$L(x) = \int _{H^2(x)}Visibility(x,x+\alpha \omega)*\frac{cos\theta}{\pi}d\omega$.
we sample the $\omega$ during intergration.

The problem I encountered is I failed to use $its.isFrame.n$ correctly and blindly copied it from normal.cpp, which caused normal direction wrong.
I modified ao.cpp and simple.cpp method to realize these two integrators.

## PA 4 Area Light source and Whitted Style Ray Tracing

In this PA I first implemented a method to generate an area light source. In computer graphics, when we need to compute the effect of an area light source, we usually sample a single point on the light source, and do the ordinary calculation in the case of single light source case.
In normal cases, the possibility we sample an emitter is $P(i)=\frac{S_i}{\Sigma S_k}$, and the possiblity we sample a mesh m on an emitter is $P(m|i)=\frac{S_{i,m}}{S_i}$, and the possibility we sample a point p on a mesh is obviously $P(p|m)=\frac{1}{S_{i,m}}$. Then we find that $P(p)=\frac{1}{\Sigma S_k}$, which means every point on an emitter has the same possibility of being sampled.
However, we now choose emitter with possibility $P(i)=\frac{1}{meshes}$ for stabability because the area size cannot represent the importance of it to this point, and $P(m|i)=\frac{S_{i,m}}{S_i}$, $P(i|m)=\frac{1}{S_{i,m}}$, so $P(p)=\frac{1}{meshes*S_i}$. In order not to introduce a coefficient in integration, we should divide the radiance with this possibility.
So we use the equation $L(x,\omega_r)=\int_{E}f_r(x,x \to y, \omega_r)G(x \to y)L_e(y,y \to x)dy$ with sampling to calculate direct shading from a light source.
In this part, I modified whitted.cpp and area.cpp to help sample and calculate.

In order to solve the problem of refraction and reflection, we should use real-raytracing to solve the effect of indirect radiance. We add a possibility $Pos$ to decide whether we should continue tracing the light, and the result should be divided by $Pos$, or the indirect radiance will be darker the direct shading. In this part, we sample a outgoing direction with the material BSDF::sample method.
Implementing Dielectric is not difficult, just give the output vector will be OK. Note that pdf is the possiblity of sampling this output vector, and it's always 0 for discrete BSDF(for the possibility is always 0 or 1, and the possibilty of get 0 is 1).
In this part, I modified whitted.cpp and dielectric.cpp.

## PA 5 Microfacet and Path Tracing

Microfacet is a mixure of mirror reflection and diffuse reflection. The sample method returns a vector by mirror reflection with random normal, or the simple diffuse result.
In the similar way, we can implement material rough dielectrics.
$f_r(\omega_i,\omega_o)=\frac{k_d}{\pi}+k_s\frac{D(\omega_h)F((\omega_h,\omega_i),\eta_e,\eta_i)G(\omega_i,\omega_o,\omega_h)}{4cos\theta_icos\theta_ocos\theta_h},\omega_h=\frac{\omega_i+\omega_o}{||\omega_i+\omega_o||}$

The estimation integrator relies all on BRDF sample and tracing. While it correctly calculates every possible ray in the scene, it may cause too much noise. If we combine BRDF light and direct light together, we can get a better result. We set the possibility of a light source (when diffused) to be $\frac{p_{BRDF}}{p_{BRDF}+p_{light}}$, the possibility of direct lighting to be $\frac{p_{light}}{p_{BRDF}+p_{light}}$. Note that $p_{BRDF}$ is the possibility of sampling this ray( $p_{BRDF}=pdf()$ ), and $p_{light}$ is the possibility of getting the point on the emitter ( $p_{light}=\frac{dis^2}{meshes*S_i*cos\theta}$ ).

I modified path_ems.cpp, path_mis.cpp and microfacet.cpp.

## Summary for RayTracing Algorithm

### Algorithm Design
#### Ray Tracing Basic Idea
Path Tracing's basic idea is to simulate the graph generating procedure in the physic world.
#### Formulas
Calculating Ray Tracing is actually calculating integral, so all the ray tracer should be regarded as integrator. Every physic model of BSDF or direct ray should be represented in the form of integral.

### Calculation Method
#### Sampling 
Continuous integral cannot be calculated by computers, but discrete sum can. We work on sampling in order to calculate integral. All the sample methods were for integral, and the possibility of every sample will actually act as coefficient in the integral.
#### Material BSDF
BSDF reflection and refraction can be used to calculate complex ray paths, but it is very slow. It uses the local color and the transmited light and the next radiance to calculate integral, and the possibility coefficient is generated in BSDF sampling.
$L(x,r_i)=\int_{p\in\partial E}bsdfeval(x)p(x,r_i,r_o)G(x\to p)L(p,r_o)$, $L(p,r_o)$ is calculated in recursion, $p(x,r_i,r_o)$ is partially generated in sampling, and another part (the possibility from pdf) of it is calculated in eval function. $G(x\to p)$ is not considered here in this case.
#### Direct lighting
Direct lighting from a single point follows Optics law, but the lighting from a whole object don't. So we use sample to get points on emitters to calculate Direct lighting integral $L(x,r_i)=\int_{p\in\partial E}bsdfeval(x)p(x,r_i,r_o)G(x\to p)L(p,r_o)$, in which $L(p,r_o)=\Phi*max(0, -r_o \cdot n_p)$ is the radiance from p, $G(x\to p)=\frac{Visibility(x\to p)}{||x-p||^2}$ represents the change of light on the way, $p(x,r_i,r_o)=(r_i \cdot r_o)$ is the diffuse possibility at x.


### Optimize and Improvement
#### Time
The biggest time consumer is finding intersection in the scene, and Octree/KDtree is now used to accelerate this procedure. Maybe ScapeGoat KDtree can be used in dynamic case. Visit by the distance of each son is also an efficient trick.
#### Noise
Noise comes from the difference of the sample method. The bigger the difference is, the more noisy is it. In order to reduce noise and correctly show the light path at the same time, we mix BSDF sampling and Direct lighting to get better performance.