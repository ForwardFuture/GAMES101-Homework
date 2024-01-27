//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    Vector3f hitColor;
    Material* m = intersection.m;
    if(intersection.happened)
    {
        Intersection pos;
        float pdf = 0;
        Vector3f dirlight, indirlight;

        if(m->hasEmission()) return m->m_emission;

        //Direct light
        sampleLight(pos, pdf);
        Vector3f dir = pos.coords - intersection.coords;
        float len = dir.norm();
        dir = dir / len;

        Intersection Checkblock = Scene::intersect(Ray(intersection.coords + dir * EPSILON, dir));
                
        if(!Checkblock.happened || Checkblock.distance >= len - 2.0 * EPSILON)
        {
            dirlight = pos.emit * m->eval(dir, - ray.direction, intersection.normal) *
                        dotProduct(dir, intersection.normal) * dotProduct(- dir, pos.normal) /
                        (len * len) / pdf;
            /*
            std::cout << "dirlight = " << dirlight << std::endl;
            std::cout << "pos.emit = " << pos.emit << std::endl;
            std::cout << "pdf = " << pdf << std::endl;
            std::cout << std::endl;
            */
        }

        //Indirect light
        if(get_random_float() < RussianRoulette)
        {
            Vector3f wi = normalize(m->sample(ray.direction, intersection.normal));
            Intersection hitObject = Scene::intersect(Ray(intersection.coords, wi));
            if(hitObject.happened && !hitObject.m->hasEmission())
            {
                indirlight = castRay(Ray(intersection.coords, wi), depth + 1) * m->eval(wi, -ray.direction, intersection.normal) * 
                                dotProduct(wi, intersection.normal) / m->pdf(wi, -ray.direction, intersection.normal) / RussianRoulette;
            }
        }

        hitColor = dirlight + indirlight;
    }
    return hitColor;
}