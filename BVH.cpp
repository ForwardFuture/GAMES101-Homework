#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    //root = recursiveBuild(primitives);
    root = SAHBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

BVHBuildNode* BVHAccel::SAHBuild(std::vector<Object*>objects)
{
    // Compute bounds of all primitives in BVH node
    if(objects.size() == 0) return nullptr;

    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    BVHBuildNode* node = new BVHBuildNode();

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = SAHBuild(std::vector{objects[0]});
        node->right = SAHBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        auto beginning = objects.begin();
        auto ending = objects.end();

        float minn, maxn;

        switch (dim)
        {
            case 0:
                minn = objects[0]->getBounds().Centroid().x;
                maxn = objects[objects.size() - 1]->getBounds().Centroid().x;
                break;
            case 1:
                minn = objects[0]->getBounds().Centroid().y;
                maxn = objects[objects.size() - 1]->getBounds().Centroid().y;
                break;
            case 2:
                minn = objects[0]->getBounds().Centroid().z;
                maxn = objects[objects.size() - 1]->getBounds().Centroid().z;
                break;
        }

        const int slice = 1000;
        float seg = (maxn - minn) / (float)slice;
        std::vector<Object*>bucket[slice];
        Bounds3 bucketbound[slice];
        int now = 0;
        
        /*
        std::cout << minn << ' ' << maxn << std::endl;
        std::cout << dim << std::endl;
        for(int i = 0; i < objects.size(); i++)
            std::cout << objects[i]->getBounds().Centroid() << std::endl;
        std::cout << std::endl;
        */

        for(int i = 0; i < objects.size(); i++)
        {
            float index = 0;
            switch (dim)
            {
                case 0:
                    index = objects[i]->getBounds().Centroid().x;
                    break;
                case 1:
                    index = objects[i]->getBounds().Centroid().y;
                    break;
                case 2:
                    index = objects[i]->getBounds().Centroid().z;
                    break;
            }
            while(index - EPSILON > (float)(now + 1) * seg + minn) now++;
            bucket[now].emplace_back(objects[i]);
        }

        for(int i = 0; i < slice; i++)
            for(int j = 0; j < bucket[i].size(); j++)
                bucketbound[i] = Union(bucketbound[i], bucket[i][j]->getBounds());

        Bounds3 prebound[slice], sufbound[slice];

        prebound[0] = bucketbound[0];
        for(int i = 1; i < slice; i++)
            prebound[i] = Union(prebound[i - 1], bucketbound[i]);
        sufbound[slice - 1] = bucketbound[slice - 1];
        for(int i = slice - 2; i >= 0; i--)
            sufbound[i] = Union(sufbound[i + 1], bucketbound[i]);

        const float Crai = 0;
        float minCost = 1e12;
        float preCost = 0;
        int middling = 0;
        int totobjects = 0;

        for(int i = 1; i < slice; i++)
        {
            totobjects += bucket[i - 1].size();
            preCost = Crai;

            if(totobjects)
                preCost += totobjects * (prebound[i - 1].SurfaceArea() / bounds.SurfaceArea());
            if(objects.size() - totobjects)
                preCost += (objects.size() - totobjects) * (sufbound[i].SurfaceArea() / bounds.SurfaceArea());

            if(preCost < minCost)
            {
                minCost = preCost;
                middling = totobjects;
            }
        }

        auto leftshapes = std::vector<Object*>(beginning, beginning + middling);
        auto rightshapes = std::vector<Object*>(beginning + middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        if(leftshapes.size())node->left = SAHBuild(leftshapes);
        if(rightshapes.size())node->right = SAHBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if(node->object) return node->object->getIntersection(ray);

    Intersection leftP, rightP;

    Vector3f MyVec = Vector3f(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
    std::array<int, 3> Myarray = {(int)(ray.direction.x > 0), (int)(ray.direction.y > 0), (int)(ray.direction.z > 0)};

    if(node->left->bounds.IntersectP(ray, MyVec, Myarray))
        leftP = BVHAccel::getIntersection(node->left, ray);

    if(node->right->bounds.IntersectP(ray, MyVec, Myarray))
        rightP = BVHAccel::getIntersection(node->right, ray);
    
    //pay attention to the occlusion
    if(leftP.happened && rightP.happened)
    {
        if(leftP.distance < rightP.distance) return leftP;
        else return rightP;
    }
    else if(leftP.happened) return leftP;
    else return rightP;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}