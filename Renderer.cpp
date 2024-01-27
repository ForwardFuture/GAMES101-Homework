//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <omp.h>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.001;
omp_lock_t lock1;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 65536;
    int sqspp = (int)sqrt((double)spp);
    float lps = 1.0 / (float)(sqspp + 1);

    //std::cout << sqspp << " " << lps << std::endl;

    float process = 0;
    float reciprocal_scene_height = 1.0 / (float)scene.height;

    std::cout << "SPP: " << spp << "\n";

    #pragma omp parallel for
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            for (int k = 0; k < spp; k++){

                float x = (2 * (i + (k % sqspp + 1) * lps) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1 - 2 * (j + (k / sqspp + 1) * lps) / (float)scene.height) * scale;
                
                //std::cout << x << " " << y << std::endl;
                
                Vector3f dir = normalize(Vector3f(-x, y, 1));

                //rotate direction in xz plane

                //double theta = 20.0 / 180.0 * M_PI;
                //dir = Vector3f(dir.x * cos(theta) - dir.z * sin(theta), dir.y, dir.z * cos(theta) + dir.x * sin(theta));

                //rotate direction in xy plane

                //float height = 1.0f;
                //dir = normalize(Vector3f(dir.x, dir.y + height, dir.z));


                framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
        }
        omp_set_lock(&lock1);
        process += reciprocal_scene_height;
        UpdateProgress(process);
        omp_unset_lock(&lock1);
    }
    
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
