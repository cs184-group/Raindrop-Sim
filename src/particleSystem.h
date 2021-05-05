#ifndef CLOTHSIM_PARTICLE_SYS_H
#define CLOTHSIM_PARTICLE_SYS_H

#include <vector>
#include "CGL/CGL.h"
#include "raindrop.h"
#include "collision/collisionObject.h"
#include "collision/plane.h"

class ParticleSystem {
public:
    ParticleSystem(unsigned int width, unsigned int height, int count, double r) {
        this->width = width;
        this->height = height;
        this->wind_f = Vector3D(0, 0, 0);
        this->velocity = TERMINAL_V;
        this->count = count;
        this->r = r;
        this->collisionMapRes = width * height;
        wetMap = (char*) calloc((width * height + 3) / 4 * 4, sizeof(char));
        collisionMap = (unsigned char*)calloc((collisionMapRes + 3) / 4 * 4, sizeof(unsigned char));
    }

    ~ParticleSystem() {
        free(wetMap);
        free(collisionMap);
    }

    const Vector3D TERMINAL_V = Vector3D(0, -7, 0);
    const double SKY_MIDPOINT = 10.0;

    void init_raindrops();
    void reset();

    char* wetMap;
    unsigned char* collisionMap;
    unsigned int width;
    unsigned int height;
    int collisionMapRes;
    int count;
    Vector3D velocity;
    Vector3D wind_f;
    double sky_midpoint;
    double r;

    void updateWind(Vector3D wind_f);
    inline Vector3D calculateHit(double x, double y, double z) const;

    void simulate(double frames_per_sec, double simulation_steps,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects);

    vector<Raindrop> raindrops;

    vector<Raindrop*> drops;

    // Spatial hashing
    unordered_map<float, vector<Raindrop *>*> map;
};


#endif //CLOTHSIM_PARTICLE_SYS_H
