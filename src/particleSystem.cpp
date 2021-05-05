#include "particleSystem.h"

void ParticleSystem::init_raindrops() {
    drops = vector<Raindrop*>();
    sky_midpoint = 10.0;

    for (int i = 0; i < 500; i += 1) {
        double x = 4 + (double(rand()) / RAND_MAX - 0.5) * 5;
        double y = (double(rand()) / RAND_MAX - 0.5) * sky_midpoint;
        double z = 4 + (double(rand()) / RAND_MAX - 0.5) * 5;
        
        drops.push_back(new Raindrop(1.0, Vector3D(x, sky_midpoint + y, z), Vector3D(0, 0, 0)));
    }

    for (int i = 0; i < 3 * collisionMapRes; i += 1) {
        this->collisionMap[i] = 255;
    }
}

void ParticleSystem::reset() {

}

void ParticleSystem::updateWind(Vector3D wind_f) {
    this->wind_f = wind_f;
}

void ParticleSystem::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations,
                              vector<CollisionObject *> *collision_objects) {
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    Vector3D accs = Vector3D(0, 0, 0);
    for (Vector3D& a : external_accelerations) {
        accs += a;
    }

    for (int i = 0; i < drops.size(); i += 1) {
        for (int j = 0; j < simulation_steps; j += 1) {

            // Reset forces
            drops[i]->forces = (accs * drops[i]->mass) + this->wind_f; 

            // correct the velocity and position
            drops[i]->vel += (drops[i]->forces / drops[i]->mass) * delta_t;

            // Correct for terminal velocity, 9 meters per second
            double speed = sqrt(dot(drops[i]->vel, drops[i]->vel));
            if (speed > 9.0) {
                drops[i]->vel = 9.0 * drops[i]->vel / speed;
            }

            drops[i]->last_pos = drops[i]->pos;
            drops[i]->pos += 0.5 * drops[i]->vel * delta_t;

            for (CollisionObject *co : *collision_objects) {
                if (typeid(*co) == typeid(Plane)) {
                    Plane* p = (Plane*) co;
                    Vector3D pos = Vector3D(0, 0, 0);
                    if (p->collide(*drops[i], pos)) {

                        for (double x = -r; x < r; x += 8.0 / width) {
                            for (double z = -r; z < r; z += 8.0 / height) {
                                Vector3D cur_pos = pos + Vector3D(x, 0, z);

                                double distance = sqrt(x * x + z * z);

                                // check that droplet hits the plane
                                if (cur_pos.x >= 0 && cur_pos.z >= 0 && cur_pos.x < 8 && cur_pos.z < 8 && distance <= this->r) {
         
                                    double x_pos = width * cur_pos.x / 8.0;
                                    double z_pos = height * cur_pos.z / 8.0;

                                    int index = round(3 * (z_pos * height + x_pos));


                                    if (collisionMap[index] <= 10) {
                                        collisionMap[index] = 0;
                                        collisionMap[index + 1] = 0;
                                        collisionMap[index + 2] = 0;
                                    } else {
                                        int d = 10 * round((this->r - distance) / this->r);  
                                        collisionMap[index] -= d;
                                        collisionMap[index + 1] -= d;
                                        collisionMap[index + 2] -= d;
                                    }
                                }

                            }
                        }

                        // If the droplet collided with the groud, replace it with a new droplet in the sky
                        double x = 4 + (double(rand()) / RAND_MAX - 0.5) * 5;
                        double y = (double(rand()) / RAND_MAX - 0.5) * sky_midpoint;
                        double z = 4 + (double(rand()) / RAND_MAX - 0.5) * 5;
                        
                        drops[i]->pos = Vector3D(x, sky_midpoint + y, z);
                        drops[i]->vel = Vector3D(0, 0, 0);

                        
                    }             
                }
                
            }
        }

        // Update the sphere
        drops[i]->s->origin = drops[i]->pos;
    }
}
