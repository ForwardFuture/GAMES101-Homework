#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D dirVec = end - start;
        float len = dirVec.norm(), lenfrag = len / (num_nodes - 1);
        dirVec /= len;
        /*
        std::cout << "1" << std::endl;

        std::cout << "start = " << start << std::endl;
        std::cout << "end = " << end << std::endl;
        std::cout << "lenfrag = " << lenfrag << std::endl;
        */

        for(int i = 1; i <= num_nodes; i++) {

            //std::cout << "2" << std::endl;

            Vector2D pre_node = start + (i - 1) * lenfrag * dirVec;

            //std::cout << "pre_node = " << pre_node << std::endl;

            masses.emplace_back(new Mass(pre_node, node_mass, false));
        }

        for(int i = 0; i < pinned_nodes.size(); i++)
            masses[pinned_nodes[i]]->pinned = true;

        for(int i = 1; i < num_nodes; i++) {

            //std::cout << "3" << std::endl;

            springs.emplace_back(new Spring(masses[i - 1], masses[i], k));

            //std::cout << "4" << std::endl;
        }
        
        //std::cout << "5" << std::endl;

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {

        //std::cout << "6" << std::endl;

        for (auto &s : springs)
        {

            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D SpringVec1 = s->m2->position - s->m1->position;
            Vector2D SpringVec2 = s->m1->position - s->m2->position;
            s->m1->forces += s->k * SpringVec1.unit() * (SpringVec1.norm() - s->rest_length);
            s->m2->forces += s->k * SpringVec2.unit() * (SpringVec2.norm() - s->rest_length);
        }

        //std::cout << "7" << std::endl;

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                float damping_factor = 0.00005;


                //std::cout << "m->forces = " << m->forces << std::endl;
                //std::cout << "m->mass = " << m->mass << std::endl;

                Vector2D a = m->forces / m->mass + gravity, v = m->velocity, p = m->position;

                //Explicit Euler Method
                //m->velocity = m->velocity * (1 - damping_factor) + delta_t * a;
                //m->position += delta_t * v;

                //Semi-implicit Euler Method
                m->velocity = m->velocity * (1 - damping_factor) + delta_t * a;
                m->position += delta_t * m->velocity;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D SpringVec = ((s->m2->position - s->m1->position).norm() - s->rest_length) / 2 * (s->m2->position - s->m1->position).unit();

            s->m1->forces += SpringVec;
            s->m2->forces += -SpringVec;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                float damping_factor = 0.00005;

                Vector2D temp_position = m->position;

                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity;
                m->position = temp_position + (temp_position - m->last_position) * (1 - damping_factor) + m->forces * delta_t * delta_t;
                m->last_position = temp_position;
                
                // TODO (Part 4): Add global Verlet damping
                
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
