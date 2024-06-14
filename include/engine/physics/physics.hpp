#pragma once
#include <functional>
#include <SFML/System/Vector2.hpp>
#include "engine/common/index_vector.hpp"
#include "engine/common/utils.hpp"
#include "constraints.hpp"


struct PhysicSolver
{
    CIVector<Particle>       objects;
    CIVector<LinkConstraint> constraints;
    // Simulator iterations count
    uint32_t solver_iterations;
    uint32_t sub_steps;

    PhysicSolver()
        : solver_iterations(1)
        , sub_steps(16)
    {}

    void update(float dt)
    {
        const float sub_step_dt = dt / to<float>(sub_steps);
        removeBrokenLinks();
        for (uint32_t i(sub_steps); i--;) {
            applyGravity();
            applyAirFriction();
            updateClothCollisions(sub_step_dt);
            updatePositions(sub_step_dt);
            solveConstraints();
            updateDerivatives(sub_step_dt);
            updateExternalCollisions();
        }
    }

    void applyGravity()
    {
        const sf::Vector2f gravity(0.0f, 1500.0f);
        for (Particle& p : objects) {
            p.forces += gravity * p.mass;
        }
    }

    void applyAirFriction()
    {
        const float friction_coef = 0.5f;
        for (Particle& p : objects) {
            p.forces -= p.velocity * friction_coef;
        }
    }

    void updatePositions(float dt)
    {
        for (Particle& p : objects) {
            p.update(dt);
        }
    }

    void updateDerivatives(float dt)
    {
        for (Particle& p : objects) {
            p.updateDerivatives(dt);
        }
    }

    void updateExternalCollisions()
    {
        for (Particle& p : objects) {
            p.updateExternalCollision();
        }
    }

    void updateClothCollisions(float dt)
    {   
        float k = 10.0f;
        dt = dt;
        std::vector<Particle> p1, p2;
        for (Particle& p : objects) {
            if (p.cloth_ID == 1) {
                p1.push_back(p);
            } else if (p.cloth_ID == 2) {
                p2.push_back(p);
            }
        }
        // for(auto iter=p1.begin(); iter!=p1.end(); ++iter){
        //     printf("p1: %f %f\n", iter->position.x, iter->position.y);
        //     printf("ID: %d\n", iter->cloth_ID);
        // }
        // for(auto iter=p2.begin(); iter!=p2.end(); ++iter){
        //     printf("p1: %f %f\n", iter->position.x, iter->position.y);
        //     printf("ID: %d\n", iter->cloth_ID);
        // }
        for(auto iter=p1.begin(); iter!=p1.end(); ++iter){
            int num = 0;
            for(auto iter2=p2.begin(); iter2!=p2.end(); ++iter2){
                // float distance = sqrt(pow(iter->position.x - iter2->position.x, 2) + pow(iter->position.y - iter2->position.y, 2));
                if (fabs(iter->position.y - iter2->position.y) < 20.0f && iter->position.x > iter2->position.x){
                    num++;
                }
            }
            //printf("num: %d\n", num);
            if (num > 0){
                // printf("num: %d\n", num);
                // printf("old x v: %f\n", iter->velocity.x);
                iter->velocity.x -= k * num;
                // printf("new x v: %f\n", iter->velocity.x);
            }
        }

        for(auto iter=p2.begin(); iter!=p2.end(); ++iter){
            int num = 0;
            for(auto iter2=p1.begin(); iter2!=p1.end(); ++iter2){
                // float distance = sqrt(pow(iter->position.x - iter2->position.x, 2) + pow(iter->position.y - iter2->position.y, 2));
                if (fabs(iter->position.y - iter2->position.y) < 20.0f && iter->position.x < iter2->position.x){
                    num++;
                }
            }
            //printf("num: %d\n", num);
            if (num > 0){
                // printf("num: %d\n", num);
                // printf("old x v: %f\n", iter->velocity.x);
                iter->velocity.x += k * num;
                // printf("new x v: %f\n", iter->velocity.x);
            }
        }
        //update the velocity in the object
        for (auto iter=p1.begin(); iter!=p1.end(); ++iter){
            for (Particle& p : objects) {
                if (p.id == iter->id){
                    p.velocity = iter->velocity;
                }
            }
        }
        for (auto iter=p2.begin(); iter!=p2.end(); ++iter){
            for (Particle& p : objects) {
                if (p.id == iter->id){
                    p.velocity = iter->velocity;
                }
            }
        }
    }

    void solveConstraints()
    {
        for (uint32_t i(solver_iterations); i--;) {
            for (LinkConstraint &l: constraints) {
                l.solve();
            }
        }
    }

    void removeBrokenLinks()
    {
        constraints.remove_if([](const LinkConstraint& c) {return !c.isValid();});
    }

    civ::ID addParticle(sf::Vector2f position, int cloth_ID = -1)
    {
        const civ::ID particle_id = objects.emplace_back(position);
        objects[particle_id].id = particle_id;
        objects[particle_id].cloth_ID = cloth_ID;
        return particle_id;
    }

    void addLink(civ::ID particle_1, civ::ID particle_2, float max_elongation_ratio = 1.5f, float strength = 1.0f)
    {
        const civ::ID link_id = constraints.emplace_back(objects.getRef(particle_1), objects.getRef(particle_2));
        constraints[link_id].id = link_id;
        constraints[link_id].max_elongation_ratio = max_elongation_ratio;
        constraints[link_id].strength = strength;
    }

    void map(const std::function<void(Particle&)>& callback)
    {
        for (Particle& p : objects) {
            callback(p);
        }
    }
};
