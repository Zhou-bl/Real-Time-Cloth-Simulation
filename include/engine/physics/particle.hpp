#pragma once
#include <SFML/System/Vector2.hpp>
#include "../common/index_vector.hpp"


struct Particle
{
    civ::ID      id            = 0;
    float        mass          = 1.0f;
    bool         moving        = true;
    int cloth_ID = -1;
    sf::Vector2f position;
    sf::Vector2f position_old;
    sf::Vector2f velocity;
    sf::Vector2f forces;

    Particle() = default;

    explicit
    Particle(sf::Vector2f pos)
        : position(pos)
        , position_old(pos)
    {}

    void update(float dt)
    {
        if (!moving) return;
        position_old = position;
        velocity += (forces / mass) * dt;
        position += velocity * dt;
    }

    void updateDerivatives(float dt)
    {
        velocity = (position - position_old) / dt;
        forces = {};
    }

    void updateExternalCollision(){
        if (position.x > 1760) {
            position.x = 1760;
            velocity.x = -0.5 * velocity.x;
        }
    }

    void move(sf::Vector2f v)
    {
        if (!moving) return;
        position += v;
    }
};

using ParticleRef = civ::Ref<Particle>;
