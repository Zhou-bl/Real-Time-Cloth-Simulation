#include "engine/window_context_handler.hpp"
#include "engine/physics/physics.hpp"
#include "renderer.hpp"
#include "wind.hpp"
#include "utils.hpp"
#include <vector>

const bool TRIANGULAR=0;

std::vector<Particle> add_cloth(uint32_t cloth_width, uint32_t cloth_height, float links_length, float start_x, float start_y, PhysicSolver& solver, int cloth_ID) {
    std::vector<Particle> cloth_particles;
    cloth_particles.clear();

    for (uint32_t y(0); y < cloth_height; ++y) {
        // This is just an arbitrary formula to make the top links stronger since
        // they are under bigger stress
        const float max_elongation = 1.2f * (2.0f - to<float>(y) / float(cloth_height));
        for (uint32_t x(0); x < cloth_width; ++x) {
            const civ::ID id = solver.addParticle(
                sf::Vector2f(start_x + to<float>(x) * links_length, start_y + to<float>(y) * links_length), cloth_ID
            );
            cloth_particles.push_back(solver.objects[id]);

            // Add left link if there is a particle on the left
            if (x > 0) {
                solver.addLink(id-1, id, max_elongation * 0.9f);
            }
            // Add top link if there is a particle on the top
            if (y > 0) {
                solver.addLink(id-cloth_width, id, max_elongation);
            } else {
                // If not, pin the particle
                solver.objects[id].moving = false;
            }

            // 扭曲性弹簧
            // Add top-left link if there is a particle on the top-left
            if (x > 0 && y > 0) {
                solver.addLink(id-cloth_width-1, id, max_elongation, 0.5f);
            }
            // Add top-right link if there is a particle on the top-right
            if (x < cloth_width - 1 && y > 0) {
                solver.addLink(id-cloth_width+1, id, max_elongation, 0.5f);
            }

            // 拉伸性弹簧
            // Add top-top link if there is a particle on the top-top
            if (y > 1) {
                solver.addLink(id-2*cloth_width, id, max_elongation, 0.1f);
            }
            // Add left-left link if there is a particle on the left-left
            if (x > 1) {
                solver.addLink(id-2, id, max_elongation, 0.1f);
            }
        }
    }

    return cloth_particles;
}

int main()
{
    const uint32_t window_width  = 1920;
    const uint32_t window_height = 1080;
    WindowContextHandler app("Cloth", sf::Vector2u(window_width, window_height), sf::Style::Default);
    // Initialize solver and renderer
    PhysicSolver solver;
    Renderer renderer(solver);

    if( TRIANGULAR){
        // 三角化
    const uint32_t cloth_width  = 75;
    const uint32_t cloth_height = 51;
    const float    links_length = 20.0f;
    const float    start_x      = (window_width - (cloth_width - 1) * links_length) * 0.5f;

    for (uint32_t y(0); y < cloth_height; ++y) {
        // This is just an arbitrary formula to make the top links stronger since
        // they are under bigger stress
        const float max_elongation = 1.2f * (2.0f - to<float>(y) / float(cloth_height));
        for (uint32_t x(0); x < ((y&1)?cloth_width-1:cloth_width); ++x) {
            const civ::ID id = solver.addParticle(
                sf::Vector2f(start_x + to<float>(x) * links_length + (y&1) * links_length / 2.0, to<float>(y) * links_length * sqrt(3.0) / 2.0)
            );
            // Add left link if there is a particle on the left
            if (x > 0) {
                solver.addLink(id-1, id, max_elongation * 0.9f);
            }
            // Add top link if there is a particle on the top
            if (y > 0) {
                if( !(( (y&1)==0) && x==0 ) )solver.addLink(id-cloth_width, id, max_elongation);
                if( !(( (y&1)==0) && x==cloth_width-1 ) )solver.addLink(id-(cloth_width-1), id, max_elongation);
            }
            // if(y>0 && (y&1)==0 ){
            //     solver.addLink(id-cloth_width-(cloth_width-1), id, max_elongation);
            // }
            if(y>1){
                solver.addLink(id-cloth_width-(cloth_width-1), id, max_elongation);
            }
            if(y==0){
                // If y==0, pin the particle
                solver.objects[id].moving = false;
            }
        }
    }
    } else {
        // 正方形化
        std::vector<Particle> cloth_particles_1 = add_cloth(25, 25, 20.0f, (window_width - (25 - 1) * 20.0f) * 0.5f, 0.0f, solver, 2),
                            cloth_particles_2 = add_cloth(25, 25, 20.0f, (window_width - (75 - 1) * 20.0f) * 0.5f, 0.0f, solver, 1);

        //Add two special particles at the right side of the cloth at x1,x2 = 1760; y1, y2 = 0, 1000
        const civ::ID id1 = solver.addParticle(sf::Vector2f(1860.0f, 0.0f));
        const civ::ID id2 = solver.addParticle(sf::Vector2f(1860.0f, 1000.0f));
        solver.addLink(id1, id2, 1.5f);
        solver.objects[id1].moving = false;
        solver.objects[id2].moving = false;

        sf::Vector2f last_mouse_position;
        bool dragging = false;
        bool erasing = false;
        // Add events callback for mouse control
        app.getEventManager().addMousePressedCallback(sf::Mouse::Right, [&](sfev::CstEv) {
            dragging = true;
            last_mouse_position = app.getWorldMousePosition();
        });
        app.getEventManager().addMouseReleasedCallback(sf::Mouse::Right, [&](sfev::CstEv) {
            dragging = false;
        });
        app.getEventManager().addMousePressedCallback(sf::Mouse::Middle, [&](sfev::CstEv) {
            erasing = true;
        });
        app.getEventManager().addMouseReleasedCallback(sf::Mouse::Middle, [&](sfev::CstEv) {
            erasing = false;
        });

        // Add 2 wind waves
        WindManager wind(to<float>(window_width));
        wind.winds.emplace_back(
            sf::Vector2f(100.0f, window_height),
            sf::Vector2f(0.0f, 0.0f),
            sf::Vector2f(1000.0f, 0.0f)
        );
        wind.winds.emplace_back(
            sf::Vector2f(20.0f, window_height),
            sf::Vector2f(0.0f, 0.0f),
            sf::Vector2f(3000.0f, 0.0f)
        );

        // Main loop
        const float dt = 1.0f / 40.0f;
        while (app.run()) {
            // Get the mouse coord in the world space, to allow proper control even with modified viewport
            const sf::Vector2f mouse_position = app.getWorldMousePosition();

            if (dragging) {
                // Apply a force on the particles in the direction of the mouse's movement
                const sf::Vector2f mouse_speed = mouse_position - last_mouse_position;
                last_mouse_position = mouse_position;
                usr::Utils::applyForceOnCloth(mouse_position, 100.0f, mouse_speed * 8000.0f, solver);
            }

            if (erasing) {
                // Delete all nodes that are in the range of the mouse
                solver.objects.remove_if([&](const Particle& p) {return usr::Utils::isInRadius(p, mouse_position, 10.0f);});
            }
            // Update physics
            wind.update(solver, dt);
            solver.update(dt);
            // Render the scene
            RenderContext& render_context = app.getRenderContext();
            render_context.clear();
            renderer.render(render_context);
            render_context.display();
        }

        return 0;
    }
}
