#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */


void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    Eigen::VectorXf goal = quadrotor_ptr->GetState() - quadrotor_ptr->GetControlState();
    float q_x, q_y, q_theta, x_dot, y_dot;
    
    int body_l=50;
    int body_h=10;
    int rotor_x=15;
    int rotor_y=6;
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    x_dot=state[4];
    y_dot=state[5];
     Sint16 body_x[4] = { q_x+body_l*cos(q_theta)+body_h*sin(q_theta),
        q_x-body_l*cos(q_theta)+body_h*sin(q_theta),
        q_x-body_l*cos(q_theta)-body_h*sin(q_theta),
        q_x+body_l*cos(q_theta)-body_h*sin(q_theta)
        };
    Sint16 body_y[4] = { q_y+body_h*cos(q_theta)-body_l*sin(q_theta),
        q_y+body_h*cos(q_theta)+body_l*sin(q_theta),
        q_y-body_h*cos(q_theta)+body_l*sin(q_theta),
        q_y-body_h*cos(q_theta)-body_l*sin(q_theta)
    };
    Sint16 plx[4]= { body_x[2]+rotor_x*cos(q_theta)+rotor_y*sin(q_theta), 
    body_x[2]-rotor_x*cos(q_theta)+rotor_y*sin(q_theta),
    body_x[2]-rotor_x*cos(q_theta)-rotor_y*sin(q_theta),
    body_x[2]+rotor_x*cos(q_theta)-rotor_y*sin(q_theta)
    }; 
    Sint16 ply[4]= { body_y[2]+rotor_y*cos(q_theta)-rotor_x*sin(q_theta),
    body_y[2]+rotor_y*cos(q_theta)+rotor_x*sin(q_theta),
    body_y[2]-rotor_y*cos(q_theta)+rotor_x*sin(q_theta),
    body_y[2]-rotor_y*cos(q_theta)-rotor_x*sin(q_theta)
    }; 
    Sint16 prx[4]= { body_x[3]+rotor_x*cos(q_theta)+rotor_y*sin(q_theta), 
    body_x[3]-rotor_x*cos(q_theta)+rotor_y*sin(q_theta),
    body_x[3]-rotor_x*cos(q_theta)-rotor_y*sin(q_theta),
    body_x[3]+rotor_x*cos(q_theta)-rotor_y*sin(q_theta)
    }; 
    Sint16 pry[4]= { body_y[3]+rotor_y*cos(q_theta)-rotor_x*sin(q_theta),
    body_y[3]+rotor_y*cos(q_theta)+rotor_x*sin(q_theta),
    body_y[3]-rotor_y*cos(q_theta)+rotor_x*sin(q_theta),
    body_y[3]-rotor_y*cos(q_theta)-rotor_x*sin(q_theta)
    }; 
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF); 
    filledPolygonColor(gRenderer.get(), body_x,  body_y, 4, 0xFF555555);
    filledPolygonColor(gRenderer.get(), plx,  ply , 4, 0xFFFF0000);
    filledPolygonColor(gRenderer.get(), prx,  pry , 4, 0xFFFF0000);
    filledCircleColor(gRenderer.get(), q_x, q_y, 2, 0xFF0000FF);
}