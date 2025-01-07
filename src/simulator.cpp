#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "shader.hpp"
#include "meshes.hpp"

extern "C" {
 #include "../include/definitions.h"
 #include "../include/Geometry.h"
 #include "../include/cm_multithreaded.h"
 #include <windows.h>
 #include "SDL2/SDL.h"
 #include "GL/glew.h"
 #include "GL/gl.h"
}

#endif // SIMULATOR_HPP

/**** Static Function prototypes ****/
static float
ParseArgv(char *argv[], const int index);

static void 
glSetAttributes();

/**** Start of main function (WinMain because sdl2 is finicky) ****/
/** Usage: ./particlesim [particle_count] [cube_size] [particle_radius] **/
int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
  // Pull command line arguments from WinMain prototype
  int argc = __argc;
  char **argv = __argv;
  // Misc.
  int particle_count, axis_count = 8, partition_count = pow(axis_count, 3), sub_steps = 12;
  float radius = 0.0, cube_size = 0.0, dt = 1e-3, yaw = 0.0, pitch = 0.0, fov = 60.0;
  float near_plane = 0.1, far_plane = 100.0, aspect_ratio = 1280.0 / 720.0;
  float last_mouse_x = 0.0, last_mouse_y = 0.0, x_offset = 0.0, y_offset = 0.0, sensititivity = 5e-5;
  std::vector<float> min(3, 0), max(3, 0);
  Object *objects;
  Cube cube;
  Cube_GL cube_gl;
  std::vector<GLfloat> cube_vertices;
  std::vector<GLuint> cube_indices;
  glm::mat4 model(1.0), view(1.0), projection(1.0), sphere_model(1.0), cube_model(1.0);
  glm::vec3 light_position(1.0), view_position(1.0), light_color(1.0);
  glm::vec3 position(1.0), target(1.0), up(1.0), direction(1.0), center(1.0), rotated_up(1.0);
  bool left_mouse_down = false, initial_mouse = true;

  /*** Instantiation of joint SDL and openGL for 3D drawing ***/
  // SDL2 window creation
  SDL_Window *window = nullptr;
  SDL_Event event;
  SDL_GLContext context;

  // Instantiate SDL2
  if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
    std::cerr << "SDL Instantiation Error: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create the window and renderer
  window = SDL_CreateWindow("Particle Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_OPENGL);
  if (window == nullptr) {
    std::cerr << "SDL window creation failure: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Set glew context to SDL context
  context = SDL_GL_CreateContext(window);
  if (context == nullptr) {
    std::cerr << "OpenGL context failure: " << SDL_GetError() << std::endl;
    return 1;
  }

  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return 1;
  }

  // Sets openGL calls for window
  glSetAttributes();
  // Shaders require set context;
  /*** End of SDL+GL instantiation ***/

  // Check number of passed arguments
  if (argc > 4 || argc == 1) {
    std::cerr << "Invalid Input count" << std::endl; 
    return 1;
  }

  // Parse input arguments and instantiate data structures
  particle_count = (int)ParseArgv(argv, 1);
  cube_size = ParseArgv(argv, 2);
  radius = ParseArgv(argv, 3);
  objects = InitializeObjects(particle_count, radius);
  cube = create_cube((Vector3){0.0, 0.0, 0.0}, (Vector3){cube_size, cube_size, cube_size}, cube_size);
  center = glm::vec3(cube.size / 2.0, cube.size / 2.0, cube.size / 2.0);
  cube_gl = Cube_GL((float)cube.size);
  Sphere_GL sphere_gl(50, 50, radius);

  // Set matrices and vectors for lighting and camera
  up = glm::vec3(0.0, 1.0, 0.0);
  rotated_up = up;
  light_position = rotated_up + center + glm::vec3(0.0, cube.size / 2.0, 0.0);
  view_position = glm::vec3(5.0, 5.0, 30.0);
  light_color = glm::vec3(1.0, 1.0, 1.0);
  target = glm::vec3(5.0, 5.0, 5.0);
  
  // Create shader
  Shader shader("shaders/vertex_shader.glsl", "shaders/fragment_shader.glsl", view_position, 
                target, up, light_position, light_color);

  model = glm::mat4(1.0);

  // Update loop
  while (1) {
    // Clear Screen to Black 
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Polls for events in SDL instance
    while (SDL_PollEvent(&event)) {
      // Quit program out
      if (event.type == SDL_QUIT) {
        exit(0);
      }
      // Checks for specific keypresses
      switch (event.type) {
        case SDL_KEYDOWN:
          // Handle Key events
          switch (event.key.keysym.sym) {
            // Escape exit
            case SDLK_ESCAPE:
              exit(0);
              break;
            // FOV sliders c and v keys
            case SDLK_c:
              fov += 1;
              break;
            case SDLK_v:
              fov -= 1;
              break;
            default:
              break;
          }
        case SDL_MOUSEBUTTONDOWN:
          if (event.button.button == SDL_BUTTON_LEFT) {
            left_mouse_down = true;
          }
          break;
        case SDL_MOUSEBUTTONUP:
          if (event.button.button == SDL_BUTTON_LEFT) {
            left_mouse_down = false;
            initial_mouse = true;
          }
          break;
        // Handle Moving Mouse
        case SDL_MOUSEMOTION:
          // Only handle if left mouse down is true
          if (left_mouse_down) {
            // Set initial mouse position
            if (initial_mouse) {
              last_mouse_x = event.motion.x;
              last_mouse_y = event.motion.y;
              initial_mouse = false;
            }

            // Calculate Mouse offsets
            x_offset = (event.motion.x - last_mouse_x) * sensititivity;
            y_offset = (event.motion.y - last_mouse_y) * sensititivity;

            // Find yaw and pitch 
            yaw -= x_offset;
            pitch += y_offset;
            pitch = (pitch > 89.0) ? 89.0 : pitch;
            pitch = (pitch < -89.0) ? -89.0 : pitch;

            // Set View Matrix
            cube_model = glm::translate(model, center);
            cube_model = glm::rotate(cube_model, yaw, glm::vec3(0.0, 1.0, 0.0));
            cube_model = glm::rotate(cube_model, pitch, glm::vec3(1.0, 0.0, 0.0));
            cube_model = glm::translate(cube_model, -center);
          }
          break;
        // Default case
        default:
          break;
      }
    }
    
    // Find the rotated light position
    rotated_up = glm::vec3(cube_model * glm::vec4(up, 0.0));
    shader.camera_.lighting_.SetPosition(rotated_up + center + glm::vec3(0.0, cube.size / 2.0, 0.0)); 

    shader.Render(fov, aspect_ratio, near_plane, far_plane, cube_gl.object_color_, cube_model);
    cube_gl.DrawCube();
    
    for (int i = 0; i < particle_count; ++i) {
      position = glm::vec3(objects[i].position.x, objects[i].position.y, objects[i].position.z);
      sphere_model = glm::translate(cube_model, position);

      shader.camera_.lighting_.SetPosition(view_position + glm::vec3(0.0, 1.0, 0.0));
      shader.Render(fov, aspect_ratio, near_plane, far_plane, sphere_gl.object_color_, sphere_model);
      sphere_gl.DrawSphere();
    }

    // Collision detection and position update
    for (int i = 0; i < sub_steps; ++i) {
      CollisionCall(cube, objects, partition_count, particle_count, axis_count);
    }
    // Update the positions of objects by one time-step
    ApplyForce(objects, dt, particle_count, Gravity);

    SDL_GL_SwapWindow(window);
    // std::cout << "Iteration Complete" << '\n';
  }

  // Free calls 
  free(objects);

  // Exit SDL
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}

/**** Simple parsing function to return a float value from an inline string input ****/
static float ParseArgv(char *argv[], const int index) {
  return std::stod(argv[index]);
}

/**** Sets the version context for openGl as well as buffers and depth ****/
static void glSetAttributes() {
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  glEnable(GL_DEPTH_TEST);
}