#pragma once

#include <iostream>
#include <utils/physics.h>
#include <utils/camera.h>
#include <utils/random.h>
#include <utils/shader.h>
#include <utils/texture_loader.h>
#include <utils/particle_system.h>

// macros
#define NUM_LIGHTS 8

// Dimensioni della finestra dell'applicazione
GLuint screenWidth = 1200, screenHeight = 900;

// parameters for time calculation (for animations)
GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

// view and projection matrices (global because we need to use them in the keyboard callback)
glm::mat4 view, projection;

// weight for the diffusive component
GLfloat Kd = 3.0f;
// roughness index for GGX shader
GLfloat alpha;
// Fresnel reflectance at 0 degree (Schlik's approximation)
GLfloat F0 = 0.9f;

// index of the current shader subroutine (= 0 in the beginning)
GLuint current_subroutine = 0;
GLuint new_subroutine = current_subroutine;

// a vector for all the shader subroutines names used and swapped in the application
vector<std::string> shaders = { "GGX", "Reflection", "GGX", "GGX", "GGX" };

// we initialize an array of booleans for each keyboard key
bool keys[1024];

// we need to store the previous mouse position to calculate the offset with the current frame
GLfloat lastX, lastY;

// we will use these value to "pass" the cursor position to the keyboard callback, in order to determine the bullet trajectory
double cursorX,cursorY;

// when rendering the first frame, we do not have a "previous state" for the mouse, so we need to manage this situation
bool firstMouse = true;

// flag to detect if beys have already done boost
bool hasBoosted1 = false;
bool hasBoosted2 = false;

// flag to activate fog
bool fogFlag = false;
// flag to activate multicolor lights
bool multiFlag = false;

// instance of the physics class
Physics bulletSimulation;

// boolean to activate/deactivate wireframe rendering
GLboolean wireframe = GL_FALSE;

// we create a camera. We pass the initial position as a parameter to the constructor. In this case, we use a "floating" camera (we pass false as last parameter)
Camera camera(glm::vec3(0.0f, 10.0f, 10.0f), GL_FALSE);

// Uniforms to be passed to shaders
// point light position and color
glm::vec3 lightPos[NUM_LIGHTS] = {
    glm::vec3(8.0f, 25.0f, 10.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(5.0f,3.0f,0.0f),
    glm::vec3(0.0f,3.0f,5.0f),
    glm::vec3(-5.0f,3.0f,0.0f),
    glm::vec3(0.0f,3.0f,-5.0f),
};
glm::vec3 lightColor[NUM_LIGHTS] = {
    glm::vec3(1.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f),
    glm::vec3(0.0f)
};
glm::vec3 currentColor = glm::vec3(0.0f,0.33f,0.66f);
GLfloat currentR;
GLfloat currentG;
GLfloat currentB;

// objects color
GLfloat bey1Color[] = {0.8f,0.8f,0.8f};
GLfloat bey2Color[] = {1.0f,0.5f,0.7f};
GLfloat arenaColor[] = {0.02f,0.08f,0.3f};

GLfloat fogColor[] = {0.5f,0.5f,0.5f};
// color of the plane
GLfloat planeMaterial[] = {0.21f,1.0f,0.24f};

// keep trace of initial Beyblade positions and forces
btTransform bey1LaunchPose;
btTransform bey2LaunchPose;
btVector3 torque1;
btVector3 torque2;
btVector3 launchDirection;

// We manage the particle systems
ParticleProps m_Particle;
ParticleSystem m_ParticleSystem;
ParticleProps boostProps1;
ParticleSystem boostSystem1;
ParticleProps boostProps2;
ParticleSystem boostSystem2;

// texture unit for the cube map
GLuint textureCube;
// tecture unit for the ground
GLuint textureGround;
