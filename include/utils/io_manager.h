#pragma once

#include <glad/glad.h>

//////////////////////////////////////////
// callback for keyboard events
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    // if ESC is pressed, we close the application
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    // if L is pressed, we activate/deactivate wireframe rendering of models
    if(key == GLFW_KEY_L && action == GLFW_PRESS)
        wireframe=!wireframe;

     if(key == GLFW_KEY_ENTER && action == GLFW_PRESS)
    {
        // RESET THE GAME: CHANGE LAUNCH POSITION, RESET ANGULAR VELOCITY, RESET FLAGS
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->setLinearVelocity(btVector3(0.0f,0.0f,0.0f));
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->setLinearVelocity(btVector3(0.0f,0.0f,0.0f));
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->setAngularVelocity(btVector3(0.0f,0.0f,0.0f));
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->setAngularVelocity(btVector3(0.0f,0.0f,0.0f));
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->applyTorqueImpulse(torque1);
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->applyTorqueImpulse(torque2);
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->applyCentralImpulse(launchDirection);
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->applyCentralImpulse(-launchDirection);
        btTransform newLaunch1 = bey1LaunchPose;
        btTransform newLaunch2 = bey2LaunchPose;
        newLaunch1.setOrigin(btVector3(
            newLaunch1.getOrigin().getX()+rand_float(),
            newLaunch1.getOrigin().getY(),
            newLaunch1.getOrigin().getZ()+rand_float()
        ));
        newLaunch2.setOrigin(btVector3(
            newLaunch2.getOrigin().getX()+rand_float(),
            newLaunch2.getOrigin().getY(),
            newLaunch2.getOrigin().getZ()+rand_float()
        ));
        bulletSimulation.dynamicsWorld->getCollisionObjectArray()[2]->setWorldTransform(newLaunch1);
        bulletSimulation.dynamicsWorld->getCollisionObjectArray()[3]->setWorldTransform(newLaunch2);
        lightColor[1] = glm::vec3(0.0f);
        lightColor[2] = glm::vec3(0.0f);
        hasBoosted1 = false;
        hasBoosted2 = false;
    }

    if((key == GLFW_KEY_1) && action == GLFW_PRESS)
    {
        if (!hasBoosted1)
        {
            btRigidBody* bey = bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0];
            if (bey->getAngularVelocity().getY() > 50)
            {
                // BOOST (angular velocity augment + light emission)
                bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->applyTorqueImpulse(btVector3(0.0f,100.0f,0.0f));
                lightColor[1] = glm::vec3(0.1f,1.0f,0.3f);
            }
            hasBoosted1 = true;
        }
    }

    if((key == GLFW_KEY_2) && action == GLFW_PRESS)
    {
        if (!hasBoosted2)
        {
            btRigidBody* bey = bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1];
            if (bey->getAngularVelocity().getY() > 50)
            {
                // BOOST (angular velocity augment + light emission)
                bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->applyTorqueImpulse(btVector3(0.0f,100.0f,0.0f));
                lightColor[2] = glm::vec3(bey2Color[0],bey2Color[1]*0.6f,bey2Color[2]*0.3f);
            }
            hasBoosted2 = true;
        }
    }

    // pressing a key number, we change the shader applied to the arena, and we manage enviromental fx

    if(key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        // change arena appearance
        new_subroutine ++;
        new_subroutine %= shaders.size();
        current_subroutine = new_subroutine;
        if (current_subroutine == 2)        // fog arena
        {
            fogFlag = true;
            arenaColor[0] = 0.29f;
            arenaColor[1] = 0.2f;
            arenaColor[2] = 0.16f;
        }
        else if (current_subroutine == 3)   // multiple colors light arena
        {
            fogFlag = false;
            multiFlag = true;
            lightColor[0] = glm::vec3(0.0f);
            arenaColor[0] = 0.1f;
            arenaColor[1] = 0.1f;
            arenaColor[2] = 0.1f;
        }
        else if (current_subroutine == 4)   // dark arena
        {
            multiFlag = false;
            arenaColor[0] = 0.4f;
            arenaColor[1] = 0.4f;
            arenaColor[2] = 0.4f;
            lightColor[0] = glm::vec3(0.1f);
            lightColor[4] = glm::vec3(0.0f);
            lightColor[5] = glm::vec3(0.0f);            
            lightColor[6] = glm::vec3(0.0f);            
            lightColor[7] = glm::vec3(0.0f);            

        }
        else                                // default arena -> mirror arena
        {
            multiFlag = false;
            lightColor[0] = glm::vec3(1.0f);
            arenaColor[0] = 0.02f;
            arenaColor[1] = 0.08f;
            arenaColor[2] = 0.3f;
        }
    }


    // we keep trace of the pressed keys
    // with this method, we can manage 2 keys pressed at the same time:
    // many I/O managers often consider only 1 key pressed at the time (the first pressed, until it is released)
    // using a boolean array, we can then check and manage all the keys pressed at the same time
    if(action == GLFW_PRESS)
        keys[key] = true;
    else if(action == GLFW_RELEASE)
        keys[key] = false;
}

//////////////////////////////////////////
// callback for mouse events
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
      // we move the camera view following the mouse cursor
      // we calculate the offset of the mouse cursor from the position in the last frame
      // when rendering the first frame, we do not have a "previous state" for the mouse, so we set the previous state equal to the initial values (thus, the offset will be = 0)

    if(firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    // we save the current cursor position in 2 global variables, in order to use the values in the keyboard callback function
    cursorX = xpos;
    cursorY = ypos;

    // offset of mouse cursor position
    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos;

    // the new position will be the previous one for the next frame
    lastX = xpos;
    lastY = ypos;

    // we pass the offset to the Camera class instance in order to update the rendering
    camera.ProcessMouseMovement(xoffset, yoffset);

}

//////////////////////////////////////////
// If one of the WASD keys is pressed, the camera is moved accordingly (the code is in utils/camera.h)
void apply_camera_movements()
{
    // if a single WASD key is pressed, then we will apply the full value of velocity v in the corresponding direction.
    // However, if two keys are pressed together in order to move diagonally (W+D, W+A, S+D, S+A), 
    // then the camera will apply a compensation factor to the velocities applied in the single directions, 
    // in order to have the full v applied in the diagonal direction  
    // the XOR on A and D is to avoid the application of a wrong attenuation in the case W+A+D or S+A+D are pressed together.  
    GLboolean diagonal_movement = (keys[GLFW_KEY_W] ^ keys[GLFW_KEY_S]) && (keys[GLFW_KEY_A] ^ keys[GLFW_KEY_D]); 
    camera.SetMovementCompensation(diagonal_movement);
    
    if(keys[GLFW_KEY_W])
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(keys[GLFW_KEY_S])
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(keys[GLFW_KEY_A])
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(keys[GLFW_KEY_D])
        camera.ProcessKeyboard(RIGHT, deltaTime);
}