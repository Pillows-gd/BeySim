/*
author: Filippo Giabelli

Real-Time Graphics Programming - a.a. 2022/2023
Master degree in Computer Science
Universita' degli Studi di Milano
*/

// Std. Includes
#include <string>

#ifdef _WIN32
    #define APIENTRY __stdcall
#endif

#include <glad/glad.h>

// GLFW library to create window and to manage I/O
#include <glfw/glfw3.h>

// another check related to OpenGL loader
// confirm that GLAD didn't include windows.h
#ifdef _WINDOWS_
    #error windows.h was included!
#endif

// classes developed to manage shaders, to load models, for FPS camera, for physical simulation, etc.
#include <utils/globals.h>
#include <utils/io_manager.h>

// we load the GLM classes used in the application
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>


////////////////// MAIN function ///////////////////////
int main()
{
  // Initialization of OpenGL context using GLFW
  glfwInit();
  // We set OpenGL specifications required for this application
  // In this case: 4.1 Core
  // If not supported by your graphics HW, the context will not be created and the application will close
  // N.B.) creating GLAD code to load extensions, try to take into account the specifications and any extensions you want to use,
  // in relation also to the values indicated in these GLFW commands
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  // we set if the window is resizable
  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);


  // we create the application's window
    GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "BeySim", nullptr, nullptr);
    if (!window)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // we put in relation the window and the callbacks
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_callback);

    // we disable the mouse cursor
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // GLAD tries to load the context set by GLFW
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        std::cout << "Failed to initialize OpenGL context" << std::endl;
        return -1;
    }

    // we define the viewport dimensions
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    // we enable Z test
    glEnable(GL_DEPTH_TEST);
    
    //the "clear" color for the frame buffer
    glClearColor(0.26f, 0.46f, 0.98f, 1.0f);

    // the Shader Program for the objects used in the application
    Shader foreground_shader = Shader("foreground_shader.vert", "foreground_shader.frag");
    // we create the Shader Program used for the environment map
    Shader skybox_shader("skybox_shader.vert", "skybox_shader.frag");


    // we load the model(s) (code of Model class is in include/utils/model.h)
    Model bey1Model("../models/driger_s_rot_uv.obj");
    Model bey2Model("../models/galux_rot.obj");
    Model arenaModel("../models/arena_high.obj");
    Model cubeModel("../models/cube.obj");
    Model sphereModel("../models/sphere.obj");
    Model coneModel("../models/cone.obj");

    // we load the cube map (we pass the path to the folder containing the 6 views)
    textureCube = LoadTextureCube("../textures/cube/Space/");
    textureGround = LoadTexture("../textures/Stars.png");

    // dimensions and position of the static plane
    // we will use the cube mesh to simulate the plane, because we need some "height" in the mesh
    // in order to make it work with the physics simulation
    glm::vec3 plane_pos = glm::vec3(0.0f, -1.0f, 0.0f);
    glm::vec3 plane_size = glm::vec3(200.0f, 0.1f, 200.0f);
    glm::vec3 plane_rot = glm::vec3(0.0f, 0.0f, 0.0f);
    
    // we create a rigid body for the plane. In this case, it is static, so we pass mass = 0;
    // in this way, the plane will not fall following the gravity force.
    btRigidBody* plane = bulletSimulation.createRigidBody(BOX,plane_pos,plane_size,plane_rot,0.0f,0.8f,0.3f,0.8,0.8);

    // same for arena
    glm::vec3 arena_pos = glm::vec3(0.0f, -1.2f, 0.0f);
    glm::vec3 arena_size = glm::vec3(0.6f, 0.6f, 0.6f);
    glm::vec3 arena_rot = glm::vec3(0.0f, 0.0f, 0.0f);


    //btRigidBody* arena = bulletSimulation.createRigidBody(BOX,arena_pos,arena_size,arena_rot,0.0f,0.3f,0.3f);
    btRigidBody* arena = bulletSimulation.createRigidBodyFromMesh(arena_pos,arena_size,arena_rot,0.0f,0.3f,0.1f,arenaModel);

    // we create 25 rigid bodies for the cubes of the scene. In this case, we use BoxShape, with the same dimensions of the cubes, as collision shape of Bullet. For more complex cases, a Bounding Box of the model may have to be calculated, and its dimensions to be passed to the physics library
    // position of the cube
    glm::vec3 bey1_pos;
    glm::vec3 bey2_pos;
    GLfloat bey1_size = 0.3f;
    GLfloat bey2_size = 0.3f;
    glm::vec3 bey1_bb = bey1Model.getBoundingBox();
    glm::vec3 bey2_bb = bey2Model.getBoundingBox();
    glm::vec3 bey1_size_scaled = glm::vec3(bey1_bb.x*bey1_size,bey1_bb.y*bey1_size,bey1_bb.z*bey1_size);
    glm::vec3 bey2_size_scaled = glm::vec3(bey2_bb.x*bey2_size,bey2_bb.y*bey2_size,bey2_bb.z*bey2_size);
    glm::vec3 bey1_rot = glm::vec3(0.0f,-0.2f,-0.2f);
    glm::vec3 bey2_rot = glm::vec3(0.0f,0.25f,-0.15f);

    // calculate BV

    // rigid body
    btRigidBody* bey1;
    btRigidBody* bey2;

    // position of each Beyblade in the grid (with a random factor), and creatino of rigid bodies
    bey1_pos = glm::vec3(-2.5f+rand_float(), 8.0f,-3.0f+rand_float());
    bey1 = bulletSimulation.createRigidBody(CONE,bey1_pos,bey1_size_scaled,bey1_rot,
      0.8f,   // MASS
      0.5f,   // FRICTION
      0.8f,   // RESTITUTION
      0.12f,   // ANGULAR DUMPING
      0.9f    // ROTATIONAL FRICION
    );
    bey2_pos = glm::vec3(2.0f+rand_float(), 7.0f,2.5f+rand_float());
    bey2 = bulletSimulation.createRigidBody(CONE,bey2_pos,bey2_size_scaled,bey2_rot,
      1.0f,   // MASS
      0.3f,   // FRICTION
      0.65f,  // RESTITUTION
      0.15f,  // ANGULAR DUMPING
      0.6f    // ROTATIONAL FRICION
    );

    // setting up initial forces
    torque1 = btVector3(0.0f,950.0f+(rand_float()*100),0.0f);
    torque2 = btVector3(0.0f,950.0f+(rand_float()*100),0.0f);

    launchDirection = btVector3(0.2f,0.0f,0.3f);
    bey1->applyTorqueImpulse(torque1);
    bey2->applyTorqueImpulse(torque2);
    bey1->applyCentralImpulse(launchDirection);
    bey2->applyCentralImpulse(-launchDirection);

  // keep trace of the initial position of the Beyblades as the application starts: this will be needed in the io_manager
  bey1LaunchPose = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform();
  bey2LaunchPose = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[3]->getWorldTransform();
  
  // we set the maximum delta time for the update of the physical simulation
  GLfloat maxSecPerFrame = 1.0f / 60.0f;

  // Projection matrix: FOV angle, aspect ratio, near and far planes
  projection = glm::perspective(45.0f, (float)screenWidth/(float)screenHeight, 0.1f, 10000.0f);

  // Model and Normal transformation matrices for the objects in the scene: we set to identity
  glm::mat4 objModelMatrix = glm::mat4(1.0f);
  glm::mat3 objNormalMatrix = glm::mat3(1.0f);
  glm::mat4 planeModelMatrix = glm::mat4(1.0f);
  glm::mat3 planeNormalMatrix = glm::mat3(1.0f);


  // Set up particle systems
  m_Particle.ColorBegin = { 1.0f, 1.0f, 1.0f , 1.0f };
  m_Particle.ColorEnd = { 1.0f, 0.9f, 0.7f , 1.0f };
  m_Particle.SizeBegin = 0.03f, m_Particle.SizeVariation = 0.1f, m_Particle.SizeEnd = 0.01f;
  m_Particle.LifeTime = 0.3f;
  m_Particle.Velocity = { 0.0f, 4.0f, 0.0f };
  m_Particle.VelocityVariation = { 20.0f, 2.0f, 20.0f  };
  m_Particle.Position = { 0.0f, 0.0f, 0.0f };
  m_Particle.PositionVariation = { 0.0f, 0.0f, 0.0f };

  boostProps1.ColorBegin = { 1.0f, 1.0f, 1.0f , 1.0f };
  boostProps1.ColorEnd = { 0.1f, 1.0f, 0.3f, 1.0f };
  boostProps1.SizeBegin = 0.02f, boostProps1.SizeVariation = 0.05f, boostProps1.SizeEnd = 0.02f;
  boostProps1.LifeTime = 0.5f;
  boostProps1.Velocity = { 0.0f, 5.0f, 0.0f };
  boostProps1.VelocityVariation = { 0.0f, 8.0f, 0.0f  };
  boostProps1.Position = { 0.0f, 0.0f, 0.0f };
  boostProps1.PositionVariation = { 0.5f, 0.0f, 0.5f};
  
  boostProps2.ColorBegin = { 1.0f, 1.0f, 1.0f , 1.0f };
  boostProps2.ColorEnd = { bey2Color[0], bey2Color[1]*0.6f, bey2Color[2]*0.3f, 1.0f };
  boostProps2.SizeBegin = 0.02f, boostProps2.SizeVariation = 0.05f, boostProps2.SizeEnd = 0.02f;
  boostProps2.LifeTime = 0.5f;
  boostProps2.Velocity = { 0.0f, 5.0f, 0.0f };
  boostProps2.VelocityVariation = { 0.0f, 8.0f, 0.0f  };
  boostProps2.Position = { 0.0f, 0.0f, 0.0f };
  boostProps2.PositionVariation = { 0.5f, 0.0f, 0.5f};

  // Rendering loop: this code is executed at each frame
  while(!glfwWindowShouldClose(window))
  {
      // we determine the time passed from the beginning
      // and we calculate time difference between current frame rendering and the previous one
      GLfloat currentFrame = glfwGetTime();
      deltaTime = currentFrame - lastFrame;
      lastFrame = currentFrame;

      // Check is an I/O event is happening
      glfwPollEvents();
      // we apply FPS camera movements
      apply_camera_movements();
      // View matrix (=camera): position, view direction, camera "up" vector
      // in this example, it has been defined as a global variable (we need it in the keyboard callback function)
      view = camera.GetViewMatrix();

      // we "clear" the frame and z buffer
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // we set the rendering mode
      if (wireframe)
          // Draw in wireframe
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      else
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

      bulletSimulation.dynamicsWorld->stepSimulation((deltaTime < maxSecPerFrame ? deltaTime : maxSecPerFrame),10);

      /////////////////// OBJECTS ////////////////////////////////////////////////
      // We "install" the selected Shader Program as part of the current rendering process
      foreground_shader.Use();
      // We search inside the Shader Program the name of a subroutine, and we get the numerical index
      GLuint index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, "GGX");
      // we activate the subroutine using the index
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);

      // we pass projection and view matrices to the Shader Program
      glUniformMatrix4fv(glGetUniformLocation(foreground_shader.Program, "projectionMatrix"), 1, GL_FALSE, glm::value_ptr(projection));
      glUniformMatrix4fv(glGetUniformLocation(foreground_shader.Program, "viewMatrix"), 1, GL_FALSE, glm::value_ptr(view));

      // we determine the position in the Shader Program of the uniform variables
      GLint objDiffuseLocation = glGetUniformLocation(foreground_shader.Program, "diffuseColor");
      GLint pointLightLocation = glGetUniformLocation(foreground_shader.Program, "pointLightPosition");
      GLint pointLightColor = glGetUniformLocation(foreground_shader.Program, "lightColor");
      GLint kdLocation = glGetUniformLocation(foreground_shader.Program, "Kd");
      GLint alphaLocation = glGetUniformLocation(foreground_shader.Program, "alpha");
      GLint f0Location = glGetUniformLocation(foreground_shader.Program, "F0");
      GLint fogFlagLocation = glGetUniformLocation(foreground_shader.Program, "fogFlag");

      // we assign the value to the uniform variable
      glUniform3fv(pointLightLocation, NUM_LIGHTS, glm::value_ptr(lightPos[0]));
      glUniform3fv(pointLightColor, NUM_LIGHTS, glm::value_ptr(lightColor[0]));
      glUniform1f(kdLocation, Kd);
      glUniform1f(alphaLocation, alpha);
      glUniform1f(f0Location, F0);
      glUniform1i(fogFlagLocation, fogFlag);

      // we make static objects less metallic
      alpha = 0.1f;
      glUniform1f(alphaLocation, alpha);


      // STATIC PLANE
      index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, "GGX_TX");
      // we activate the subroutine using the index
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);

      // we use a specific color for the plane
      glUniform3fv(objDiffuseLocation, 1, planeMaterial);

      // The plane is static, so its Collision Shape is not subject to forces, and it does not move. Thus, we do not need to use dynamicsWorld to acquire the rototraslations, but we can just use directly glm to manage the matrices
      // if, for some reason, the plane becomes a dynamic rigid body, the following code must be modified
      // we reset to identity at each frame
      planeModelMatrix = glm::mat4(1.0f);
      planeNormalMatrix = glm::mat3(1.0f);
      planeModelMatrix = glm::translate(planeModelMatrix, plane_pos);
      planeModelMatrix = glm::scale(planeModelMatrix, plane_size);
      planeNormalMatrix = glm::inverseTranspose(glm::mat3(view*planeModelMatrix));
      glUniformMatrix4fv(glGetUniformLocation(foreground_shader.Program, "modelMatrix"), 1, GL_FALSE, glm::value_ptr(planeModelMatrix));
      glUniformMatrix3fv(glGetUniformLocation(foreground_shader.Program, "normalMatrix"), 1, GL_FALSE, glm::value_ptr(planeNormalMatrix));
      GLint textureLocation = glGetUniformLocation(foreground_shader.Program, "tex");
      GLint repeatLocation = glGetUniformLocation(foreground_shader.Program, "repeat");

      // we activate the texture with id 1, and we bind the id to the loaded texture data
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, textureGround);
      // we pass the id of the texture (= to number X in GL_TEXTUREX at line 327) and the number of repetitions for the plane
      glUniform1i(textureLocation, 1);
      glUniform1f(repeatLocation, 80.0f);

      // we render the plane
      cubeModel.Draw();
      planeModelMatrix = glm::mat4(1.0f);

      /////
      // ARENA
      index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, shaders[current_subroutine].c_str());
      // we activate the subroutine using the index
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);
      glUniform3fv(objDiffuseLocation, 1, arenaColor);

      // we activate the cube map
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_CUBE_MAP, textureCube);

      // we determine the position in the Shader Program of the uniform variables
      GLint tCubeLocation = glGetUniformLocation(foreground_shader.Program, "tCube");

      // The arena is static, so its Collision Shape is not subject to forces, and it does not move. Thus, we do not need to use dynamicsWorld to acquire the rototraslations, but we can just use directly glm to manage the matrices
      // if, for some reason, the plane becomes a dynamic rigid body, the following code must be modified
      // we reset to identity at each frame
      objModelMatrix = glm::mat4(1.0f);
      objNormalMatrix = glm::mat3(1.0f);
      objModelMatrix = glm::translate(objModelMatrix, arena_pos);
      objModelMatrix = glm::scale(objModelMatrix, arena_size);
      objNormalMatrix = glm::inverseTranspose(glm::mat3(view*objModelMatrix));
      glUniformMatrix4fv(glGetUniformLocation(foreground_shader.Program, "modelMatrix"), 1, GL_FALSE, glm::value_ptr(objModelMatrix));
      glUniformMatrix3fv(glGetUniformLocation(foreground_shader.Program, "normalMatrix"), 1, GL_FALSE, glm::value_ptr(objNormalMatrix));
      glUniform3fv(glGetUniformLocation(foreground_shader.Program, "cameraPosition"), 1, glm::value_ptr(camera.Position));
      // we assign the value to the uniform variable
      glUniform1i(tCubeLocation, 0);
      // we render the arena
      arenaModel.Draw();
      objModelMatrix = glm::mat4(1.0f);

      
      index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, "GGX");
      // we activate the subroutine using the index
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);

      /////
      // DYNAMIC OBJECTS (FALLING CUBES + BULLETS)
      // array of 16 floats = "native" matrix of OpenGL. We need it as an intermediate data structure to "convert" the Bullet matrix to a GLM matrix
      GLfloat matrix[16];
      btTransform transform;

      // we need two variables to manage the rendering of both cubes and bullets
      glm::vec3 obj_size = glm::vec3(1.0f,1.0f,1.0f);
      Model* objectModel;

      // we ask Bullet to provide the total number of Rigid Bodies in the scene
      // at the beginning they are 26 (the static plane + the falling cubes), but we can add several bullets by pressing the space key
      int num_cobjs = bulletSimulation.dynamicsWorld->getNumCollisionObjects();
      
      // we keep trace of the collisions between bey1 and bey2
      btPersistentManifold* collision = bulletSimulation.dispatcher->getNewManifold(
        bulletSimulation.dynamicsWorld->getCollisionObjectArray()[2],
        bulletSimulation.dynamicsWorld->getCollisionObjectArray()[3]
        );
      btManifoldPoint contact_point;

      // we cycle among all the Rigid Bodies (starting from 1 to avoid the plane)
      for (GLuint i=2; i<num_cobjs;i++)
      {
        // the first 25 objects are the falling cubes
          if (i == 2)
          {
              // we point objectModel to the cube
              objectModel = &bey1Model;
              obj_size = glm::vec3(bey1_size);
              //DEBUG WITH A CONE
              //objectModel = &coneModel;
              //GLfloat bey_cone_ratio = 0.56f;
              //obj_size = glm::vec3(bey1_size_scaled.x*bey_cone_ratio,bey1_size_scaled.y*bey_cone_ratio,bey1_size_scaled.z*bey_cone_ratio);
              // we pass red color to the shader
              glUniform3fv(objDiffuseLocation, 1, bey1Color);
              btVector3 curPos = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
              lightPos[1] = glm::vec3(curPos.getX(),curPos.getY(),curPos.getZ());
          }
          else if (i == 3)
          {
              // we point objectModel to the cube
              objectModel = &bey2Model;
              obj_size = glm::vec3(bey2_size);
              // DEBUG WITH A CONE:
              //objectModel = &coneModel;
              //GLfloat bey_cone_ratio = 0.56f;
              //obj_size = glm::vec3(bey2_size_scaled.x*bey_cone_ratio,bey2_size_scaled.y*bey_cone_ratio,bey2_size_scaled.z*bey_cone_ratio);
              // we pass red color to the shader
              glUniform3fv(objDiffuseLocation, 1, bey2Color);
              btVector3 curPos = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
              lightPos[2] = glm::vec3(curPos.getX(),curPos.getY(),curPos.getZ());
          }


          // we take the Collision Object from the list
          btCollisionObject* obj = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[i];
          // we upcast it in order to use the methods of the main class RigidBody
          btRigidBody* body = btRigidBody::upcast(obj);

          // we take the transformation matrix of the rigid boby, as calculated by the physics engine
          body->getMotionState()->getWorldTransform(transform);

          // we convert the Bullet matrix (transform) to an array of floats
          transform.getOpenGLMatrix(matrix);
          
          // we reset to identity at each frame
          objModelMatrix = glm::mat4(1.0f);
          objNormalMatrix = glm::mat3(1.0f);

          // we create the GLM transformation matrix
          // 1) we convert the array of floats to a GLM mat4 (using make_mat4 method)
          // 2) Bullet matrix provides rotations and translations: it does not consider scale (usually the Collision Shape is generated using directly the scaled dimensions). If, like in our case, we have applied a scale to the original model, we need to multiply the scale to the rototranslation matrix created in 1). If we are working on an imported and not scaled model, we do not need to do this
          objModelMatrix = glm::make_mat4(matrix) * glm::scale(objModelMatrix, obj_size);
          // we create the normal matrix
          objNormalMatrix = glm::inverseTranspose(glm::mat3(view*objModelMatrix));
          glUniformMatrix4fv(glGetUniformLocation(foreground_shader.Program, "modelMatrix"), 1, GL_FALSE, glm::value_ptr(objModelMatrix));
          glUniformMatrix3fv(glGetUniformLocation(foreground_shader.Program, "normalMatrix"), 1, GL_FALSE, glm::value_ptr(objNormalMatrix));
          
          
          // we make dynamic object more metallic
          alpha = 0.1f;
          glUniform1f(alphaLocation, alpha);

          // we render the model
          // N.B.) if the number of models is relatively low, this approach (we render the same mesh several time from the same buffers) can work. If we must render hundreds or more of copies of the same mesh,
          // there are more advanced techniques to manage Instanced Rendering (see https://learnopengl.com/#!Advanced-OpenGL/Instancing for examples).
          objectModel->Draw();

          // we "reset" the matrix
          objModelMatrix = glm::mat4(1.0f);

      }

      // turn off boost lights gradually, if already boosted
      for (int i=1; i<3; i++)
        for (int c=0; c<3; c++)
            if (lightColor[i][c] > 0.0f)
                lightColor[i][c] -= lightColor[i][c] > 0.01f ? lightColor[i][c]*deltaTime*0.5f : lightColor[i][c];
      for (int c=0; c<3; c++)
        if (lightColor[3][c] > 0.0f)
            lightColor[3][c] -= lightColor[3][c] > 0.01f ? lightColor[3][c]*deltaTime*20.0f : lightColor[3][c];
      
      // geometry to compute the contact point
      btVector3 a = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[2]->getWorldTransform().getOrigin();
      btVector3 b = bulletSimulation.dynamicsWorld->getCollisionObjectArray()[3]->getWorldTransform().getOrigin();
      btVector3 a_to_b = b-a;
      btVector3 b_to_a = a-b;
      btVector3 a_ray = a_to_b.normalized()*btVector3(bey1_size_scaled.x,bey1_size_scaled.y,bey1_size_scaled.z)*0.5;
      btVector3 b_ray = b_to_a.normalized()*btVector3(bey2_size_scaled.x,bey2_size_scaled.y,bey2_size_scaled.z)*0.5;
      btVector3 a_edge = a+a_ray;
      // threshold to detect collision (the higher: the larger the hitbox)
      btScalar epsilon = 0.2;
      
      // update sparkles attributes
      m_Particle.Position = a_edge;
      m_ParticleSystem.OnUpdate(deltaTime);

      // keep trace of current total angular velocity to determin whether to emit sparkles or not
      btScalar currentPower = bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[0]->getAngularVelocity().norm() +
        bulletSimulation.dynamicsWorld->getNonStaticRigidBodies()[1]->getAngularVelocity().norm();

    // detect collision and emit sparkles and light if detected
	  if (a_to_b.length()-epsilon <= a_ray.length()+b_ray.length())
      {
        lightPos[3] = glm::vec3(a_edge.getX(),a_edge.getY(),a_edge.getZ());
        for (int i = 0; i < (int)currentPower/200; i++)
            m_ParticleSystem.Emit(m_Particle);
        if (currentPower > 200)
            lightColor[3] += glm::vec3(0.1f*currentPower/1000,0.08f*currentPower/1000,0.05f*currentPower/1000);  
      }

      // update boost particles position and attributes
      boostProps1.Position = a + btVector3(0.0f,bey1_size_scaled.y,0.0f);
      boostSystem1.OnUpdate(deltaTime/2);
      boostProps2.Position = b + btVector3(0.0f,bey2_size_scaled.y,0.0f);
      boostSystem2.OnUpdate(deltaTime/2);

      // keep trace of current boost light intensity to determin if particles have to be emitted too
      float boostLightIntensity1 = lightColor[1][0]+lightColor[1][1]+lightColor[1][2];
      float boostLightIntensity2 = lightColor[2][0]+lightColor[2][1]+lightColor[2][2];

      if (boostLightIntensity1/3 - 0.2f > 0.0f)
        boostSystem1.Emit(boostProps1);
    
      if (boostLightIntensity2/3 - 0.2f > 0.0f)
        boostSystem2.Emit(boostProps2);

      // change multiple arena lights color
      GLfloat phase = glfwGetTime()*0.3f - (int)(glfwGetTime()*0.3f);
      currentColor = glm::vec3(
        0.3f*(0.5f*sin(2*_Pi*phase) + 0.5f),
        0.3f*(0.5f*sin(2*_Pi*phase + 2*_Pi*0.33f) + 0.5f),
        0.3f*(0.5f*sin(2*_Pi*phase + 2*_Pi*0.66f) + 0.5f)
      );
      currentR = currentColor[0];
      currentG = currentColor[1];
      currentB = currentColor[2];
      if (multiFlag)
      {
        lightColor[4] = glm::vec3(currentR,currentG,currentB);            
        lightColor[5] = glm::vec3(currentG,currentR,currentB);            
        lightColor[6] = glm::vec3(currentB,currentG,currentR);            
        lightColor[7] = glm::vec3(currentR,currentB,currentG);            
      }

      index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, "Particles");
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);

      // render particles
      m_ParticleSystem.OnRender(foreground_shader, camera);
      boostSystem1.OnRender(foreground_shader, camera);
      boostSystem2.OnRender(foreground_shader, camera);

      index = glGetSubroutineIndex(foreground_shader.Program, GL_FRAGMENT_SHADER, "GGX");
      glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &index);

        /////////////////// SKYBOX ////////////////////////////////////////////////
        // we use the cube to attach the 6 textures of the environment map.
        // we render it after all the other objects, in order to avoid the depth tests as much as possible.
        // we will set, in the vertex shader for the skybox, all the values to the maximum depth. Thus, the environment map is rendered only where there are no other objects in the image (so, only on the background).
        //Thus, we set the depth test to GL_LEQUAL, in order to let the fragments of the background pass the depth test (because they have the maximum depth possible, and the default setting is GL_LESS)
        glDepthFunc(GL_LEQUAL);
        skybox_shader.Use();
        // we activate the cube map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureCube);
         // we pass projection and view matrices to the Shader Program of the skybox
        glUniformMatrix4fv(glGetUniformLocation(skybox_shader.Program, "projectionMatrix"), 1, GL_FALSE, glm::value_ptr(projection));
        // to have the background fixed during camera movements, we have to remove the translations from the view matrix
        // thus, we consider only the top-left submatrix, and we create a new 4x4 matrix
        view = glm::mat4(glm::mat3(view)); // Remove any translation component of the view matrix
        glUniformMatrix4fv(glGetUniformLocation(skybox_shader.Program, "viewMatrix"), 1, GL_FALSE, glm::value_ptr(view));

        // we determine the position in the Shader Program of the uniform variables
        textureLocation = glGetUniformLocation(skybox_shader.Program, "tCube");
        // we assign the value to the uniform variable
        glUniform1i(textureLocation, 0);

        // we render the cube with the environment map
        cubeModel.Draw();
        // we set again the depth test to the default operation for the next frame
        glDepthFunc(GL_LESS);

      // we swap buffers
      glfwSwapBuffers(window);
  }

  // when I exit from the graphics loop, it is because the application is closing
  // we delete the Shader Programs
  foreground_shader.Delete();
  // we delete the data of the physical simulation
  bulletSimulation.Clear();
  // we deallocate particle mesh memory
  m_ParticleSystem.Destroy();
  // we disable transparency
  glDisable(GL_BLEND);
  // we close and delete the created context
  glfwTerminate();
  return 0;
}

