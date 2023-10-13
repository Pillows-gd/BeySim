/*
Physics class:
- initialization of the physics simulation using the Bullet library

The class sets up the collision manager and the resolver of the constraints, using basic general-purposes methods provided by the library. Advanced and multithread methods are available, please consult Bullet documentation and examples

createRigidBody method sets up a Box or Sphere Collision Shape. For other Shapes, you must extend the method.

author: Davide Gadia

Real-Time Graphics Programming - a.a. 2022/2023
Master degree in Computer Science
Universita' degli Studi di Milano
*/

#pragma once

#include <bullet/btBulletDynamicsCommon.h>
#include <utils/model.h>

//enum to identify the 2 considered Collision Shapes
enum shapes{ BOX, SPHERE, CONE };

///////////////////  Physics class ///////////////////////
class Physics
{
public:

    btDiscreteDynamicsWorld* dynamicsWorld; // the main physical simulation class
    btAlignedObjectArray<btCollisionShape*> collisionShapes; // a vector for all the Collision Shapes of the scene
    btDefaultCollisionConfiguration* collisionConfiguration; // setup for the collision manager
    btCollisionDispatcher* dispatcher; // collision manager
    btBroadphaseInterface* overlappingPairCache; // method for the broadphase collision detection
    btSequentialImpulseConstraintSolver* solver; // constraints solver


    //////////////////////////////////////////
    // constructor
    // we set all the classes needed for the physical simulation
    Physics()
    {
        // Collision configuration, to be used by the collision detection class
        // collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
        this->collisionConfiguration = new btDefaultCollisionConfiguration();

        // default collision dispatcher (= collision detection method). For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        this->dispatcher = new btCollisionDispatcher(this->collisionConfiguration);

        // btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
        this->overlappingPairCache = new btDbvtBroadphase();

        // we set a ODE solver, which considers forces, constraints, collisions etc., to calculate positions and rotations of the rigid bodies.
        // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        this->solver = new btSequentialImpulseConstraintSolver();

        //  DynamicsWorld is the main class for the physical simulation
        this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher,this->overlappingPairCache,this->solver,this->collisionConfiguration);

        // we set the gravity force
        this->dynamicsWorld->setGravity(btVector3(0.0f,-9.82f*5.00f,0.0f));   // !! GRAVITY CHANGED TO MAKE THE SIMULATION MORE ACCURATE !!
    }

    //////////////////////////////////////////
    // Method for the creation of a rigid body, based on a Box or Sphere Collision Shape
    // The Collision Shape is a reference solid that approximates the shape of the actual object of the scene. The Physical simulation is applied to these solids, and the rotations and positions of these solids are used on the real models.
    btRigidBody* createRigidBody(int type, glm::vec3 pos, glm::vec3 size, glm::vec3 rot, float m, float friction , float restitution,  float a_dumping = 0.3f, float r_friction = 0.3f)
    {

        btCollisionShape* cShape = NULL;

        // we convert the glm vector to a Bullet vector
        btVector3 position = btVector3(pos.x,pos.y,pos.z);

        // we set a quaternion from the Euler angles passed as parameters
        btQuaternion rotation;
        rotation.setEuler(rot.x,rot.y,rot.z);

        // Box Collision shape
        if (type == BOX)
        {
            // we convert the glm vector to a Bullet vector
            btVector3 dim = btVector3(size.x,size.y,size.z);
            // BoxShape
            cShape = new btBoxShape(dim);
        }
        // Sphere Collision Shape (in this case we consider only the first component)
        else if (type == SPHERE)
            cShape = new btSphereShape(size.x);
        else if (type == CONE)
        {
            cShape = new btConeShape(size.x/2,size.y);
            rotation.setEuler(rot.x,rot.y+3.1415f,rot.z);
        }
        // we add this Collision Shape to the vector
        this->collisionShapes.push_back(cShape);

        // We set the initial transformations
        btTransform objTransform;
        objTransform.setIdentity();
        objTransform.setRotation(rotation);
        // we set the initial position (it must be equal to the position of the corresponding model of the scene)
        objTransform.setOrigin(position);

        // if objects has mass = 0 -> then it is static (it does not move and it is not subject to forces)
        btScalar mass = m;
        bool isDynamic = (mass != 0.0f);

        // if it is dynamic (mass > 0) then we calculates local inertia
        btVector3 localInertia(0.0f,0.0f,0.0f);
        if (isDynamic)
            cShape->calculateLocalInertia(mass,localInertia);

        // we initialize the Motion State of the object on the basis of the transformations
        // using the Motion State, the physical simulation will calculate the positions and rotations of the rigid body
        btDefaultMotionState* motionState = new btDefaultMotionState(objTransform);

        // we set the data structure for the rigid body
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,cShape,localInertia);
        // we set friction and restitution
        rbInfo.m_friction = friction;
        rbInfo.m_restitution = restitution;

        // if the Collision Shape is a sphere
        if (type == SPHERE || type == CONE){
            // the sphere touches the plane on the plane on a single point, and thus the friction between sphere and the plane does not work -> the sphere does not stop
            // to avoid the problem, we apply the rolling friction together with an angular damping (which applies a resistence during the rolling movement), in order to make the sphere to stop after a while
            rbInfo.m_angularDamping = a_dumping;
            rbInfo.m_rollingFriction = r_friction;
        }

        // we create the rigid body
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        this->dynamicsWorld->addRigidBody(body);

        // the function returns a pointer to the created rigid body
        // in a standard simulation (e.g., only objects falling), it is not needed to have a reference to a single rigid body, but in some cases (e.g., the application of an impulse), it is needed.
        return body;
    }
    btRigidBody* createRigidBodyFromMesh(glm::vec3 pos, glm::vec3 size, glm::vec3 rot, float m, float friction , float restitution, const Model& shape)
    {

        // we convert the glm vector to a Bullet vector
        btVector3 position = btVector3(pos.x,pos.y,pos.z);

        // we set a quaternion from the Euler angles passed as parameters
        btQuaternion rotation;
        rotation.setEuler(rot.x,rot.y,rot.z);

        auto targetIndices = shape.meshes[0].indices;
        auto targetVertices = shape.meshes[0].vertices;

            // first create a btTriangleIndexVertexArray
        // NOTE: we must track this pointer and delete it when all shapes are done with it!
        btTriangleIndexVertexArray* data = new btTriangleIndexVertexArray;

        // add an empty mesh (data makes a copy)
        btIndexedMesh tempMesh;
        data->addIndexedMesh(tempMesh, PHY_FLOAT);

        // get a reference to internal copy of the empty mesh
        btIndexedMesh& mesh = data->getIndexedMeshArray()[0];

        // allocate memory for the mesh
        const int32_t VERTICES_PER_TRIANGLE = 3;
        size_t numIndices = targetIndices.size();
        mesh.m_numTriangles = numIndices / VERTICES_PER_TRIANGLE;
        if (numIndices < std::numeric_limits<int16_t>::max()) {
            // we can use 16-bit indices
            mesh.m_triangleIndexBase = new unsigned char[sizeof(int16_t) * (size_t)numIndices];
            mesh.m_indexType = PHY_SHORT;
            mesh.m_triangleIndexStride = VERTICES_PER_TRIANGLE * sizeof(int16_t);
        } else {
            // we need 32-bit indices
            mesh.m_triangleIndexBase = new unsigned char[sizeof(int32_t) * (size_t)numIndices];
            mesh.m_indexType = PHY_INTEGER;
            mesh.m_triangleIndexStride = VERTICES_PER_TRIANGLE * sizeof(int32_t);
        }
        mesh.m_numVertices = targetVertices.size();
        mesh.m_vertexBase = new unsigned char[VERTICES_PER_TRIANGLE * sizeof(btScalar) * (size_t)mesh.m_numVertices];
        mesh.m_vertexStride = VERTICES_PER_TRIANGLE * sizeof(btScalar);

        // copy vertices into mesh
        btScalar* vertexData = static_cast<btScalar*>((void*)(mesh.m_vertexBase));
        for (int32_t i = 0; i < mesh.m_numVertices; ++i) {
            int32_t j = i * VERTICES_PER_TRIANGLE;
            const auto& point = targetVertices[i];
            vertexData[j] = point.Position.x*size[0];
            vertexData[j + 1] = point.Position.y*size[1];
            vertexData[j + 2] = point.Position.z*size[2];
        }
        // copy indices into mesh
        if (numIndices < std::numeric_limits<int16_t>::max()) {
            // 16-bit case
            int16_t* indices = static_cast<int16_t*>((void*)(mesh.m_triangleIndexBase));
            for (int32_t i = 0; i < numIndices; ++i) {
                indices[i] = (int16_t)targetIndices[i];
            }
        } else {
            // 32-bit case
            int32_t* indices = static_cast<int32_t*>((void*)(mesh.m_triangleIndexBase));
            for (int32_t i = 0; i < numIndices; ++i) {
                indices[i] = targetIndices[i];
            }
        }

        // create the shape
        // NOTE: we must track this pointer and delete it when all btCollisionObjects that use it are done with it!
        const bool USE_QUANTIZED_AABB_COMPRESSION = true;
        btBvhTriangleMeshShape* cShape = new btBvhTriangleMeshShape(data, USE_QUANTIZED_AABB_COMPRESSION);

        // we add this Collision Shape to the vector
        this->collisionShapes.push_back(cShape);

        // We set the initial transformations
        btTransform objTransform;
        objTransform.setIdentity();
        objTransform.setRotation(rotation);
        // we set the initial position (it must be equal to the position of the corresponding model of the scene)
        objTransform.setOrigin(position);

        // if objects has mass = 0 -> then it is static (it does not move and it is not subject to forces)
        btScalar mass = m;
        bool isDynamic = (mass != 0.0f);

        // if it is dynamic (mass > 0) then we calculates local inertia
        btVector3 localInertia(0.0f,0.0f,0.0f);
        if (isDynamic)
            cShape->calculateLocalInertia(mass,localInertia);

        // we initialize the Motion State of the object on the basis of the transformations
        // using the Motion State, the physical simulation will calculate the positions and rotations of the rigid body
        btDefaultMotionState* motionState = new btDefaultMotionState(objTransform);

        // we set the data structure for the rigid body
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,cShape,localInertia);
        // we set friction and restitution
        rbInfo.m_friction = friction;
        rbInfo.m_restitution = restitution;

        // we create the rigid body
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        this->dynamicsWorld->addRigidBody(body);

        // the function returns a pointer to the created rigid body
        // in a standard simulation (e.g., only objects falling), it is not needed to have a reference to a single rigid body, but in some cases (e.g., the application of an impulse), it is needed.
        return body;
    }

    //////////////////////////////////////////
    // We delete the data of the physical simulation when the program ends
    void Clear()
    {
        //we remove the rigid bodies from the dynamics world and delete them
        for (int i=this->dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
        {
            // we remove all the Motion States
            btCollisionObject* obj = this->dynamicsWorld->getCollisionObjectArray()[i];
            // we upcast in order to use the methods of the main class RigidBody
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            this->dynamicsWorld->removeCollisionObject( obj );
            delete obj;
        }

        //delete dynamics world
        delete this->dynamicsWorld;

        //delete solver
        delete this->solver;

        //delete broadphase
        delete this->overlappingPairCache;

        //delete dispatcher
        delete this->dispatcher;

        delete this->collisionConfiguration;

        this->collisionShapes.clear();
    }
};