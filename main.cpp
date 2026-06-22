/*
This simulates a chain as you can see the ball immediately stops as it reaches the bottom
unlike a spring/bungee, a chain is sturdy and heavy and doesnt have the elasticity to 
conserve energy and bring the ball back up, as such it falls down to its maximum length without
any form of bounce
*/

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <iostream>
#include <string>

#include <fstream>
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include <list>
#include <limits>

using namespace std::chrono_literals;

constexpr std::chrono::nanoseconds timestep(12ms);


class MyVector
{
public:
    float x = 0;
    float y = 0;
	float z = 0;

    MyVector Direction();

    MyVector operator+(const MyVector& other) const {
        return { x + other.x, y + other.y, z + other.z };
    }
    MyVector operator-(const MyVector& other) const {
        return { x - other.x, y - other.y, z - other.z };
    }
    MyVector operator*(float scalar) const {
        return { x * scalar, y * scalar, z * scalar };
    }
    friend MyVector operator*(float scalar, const MyVector& vec) {
        return { vec.x * scalar, vec.y * scalar, vec.z * scalar };
    }
    MyVector Normalize() {
        float mag = Magnitude();
        if (mag <= 0) return { 0, 0, 0 };
        return { x / mag, y / mag, z / mag };
    }
    float Magnitude() {
        return sqrtf(x * x + y * y + z * z);
    }
    float Dot(const MyVector& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

class P6Particle
{
public:
    float mass = 0;
    float damping = 0.8;
    glm::vec3 Color = glm::vec3(1.0f);
    MyVector Position;
    MyVector Velocity;
    MyVector Acceleration;
    void AddForce(MyVector force) {
		accumulatedForce = accumulatedForce + force;
    }
    void ResetForce() {
		float d_mass = glm::max(std::numeric_limits<float>::min(), mass);
        Acceleration = accumulatedForce * (1.0f / d_mass);
		accumulatedForce = MyVector{ 0, 0, 0 };
    }


protected:
	MyVector accumulatedForce = MyVector{ 0, 0, 0 };
    bool destroyed = false;

    void UpdatePosition(float time) {
        Position = Position + (Velocity * time) + ((1.0f / 2.0f) * (Acceleration * time * time));
    }
    void UpdateVelocity(float time) {
		float d_mass = glm::max(std::numeric_limits<float>::min(), mass);

        Velocity = Velocity + (Acceleration * time);
		Velocity = Velocity * powf(damping, time);
    }

public:
    void Update(float time) {
		ResetForce();
        UpdatePosition(time);
        UpdateVelocity(time);
    }

    void Destroy() {
        destroyed = true;
    }
    bool isDestroyed() {
        return destroyed;
    }

    void Draw(GLuint shaderProg, GLuint VAO, GLuint transformLoc, std::vector<GLuint>& mesh_indices) {
        // update transform from position
		if (isDestroyed()) return;
        
        glm::mat4 transform = glm::mat4(1.0f);
        transform = glm::translate(transform, glm::vec3(Position.x, Position.y, Position.z));
        transform = glm::scale(transform, glm::vec3(0.05f, 0.05f, 0.05f)); //sphere size 
        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

        // send color to shader
        GLuint colorLoc = glGetUniformLocation(shaderProg, "color");
        glUniform3fv(colorLoc, 1, glm::value_ptr(Color));

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)mesh_indices.size(), GL_UNSIGNED_INT, 0);

    } 
};

class ParticleContact
{
public:
	P6Particle* particles[2];
    float restitution;
	MyVector contactNormal;
    float depth;
    float getSeparatingSpeed() {
        MyVector velocity = particles[0]->Velocity;
        if (particles[1]) {
            velocity = velocity - particles[1]->Velocity;
        }
        return velocity.Dot(contactNormal);
    }
protected:
    void ResolveVelocity(float time) {
		float separatingSpeed = getSeparatingSpeed();

        if (separatingSpeed > 0) return;
        float newSS = -separatingSpeed * restitution;
        float deltaSpeed = newSS - separatingSpeed;
        float totalMass = (float)1 / particles[0]->mass;
        if (particles[1]) {
            totalMass += (float)1 / particles[1]->mass;
        }
		if (totalMass <= 0) return;
		float impulse_mag = deltaSpeed / totalMass;
		MyVector Impulse = contactNormal * impulse_mag;
		MyVector V_a = Impulse * ((float)1 / particles[0]->mass);
        particles[0]->Velocity = particles[0]->Velocity + V_a;
        if (particles[1]) {
            MyVector V_b = Impulse * ((float)1 / particles[1]->mass);
            particles[1]->Velocity = particles[1]->Velocity - V_b;
		}
    }

    void ResolveInterpenetration(float time) {
        if (depth <= 0) return;
        float totalMass = (float)1 / particles[0]->mass;
        if(particles[1]) {
            totalMass += (float)1 / particles[1]->mass;
		}
        if (totalMass <= 0) return;
		float totalMoveByMass = depth / totalMass;
		MyVector moveByMass = contactNormal * totalMoveByMass;
		MyVector P_a = moveByMass * ((float)1 / particles[0]->mass);
        particles[0]->Position = particles[0]->Position + P_a;
        if (particles[1]) {
            MyVector P_b = moveByMass * (-(float)1 / particles[1]->mass);
            particles[1]->Position = particles[1]->Position + P_b;
        }

        depth = 0;
    }

public: 
    void Resolve(float time) {
        ResolveVelocity(time);
        ResolveInterpenetration(time);
	}
};

class ForceGenerator
{
public:
    virtual void UpdateForce(P6Particle* p, float time) {
        p->AddForce (MyVector{ 0, 0, 0 });
    }
};

class GravityForceGenerator : public ForceGenerator
{
private:
    MyVector Gravity = MyVector{ 0, 9.8f, 0 };
public:
    GravityForceGenerator(const MyVector gravity) : Gravity(gravity) {}
    void UpdateForce(P6Particle* particle, float time) override {
        if (particle->mass <= 0) return;
        MyVector force = Gravity * particle->mass;
        particle->AddForce(force);
    }
};

class DragForceGenerator : public ForceGenerator
{
private:
    float k1 = 0.74f; // friction
	float k2 = 0.57f; // drag
public:
    DragForceGenerator() {}
	DragForceGenerator(float _k1, float _k2) : k1(_k1), k2(_k2) {}
    void UpdateForce (P6Particle* particle, float time) override {
        MyVector force = MyVector{ 0, 0, 0 };
		MyVector currV = particle->Velocity;

		float mag = currV.Magnitude();
		if (mag <= 0) return;

		float dragF = (k1 * mag) + (k2 * mag);
        MyVector dir = currV.Normalize();

        particle->AddForce(dir * -dragF);
	}
};

class AnchoredSpring : public ForceGenerator
{
private: 
    MyVector anchorPoint;
    float springConstant = 0;
	float restLength = 0;

public: 
	AnchoredSpring(MyVector pos, float _springConst, float _restLen) : anchorPoint(pos), springConstant(_springConst), restLength(_restLen) {}

    void UpdateForce (P6Particle* particle, float time) override {
		MyVector pos = particle->Position;
        MyVector force = pos - anchorPoint;
		float mag = force.Magnitude();
		float springforce = -springConstant * abs(mag - restLength);
        force = force.Normalize();
        force = force * springforce;
		particle->AddForce(force);
    }
};

class ParticleSpring : public ForceGenerator
{
private:
    P6Particle* otherParticle;
    float springConstant;
    float restLength;
public:
    ParticleSpring(P6Particle* particle, float _springConst, float _restLen) : otherParticle(particle), springConstant(_springConst), restLength(_restLen) {

    }

    void UpdateForce(P6Particle* particle, float time) override {
        MyVector pos = particle->Position;
        MyVector force = pos - otherParticle->Position;
        float mag = force.Magnitude();
        float springforce = -springConstant * abs(mag - restLength);
        force = force.Normalize();
        force = force * springforce;
        particle->AddForce(force);
    }
};

class ParticleLink : public ForceGenerator
{
public:
    P6Particle* particles[2];
    virtual ParticleContact* GetContact() { return nullptr; };

protected: 
    float CurrentLength() {
        MyVector ret = particles[0]->Position - particles[1]->Position;
        return ret.Magnitude();
	}

};

class Rod : public ParticleLink
{
public: 
    float length = 1;
    float restitution = 0;

    ParticleContact* GetContact() override {
		float currLen = CurrentLength();
        if (currLen <= length) {
            return nullptr;
        }
		ParticleContact* ret = new ParticleContact();
        ret->particles[0] = particles[0];
        ret->particles[1] = particles[1];
        MyVector dir = (particles[1]->Position - particles[0]->Position).Normalize();

        if (currLen > length) {
            ret->contactNormal = dir;
            ret->depth = currLen - length;
        }
        else {
            ret->contactNormal = dir * -1.0f;
            ret->depth = length - currLen;
        }
        ret->restitution = restitution;
		return ret;
    }
};

class ContactResolver
{
protected: 
    unsigned current_iterations = 0;
public: 
    unsigned max_iterations;
    unsigned iterationsUsed = 0;
	ContactResolver(unsigned _maxiterations) : max_iterations(_maxiterations) {}
    void ResolveContacts(std::vector<ParticleContact*>& contacts, float time) {
        current_iterations = 0;

        while (current_iterations < max_iterations) {
            // 1. Find the contact with the worst (most negative) separating speed
            float maxSeverity = 0.0f;
            int worstIndex = -1;

            for (int i = 0; i < (int)contacts.size(); i++) {
                float sepSpeed = contacts[i]->getSeparatingSpeed();
                if (sepSpeed < maxSeverity) {
                    maxSeverity = sepSpeed;
                    worstIndex = i;
                }
            }

            // 2. If nothing is colliding (all separating speeds >= 0), we're done
            if (worstIndex == -1) {
                break;
            }

            // 3. Resolve the worst one
            contacts[worstIndex]->Resolve(time);

            current_iterations++;
        }
    }
};

class ForceRegistry
{
protected:
    struct ParticleForceRegistry
    {
        P6Particle* particle;
        ForceGenerator* generator;
    };

    std::list<ParticleForceRegistry> Registry;
public:
    void Add(P6Particle* particle, ForceGenerator* generator) {
        ParticleForceRegistry toAdd;

        toAdd.particle = particle;
        toAdd.generator = generator;

        Registry.push_back(toAdd);
    }

    void Remove(P6Particle* particle, ForceGenerator* generator) {
        Registry.remove_if(
            [particle, generator](ParticleForceRegistry reg) {
                return reg.particle == particle && reg.generator == generator;
            }
        );
    }

    void Clear() {
        Registry.clear();
    }

    void updateForces(float time) {
        for (std::list<ParticleForceRegistry>::iterator i = Registry.begin();
            i != Registry.end();
            i++) {
            i->generator->UpdateForce(i->particle, time);
        }
    }
};

namespace P6 {
    class PhysicsWorld
    {
    protected:
        ContactResolver contactResolver = ContactResolver(20);
        void GenerateContacts() {
            Contacts.clear();
            for (std::list<ParticleLink*>::iterator l = Links.begin();
                l != Links.end();
                l++) {
                ParticleContact* contact = (*l)->GetContact();
                if (contact) {
                    Contacts.push_back(contact);
                }
		    }
        }
    public:
		ForceRegistry forceRegistry;
        std::list<P6Particle*> Particles;
		std::list<ParticleLink*> Links;
		std::vector<ParticleContact*> Contacts;

        void AddContact(P6Particle* P1, P6Particle* P2, float restitution, MyVector contactNormal) {
			ParticleContact* toAdd = new ParticleContact();

            toAdd->particles[0] = P1;
            toAdd->particles[1] = P2;
            toAdd->restitution = restitution;
            toAdd->contactNormal = contactNormal;
			Contacts.push_back(toAdd);
        }

        void AddParticle(P6Particle* toAdd) {
            Particles.push_back(toAdd);
			forceRegistry.Add(toAdd, &Gravity);
            forceRegistry.Add(toAdd, &Drag);
			//forceRegistry.Add(toAdd, &Spring);

        }

        void Update(float time) {
			UpdateParticleList();
			forceRegistry.updateForces(time);

            for (std::list<P6Particle*>::iterator p = Particles.begin();
                p != Particles.end();
                p++
                )
            {
                (*p)->Update(time);
            }

            GenerateContacts();

            if (Contacts.size() > 0) {
                contactResolver.ResolveContacts(Contacts, time);
			}
        }

    private: // FORCE GENERATORS HERE, CHANGE PHYSICS
		GravityForceGenerator Gravity = GravityForceGenerator(MyVector{ 0, -9.8f, 0 });
        void UpdateParticleList() {
			Particles.remove_if([](P6Particle* p) { return p->isDestroyed(); });
        }
		DragForceGenerator Drag = DragForceGenerator(0.14, 0.1);
		AnchoredSpring Spring = AnchoredSpring(MyVector{ 0, 0.5 , 0}, 10, 0.01);
    };
}


bool AtCenter(P6Particle& p, float threshold = 0.1f) {
    return (p.Position.x > -threshold && p.Position.x < threshold &&
        p.Position.y > -threshold && p.Position.y < threshold);
}

void DrawLine(GLuint shaderProg, GLuint transformLoc, MyVector from, MyVector to) {
    float vertices[] = {
        from.x, from.y, from.z,
        to.x,   to.y,   to.z
    };

    GLuint lineVAO, lineVBO;
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glm::mat4 identity = glm::mat4(1.0f);
    glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(identity));

    GLuint colorLoc = glGetUniformLocation(shaderProg, "color");
    glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);

    glDrawArrays(GL_LINES, 0, 2);

    glDeleteBuffers(1, &lineVBO);
    glDeleteVertexArrays(1, &lineVAO);
}

int main(void)
{
    GLFWwindow* window;

    std::fstream vertSrc("Shaders/sample.vert");
    std::stringstream vertBuff;
    vertBuff << vertSrc.rdbuf();
    std::string vertS = vertBuff.str();
    const char* v = vertS.c_str();

    std::fstream fragSrc("Shaders/sample.frag");
    std::stringstream fragBuff;
    fragBuff << fragSrc.rdbuf();
    std::string fragS = fragBuff.str();
    const char* f = fragS.c_str();

	using clock = std::chrono::high_resolution_clock;
	auto curr_time = clock::now();
	auto prev_time = curr_time;
	std::chrono::nanoseconds curr_ns(0);

	P6::PhysicsWorld pWorld = P6::PhysicsWorld();

    P6Particle anchor;
    anchor.Position = MyVector{ 0, 0.5f, 0 };
    anchor.mass = std::numeric_limits<float>::max();

    P6Particle sphere;
    //sphere.Velocity = MyVector{ -3, 0, 0 }; // negative = left
    sphere.Color = glm::vec3(0.4f, 0.6f, 0);
    sphere.Acceleration = MyVector{ 0, 0, 0 };
	sphere.mass = 0.01f; // small so chain can be seen
	sphere.Position = MyVector{ 0, 0.55f, 0 };
	pWorld.AddParticle(&sphere);

    Rod* chainRod = new Rod();
    chainRod->particles[0] = &sphere;
    chainRod->particles[1] = &anchor;
    chainRod->length = 0.6f; 
    chainRod->restitution = 0.0f;  

    pWorld.Links.push_back(chainRod);

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "Assignment4 - Dayrit, Ryan Nathan A.", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    gladLoadGL();

	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &v, NULL);
	glCompileShader(vertexShader);

	GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragShader, 1, &f, NULL);
	glCompileShader(fragShader);

	GLuint shaderProg = glCreateProgram();
	glAttachShader(shaderProg, vertexShader);
	glAttachShader(shaderProg, fragShader);

    std::string path = "3D/sphere.obj";
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warning, error;
    tinyobj::attrib_t attributes;

    
    bool success = tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error, path.c_str());

    std::vector<GLuint> mesh_indices;

    for (int i = 0; i < shapes[0].mesh.indices.size(); i++) {
        mesh_indices.push_back(shapes[0].mesh.indices[i].vertex_index);
    }

    GLuint VAO, VBO, EBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

	glLinkProgram(shaderProg);
    glDeleteShader(vertexShader);
    glDeleteShader(fragShader);
	glUseProgram(shaderProg);
    glm::mat4 transform = glm::mat4(1.0f);
    transform = glm::scale(transform, glm::vec3(0.2f, 0.2f, 0.2f));
    GLuint transformLoc = glGetUniformLocation(shaderProg, "transform");
    glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
        attributes.vertices.size() * sizeof(float),
        attributes.vertices.data(),
        GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        mesh_indices.size() * sizeof(GLuint),
        mesh_indices.data(),
        GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);


    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);



		curr_time = clock::now();
        // duration check
		auto dur = std::chrono::duration_cast<std::chrono::nanoseconds> (curr_time - prev_time);
		prev_time = curr_time;

		curr_ns += dur;

        if (curr_ns >= timestep) {
            constexpr float timestep_sec = timestep.count() / (float)(1E09);

            std::cout << "P6 Update" << std::endl;
			pWorld.Update(timestep_sec);

            /*
            if (AtCenter(sphere)) {
                sphere.Destroy();
			}
            */
            if (sphere.Position.x <= -0.9f || sphere.Position.x >= 0.9f) {
                sphere.Velocity.x = -sphere.Velocity.x; 
            }
            if (sphere.Position.y <= -0.89f || sphere.Position.y >= 0.89f) {
                sphere.Velocity.y = -sphere.Velocity.y; 
            }
            curr_ns -= timestep;
        }

        sphere.Draw(shaderProg, VAO, transformLoc, mesh_indices);
		DrawLine(shaderProg, transformLoc, sphere.Position, MyVector{ 0, 0.5f, 0 }); // same point where anchor is located
        std::cout << "Normal Update\n";


        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }



    glfwTerminate();
    return 0;
}