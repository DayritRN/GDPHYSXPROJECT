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

using namespace std::chrono_literals;

constexpr std::chrono::nanoseconds timestep(12ms);

class MyVector
{
public:
    float x = 0;
    float y = 0;
	float z = 0;

    float Magnitude();
    MyVector Direction();

    MyVector operator+(const MyVector& other) const {
        return { x + other.x, y + other.y, z + other.z };
    }
    MyVector operator*(float scalar) const {
        return { x * scalar, y * scalar, z * scalar };
    }
    friend MyVector operator*(float scalar, const MyVector& vec) {
        return { vec.x * scalar, vec.y * scalar, vec.z * scalar };
    }
};

class P6Particle
{
public:
    float mass = 0;
    glm::vec3 Color = glm::vec3(1.0f);
    MyVector Position;
    MyVector Velocity;
    MyVector Acceleration;


protected:
    void UpdatePosition(float time) {
        Position = Position + (Velocity * time) + ((1.0f / 2.0f) * (Acceleration * time * time));
    }
    void UpdateVelocity(float time) {
        Velocity = Velocity + (Acceleration * time);
    }

public:
    void Update(float time) {
        UpdatePosition(time);
        UpdateVelocity(time);
    }
    void Draw(GLuint shaderProg, GLuint VAO, GLuint transformLoc, std::vector<GLuint>& mesh_indices) {
        // update transform from position
        glm::mat4 transform = glm::mat4(1.0f);
        transform = glm::translate(transform, glm::vec3(Position.x, Position.y, Position.z));
        transform = glm::scale(transform, glm::vec3(0.1f, 0.1f, 0.1f));
        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

        // send color to shader
        GLuint colorLoc = glGetUniformLocation(shaderProg, "color");
        glUniform3fv(colorLoc, 1, glm::value_ptr(Color));

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)mesh_indices.size(), GL_UNSIGNED_INT, 0);
    } 
};


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


    P6Particle sphere;
    sphere.Velocity = MyVector{ -3, 0, 0 };   // negative = left
    sphere.Color = glm::vec3(0.9f, 0, 0);
    sphere.Acceleration = MyVector{ 0, 0, 0 };

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(800, 800, "RN Dayrit", NULL, NULL);
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
            sphere.Update(timestep_sec);
            if (sphere.Position.x <= -1.0f || sphere.Position.x >= 1.0f) {
                sphere.Velocity.x = -sphere.Velocity.x; 
            }
            if (sphere.Position.y <= -1.0f || sphere.Position.y >= 1.0f) {
                sphere.Velocity.y = -sphere.Velocity.y; 
            }
            curr_ns -= timestep;
        }

        sphere.Draw(shaderProg, VAO, transformLoc, mesh_indices);
        std::cout << "Normal Update\n";


        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }



    glfwTerminate();
    return 0;
}