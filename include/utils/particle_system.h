#pragma once

#include <glm/gtc/type_ptr.hpp>

#include <glm/gtx/compatibility.hpp>


struct ParticleProps
{
	btVector3 Position;
	btVector3 PositionVariation;
	btVector3 Velocity, VelocityVariation;
	glm::vec4 ColorBegin, ColorEnd;
	float SizeBegin, SizeEnd, SizeVariation;
	float LifeTime = 1.0f;
};

class ParticleSystem
{
public:
	ParticleSystem()
	{
		m_ParticlePool.resize(1000);
	}

	void OnUpdate(GLfloat ts)
	{
		for (auto& particle : m_ParticlePool)
		{
			if (!particle.Active)
				continue;

			if (particle.LifeRemaining <= 0.0f)
			{
				particle.Active = false;
				continue;
			}

			particle.LifeRemaining -= ts;
			particle.Velocity[1] -= ts*9.8f;
			particle.Position += particle.Velocity * (float)ts;
			particle.Rotation = glm::quat(glm::vec3(
				particle.Velocity.angle(btVector3(particle.Rotation[0],0.0f,0.0f)),
				particle.Velocity.angle(btVector3(0.0f,particle.Rotation[1],0.0f)),
				particle.Velocity.angle(btVector3(0.0f,0.0f,particle.Rotation[2]))
				));
		}
	}

	void OnRender(Shader& shader, Camera& camera)
	{
		if (!m_QuadVA)
		{
			float vertices[] = {
				-0.5f, -0.5f, 0.0f,
				0.5f, -0.5f, 0.0f,
				0.5f,  0.5f, 0.0f,
				-0.5f,  0.5f, 0.0f
			};
	

			glGenVertexArrays(1, &m_QuadVA);
			glBindVertexArray(m_QuadVA);

			glGenBuffers(1, &quadVB);
			glBindBuffer(GL_ARRAY_BUFFER, quadVB);
			glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

			GLuint indices[] = {
				0, 1, 2,
				2, 3, 0
			};

			glGenBuffers(1, &quadIB);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadIB);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

			glEnableVertexAttribArray(0);
    		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);

    		glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind
    		glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO


			m_ParticleShaderColor = glGetUniformLocation(shader.Program, "particleColor");
		}


		for (auto& particle : m_ParticlePool)
		{
			if (!particle.Active)
				continue;

			// Fade away particles
			float life = particle.LifeRemaining / particle.LifeTime;
			glm::vec4 color = glm::lerp(particle.ColorEnd, particle.ColorBegin, life);
			//color.a = color.a * life;

			float size = glm::lerp(particle.SizeEnd, particle.SizeBegin, life);
			
			glm::mat4 particleModelMatrix = glm::mat4(1.0f);
			glm::mat3 particleNormalMatrix = glm::mat3(1.0f);
			particleModelMatrix = glm::translate(particleModelMatrix, glm::vec3(particle.Position.getX(),particle.Position.getY(),particle.Position.getZ()));
			particleModelMatrix = glm::rotate(particleModelMatrix, particle.Rotation[0], glm::vec3(1.0f,0.0f,0.0f));
			particleModelMatrix = glm::rotate(particleModelMatrix, particle.Rotation[1], glm::vec3(0.0f,1.0f,0.0f));
			particleModelMatrix = glm::rotate(particleModelMatrix, particle.Rotation[2], glm::vec3(0.0f,0.0f,1.0f));
			particleModelMatrix = glm::scale(particleModelMatrix, glm::vec3(size));
			particleNormalMatrix = glm::transpose(glm::mat3(camera.GetViewMatrix()*particleModelMatrix));
			particleNormalMatrix = glm::inverse(particleNormalMatrix);
			glUniformMatrix4fv(glGetUniformLocation(shader.Program, "modelMatrix"), 1, GL_FALSE, glm::value_ptr(particleModelMatrix));
			glUniformMatrix3fv(glGetUniformLocation(shader.Program, "normalMatrix"), 1, GL_FALSE, glm::value_ptr(particleNormalMatrix));


			glUniform4fv(m_ParticleShaderColor, 1, glm::value_ptr(color));
			glBindVertexArray(m_QuadVA);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);

			particleModelMatrix = glm::mat4(1.0f);
		}
	}

	void Emit(const ParticleProps& particleProps)
	{
		Particle& particle = m_ParticlePool[m_PoolIndex];
		particle.Active = true;

		// Position
		particle.Position = particleProps.Position;
		particle.Position[0] += particleProps.PositionVariation[0] * (rand_float() - 0.5f);
		particle.Position[1] += particleProps.PositionVariation[1] * (rand_float() - 0.5f);
		particle.Position[2] += particleProps.PositionVariation[2] * (rand_float() - 0.5f);

		// Velocity
		particle.Velocity = particleProps.Velocity;
		particle.Velocity[0] += particleProps.VelocityVariation[0] * (rand_float() - 0.5f);
		particle.Velocity[1] += particleProps.VelocityVariation[1] * (rand_float() - 0.5f);
		particle.Velocity[2] += particleProps.VelocityVariation[2] * (rand_float() - 0.5f);

		particle.Rotation = glm::vec3(
			particle.Velocity.angle(btVector3(1.0f,0.0f,0.0f)),
			particle.Velocity.angle(btVector3(0.0f,1.0f,0.0f)),
			particle.Velocity.angle(btVector3(0.0f,0.0f,1.0f))
		);
		
		// Color
		particle.ColorBegin = particleProps.ColorBegin;
		particle.ColorEnd = particleProps.ColorEnd;

		particle.LifeTime = particleProps.LifeTime;
		particle.LifeRemaining = particleProps.LifeTime;
		particle.SizeBegin = particleProps.SizeBegin + particleProps.SizeVariation * (rand_float() - 0.5f);
		particle.SizeEnd = particleProps.SizeEnd;

		m_PoolIndex = ++m_PoolIndex % m_ParticlePool.size();
	}

	void Destroy()
	{
		glDeleteVertexArrays(1, &m_QuadVA);
    	glDeleteBuffers(1, &quadVB);
    	glDeleteBuffers(1, &quadIB);
	}


private:
	struct Particle
	{
		btVector3 Position;
		btVector3 Velocity;
		glm::vec4 ColorBegin, ColorEnd;
		glm::quat Rotation = glm::vec3(0.0f);
		GLfloat SizeBegin, SizeEnd;

		GLfloat LifeTime = 1.0f;
		GLfloat LifeRemaining = 0.0f;

		GLboolean Active = false;
	};
	std::vector<Particle> m_ParticlePool;
	GLuint m_PoolIndex = 0;

	GLuint m_QuadVA = 0;
	GLuint quadVB, quadIB;
	GLint m_ParticleShaderViewProj, m_ParticleShaderTransform, m_ParticleShaderColor;
};