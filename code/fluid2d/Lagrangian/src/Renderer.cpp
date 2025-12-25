#include "Lagrangian/include/Renderer.h"

#include <iostream>
#include <fstream>
#include "Configure.h"

// ������Ⱦ������Ҫʵ��,�漰OpenGL������ʹ��
// ���Ķ����ļ�֮ǰ,��ȷ������OpenGL�л����˽�

namespace FluidSimulation
{

    namespace Lagrangian2d
    {

        Renderer::Renderer()
        {
        }

        // ��ʼ����Ⱦ��
        int32_t Renderer::init()
        {
            extern std::string shaderPath;

            // ���ز�����������ɫ��
            std::string particleVertShaderPath = shaderPath + "/DrawParticles2d.vert";
            std::string particleFragShaderPath = shaderPath + "/DrawParticles2d.frag";
            shader = new Glb::Shader();
            shader->buildFromFile(particleVertShaderPath, particleFragShaderPath);

            // ���ɶ����������(VAO)
            glGenVertexArrays(1, &VAO);
            // ����λ�õĶ��㻺�����(VBO)
            glGenBuffers(1, &positionVBO);
            // �����ܶȵĶ��㻺�����(VBO)
            glGenBuffers(1, &densityVBO);

            // ����֡�������(FBO)
            glGenFramebuffers(1, &FBO);
            // ��֡����
            glBindFramebuffer(GL_FRAMEBUFFER, FBO);

            // ���ɲ���������
            glGenTextures(1, &textureID);
            glBindTexture(GL_TEXTURE_2D, textureID);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glBindTexture(GL_TEXTURE_2D, 0);

            // ���������ӵ�֡����
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureID, 0);

            // ������Ⱦ�������(RBO)
            glGenRenderbuffers(1, &RBO);
            glBindRenderbuffer(GL_RENDERBUFFER, RBO);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, imageWidth, imageHeight);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);

            // ��RBO���ӵ�֡����
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            {
                Glb::Logger::getInstance().addLog("Error: Framebuffer is not complete!");
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            // �����ӿڴ�С
            glViewport(0, 0, imageWidth, imageHeight);

            return 0;
        }

        // ��������ϵͳ
        void Renderer::draw(ParticleSystem2d& ps)
        {
            std::vector<ParticleInfo2d> renderParticles;
            size_t solidPointNum = 0;
            for (const auto& body : ps.solids)
            {
                solidPointNum += body.worldPoints.size();
            }
            renderParticles.reserve(ps.particles.size() + solidPointNum);
            renderParticles.insert(renderParticles.end(), ps.particles.begin(), ps.particles.end());
            for (const auto& body : ps.solids)
            {
                for (const auto& sp : body.worldPoints)
                {
                    ParticleInfo2d p;
                    p.position = sp;
                    p.velocity = glm::vec2(0.0f);
                    p.accleration = glm::vec2(0.0f);
                    p.density = 5000.0f;
                    p.pressure = 0.0f;
                    p.pressDivDens2 = 0.0f;
                    p.blockId = 0;
                    renderParticles.push_back(p);
                }
            }

            // ��VAO
            glBindVertexArray(VAO);

            // �󶨲�����λ��VBO
            glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
            glBufferData(GL_ARRAY_BUFFER, renderParticles.size() * sizeof(ParticleInfo2d), renderParticles.data(), GL_STATIC_DRAW);

            // ����λ������
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInfo2d), (void*)offsetof(ParticleInfo2d, position));
            glEnableVertexAttribArray(0);

            // �����ܶ�����
            glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleInfo2d), (void*)offsetof(ParticleInfo2d, density));
            glEnableVertexAttribArray(1);

            glBindVertexArray(0);

            // ������������
            particleNum = renderParticles.size();

            // ��֡���忪ʼ��Ⱦ
            glBindFramebuffer(GL_FRAMEBUFFER, FBO);

            // ��ջ�����
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // ׼����Ⱦ
            glBindVertexArray(VAO);
            shader->use();
            shader->setFloat("scale", ps.scale);

            // ���õ㾫��
            glEnable(GL_PROGRAM_POINT_SIZE);

            // ������������
            glDrawArrays(GL_POINTS, 0, particleNum);

            // ���֡����
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        // ��ȡ��Ⱦ���������ID
        GLuint Renderer::getRenderedTexture()
        {
            return textureID;
        }
    }

}
