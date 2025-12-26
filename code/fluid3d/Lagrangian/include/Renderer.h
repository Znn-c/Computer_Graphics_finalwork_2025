#pragma once
#ifndef __LAGRANGIAN_3D_RENDERER_H__
#define __LAGRANGIAN_3D_RENDERER_H__

#include <glm/glm.hpp>
#include <glad/glad.h>
#include "glfw3.h"
#include "Shader.h"
#include "Container.h"
#include "Camera.h"
#include "Global.h"
#include "Configure.h"
#include <Logger.h>
#include "ParticleSystem3d.h"

namespace FluidSimulation
{
    namespace Lagrangian3d
    {
        // �������շ�������Ⱦ����
        // ������ά����ϵͳ���ӻ�
        class Renderer
        {
        public:
            Renderer(){};

            void init();                          // ��ʼ����Ⱦ��
            GLuint getRenderedTexture();          // ��ȡ��Ⱦ���������ID
            void draw(ParticleSystem3d &ps);      // ��������ϵͳ

        private:
            void MakeVertexArrays();              // ������������

        private:
            // ��ɫ��������
            Glb::Shader *shader = nullptr;        // ��ɫ������
            Glb::Container *container = nullptr;   // ��������

            // OpenGL��Ⱦ����
            GLuint FBO = 0;                       // ֡�������
            GLuint RBO = 0;                       // ��Ⱦ�������
            GLuint VAO = 0;                       // �����������
            GLuint VBO = 0;                       // ���㻺�����
            GLuint textureID = 0;                 // ��Ⱦ����ID

            int32_t particleNum = 0;              // ��������
        };
    }
}

#endif
