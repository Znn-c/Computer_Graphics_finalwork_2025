/**
 * ParticleSystem2d.h: 2D����ϵͳͷ�ļ�
 * ����������ӵ�2D����������ݽṹ
 */

#pragma once
#ifndef __PARTICAL_SYSTEM_2D_H__
#define __PARTICAL_SYSTEM_2D_H__

#include <vector>
#include <list>
#include <cstdint>
#include <cfloat>
#include <cmath>
#include <glm/glm.hpp>
#include "Global.h"

#include "Configure.h"

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        /**
         * 2D������Ϣ�ṹ��
         * �洢���ӵ���������
         */
        struct ParticleInfo2d
        {
            alignas(8) glm::vec2 position;      // λ��
            alignas(8) glm::vec2 velocity;      // �ٶ�
            alignas(8) glm::vec2 accleration;    // ���ٶ�
            alignas(4) float density;            // �ܶ�
            alignas(4) float pressure;          // ѹ��
            alignas(4) float pressDivDens2;      // ѹ�������ܶȵ�ƽ��
            alignas(4) uint32_t blockId;         // ���������ID
        };

        struct SolidBody2d
        {
            std::vector<glm::vec2> localPoints;
            std::vector<glm::vec2> worldPoints;

            glm::vec2 position = glm::vec2(0.0f);
            float angle = 0.0f;

            glm::vec2 velocity = glm::vec2(0.0f);
            float angularVelocity = 0.0f;

            float radius = 0.0f;
            float mass = 1.0f;
            float invMass = 1.0f;
            float inertia = 1.0f;
            float invInertia = 1.0f;

            glm::vec2 force = glm::vec2(0.0f);
            float torque = 0.0f;
        };

        /**
         * 2D����ϵͳ��
         * �����������ӵ��������ԺͿռ�����
         */
        class ParticleSystem2d
        {
        public:
            ParticleSystem2d();
            ~ParticleSystem2d();

            void setContainerSize(glm::vec2 containerCorner, glm::vec2 containerSize);
            int32_t addFluidBlock(glm::vec2 corner, glm::vec2 size, glm::vec2 v0, float particleSpace);
            int32_t addSolidDisk(glm::vec2 center, float radius, glm::vec2 v0, float mass, float pointSpace);
            uint32_t getBlockIdByPosition(glm::vec2 position);
            void updateBlockInfo();
            void updateSolidWorldPoints();

        public:
            // ���Ӳ���
            float supportRadius = Lagrangian2dPara::supportRadius;
            float supportRadius2 = supportRadius * supportRadius;
            float particleRadius = Lagrangian2dPara::particleRadius;
            float particleDiameter = Lagrangian2dPara::particleDiameter;
            float particleVolume = particleDiameter * particleDiameter;

            // �洢ȫ��������Ϣ
            std::vector<ParticleInfo2d> particles;
            std::vector<SolidBody2d> solids;

            // ��������
            glm::vec2 lowerBound = glm::vec2(FLT_MAX);
            glm::vec2 upperBound = glm::vec2(-FLT_MAX);
            glm::vec2 containerCenter = glm::vec2(0.0f);
            float scale = Lagrangian2dPara::scale;
            
            // Block�ṹ�������ٽ�������
            glm::uvec2 blockNum = glm::uvec2(0);
            glm::vec2 blockSize = glm::vec2(0.0f);
            std::vector<glm::uvec2> blockExtens;
            std::vector<int32_t> blockIdOffs;
        };
    }
}

#endif // !PARTICAL_SYSTEM_H
