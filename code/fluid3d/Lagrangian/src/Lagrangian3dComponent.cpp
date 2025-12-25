/**
 * Lagrangian3dComponent.cpp: 3D���������������ʵ���ļ�
 * ʵ������ĳ�ʼ�����������Ⱦ����
 */

#include "Lagrangian3dComponent.h"

namespace FluidSimulation
{
    namespace Lagrangian3d
    {
        /**
         * �ر�������ͷ���Դ
         */
        void Lagrangian3dComponent::shutDown()
        {
            delete renderer, solver, ps;
            renderer = NULL;
            solver = NULL;
            ps = NULL;
        }

        /**
         * ��ʼ�����
         * ������Ⱦ��������ϵͳ�������
         */
        void Lagrangian3dComponent::init()
        {
            if (renderer != NULL || solver != NULL || ps != NULL)
            {
                shutDown();
            }
            Glb::Timer::getInstance().clear();

            renderer = new Renderer();
            renderer->init();

            ps = new ParticleSystem3d();
            ps->setContainerSize(glm::vec3(0.0, 0.0, 0.0), glm::vec3(1, 1, 1));
            
            auto isValidBlock = [](const Lagrangian3dPara::FluidBlock& b) {
                if (!(b.upperCorner.x > b.lowerCorner.x && b.upperCorner.y > b.lowerCorner.y && b.upperCorner.z > b.lowerCorner.z))
                {
                    return false;
                }
                if (!(b.particleSpace > 0.0f))
                {
                    return false;
                }
                return true;
            };

            auto defaultBlock = []() {
                Lagrangian3dPara::FluidBlock b;
                b.lowerCorner = glm::vec3(0.1f, 0.1f, 0.55f);
                b.upperCorner = glm::vec3(0.9f, 0.9f, 0.9f);
                b.initVel = glm::vec3(0.0f, 0.0f, 0.0f);
                b.particleSpace = 0.04f;
                return b;
            };

            bool hasValidBlock = false;
            for (int i = 0; i < Lagrangian3dPara::fluidBlocks.size(); i++)
            {
                if (isValidBlock(Lagrangian3dPara::fluidBlocks[i]))
                {
                    hasValidBlock = true;
                    break;
                }
            }
            if (!hasValidBlock)
            {
                Lagrangian3dPara::fluidBlocks.clear();
                Lagrangian3dPara::fluidBlocks.push_back(defaultBlock());
            }

            for (int i = 0; i < Lagrangian3dPara::fluidBlocks.size(); i++) {
                if (!isValidBlock(Lagrangian3dPara::fluidBlocks[i]))
                {
                    continue;
                }
                ps->addFluidBlock(Lagrangian3dPara::fluidBlocks[i].lowerCorner, Lagrangian3dPara::fluidBlocks[i].upperCorner,
                    Lagrangian3dPara::fluidBlocks[i].initVel, Lagrangian3dPara::fluidBlocks[i].particleSpace);
            }

            if (ps->particles.empty())
            {
                Lagrangian3dPara::fluidBlocks.clear();
                Lagrangian3dPara::fluidBlocks.push_back(defaultBlock());
                ps->addFluidBlock(Lagrangian3dPara::fluidBlocks[0].lowerCorner, Lagrangian3dPara::fluidBlocks[0].upperCorner,
                    Lagrangian3dPara::fluidBlocks[0].initVel, Lagrangian3dPara::fluidBlocks[0].particleSpace);
            }

            ps->updateBlockInfo();
            Glb::Logger::getInstance().addLog("3d Particle System initialized. particle num: " + std::to_string(ps->particles.size()));

            solver = new Solver(*ps);
        }

        void Lagrangian3dComponent::simulate()
        {
            for (int i = 0; i < Lagrangian3dPara::substep; i++)
            {
                ps->updateBlockInfo();
                solver->solve();
            }
        }

        GLuint Lagrangian3dComponent::getRenderedTexture()
        {
            renderer->draw(*ps);
            return renderer->getRenderedTexture();
        }
    }
}
