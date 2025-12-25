/**
 * Lagrangian2dComponent.cpp: 2D���������������ʵ���ļ�
 * ʵ������ĳ�ʼ�����������Ⱦ����
 */

#include "Lagrangian2dComponent.h"

namespace FluidSimulation
{
    namespace Lagrangian2d
    {
        /**
         * �ر�������ͷ���Դ
         */
        void Lagrangian2dComponent::shutDown()
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
        void Lagrangian2dComponent::init()
        {
            if (renderer != NULL || solver != NULL || ps != NULL)
            {
                shutDown();
            }

            Glb::Timer::getInstance().clear();

            // initialize renderer
            renderer = new Renderer();
            renderer->init();

            // initialize particle system
            // set the container's size
            ps = new ParticleSystem2d();
            ps->setContainerSize(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f));

            auto isValidBlock = [](const Lagrangian2dPara::FluidBlock& b) {
                if (!(b.upperCorner.x > b.lowerCorner.x && b.upperCorner.y > b.lowerCorner.y))
                {
                    return false;
                }
                if (!(b.particleSpace > 0.0f))
                {
                    return false;
                }
                return true;
            };

            auto defaultFountainBlock = []() {
                Lagrangian2dPara::FluidBlock b;
                b.lowerCorner = glm::vec2(-0.9f, -1.0f);
                b.upperCorner = glm::vec2(0.9f, -0.6f);
                b.initVel = glm::vec2(0.0f, 0.0f);
                b.particleSpace = 0.02f;
                return b;
            };

            bool hasValidBlock = false;
            for (int i = 0; i < Lagrangian2dPara::fluidBlocks.size(); i++)
            {
                if (isValidBlock(Lagrangian2dPara::fluidBlocks[i]))
                {
                    hasValidBlock = true;
                    break;
                }
            }
            if (!hasValidBlock)
            {
                Lagrangian2dPara::fluidBlocks.clear();
                Lagrangian2dPara::fluidBlocks.push_back(defaultFountainBlock());
            }

            for (int i = 0; i < Lagrangian2dPara::fluidBlocks.size(); i++) {
                if (!isValidBlock(Lagrangian2dPara::fluidBlocks[i]))
                {
                    continue;
                }
                ps->addFluidBlock(Lagrangian2dPara::fluidBlocks[i].lowerCorner, Lagrangian2dPara::fluidBlocks[i].upperCorner,
                    Lagrangian2dPara::fluidBlocks[i].initVel, Lagrangian2dPara::fluidBlocks[i].particleSpace);
            }

            if (ps->particles.empty())
            {
                Lagrangian2dPara::fluidBlocks.clear();
                Lagrangian2dPara::fluidBlocks.push_back(defaultFountainBlock());
                ps->addFluidBlock(Lagrangian2dPara::fluidBlocks[0].lowerCorner, Lagrangian2dPara::fluidBlocks[0].upperCorner,
                    Lagrangian2dPara::fluidBlocks[0].initVel, Lagrangian2dPara::fluidBlocks[0].particleSpace);
            }

            ps->addSolidDisk(glm::vec2(0.0f, -0.2f), 0.15f, glm::vec2(0.0f, 0.0f), 30.0f, 0.02f);

            ps->updateBlockInfo();

            Glb::Logger::getInstance().addLog("2d Particle System initialized. particle num: " + std::to_string(ps->particles.size()));

            solver = new Solver(*ps);
        }

        void Lagrangian2dComponent::simulate()
        {
            for (int i = 0; i < Lagrangian2dPara::substep; i++)
            {
                ps->updateBlockInfo();
                solver->solve();
            }
        }

        GLuint Lagrangian2dComponent::getRenderedTexture()
        {
            renderer->draw(*ps);
            return renderer->getRenderedTexture();
        }
    }
}
