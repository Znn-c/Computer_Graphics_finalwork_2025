#include "ParticleSystem2d.h"
#include <iostream>
#include "Global.h"
#include <unordered_set>
#include <algorithm>
#include <cstdint>
#include <cfloat>

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        ParticleSystem2d::ParticleSystem2d()
        {
        }

        ParticleSystem2d::~ParticleSystem2d()
        {
        }

        // ???????????????§³
        void ParticleSystem2d::setContainerSize(glm::vec2 lower, glm::vec2 upper)
        {
            // ???????
            lower *=  Lagrangian2dPara::scale;
            upper *= Lagrangian2dPara::scale;

            // ??????,?????????????????
            lowerBound = lower - supportRadius + particleDiameter;
            upperBound = upper + supportRadius - particleDiameter;
            containerCenter = (lowerBound + upperBound) / 2.0f;

            glm::vec2 size = upperBound - lowerBound;

            // ?????????????§³
            blockNum.x = floor(size.x / supportRadius);
            blockNum.y = floor(size.y / supportRadius);
            blockSize = glm::vec2(size.x / blockNum.x, size.y / blockNum.y);

            // ??????????????
            blockIdOffs.resize(9);
            int p = 0;
            for (int j = -1; j <= 1; j++)
            {
                for (int i = -1; i <= 1; i++)
                {
                    blockIdOffs[p] = blockNum.x * j + i;
                    p++;
                }
            }

            // ???????????
            particles.clear();
            solids.clear();
        }

        // ?????????
        int32_t ParticleSystem2d::addFluidBlock(glm::vec2 lowerCorner, glm::vec2 upperCorner, glm::vec2 v0, float particleSpace)
        {
            // ???????
            lowerCorner *= Lagrangian2dPara::scale;
            upperCorner *= Lagrangian2dPara::scale;

            glm::vec2 size = upperCorner - lowerCorner;

            // ?????
            if (lowerCorner.x < lowerBound.x ||
                lowerCorner.y < lowerBound.y ||
                upperCorner.x > upperBound.x ||
                upperCorner.y > upperBound.y)
            {
                return 0;
            }

            // ????????????
            glm::uvec2 particleNum = glm::uvec2(size.x / particleSpace, size.y / particleSpace);
            std::vector<ParticleInfo2d> tempParticles(particleNum.x * particleNum.y);

            // ?????????,????????¦Ë??????
            Glb::RandomGenerator rand;
            int p = 0;
            // ?????????????????
            for (int idX = 0; idX < particleNum.x; idX++)
            {
                for (int idY = 0; idY < particleNum.y; idY++)
                {
                    // ?????????????????????
                    float x = (idX + rand.GetUniformRandom()) * particleSpace;
                    float y = (idY + rand.GetUniformRandom()) * particleSpace;

                    // ????????????
                    tempParticles[p].position = lowerCorner + glm::vec2(x, y);
                    tempParticles[p].blockId = getBlockIdByPosition(tempParticles[p].position);
                    tempParticles[p].density = Lagrangian2dPara::density;
                    tempParticles[p].velocity = v0;
                    p++;
                }
            }

            // ?????????????????
            particles.insert(particles.end(), tempParticles.begin(), tempParticles.end());
            return particles.size();
        }

        int32_t ParticleSystem2d::addSolidDisk(glm::vec2 center, float radius, glm::vec2 v0, float mass, float pointSpace)
        {
            center *= Lagrangian2dPara::scale;
            radius *= Lagrangian2dPara::scale;

            if (!(radius > 0.0f) || !(pointSpace > 0.0f) || !(mass > 0.0f))
            {
                return solids.size();
            }

            if (center.x - radius < lowerBound.x || center.y - radius < lowerBound.y ||
                center.x + radius > upperBound.x || center.y + radius > upperBound.y)
            {
                return solids.size();
            }

            SolidBody2d body;
            body.position = center;
            body.velocity = v0;
            body.radius = radius;
            body.mass = mass;
            body.invMass = 1.0f / mass;
            body.inertia = 0.5f * mass * radius * radius;
            body.invInertia = body.inertia > 0.0f ? (1.0f / body.inertia) : 0.0f;

            float r2 = radius * radius;
            for (float y = -radius; y <= radius; y += pointSpace)
            {
                for (float x = -radius; x <= radius; x += pointSpace)
                {
                    glm::vec2 p(x, y);
                    float d2 = glm::dot(p, p);
                    float inner = (radius - pointSpace) > 0.0f ? (radius - pointSpace) : 0.0f;
                    float inner2 = inner * inner;
                    if (d2 <= r2 && d2 >= inner2)
                    {
                        body.localPoints.push_back(p);
                    }
                }
            }

            solids.push_back(std::move(body));
            updateSolidWorldPoints();
            return solids.size();
        }

        // ????¦Ë??????ID
        uint32_t ParticleSystem2d::getBlockIdByPosition(glm::vec2 position)
        {
            // ?????
            if (position.x < lowerBound.x ||
                position.y < lowerBound.y ||
                position.x > upperBound.x ||
                position.y > upperBound.y)
            {
                return -1;
            }

            // ?????????
            glm::vec2 deltePos = position - lowerBound;
            uint32_t c = floor(deltePos.x / blockSize.x);
            uint32_t r = floor(deltePos.y / blockSize.y);
            return r * blockNum.x + c;
        }

        // ????????
        void ParticleSystem2d::updateBlockInfo()
        {
            // ?????ID?????????????
            std::sort(particles.begin(), particles.end(),
                      [=](ParticleInfo2d &first, ParticleInfo2d &second)
                      {
                          return first.blockId < second.blockId;
                      });

            // ?????????????????¦¶
            blockExtens = std::vector<glm::uvec2>(blockNum.x * blockNum.y, glm::uvec2(0, 0));
            if (particles.empty())
            {
                return;
            }

            uint32_t curBlockId = particles[0].blockId;
            uint32_t left = 0;
            uint32_t right = 0;
            for (right = 0; right < particles.size(); right++)
            {
                if (particles[right].blockId != curBlockId)
                {
                    blockExtens[curBlockId] = glm::uvec2(left, right);
                    left = right;
                    curBlockId = particles[right].blockId;
                }
            }
            blockExtens[curBlockId] = glm::uvec2(left, right);
        }

        void ParticleSystem2d::updateSolidWorldPoints()
        {
            for (auto& body : solids)
            {
                float c = std::cos(body.angle);
                float s = std::sin(body.angle);
                body.worldPoints.resize(body.localPoints.size());
                for (size_t i = 0; i < body.localPoints.size(); i++)
                {
                    const glm::vec2 lp = body.localPoints[i];
                    body.worldPoints[i] = body.position + glm::vec2(c * lp.x - s * lp.y, s * lp.x + c * lp.y);
                }
            }
        }
    }
}
