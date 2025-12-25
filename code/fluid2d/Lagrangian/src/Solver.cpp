/**
 * Solver.cpp: 2D�����������������ʵ���ļ�
 * ʵ�ֻ������ӵ�2D��������㷨
 */

#include "Lagrangian/include/Solver.h"
#include "Global.h"
#include <iostream>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{

    namespace Lagrangian2d
    {
        static inline float Poly6Kernel2d(float r2, float h, float h2)
        {
            if (r2 >= h2)
            {
                return 0.0f;
            }
            float x = h2 - r2;
            float h8 = h2 * h2 * h2 * h2;
            float c = 4.0f / (glm::pi<float>() * h8);
            return c * x * x * x;
        }

        static inline glm::vec2 SpikyGradKernel2d(const glm::vec2& r, float rLen, float h)
        {
            if (rLen <= 0.0f || rLen >= h)
            {
                return glm::vec2(0.0f);
            }
            float x = h - rLen;
            float h5 = h * h * h * h * h;
            float c = -30.0f / (glm::pi<float>() * h5);
            return c * x * x * (r / rLen);
        }

        static inline float ViscLaplacianKernel2d(float rLen, float h)
        {
            if (rLen >= h)
            {
                return 0.0f;
            }
            float h5 = h * h * h * h * h;
            float c = 40.0f / (glm::pi<float>() * h5);
            return c * (h - rLen);
        }

        /**
         * ���캯�������������ϵͳ������
         * @param ps ����ϵͳ����
         */
        Solver::Solver(ParticleSystem2d &ps) : mPs(ps)
        {
        }

        /**
         * ������巽��
         * ʵ��һ����������ķ������
         */
        void Solver::solve()
        {
            const float dt = Lagrangian2dPara::dt;
            const float restDensity = Lagrangian2dPara::density;
            const float stiffness = Lagrangian2dPara::stiffness;
            const float exponent = Lagrangian2dPara::exponent;
            const float viscosity = Lagrangian2dPara::viscosity;

            const float h = mPs.supportRadius;
            const float h2 = mPs.supportRadius2;
            const float mass = restDensity * mPs.particleVolume;

            const glm::vec2 gravity = glm::vec2(Lagrangian2dPara::gravityX, -Lagrangian2dPara::gravityY);

            const float nozzleHalfWidth = 3.0f * h;
            const float nozzleHeight = 3.0f * h;
            const float nozzleSpeed = Lagrangian2dPara::maxVelocity;
            const glm::vec2 nozzleCenter = glm::vec2(mPs.containerCenter.x, mPs.lowerBound.y + 4.0f * mPs.particleDiameter);

            const float solidStiffness = 8000.0f;
            const float solidDamping = 120.0f;

            for (auto& p : mPs.particles)
            {
                p.density = 0.0f;
                p.pressure = 0.0f;
                p.pressDivDens2 = 0.0f;
                p.accleration = glm::vec2(0.0f);
            }

            mPs.updateSolidWorldPoints();
            for (auto& body : mPs.solids)
            {
                body.force = body.mass * gravity;
                body.torque = 0.0f;
            }

            auto visitNeighbors = [&](uint32_t blockId, auto&& visitor)
            {
                if (blockId >= mPs.blockNum.x * mPs.blockNum.y)
                {
                    return;
                }
                int32_t c0 = static_cast<int32_t>(blockId % mPs.blockNum.x);
                int32_t r0 = static_cast<int32_t>(blockId / mPs.blockNum.x);
                for (int32_t dr = -1; dr <= 1; dr++)
                {
                    for (int32_t dc = -1; dc <= 1; dc++)
                    {
                        int32_t c = c0 + dc;
                        int32_t r = r0 + dr;
                        if (c < 0 || r < 0 || c >= static_cast<int32_t>(mPs.blockNum.x) || r >= static_cast<int32_t>(mPs.blockNum.y))
                        {
                            continue;
                        }
                        uint32_t nb = static_cast<uint32_t>(r) * mPs.blockNum.x + static_cast<uint32_t>(c);
                        const glm::uvec2 ext = mPs.blockExtens[nb];
                        for (uint32_t j = ext.x; j < ext.y; j++)
                        {
                            visitor(j);
                        }
                    }
                }
            };

            for (size_t i = 0; i < mPs.particles.size(); i++)
            {
                auto& pi = mPs.particles[i];
                const uint32_t myBlockId = pi.blockId;

                float dens = 0.0f;
                visitNeighbors(myBlockId,
                               [&](uint32_t j)
                               {
                                   const auto& pj = mPs.particles[j];
                                   glm::vec2 r = pi.position - pj.position;
                                   float r2 = glm::dot(r, r);
                                   dens += mass * Poly6Kernel2d(r2, h, h2);
                               });

                pi.density = (std::max)(dens, restDensity * 0.5f);
                float ratio = pi.density / restDensity;
                pi.pressure = stiffness * (std::pow(ratio, exponent) - 1.0f);
                if (pi.pressure < 0.0f)
                {
                    pi.pressure = 0.0f;
                }
                pi.pressDivDens2 = pi.pressure / (pi.density * pi.density);
            }

            for (size_t i = 0; i < mPs.particles.size(); i++)
            {
                auto& pi = mPs.particles[i];
                glm::vec2 aPressure(0.0f);
                glm::vec2 aVisc(0.0f);

                const uint32_t myBlockId = pi.blockId;
                visitNeighbors(myBlockId,
                               [&](uint32_t j)
                               {
                                   if (j == i)
                                   {
                                       return;
                                   }

                                   const auto& pj = mPs.particles[j];
                                   glm::vec2 r = pi.position - pj.position;
                                   float r2 = glm::dot(r, r);
                                   if (r2 >= h2)
                                   {
                                       return;
                                   }
                                   float rLen = std::sqrt(r2);

                                   glm::vec2 gradW = SpikyGradKernel2d(r, rLen, h);
                                   float lapW = ViscLaplacianKernel2d(rLen, h);

                                   float pressTerm = (pi.pressDivDens2 + pj.pressDivDens2);
                                   aPressure += -mass * pressTerm * gradW;

                                   aVisc += viscosity * mass * (pj.velocity - pi.velocity) / pj.density * lapW;
                               });

                glm::vec2 aSolid(0.0f);
                for (auto& body : mPs.solids)
                {
                    for (const auto& sp : body.worldPoints)
                    {
                        glm::vec2 r = pi.position - sp;
                        float r2 = glm::dot(r, r);
                        if (r2 >= h2)
                        {
                            continue;
                        }
                        float rLen = std::sqrt(r2);
                        if (rLen <= 1e-6f)
                        {
                            continue;
                        }

                        float x = h - rLen;
                        glm::vec2 n = r / rLen;

                        glm::vec2 relPos = sp - body.position;
                        glm::vec2 vSolid = body.velocity + body.angularVelocity * glm::vec2(-relPos.y, relPos.x);
                        glm::vec2 relVel = pi.velocity - vSolid;

                        glm::vec2 f = solidStiffness * x * x * n - solidDamping * x * relVel;

                        aSolid += f / mass;
                        body.force -= f;
                        body.torque -= (relPos.x * f.y - relPos.y * f.x);
                    }
                }

                pi.accleration = aPressure + aVisc + aSolid + gravity;
            }

            for (auto& p : mPs.particles)
            {
                if (p.position.y <= mPs.lowerBound.y + nozzleHeight && std::abs(p.position.x - nozzleCenter.x) <= nozzleHalfWidth)
                {
                    p.velocity.y = (std::max)(p.velocity.y, nozzleSpeed);
                }

                p.velocity += dt * p.accleration;
                float v2 = glm::dot(p.velocity, p.velocity);
                float maxV = Lagrangian2dPara::maxVelocity;
                if (v2 > maxV * maxV)
                {
                    p.velocity *= (maxV / std::sqrt(v2));
                }

                p.position += dt * p.velocity;

                const float eps = Lagrangian2dPara::eps;
                const float atten = Lagrangian2dPara::velocityAttenuation;

                if (p.position.x < mPs.lowerBound.x + eps)
                {
                    p.position.x = mPs.lowerBound.x + eps;
                    p.velocity.x *= -atten;
                }
                else if (p.position.x > mPs.upperBound.x - eps)
                {
                    p.position.x = mPs.upperBound.x - eps;
                    p.velocity.x *= -atten;
                }

                if (p.position.y < mPs.lowerBound.y + eps)
                {
                    p.position.y = mPs.lowerBound.y + eps;
                    p.velocity.y *= -atten;
                }
                else if (p.position.y > mPs.upperBound.y - eps)
                {
                    p.position.y = mPs.upperBound.y - eps;
                    p.velocity.y *= -atten;
                }

                p.blockId = mPs.getBlockIdByPosition(p.position);
            }

            for (auto& body : mPs.solids)
            {
                body.velocity += dt * body.force * body.invMass;
                body.angularVelocity += dt * body.torque * body.invInertia;

                body.velocity *= 0.999f;
                body.angularVelocity *= 0.999f;

                body.position += dt * body.velocity;
                body.angle += dt * body.angularVelocity;
            }

            mPs.updateSolidWorldPoints();

            const float eps = Lagrangian2dPara::eps;
            const float atten = Lagrangian2dPara::velocityAttenuation;
            for (auto& body : mPs.solids)
            {
                if (body.worldPoints.empty())
                {
                    continue;
                }

                glm::vec2 mn(FLT_MAX);
                glm::vec2 mx(-FLT_MAX);
                for (const auto& p : body.worldPoints)
                {
                    mn.x = (std::min)(mn.x, p.x);
                    mn.y = (std::min)(mn.y, p.y);
                    mx.x = (std::max)(mx.x, p.x);
                    mx.y = (std::max)(mx.y, p.y);
                }

                bool collided = false;
                if (mn.x < mPs.lowerBound.x + eps)
                {
                    body.position.x += (mPs.lowerBound.x + eps - mn.x);
                    body.velocity.x *= -atten;
                    collided = true;
                }
                else if (mx.x > mPs.upperBound.x - eps)
                {
                    body.position.x -= (mx.x - (mPs.upperBound.x - eps));
                    body.velocity.x *= -atten;
                    collided = true;
                }

                if (mn.y < mPs.lowerBound.y + eps)
                {
                    body.position.y += (mPs.lowerBound.y + eps - mn.y);
                    body.velocity.y *= -atten;
                    collided = true;
                }
                else if (mx.y > mPs.upperBound.y - eps)
                {
                    body.position.y -= (mx.y - (mPs.upperBound.y - eps));
                    body.velocity.y *= -atten;
                    collided = true;
                }

                if (collided)
                {
                    body.angularVelocity *= atten;
                }
            }

            mPs.updateSolidWorldPoints();
        }
    }
}
