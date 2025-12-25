/**
 * Solver.cpp: 3D�����������������ʵ���ļ�
 * ʵ�ֻ������ӵ�3D��������㷨
 */

#include "fluid3d/Lagrangian/include/Solver.h"
#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace FluidSimulation
{

	namespace Lagrangian3d
	{
		static inline float Poly6Kernel3d(float r2, float h, float h2)
		{
			if (r2 >= h2)
			{
				return 0.0f;
			}
			float x = h2 - r2;
			float h4 = h2 * h2;
			float h8 = h4 * h4;
			float h9 = h8 * h;
			float c = 315.0f / (64.0f * glm::pi<float>() * h9);
			return c * x * x * x;
		}

		static inline glm::vec3 SpikyGradKernel3d(const glm::vec3& r, float rLen, float h)
		{
			if (rLen <= 0.0f || rLen >= h)
			{
				return glm::vec3(0.0f);
			}
			float x = h - rLen;
			float h2 = h * h;
			float h4 = h2 * h2;
			float h6 = h4 * h2;
			float c = -45.0f / (glm::pi<float>() * h6);
			return c * x * x * (r / rLen);
		}

		static inline float ViscLaplacianKernel3d(float rLen, float h)
		{
			if (rLen >= h)
			{
				return 0.0f;
			}
			float h2 = h * h;
			float h4 = h2 * h2;
			float h6 = h4 * h2;
			float c = 45.0f / (glm::pi<float>() * h6);
			return c * (h - rLen);
		}

		/**
		 * ���캯������������ϵͳ����
		 * @param ps ����ϵͳ����
		 */
		Solver::Solver(ParticleSystem3d &ps) : mPs(ps)
		{
			// ���캯��,��������ϵͳ����
		}

		/**
		 * ������巽��
		 * ʵ��һ��3D��������ķ������
		 */
		void Solver::solve()
		{
			const float dt = Lagrangian3dPara::dt;
			const float restDensity = Lagrangian3dPara::density;
			const float stiffness = Lagrangian3dPara::stiffness;
			const float exponent = Lagrangian3dPara::exponent;
			const float viscosity = Lagrangian3dPara::viscosity;

			const float h = mPs.supportRadius;
			const float h2 = mPs.supportRadius2;
			const float mass = restDensity * mPs.particleVolume;

			const glm::vec3 gravity = glm::vec3(Lagrangian3dPara::gravityX, Lagrangian3dPara::gravityY, -Lagrangian3dPara::gravityZ);

			for (auto& p : mPs.particles)
			{
				p.density = 0.0f;
				p.pressure = 0.0f;
				p.pressDivDens2 = 0.0f;
				p.accleration = glm::vec3(0.0f);
			}

			auto visitNeighbors = [&](uint32_t blockId, auto&& visitor)
			{
				const uint32_t blocksXY = mPs.blockNum.x * mPs.blockNum.y;
				const uint32_t totalBlocks = blocksXY * mPs.blockNum.z;
				if (blockId >= totalBlocks)
				{
					return;
				}

				int32_t c0 = static_cast<int32_t>(blockId % mPs.blockNum.x);
				int32_t r0 = static_cast<int32_t>((blockId / mPs.blockNum.x) % mPs.blockNum.y);
				int32_t h0 = static_cast<int32_t>(blockId / blocksXY);

				for (int32_t dz = -1; dz <= 1; dz++)
				{
					for (int32_t dy = -1; dy <= 1; dy++)
					{
						for (int32_t dx = -1; dx <= 1; dx++)
						{
							int32_t c = c0 + dx;
							int32_t r = r0 + dy;
							int32_t z = h0 + dz;
							if (c < 0 || r < 0 || z < 0 ||
								c >= static_cast<int32_t>(mPs.blockNum.x) ||
								r >= static_cast<int32_t>(mPs.blockNum.y) ||
								z >= static_cast<int32_t>(mPs.blockNum.z))
							{
								continue;
							}

							uint32_t nb = static_cast<uint32_t>(z) * blocksXY +
								static_cast<uint32_t>(r) * mPs.blockNum.x +
								static_cast<uint32_t>(c);
							const glm::uvec2 ext = mPs.blockExtens[nb];
							for (uint32_t j = ext.x; j < ext.y; j++)
							{
								visitor(j);
							}
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
						glm::vec3 r = pi.position - pj.position;
						float r2 = glm::dot(r, r);
						dens += mass * Poly6Kernel3d(r2, h, h2);
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
				glm::vec3 aPressure(0.0f);
				glm::vec3 aVisc(0.0f);

				const uint32_t myBlockId = pi.blockId;
				visitNeighbors(myBlockId,
					[&](uint32_t j)
					{
						if (j == i)
						{
							return;
						}

						const auto& pj = mPs.particles[j];
						glm::vec3 r = pi.position - pj.position;
						float r2 = glm::dot(r, r);
						if (r2 >= h2)
						{
							return;
						}
						float rLen = std::sqrt(r2);

						glm::vec3 gradW = SpikyGradKernel3d(r, rLen, h);
						float lapW = ViscLaplacianKernel3d(rLen, h);

						float pressTerm = (pi.pressDivDens2 + pj.pressDivDens2);
						aPressure += -mass * pressTerm * gradW;

						aVisc += viscosity * mass * (pj.velocity - pi.velocity) / pj.density * lapW;
					});

				pi.accleration = aPressure + aVisc + gravity;
			}

			for (auto& p : mPs.particles)
			{
				p.velocity += dt * p.accleration;

				float v2 = glm::dot(p.velocity, p.velocity);
				float maxV = Lagrangian3dPara::maxVelocity;
				if (v2 > maxV * maxV)
				{
					p.velocity *= (maxV / std::sqrt(v2));
				}

				p.position += dt * p.velocity;

				const float eps = Lagrangian3dPara::eps;
				const float atten = Lagrangian3dPara::velocityAttenuation;

				if (p.position.x < mPs.containerLower.x + eps)
				{
					p.position.x = mPs.containerLower.x + eps;
					p.velocity.x *= -atten;
				}
				else if (p.position.x > mPs.containerUpper.x - eps)
				{
					p.position.x = mPs.containerUpper.x - eps;
					p.velocity.x *= -atten;
				}

				if (p.position.y < mPs.containerLower.y + eps)
				{
					p.position.y = mPs.containerLower.y + eps;
					p.velocity.y *= -atten;
				}
				else if (p.position.y > mPs.containerUpper.y - eps)
				{
					p.position.y = mPs.containerUpper.y - eps;
					p.velocity.y *= -atten;
				}

				if (p.position.z < mPs.containerLower.z + eps)
				{
					p.position.z = mPs.containerLower.z + eps;
					p.velocity.z *= -atten;
				}
				else if (p.position.z > mPs.containerUpper.z - eps)
				{
					p.position.z = mPs.containerUpper.z - eps;
					p.velocity.z *= -atten;
				}

				p.blockId = mPs.getBlockIdByPosition(p.position);
			}
		}
	}
}
