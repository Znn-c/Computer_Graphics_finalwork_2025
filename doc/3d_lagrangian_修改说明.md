# 3D 拉格朗日（粒子/SPH）部分修改说明（用于实验报告）

本文档记录我在 `code/fluid3d/Lagrangian` 下对 3D 拉格朗日流体模块做的补全与修正，目标是：在不改变现有框架（UI/渲染/组件组织方式）的前提下，让 3D 粒子法像 2D 一样“动起来”，并修复运行中暴露出的几个小一致性问题（例如粒子轻微溢出白色容器线框）。

整体思路是复用你在 2D 拉格朗日实现中已经验证过的结构：

- `ParticleSystem3d`：粒子数据结构 + 空间划分（block）加速邻域访问
- `Solver`：每帧（每 substep）推进一次 SPH 动力学更新
- `Renderer`：把粒子缓冲上传到 GPU，用点精灵绘制
- `Lagrangian3dComponent`：把 solver、particle system、renderer 串起来

---

## 1. 你原始 3D 代码的状态与问题定位

### 1.1 Solver 未实现（核心缺失）

原始 `Solver::solve()` 只有 TODO 注释，没有任何密度/压力/受力/积分/碰撞更新逻辑，导致粒子不会形成流体运动。

- 文件：`code/fluid3d/Lagrangian/src/Solver.cpp`
- 关键函数：`FluidSimulation::Lagrangian3d::Solver::solve()`

### 1.2 渲染数据布局与 shader 不一致（潜在渲染异常）

顶点着色器 `DrawParticles3d.vert` 中：

```glsl
layout(location = 0) in vec3 position;
layout(location = 1) in float density;
```

但原 3D renderer 的 VAO 里把 `density` 当成了 `vec2`（size=2）来解释，这会导致 attribute 读取错位，出现颜色/形态异常，甚至影响后续 attribute 的正确性。

- shader：`code/resources/shaders/DrawParticles3d.vert`
- 渲染：`code/fluid3d/Lagrangian/src/Renderer.cpp` 的 `MakeVertexArrays()`

### 1.3 空间划分 blockExtens 统计存在边界条件 bug（会让邻域访问错乱）

原 `ParticleSystem3d::updateBlockInfo()` 默认从 `curBlockId = 0` 开始统计，但粒子排序后第一个粒子的 `blockId` 不一定是 0（尤其是粒子初始块不在容器左下前角），会导致：

- `blockExtens[0]` 被错误写入
- 后续真正的 `blockId` 范围可能不完整/错位
- solver 邻域遍历时访问到错误粒子范围，表现为“粒子穿墙、局部炸开、密度计算异常”等

- 文件：`code/fluid3d/Lagrangian/src/ParticleSystem3d.cpp`
- 函数：`ParticleSystem3d::updateBlockInfo()`

### 1.4 粒子碰撞边界与白色容器线框不一致（视觉“溢出一点点”）

你的 3D 场景里白色容器线框由 `Glb::Container` 绘制，几何是 `[0,1]^3`（或 resetSize 后的盒子）。

而粒子系统里为了邻域搜索，把边界 `lowerBound/upperBound` 扩展了 `supportRadius`（这是合理的“搜索 padding”），但如果把这个扩展边界用于碰撞，就会出现：

- 粒子在视觉上可以到达容器线框外一点点（因为碰撞用的是更大的 box）

---

## 2. 修改总览（哪些文件改了）

### 2.1 3D Solver：补全 SPH 更新

- 修改：`code/fluid3d/Lagrangian/src/Solver.cpp`
- 内容：实现密度/压力/压力力/黏性力/重力/积分/边界碰撞/更新 blockId

### 2.2 3D Renderer：修正 VAO attribute 尺寸与 VBO 绑定目标

- 修改：`code/fluid3d/Lagrangian/src/Renderer.cpp`
- 内容：
  - `density` attribute 从 size=2 改为 size=1（匹配 shader）
  - 上传数据改回 `GL_ARRAY_BUFFER`（与 `glVertexAttribPointer` 的 VBO 目标一致）

### 2.3 3D ParticleSystem：修正 blockExtens 统计逻辑 + 引入“可视容器边界”

- 修改：`code/fluid3d/Lagrangian/include/ParticleSystem3d.h`
- 修改：`code/fluid3d/Lagrangian/src/ParticleSystem3d.cpp`
- 内容：
  - `updateBlockInfo()` 从 `particles[0].blockId` 开始统计，处理空粒子情况
  - 新增 `containerLower/containerUpper`（用于碰撞与可视容器对齐）
  - `setContainerSize()` 里 `corner` 同样应用 scale，并正确设置容器边界
  - `addFluidBlock()` 用 `containerLower/containerUpper` 校验合法性（避免初始块越界）

### 2.4 3D Component：添加初始化兜底，避免“空场景”

- 修改：`code/fluid3d/Lagrangian/src/Lagrangian3dComponent.cpp`
- 内容：当配置 `fluidBlocks` 不合法或生成粒子失败时，回退到默认 fluid block

### 2.5 构建：清理 CMake 缓存（解决 cache 目录复用）

这个不是代码逻辑修改，但属于实际跑通工程的重要步骤：

- 删除了 `code/out/build/x64-Release` 下旧的 `CMakeCache.txt`（它引用了另一个项目路径 `graphics-lab6`）

---

## 3. 关键实现细节（Solver：3D SPH 推进）

### 3.1 粒子状态与参数来源

粒子结构体 `particle3d`（位置/速度/加速度/密度/压力等）：

- 文件：`code/fluid3d/Lagrangian/include/ParticleSystem3d.h`

全局参数来自 `Configure.cpp` 的 `Lagrangian3dPara`：

- `dt`、`density`（rest density）、`stiffness`、`exponent`、`viscosity`
- `supportRadius`（核函数支持半径）
- `maxVelocity`（速度上限）
- `velocityAttenuation`（碰撞反弹衰减）
- `eps`（贴边安全距离）

### 3.2 邻域搜索：基于 block 的 3×3×3 邻块遍历

为了避免 \(O(n^2)\) 的全粒子两两计算，代码沿用 2D 的“block 空间划分 + 排序 + 每块粒子区间”的做法：

1. 每个粒子通过 `getBlockIdByPosition()` 获得所在 blockId
2. 每 substep 前调用 `updateBlockInfo()`：
   - 按 blockId 对粒子排序
   - 统计每个 blockId 的粒子范围 `[begin, end)` 存到 `blockExtens`
3. 在 solver 里对某个粒子，只访问其 3×3×3 邻块（含自己块）里的粒子区间

在 `Solver.cpp` 中实现了一个局部 lambda：

- `visitNeighbors(blockId, visitor)`：遍历邻块并对区间内的粒子索引 `j` 调用 visitor

### 3.3 核函数（Kernel）

实现了 3D 版本的常用 SPH 核函数（形式与 2D 对应，系数换成 3D 标准常数）：

1. Poly6（用于密度）：
   \[
   W_{poly6}(r) = \frac{315}{64\pi h^9}(h^2-r^2)^3
   \]
   代码：`Poly6Kernel3d(r2, h, h2)`

2. Spiky 的梯度（用于压力力）：
   \[
   \nabla W_{spiky}(r) = -\frac{45}{\pi h^6}(h-r)^2 \frac{r}{|r|}
   \]
   代码：`SpikyGradKernel3d(r, rLen, h)`

3. Viscosity 的 Laplacian（用于黏性）：
   \[
   \nabla^2 W_{visc}(r) = \frac{45}{\pi h^6}(h-r)
   \]
   代码：`ViscLaplacianKernel3d(rLen, h)`

### 3.4 密度与压力

密度：

\[
\rho_i = \sum_j m \cdot W(|x_i-x_j|)
\]

其中 `mass = restDensity * particleVolume`。

压力状态方程采用与 2D 一致的 Tait 形式：

\[
p_i = k \left( \left(\frac{\rho_i}{\rho_0}\right)^\gamma - 1 \right)
\]

并做了非负截断（避免稀疏区出现负压导致吸附爆炸）：

- `if (pressure < 0) pressure = 0;`
- 同时预计算 `pressDivDens2 = p / (rho^2)`，后面压力力复用

### 3.5 力项与加速度

压力力（对称形式，减少数值偏差）：

\[
a_i^{pressure} = -\sum_{j\neq i} m \left(\frac{p_i}{\rho_i^2} + \frac{p_j}{\rho_j^2}\right)\nabla W_{spiky}
\]

黏性：

\[
a_i^{visc} = \mu \sum_{j\neq i} m \frac{(v_j - v_i)}{\rho_j} \nabla^2 W_{visc}
\]

重力：

- 代码里设为 `glm::vec3(gx, gy, -gz)`
- 原因：默认参数里 `initVel.z = -1.0` 且希望 `gravityZ=9.8` 表示向 **-Z** 下落，因此取负号来与初始运动方向一致

最终：

```cpp
pi.accleration = aPressure + aVisc + gravity;
```

### 3.6 时间积分（显式欧拉）与速度上限

采用最简单的显式欧拉（与 2D 保持一致）：

- `v += dt * a`
- `x += dt * v`

另外加入最大速度截断（避免少数粒子数值爆炸带坏整体）：

- 若 `|v| > maxVelocity`，则缩放回 `maxVelocity`

### 3.7 边界碰撞（关键修复：对齐白色容器线框）

为了解决“粒子溢出白色边框”的问题，引入两套边界：

1. `containerLower/containerUpper`：可视容器边界（严格对齐 `Glb::Container` 的盒子）
2. `lowerBound/upperBound`：邻域搜索/分块边界（在容器基础上 padding 了 `supportRadius`）

碰撞 clamp 使用 `containerLower/containerUpper`：

- 小于下界：位置夹到 `lower + eps`，速度反弹并乘 `velocityAttenuation`
- 大于上界：位置夹到 `upper - eps`，速度反弹并乘 `velocityAttenuation`

这样粒子就不会再“合法地”跑到容器线框外。

相关代码位置：

- `code/fluid3d/Lagrangian/src/Solver.cpp`：碰撞处理段
- `code/fluid3d/Lagrangian/src/ParticleSystem3d.cpp`：`setContainerSize()` 设置两套边界

---

## 4. 关键实现细节（ParticleSystem：空间划分修复）

### 4.1 修复 updateBlockInfo 的起始 blockId

原先默认从 `curBlockId=0` 开始统计，会在“粒子第一个 blockId 不是 0”时写错。

修改后逻辑：

- 如果 `particles.empty()` 直接返回
- 否则：
  - `curBlockId = particles[0].blockId`
  - `left = 0`
  - 扫描 `right`，遇到 blockId 变化就写入 `blockExtens[curBlockId] = [left, right)`

这样 `blockExtens` 才能正确反映“每个 block 里有哪些粒子”。

位置：`code/fluid3d/Lagrangian/src/ParticleSystem3d.cpp` 的 `updateBlockInfo()`

### 4.2 修复 setContainerSize 的 corner 缩放与边界定义

你在 3D component 中调用：

```cpp
ps->setContainerSize(glm::vec3(0,0,0), glm::vec3(1,1,1));
```

但粒子系统内部会把 `size` 乘 scale，如果 `corner` 不乘 scale，会导致容器在“缩放坐标系”下有轻微偏差。

修改后：

- `corner *= scale`
- `size *= scale`
- `containerLower = corner`
- `containerUpper = corner + size`
- `lowerBound/upperBound` = 在 container 上 padding 支持半径（用于邻域搜索）

---

## 5. 关键实现细节（Renderer：修复 attribute 与缓冲绑定）

### 5.1 density attribute 尺寸修正

shader 明确要求 `density` 是 `float`（1 个分量），因此 VAO 应该：

```cpp
glVertexAttribPointer(1, 1, GL_FLOAT, ... offsetof(particle3d, density));
```

而不是 `size=2`。

位置：`code/fluid3d/Lagrangian/src/Renderer.cpp` 的 `MakeVertexArrays()`

### 5.2 上传 VBO 的绑定目标修正

由于 VAO 里用 `GL_ARRAY_BUFFER` 记录了 VBO 绑定关系，上传数据时也应该用同一个目标更一致：

- 改为 `glBindBuffer(GL_ARRAY_BUFFER, VBO); glBufferData(GL_ARRAY_BUFFER, ...)`

位置：`code/fluid3d/Lagrangian/src/Renderer.cpp` 的 `draw()`

---

## 6. 3D 初始化鲁棒性（避免空粒子导致 solver/渲染异常）

为避免配置 `Lagrangian3dPara::fluidBlocks` 不合法（例如 upper <= lower）或生成粒子数为 0 导致：

- solver 邻域/排序逻辑边界条件被触发
- 画面无内容

我在 `Lagrangian3dComponent::init()` 中加入：

- `isValidBlock()`：检查块体积与 particleSpace
- `defaultBlock()`：提供一个合理默认流体块（在容器内部）
- 若没有任何合法块或生成后 `ps->particles.empty()`，则回退到默认块重新生成

位置：`code/fluid3d/Lagrangian/src/Lagrangian3dComponent.cpp` 的 `init()`

---

## 7. 现象与参数调试建议（写报告可用）

1. 如果出现“局部爆炸/飞散”：
   - 降低 `dt` 或提高 `substep`
   - 降低 `stiffness` 或 `exponent`
   - 增大 `viscosity`（但过大可能会“凝固”）

2. 如果粒子贴边抖动明显：
   - 增大 `eps`（例如 `1e-4`）
   - 或减小 `velocityAttenuation`（反弹更弱）

3. 如果感觉下落方向不符合预期：
   - 检查 `Configure.cpp` 中 `gravityX/Y/Z` 与初始速度方向
   - 当前实现把 `gravityZ` 施加到 `-Z` 方向（与默认 `initVel.z=-1` 的场景一致）

---

## 8. 修改点快速索引（报告引用方便）

- SPH solver 主体：`code/fluid3d/Lagrangian/src/Solver.cpp` 的 `Solver::solve()`
- Kernel 实现：同上文件的 `Poly6Kernel3d / SpikyGradKernel3d / ViscLaplacianKernel3d`
- blockExtens 修复：`code/fluid3d/Lagrangian/src/ParticleSystem3d.cpp` 的 `updateBlockInfo()`
- 可视容器边界字段：`code/fluid3d/Lagrangian/include/ParticleSystem3d.h`（`containerLower/containerUpper`）
- 碰撞边界对齐：`code/fluid3d/Lagrangian/src/Solver.cpp`（clamp 到 `containerLower/containerUpper`）
- renderer attribute 修复：`code/fluid3d/Lagrangian/src/Renderer.cpp` 的 `MakeVertexArrays()`

