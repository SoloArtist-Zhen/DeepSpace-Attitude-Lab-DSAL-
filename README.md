# DeepSpace Attitude Lab (DSAL) — 航天姿态控制交互实验室

> 一键运行的 **Streamlit 工程**：刚体四元数动力学 + 反作用轮 + 干扰（重力梯度、SRP） + 控制器（Detumble / PID / LQR）。
> 内置 **10+ 高质量图**、**3D 单位球动画**，并支持 **一键导出 Exam Pack（PNG/GIF+摘要）与 PPTX（12 张图双栏排版）**。

---

## 目录 / Table of Contents
- [快速开始](#快速开始)
- [项目亮点](#项目亮点)
- [UI 使用指南](#ui-使用指南)
- [核心模型与方法](#核心模型与方法)
- [场景与控制器](#场景与控制器)
- [导出功能（Exam Pack / PPTX）](#导出功能exam-pack--pptx)
- [文件结构](#文件结构)
- [复试演示建议（3 分钟脚本）](#复试演示建议3-分钟脚本)
- [性能与稳定性建议](#性能与稳定性建议)
- [常见问题](#常见问题)
- [致谢与参考](#致谢与参考)
- [许可协议](#许可协议)

---

## 快速开始

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate

pip install -r requirements.txt
streamlit run app.py
```

> 第一次运行：点击左侧参数后，按下 **Run Simulation**，即可生成图表与动画。默认参数即可出效果。

**占位图（仓库封面/动图）：**

![截屏2025-10-29 05.36.07.png](../../Desktop/%E6%88%AA%E5%B1%8F2025-10-29%2005.36.07.png)
---

## 项目亮点

- 🛰️ **工程化**：四元数动力学、反作用轮饱和/动量卸载、重力梯度与太阳辐射压（简化）扰动。
- 🕹️ **交互友好**：Streamlit 滑块/下拉菜单，默认参数即可稳定收敛，适合「小白一键跑」。
- 📈 **十几张图**：角速度/误差角/控制力矩/能量/相图/累积代价等**10+ 图**自动生成。
- 🌐 **3D 动画**：单位球面上的指向路径（Plotly 动画按钮 Play）。
- 📦 **一键导出**：
  - Exam Pack：PNG（12 张）+ 小 GIF + `summary.md`
  - PPTX：**12 张图双栏排版**，直接可用于复试/答辩。
- 🧪 **多控制器**：Detumble、PID、LQR，支持对比能量衰减与控制代价。

**占位图（功能一览）：**
![Features Overview 占位](docs/img/features_overview.png) <!-- TODO: 替换为功能总览图 -->

---

## UI 使用指南

1. **左侧栏**设置：选择 *Scenario* 与 *Controller*，设定仿真时长 `Duration`、步长 `Step`、初始角速度/姿态、力矩/轮动量上限，以及是否开启干扰。
2. 点击 **Run Simulation** → 进入下方 **Tabs** 浏览结果：
   - **📊 Time Series**：角速度、四元数向量、速率/力矩/轮动量模值
   - **🧭 Error & Energy**：误差角、能量、误差角变化率、累积控制代价
   - **🌀 Phase & Spectra**：`ωx-ωy` 相图（可扩展频谱）
   - **🌐 3D Animation**：单位球面指向轨迹（Plotly 动画）
   - **📦 Exports**：一键导出 ZIP/PPTX

**占位图（UI 引导）：**
![UI Guide 占位](docs/img/ui_guide.png) <!-- TODO: 替换为带标注的 UI 截图 -->

---

## 核心模型与方法

**刚体姿态动力学（含轮系动量）**

- 刚体动量守恒：
  \[
  \mathbf{J}\,\dot{\boldsymbol\omega} + \boldsymbol\omega \times (\mathbf{J}\,\boldsymbol\omega + \mathbf{h})
  = \boldsymbol\tau + \boldsymbol\tau_d,\quad
  \dot{\mathbf{h}} = -\boldsymbol\tau.
  \]
- 四元数运动学：
  \[
  \dot{\mathbf{q}} = \tfrac{1}{2} \Omega(\boldsymbol\omega)\,\mathbf{q},
  \]
  其中 \(\mathbf{q}=[w,x,y,z]^\top\) 并保持归一化。

**外扰（简化可教学版本）**

- **重力梯度力矩**：
  \[
  \boldsymbol\tau_{gg} = 3\,\frac{\mu}{r^3}\,\hat{\mathbf{r}}\times(\mathbf{J}\,\hat{\mathbf{r}}).
  \]
- **太阳辐射压（SRP）力矩**（粗略）：
  \[
  \boldsymbol\tau_{srp} = P_\odot A C\, (\mathbf{r}_{cp} \times \hat{\mathbf{s}}_{b}),
  \]
  其中 \(P_\odot \approx 4.5\times10^{-6}\,\mathrm{N/m^2}\)。

**控制律（内置 3 类）**

- **Detumble（安全模式）**：\(\boldsymbol\tau=-K_d\,\boldsymbol\omega\)（B-dot 思想的简化 PD）。
- **PID（四元数误差 + 角速率）**：
  \[
  \boldsymbol\tau = -K_p\,\mathbf{e}_q - K_d (\boldsymbol\omega - \boldsymbol\omega_{ref}),\quad
  \mathbf{e}_q = \text{vec}(\,\mathbf{q}_{ref}\otimes\mathbf{q}^{-1}\,).
  \]
- **LQR（小角度线化）**：令 \(\mathbf{x}=[\boldsymbol\theta;\,\boldsymbol\omega]\)，
  \(\dot{\mathbf{x}}=\mathbf{A}\mathbf{x}+\mathbf{B}\mathbf{u}\)，求解 CARE 得到 \(\mathbf{K}\)，\(\mathbf{u}=-\mathbf{K}\mathbf{x}\)。

**占位图（错误角/能量）：**
![Error & Energy 占位](docs/img/error_energy.png) <!-- TODO: 替换为误差角与能量图 -->

---

## 场景与控制器

- **Scenes**：
  - *Safe-Mode Detumble*（去旋保障）
  - *Sun-Pointing*（太阳指向）
  - *Nadir-Pointing*（对地方向）
  - *Target Scan*（按恒速偏航扫掠）
- **Controllers**：Detumble / PID / LQR（自动求 CARE）
- **参数**：
  - `torque_max` 轮力矩限幅、`h_rw_max` 轮动量上限（触发简化卸载）；
  - `Enable disturbances (GG + SRP)` 开关扰动。

**占位图（不同控制器对比）：**
![Controller Compare 占位](docs/img/controller_compare.png) <!-- TODO: 替换为 PID vs LQR 对比图 -->

---

## 导出功能（Exam Pack / PPTX）

- **Exam Pack（ZIP）**：自动打包 **12 张 PNG** + **小 GIF**（指向路径 2D 投影）+ `summary.md`。
- **PPTX 导出**：自动把 12 张图 **双栏排版（6 页）**，开头有标题页，可直接用于复试/答辩。

**占位图（PPT 预览）：**
![PPT Preview 占位](docs/img/ppt_preview.png) <!-- TODO: 替换为 PPT 预览截图 -->

---

## 文件结构

```
DeepSpace-Attitude-Lab/
├─ app.py                 # Streamlit 界面入口（Run Simulation / Tabs / Exports）
├─ dynamics.py            # 四元数动力学、外扰、轮动量与积分器
├─ controllers.py         # Detumble / PID / LQR 控制律
├─ scenarios.py           # 场景定义（Sun/Nadir/Scan 等参考姿态）
├─ plotting.py            # Matplotlib/Plotly 绘图与 Zip/PPTX 导出
├─ requirements.txt
├─ README.md
└─ docs/
   └─ img/                # 你的截图/插图放这里（预留占位）
```

**占位图（项目结构图）：**
![Project Structure 占位](docs/img/structure.png) <!-- TODO: 替换为结构示意图 -->

---

## 复试演示建议（3 分钟脚本）

1. **Detumble 开场**（展示去旋收敛曲线 + 误差角 < 1° 时间）  
2. **PID vs LQR 对比**（能量衰减、响应速度与过冲的权衡）  
3. **3D 指向动画**（播放按钮，演示路径收敛/扫描）  
4. **一键导出**（PPTX/ZIP）并简述工程化能力（限幅、卸载、干扰建模、UI/导出链路）  

**占位图（演示路线海报）：**
![Demo Flow 占位](docs/img/demo_flow.png) <!-- TODO: 替换为复试讲解流程图 -->

---

## 性能与稳定性建议

- 若动画卡顿：`Duration` 设为 120 s，`Step` 设为 0.08–0.10 s。
- 若力矩饱和明显：适当增大 `torque_max` 或放宽 `h_rw_max`。
- 需要更“稳”的收敛：PID 可提高 `Kp` 并适当增大 `Kd`，注意避免过冲。

**占位图（性能建议示意）：**
![Perf Tips 占位](docs/img/perf_tips.png) <!-- TODO: 替换为性能建议相关图 -->

---

## 常见问题

- **Q: PPTX 导出报错 `ModuleNotFoundError: pptx`？**  
  A: 重新执行依赖安装：`pip install -r requirements.txt`。

- **Q: 图像不显示或 Matplotlib 后端问题？**  
  A: 已固定 `matplotlib==3.8.4`；如仍有问题，可尝试 `pip uninstall matplotlib && pip install "matplotlib<3.9"`。

- **Q: 想要“自动带姓名/学校/专业”的标题页？**  
  A: 可在 `plotting.py::make_exam_ppt` 中修改标题与副标题，或告诉我们代为添加。

**占位图（FAQ 预览）：**
![FAQ 占位](docs/img/faq.png) <!-- TODO: 替换为 FAQ 截图 -->

---

## 致谢与参考

- 经典航天姿态控制教材与公开资料（四元数、重力梯度、LQR 等）。
- Plotly/Matplotlib/Streamlit 开源社区。

> 注：为教学演示而做出合理简化；参数非任务级设计，仅用于交互理解与对比。
