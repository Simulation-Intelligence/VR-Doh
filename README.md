# MochiPound

A mixed-reality mochi-pounding game for Meta Quest 3. One player kneads virtual mochi with tracked hands while another swings a virtual kine (mallet) into the usu (mortar). The dough is a real-time particle simulation tuned to look like high-speed mochitsuki: it dents under a strike, follows the kine up into a tent, lets go, and falls back into a mound.

This is a fork of [VR-Doh](https://github.com/Simulation-Intelligence/VR-Doh). The original VR-Doh README is kept below.

## Status

Work in progress on the `unity6-upgrade` branch. There are two ways to run it:

| | Standalone (current direction) | Quest Link (legacy) |
|---|---|---|
| Scene | `Assets/Scenes/MochiStandalone.unity` | `Assets/Scenes/MochiPounding.unity` |
| Dough solver | Unity compute shaders, no native plugins | Taichi AOT module on the PC GPU |
| Runs on | Quest 3 by itself, or the Unity Editor | PC, with the headset over Link |

The standalone build runs on a Quest 3 with passthrough and holds 72 fps with the dough at rest; performance during strikes and kneading has not been measured yet. The table of what works and what does not is in [AGENTS.md](AGENTS.md#target-behaviour).

## Requirements

- Unity 6000.6.9f1 with Android Build Support (for the standalone build)
- Meta Quest 3
- For the Editor: the Meta XR Simulator, or Quest Link
- Optional command-line tools used throughout the docs: the Unity CLI (`unity`) and Meta's `metavr` (`npx -y metavr@latest`)

## Quick start

1. Clone and open the folder in Unity 6000.6.9f1. Ignore the "Problem detected while opening the Scene file" and missing `OVRVirtualKeyboard` messages; they come from the original project.
2. Open `Assets/Scenes/MochiStandalone.unity`.
3. Activate the simulator (`Window > Meta > Meta XR Simulator > Activate`) or connect over Link, then press Play. The usu appears in front of you with the mochi in it.
4. Pinch the kine handle to pick it up and swing; pinch the usu rim to move the whole station. Open hands push the dough; a closed fist grips it.

To build for the headset, see "Building for Quest" in [AGENTS.md](AGENTS.md#building-for-quest-standalone).

## Working on it

[AGENTS.md](AGENTS.md) is the developer guide: architecture, how the solver fakes the sticky-tent look, the test recorder, build steps and known problems. It is written so an AI coding agent can work from it directly; `CLAUDE.md` points Claude Code at the same file.

---

# Original project

# VR-Doh: Hands-on 3D Modeling in Virtual Reality

**ACM Transactions on Graphics (SIGGRAPH), 2025**

## Authors

[Zhaofeng Luo*](https://roushelfy.github.io/)<sup>1,2</sup>, [Zhitong Cui*](https://scholar.google.com/citations?user=4BOa6U4AAAAJ&hl=en)<sup>1,3</sup>, [Shijian Luo](https://person.zju.edu.cn/en/0099094)<sup>3</sup>, [Mengyu Chu](https://rachelcmy.github.io/)<sup>2,4</sup>, [Minchen Li](https://www.cs.cmu.edu/~minchenl/)<sup>1</sup>  

<sup>1</sup> Carnegie Mellon University  
<sup>2</sup> Peking University  
<sup>3</sup> Zhejiang University  
<sup>4</sup> State Key Lab of General AI  

\*Equal Contribution  

## Links

- [📄 Preprint](https://arxiv.org/abs/2412.00814)  
- [💻 Code](https://github.com/Simulation-Intelligence/VR-Doh)  
- [📊 Project Page](https://simulation-intelligence.github.io/VR-Doh/)  

## System Requirements

Currently tested only on Meta Quest 3. Support for other VR headset models requires manual adjustment of related settings.

## Getting Started

The scene is pre-configured. To quickly experience VR-Doh, follow these steps:

1. **Clone the project**

    ```bash
    git clone https://github.com/Simulation-Intelligence/VR-Doh
    ```

2. **Add project to Unity**

    - In Unity Hub, select **"Add" → "Add project from disk"**
    - Select the cloned folder

3. **Install and configure Meta Quest Link**

    - Ensure Meta Quest Link is installed on your PC
    - Connect properly with Quest 3

4. **Configure Quest Link settings**

    - In **Settings → Beta**, enable **"Developer Runtime Features"** and all related options
    - In **Settings → General**, ensure **"Set Meta Quest Link as the active OpenXR Runtime"** is checked

5. **Open Unity project**

    - If errors occur during the process, select **"Ignore"**
    - If prompted about Oculus input changes requiring restart, select **"Restart"**

6. **Load the scene**

    - Open `Assets/Scenes/MPM 88_UI2` scene

7. **Run the project**

    - Ensure your PC and Quest 3 are properly connected with Quest Link mode active
    - Click **"Play"** at the top of the screen in Unity to enter

## Code Architecture

### Physics Simulation Core

The main physics simulation is implemented in: `scripts/mpm3D_gaussian_part.kernel.py`

This part uses **Taichi** to compile kernels into AOT modules (default location:  `Assets/Resources/TaichiModules/mpm3DGaussian_part_mat.kernel.tcm`).

In Unity, the module is imported via C# with main code in:
`Assets/Scripts/Mpm3DMarching.cs`

### Parameter Configuration

While users can adjust various parameters through the UI after launching the scene, some parameters are not yet exposed. Currently, all algorithms and parameters mentioned in the paper can be adjusted through the following method:

- `Test_Example_1` and `Test_Example_2` in the scene are the default startup objects.
- You can **pre-adjust parameters** in their **Mpm 3D Marching** component in the Unity Editor.

#### Common Parameters

- **Render Type:**
  - Select *Marching Cubes* for modeling from scratch
  - Select *Gaussian Splats* for editing 3D-GS models

- If using *Gaussian Splats*:
  - Enable the **Gaussian Splat Renderer** component on the object
  - Select a target Asset (created via `Tools → Gaussian Splats → CreateGaussianSplatAsset`)
  - Ensure **quality is set to very high**

- **Decoupling of physical and appearance representations:**
  - Enable `Use_gaussian_acceleration`
  - Adjust `Gaussian_simulate_ratio`

## Troubleshooting

If you encounter any deployment or runtime issues, please:

- Submit an [Issue](https://github.com/Simulation-Intelligence/VR-Doh/issues)
- Email contact: [zhaofen2@andrew.cmu.edu](mailto:zhaofen2@andrew.cmu.edu)

## Citation

If you use VR-Doh in your research, please cite our paper:

```bibtex
@article{luo2025vrdoh,
  author = {Zhaofeng Luo and Zhitong Cui and Shijian Luo and Mengyu Chu and Minchen Li},
  title = {VR-Doh: Hands-on 3D Modeling in Virtual Reality},
  year = {2025},
  issue_date = {July 2025},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  volume = {44},
  number = {4},
  issn = {0730-0301},
  url = {https://doi.org/10.1145/3731154},
  doi = {10.1145/3731154},
  journal = {ACM Trans. Graph.},
  month = jul,
  numpages = {12},
  keywords = {Virtual Reality, 3D Modeling, Elastoplasticity Simulation, Material Point Method, Human-Computer Interaction}
}
```
