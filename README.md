# Computer Graphics - Advanced Hybrid 3D Renderer (C++)

> **2025 Computer Graphics Coursework | Score: TBD 🏆**

An industrial-grade, hybrid 3D rendering engine written in **Modern C++ (C++20)**. This project implements a comprehensive graphics pipeline that seamlessly switches between **Real-Time Rasterization** and **Physically Based Path Tracing**. It demonstrates advanced rendering algorithms including Photon Mapping, Monte Carlo Integration, and Depth of Field, built upon a highly optimized, data-oriented architecture.

## 🚀 Key Highlights

### 1. Hybrid Rendering Pipeline

A flexible architecture supporting instantaneous mode switching without reloading geometry.

- **Rasterizer:** Features scanline triangle filling with **perspective-correct texture mapping**, Sutherland-Hodgman clipping, and fuzzy depth testing to eliminate Z-fighting in wireframe mode.
- **Path Tracer:** A physically based renderer (PBR) supporting Global Illumination, Area Lights (Soft Shadows), and transparent dielectrics with Fresnel equations.
- **Advanced Optics:** Implemented **Depth of Field (DoF)** using concentric disk sampling to simulate thin-lens cameras with physically accurate Bokeh effects.

### 2. High-Performance Engineering

Optimized for maximum CPU throughput using modern C++ techniques.

- **Multi-Threaded Tiling:** The renderer uses `std::jthread` and `std::barrier` to implement a lock-free, tile-based rendering system, maximizing core utilization.
- **Unified BVH Accelerator:** Implemented a Surface Area Heuristic (SAH) **Bounding Volume Hierarchy (BVH)** used by both the Ray Tracer and Photon Map for $O(\log n)$ intersection queries.
- **Photon Tracing**: Multithreaded photon emission allows for rapid caustic map generation without blocking the main UI thread.
- **Memory Optimization:**
  - **Data-Oriented Design:** Geometry is stored in flat global buffers (`World`-centric) with `Face` structs storing indices rather than pointers, significantly improving CPU cache locality.
  - **Custom Containers:** Utilized a custom `InplaceVector` (stack-allocated) to eliminate heap allocation overhead during hot-path polygon clipping, see [std::inplace_vector - cppreference.com](https://en.cppreference.com/w/cpp/container/inplace_vector.html) if you're using C++26 standard.

### 3. Advanced Global Illumination

Features a simplified but effective implementation of **Photon Mapping** to handle complex light transport phenomena.

- **Caustics:** Capable of rendering focused light patterns through refractive objects (e.g., glass spheres).
- **Optimized Spatial Indexing:** Replaced standard hash maps with a **flattened 1D Grid** for photon storage, ensuring $O(1)$ access and contiguous memory layout for high-performance radiance estimation.

### 4. Post-Processing & Extras

- **ACES Filmic Tone Mapping:** Converts High Dynamic Range (HDR) linear light to displayable sRGB, preserving details in bright highlights.
- **Gamma Correction:** Accurate linear-to-sRGB pipeline.
- **Video Export:** Integrated `ffmpeg` pipe for direct `.mp4` recording without relying on screen capture.

## 🛠 Features Implemented

This renderer implements **ALL** features required for the highest marking band (>75%).

| **Category**      | **Feature**                                           | **Status**    |
| ----------------- | ----------------------------------------------------- | ------------- |
| **Core**          | OBJ Loading / Multi-model / Camera Control            | ✅ Implemented |
| **Rasterization** | Wireframe / Flat / Texture Mapping / Z-Buffer         | ✅ Implemented |
| **Shading**       | Gouraud / Phong / Interpolated Normals                | ✅ Implemented |
| **Shadows**       | Hard Shadows / **Soft Shadows (Area Light)**          | ✅ Implemented |
| **Materials**     | Mirror Reflection / **Refraction (Glass)** / Metallic | ✅ Implemented |
| **Advanced**      | **Photon Mapping (Caustics)**                         | ✅ Implemented |
| **Advanced**      | **Depth of Field (Bokeh)**                            | ✅ Implemented |
| **Advanced**      | Environment Mapping (HDR)                             | ✅ Implemented |

## 📸 Gallery

### Physically Based Path Tracing

![PBR-Tracer](./assets/pbr-tracer.png)

### Depth of Field & Bokeh

> Demonstrating Thin-Lens Simulation with adjustable Aperture and Focal Distance.
>
> (Place your DoF image here)

### Caustics via Photon Mapping

> Demonstrating light focusing through dielectric spheres.
>
> (Place an image showing caustics here)

## 🎮 Controls

The renderer supports real-time interaction even during ray tracing (progressive accumulation resets on movement).

- **Render Modes:** `1` Wireframe | `2` Rasterized | `3` Ray Tracing | `4` Depth of Field
- **Movement:** `W/A/S/D` Move | `Space/C` Up/Down | `Q/E` Roll
- **Camera:** `Mouse Drag` Look | `O` Orbit Mode
- **DoF Control:** `Scroll` Focal Dist | `+/-` Aperture Size
- **Toggles:** `G` Gamma (1.0/2.2) | `P` Caustics
- **System:** `Ctrl+S` Screenshot | `Ctrl+R` Record Video

## 💻 Usage

### 🏫 Lab Environment Setup

This project leverages **C++20** features (e.g., `std::barrier`, `std::jthread`). If running on University Lab machines, you **MUST** load a modern GCC version before compiling:

```bash
module add gcc/13.4.0
```

### 🔨 Building the Project

Use CMake to build the project in Release mode for optimal rendering performance:

```bash
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

If you are compiling on a Lab machine, then run the following commands instead:

```bash
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/opt/gcc/13.4.0/bin/g++
cmake --build build -j$(nproc)
```


### 🚀 Running the Renderer

The renderer accepts OBJ models or custom Scene description files (`.txt`) as command-line arguments.

**Basic Usage:**

```
# Run with a single OBJ file or TXT file
./build/CG-CW ./model/cornell-box.obj

# Run with multiple files
./build/CG-CW ./model/cornell-box.obj ./model/sphere.obj

# Run with multiple files and an environment map
./build/CG-CW ./model/cornell-box.obj ./model/sphere.obj ./model/qwantani_dusk_2_puresky_4k.hdr
```

> **Note:** Ensure your working directory allows the executable to find the model paths provided. The renderer requires at least one argument.

*Created by Zik Zhao | University of Bristol, 2025*