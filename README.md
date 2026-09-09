# Interactive 3D City Scene

A real-time C++ graphics coursework project built with OpenGL, FreeGLUT, and GLU. The program constructs an explorable city without relying on a game engine: geometry transforms, texture loading, transparency, depth and stencil state, camera motion, and object animation are managed directly in the rendering code.

The scene combines static architecture with moving ground and airborne objects to demonstrate how multiple graphics techniques interact inside one render loop.

## Scene Overview

The city includes:

- buildings and textured façades
- ground, water, and a textured sky sphere
- billboarded trees
- ground cars and hovering vehicles
- trains following two tracks
- drones moving along figure-eight paths
- an animated robot
- a rotating Ferris wheel
- waves and other time-dependent scene elements
- a movable camera and two-dimensional overlay text

The current source contains 12 ground cars, 6 hover cars, and 10 figure-eight drones. Texture setup creates 27 BMP-backed texture objects for the scene.

## Rendering Techniques

### Hierarchical modelling

Complex objects are assembled using OpenGL's model-view matrix stack. Repeated combinations of `glPushMatrix`, `glPopMatrix`, translation, rotation, and scaling create local coordinate systems for object parts while keeping transformations isolated.

This approach is used across architecture, vehicles, animated objects, and repeated scene elements.

### Texture loading

BMP resources are loaded manually and converted into OpenGL textures. The implementation configures texture parameters, supplies image data, and handles resource-specific colour conversion. Several textures use a colour-key-to-alpha transformation to produce transparent regions.

Texture paths are relative to the project directory, so the `texture/` folder must remain beside the executable in the expected layout.

### Transparency, depth, and stencil state

The renderer coordinates:

- depth testing for correct visibility ordering
- alpha testing for cut-out textures
- blending for semi-transparent surfaces
- stencil operations for selected rendering effects and regions

Transparent objects require careful ordering and explicit state restoration. The program therefore saves and restores projection, model-view, lighting, texture, depth, blending, and stencil state around specialised passes and overlay rendering.

### Billboarding

Trees use textured quads that rotate to face the camera. Camera direction is derived from the model-view matrix, allowing many low-polygon trees to maintain a visually meaningful orientation without full three-dimensional tree geometry.

### Camera and projection

The program manages the projection and model-view matrices directly. Keyboard input updates camera position and orientation, while movement constraints keep the viewer within the intended scene region. Two-dimensional text overlays temporarily switch projection state before restoring the 3D camera.

### Animation

Animated objects update from elapsed time and per-object state. Different motion patterns are used for cars, trains, hover vehicles, drones, the robot, Ferris wheel, water, and the sky. Figure-eight drone paths demonstrate parametric motion rather than simple linear translation.

## Repository Structure

```text
.
├── texture/                         # BMP assets used by the renderer
├── CPT 205 Assessment2.cpp          # complete C++ scene and interaction logic
├── CPT 205 Assessment2.exe          # prebuilt Windows executable, if retained
├── README.md
└── *.pdf                            # coursework report or supporting document
```

The main C++ source is approximately 3,583 lines. The project is intentionally concentrated in one translation unit because it was submitted as coursework; a production refactor would separate scene objects, asset management, input, camera logic, and rendering passes.

## Requirements

- A C++ compiler with OpenGL support
- OpenGL libraries available on the platform
- FreeGLUT
- GLU
- Windows is the directly represented build environment in this repository

The code uses the traditional fixed-function OpenGL pipeline. It does not require a shader compiler, but it also does not represent a modern core-profile shader architecture.

## Build

The exact compiler and library paths depend on the local FreeGLUT installation. A typical MinGW command is:

```bash
g++ "CPT 205 Assessment2.cpp" -o city-scene.exe -lfreeglut -lopengl32 -lglu32
```

If FreeGLUT headers or libraries are not installed in the compiler's default search paths, add the appropriate `-I` and `-L` options for the local environment.

For reproducibility, document the tested compiler version, FreeGLUT build, CPU architecture, and Windows version when publishing a binary.

## Run

Keep the executable in a location from which the relative `texture/` paths resolve, then launch it from that working directory.

```powershell
.\city-scene.exe
```

If using the prebuilt executable, its filename may differ from the example above. Building from source is preferred when the compiler environment is available.

## Controls

The program provides keyboard controls for camera movement, viewing direction, and animation or scene options. Refer to the on-screen overlay and the input-handling section of `CPT 205 Assessment2.cpp` for the exact key mapping in the current revision.

When documenting or demonstrating the project, include one screenshot of the initial scene and at least one screenshot showing camera movement or animated objects. This makes the visual result and controls easier to understand than a text-only description.

## Technical Design Notes

The scene is a useful example of stateful rendering. A visually small feature can depend on several interacting concerns:

1. object geometry and its local coordinate system
2. texture memory, coordinates, and filtering
3. depth and transparency ordering
4. active lighting and material state
5. animation time and object-specific state
6. restoration of global OpenGL state before the next object is drawn

The implementation demonstrates direct responsibility for these interactions rather than delegating them to an engine.

## Verification Checklist

After building, verify:

- all BMP textures load from the expected relative paths
- the sky, ground, water, buildings, and transparent trees render correctly
- the camera responds to every documented input and respects movement limits
- trains, cars, drones, the robot, Ferris wheel, water, and sky animate without corrupting other objects
- transparent objects do not disable depth or blending state for later draws
- the 2D overlay does not permanently alter the 3D projection
- resizing the window preserves a usable viewport and projection

## Limitations

- The renderer uses fixed-function OpenGL rather than programmable shaders and buffer-oriented modern OpenGL.
- A single large source file makes extension and testing harder than a modular scene architecture.
- BMP loading and relative asset paths are intentionally simple and platform-oriented.
- Frame-rate behaviour depends on the graphics driver and hardware.
- The included Windows executable may not run on every architecture or security configuration; source builds are more transparent and reproducible.

## Public Repository Notes

- State which compiler and FreeGLUT version produced any committed executable.
- Explain the source and permitted use of every texture asset.
- Identify the purpose of the included PDF.
- Before publishing, confirm that the PDF contains no student number, private feedback, private contact details, or other information that should remain private.
- Add a licence only after confirming that both the code and bundled assets can be distributed under it.

## What the Project Demonstrates

- Direct manipulation of the OpenGL model-view and projection pipeline
- Hierarchical modelling and repeated-object construction
- Manual texture and transparency handling
- Camera-facing billboards and low-cost visual approximation
- Coordination of geometry, memory, render order, and animation state
- Design of a real-time interactive scene without a game engine

## Academic Context

This repository was developed as a computer graphics coursework project. It is presented as evidence of graphics programming and technical implementation, not as a modern production rendering engine.
