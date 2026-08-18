# Interactive 3D City Scene

This repository contains a CPT205 assessment project written in C++ with OpenGL and FreeGLUT. It renders an interactive, textured 3D city scene with animated vehicles, trains, drones, buildings, a Ferris wheel, and day/night visual effects.

## Contents

```text
.
|-- CPT 205 Assessment2.cpp       # Main application source code
|-- CPT 205 Assessment2.exe       # Supplied Windows executable
|-- CPT 205 Assessment2.pdf       # Assessment document
`-- texture/                      # Bitmap textures required by the scene
```

## Requirements

To build the application from source, configure a C++ compiler with:

- FreeGLUT
- OpenGL
- GLU

The supplied executable is intended for Windows. When running either a compiled build or the supplied executable, keep the `texture/` directory in the same location because the program loads texture assets using relative paths.

## Build and Run

The following is an example command for a MinGW-based Windows environment after FreeGLUT, OpenGL, and GLU have been installed and configured:

```bash
g++ "CPT 205 Assessment2.cpp" -o "CPT 205 Assessment2.exe" -lfreeglut -lopengl32 -lglu32
```

Then run the generated executable from the repository root:

```bash
"CPT 205 Assessment2.exe"
```

## Controls

| Input | Action |
| --- | --- |
| Arrow keys | Move the camera horizontally and forwards/backwards. |
| `W` / `S` | Look up / down. |
| `A` / `D` | Rotate the camera left / right. |
| `Q` / `E` | Move the camera up / down. |
| `H` | Toggle the high-altitude view. |
| `1` / `2` | Start/stop or show/hide the train. |
| `3` / `4` | Start/stop or show/hide the car. |
| `5` / `6` | Start/stop or show/hide the drones. |

## Notes

This is an academic graphics project. The included source code, textures, executable, and report are provided for demonstration and coursework review.
