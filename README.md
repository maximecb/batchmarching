# SolidScript

A minimalistic functional programming language to define 3D shapes using implicit surfaces
(f-rep, isosurfaces, signed distance fields). A variety of CSG operations and other
functional operators are supported. Multiple examples included in the [examples](examples)
directory.

Goals:
- Define a language that is simple, concise and easy to learn
- Keep the toolchain portable, easy to install and easy to use
- Minimize external dependencies to safeguard the above
- Get to the point where we can export STL meshes of reasonable quality for 3D printing

SolidScript is implemented in Rust and draws inspiration from OpenSCAD, libfive,
ImplicitCAD and work done by Inigo Quilez.

As this language is still a prototype, it is subject to breaking changes.

## The SolidScript Language

The language is dynamically typed to make it more approachable for newcomers and
those who have not programmed before, such as people interested primarily in 3D
printing or those coming from a CAD background.

All numbers are 64-bit floats (doubles) to simplify the interpreter. The language
supports closures, which makes it easy to combine distance fields using
functional operators.

The output of each script is the last expression at the end of the file. This is
expected to be a function/closure representing a signed distance field.

Dimensions are assumed to be in millimeters, but it should be easy to resize a
model whose dimensions are in another system of units using a simple scaling
transformation.

## Usage

To load/view an example file:
```sh
cargo run -- --view examples/box.ssc
```

## Build Instructions

Dependencies:
- The [Rust toolchain](https://www.rust-lang.org/tools/install)
- The [SDL2 libraries](https://wiki.libsdl.org/SDL2/Installation)

### Installing Rust and SDL2 on macOS

Install the Rust toolchain:
```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install the SDL2 package:
```sh
brew install sdl2
```

Add this to your `~/.zprofile`:
```sh
export LIBRARY_PATH="$LIBRARY_PATH:$(brew --prefix)/lib"
```

### Installing Rust and SDL2 on Debian/Ubuntu

Install the Rust toolchain:
```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install the SDL2 package:
```sh
sudo apt-get install libsdl2-dev
```

### Installing Rust and SDL2 on Windows

Follow the Windows-specific instructions to [install the Rust toolchain](https://www.rust-lang.org/tools/install).

Get `SDL2.dll` from one of [SDL2 Releases](https://github.com/libsdl-org/SDL/releases).

Copy `SDL2.dll` (unzip) to the `vm/` folder.

### Compiling the Project

```sh
cargo build
```

You can run the test suite as follows:
```sh
cargo test
```

To verify that a source file compiles without viewing it:
```sh
cargo run examples/box.ssc
```
