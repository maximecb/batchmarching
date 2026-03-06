# Adaptive Ray Batch Marching (ARBM)

## Running the Demo

Press the numbers 1, 2, 3, 4 on your keyboard to toggle between 4 different raymarching algorithms:
1. Standard ray marching
2. Accelerated raymarching with over-relaxation (Balint & Valasek 2018)
3. Adaptive ray batch marching (ARBM)
4. ARBM with approximate/interpolated shading

```
cargo run --release
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
