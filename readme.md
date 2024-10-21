# Triangles3D

## Program Purpose

This program checks if two of triangles intersect in space.

## How to build

Firstly clone this repo and go root directory.
```
git clone git@github.com:R0flan4k/Triangles3D.git
cd Triangles3D
```

If you have never used conan on your machine before, please run:
```
conan profile detect --force
```

Now you can build the project:
```
conan install . --build=missing -s compiler.cppstd=gnu20
cmake . -B build/Release -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release
```
