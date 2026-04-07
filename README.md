# FiDiCoM

FiDiCoM is a C++ research codebase for spatial analysis using geometric network generation.
Shortest path calculation, and visibility analysis are available.
Applications are follows:
- desire path simulation
- facility location problem

## Requirements

The project is built with CMake and depends on these C++ libraries:

- CGAL
- Boost
- Eigen3
- pagmo2
- OpenMP
- nlohmann-json

On Ubuntu, the included Dockerfile installs the required system packages and builds pagmo2 from source.

## Repository Layout

- `cpp/`: source code and CMake files
- `docker/`: Docker image and container scripts

## Build

### Native build

Release:

```bash
cd cpp
cmake -S . -B ../build -DCMAKE_BUILD_TYPE=Release
cmake --build ../build
```

Debug:

```bash
cd cpp
cmake -S . -B ../debug -DCMAKE_BUILD_TYPE=Debug
cmake --build ../debug
```

The executable is created as `build/main` or `debug/main`.

### Docker build

```bash
cd docker
docker build -t fidicom:latest .
docker run -itd \
	-v /your/path/to/project/directory:/home/workspace \
	--detach-keys "ctrl-x" \
	--name fidicom \
	fidicom:latest
```

## Run

The program selects behavior by numeric mode:

```bash
./build/main <mode>
```

Examples:

- `./build/main 1`: 2D network generation, shortest path, visibility, and reachability output using files under `/home/workspace/data/`
- `./build/main 16`: AED location planning mode; after launch, enter a dataset folder path such as `data/simulation_UTKomaba2`

Some other modes still use hard-coded absolute paths such as `/home/builder/workspace/...`, so they may require path changes before running outside the original environment.

## Notes

- Generated outputs are written into the `data/` tree, which is ignored by git.
