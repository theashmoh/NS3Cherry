# Sionna RT setup for `leo-link-regen-RT`

`leo-link-regen-RT` (Example 2 in [`README.md`](README.md)) replaces the statistical channel
with [Sionna RT](https://nvlabs.github.io/sionna/rt/) ray tracing. ns-3.48 embeds Python through
pybind11 and imports `sionna.rt` in process; there is no server to start. The other examples
do not need any of this.

## Install

Tested with Sionna RT v2: `sionna-rt` 2.1.0, Dr.Jit 1.5.0, Mitsuba 3.9.1, pybind11 3.1.0 and
Python 3.12. A virtual environment keeps them apart from the system packages:

```bash
python3 -m venv ~/sionna-venv
~/sionna-venv/bin/pip install sionna-rt pybind11 cppyy
```

## Configure

ns-3 detects Sionna by importing it with the Python interpreter CMake finds, so point CMake at
the virtual environment's interpreter. ns-3 also asks CMake for the Python interpreter only,
not the development headers and library that pybind11 needs, so supply those by hand (adjust
3.12 to your Python version):

```bash
cmake -S . -B cmake-cache -DNS3_EXAMPLES=ON \
  -DPython3_EXECUTABLE=$HOME/sionna-venv/bin/python \
  -DCMAKE_CXX_FLAGS="-isystem /usr/include/python3.12" \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lpython3.12"
```

The library must go in `CMAKE_CXX_STANDARD_LIBRARIES`: flags in `CMAKE_EXE_LINKER_FLAGS` come
before the object files, where the linker discards them. Look for
`Sionna-RT support enabled: all required dependencies were found.` in the output, then build:

```bash
cmake --build cmake-cache --target leo-link-regen-RT
```

`leo-link-regen-RT` is only configured when Sionna and Eigen3 are both found.

## Run

The embedded interpreter is the system Python that ns-3 linked against, so activating the
virtual environment changes nothing. Put its packages on `PYTHONPATH` instead:

```bash
export PYTHONPATH=$HOME/sionna-venv/lib/python3.12/site-packages
```

**GPU.** At configure time ns-3 may print `CUDA toolkit (nvcc) not found. Dr.Jit will fall
back to the CPU/LLVM backend`. Dr.Jit uses the CUDA driver directly and ships its own
kernels, so an NVIDIA GPU with its driver is used without the toolkit; the message only
reflects whether `nvcc` is installed.

## Placing the terminal

Stand the terminal on open ground. Under a roof every path is blocked, the received power is
zero, and the run delivers nothing without an error. To check a spot, place a receiver there
and a transmitter straight above it, and count the paths the solver finds: zero means the spot
is covered.

Other sites: `ntn/tools/osm_to_sionna.py` turns a GeoJSON of OpenStreetMap footprints with a
`height_m` property into a scene, as `ntn/data/uw-campus/` was built. Sionna's built-in scenes
(`munich`, `san_francisco`, `etoile`, `florence`) also work, but their origins can sit inside a
building.
