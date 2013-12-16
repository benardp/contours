Contours
==

This is the research prototype implementing the occluding contours
extraction method described in the following publication:

[Computing Smooth Surface Contours with Accurate Topology.](http://www.labri.fr/perso/pbenard/publications/contours.html)
Pierre Bénard, Aaron Hertzmann, Michael Kass


Note: This is research software. As such, it may fail to run, crash,
or otherwise not perform as expected. It is not intended for regular
use in any kind of production pipeline.

-

### Dependencies

The code is based on a skimmed and tweaked version of
[OpenSubdiv](http://graphics.pixar.com/opensubdiv/) and an heavily
modified version of [Freestyle](http://freestyle.sourceforge.net/)
(included).  The mesh generation algorithm is implemented as a
RifFilter; you will either need
[Pixar's RenderMan](http://renderman.pixar.com/) or
[3Delight Studio Pro](http://www.3delight.com/) (free licence
available) to build and run it.

Required:
* [cmake](http://www.cmake.org/cmake/resources/software.html)
* [Qt 4.8 libraries](http://qt-project.org/downloads) (Freestyle)
* [libQGLViewer](http://www.libqglviewer.com/) (Freestyle)

Optional:
* [GLEW](http://sourceforge.net/projects/glew/) (Linux only)
* [OpenMP](http://openmp.org/wp/) (OpenSubdiv)
* [TBB](https://www.threadingbuildingblocks.org/) (OpenSubdiv)

-

# Build instructions
On Linux (gcc) and OS X (clang), not tested on Windows.

__Clone the repository:__

From the GitShell, Cygwin or the CLI :

````
git clone https://github.com/benardp/contours
````

__Generate Makefiles and build the project:__

Assuming that you want the binaries installed into a "build" directory
at the root of the source tree :

````
cd contours
mkdir build
cd build
cmake ..
make
````

-

# Usage

We provide scripts to run the mesh generation algorithm and contours
extraction code on a sequence of RIB files.

All the settings need to be set in `build/scripts/settings.py`. Specify the
local paths to the input and output shots directories in the variables
`mainTestShotFolder` and `mainOutputFolder` (mandatory). 

Then, to run the code:

````
# from the "build" directory
cd scripts
python npr.py
````
