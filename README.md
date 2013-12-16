Contours
==

This is the research prototype implementing the occluding contours
extraction method described in the following publication:

[Computing Smooth Surface Contours with Accurate Topology.](http://www.labri.fr/perso/pbenard/publications/contours.html)
Pierre Bénard, Aaron Hertzmann, Michael Kass. ACM Transactions on
Graphics, 2013


Note: This is research software. As such, it may fail to run, crash,
or otherwise not perform as expected. It is not intended for regular
use in any kind of production pipeline.

-

### Dependencies

The code is based on a skimmed and tweaked version of
[OpenSubdiv](http://graphics.pixar.com/opensubdiv/) and a heavily
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
* [SWIG](http://www.swig.org/) (Freestyle)

Optional:
* [GLEW](http://sourceforge.net/projects/glew/) (Linux only)
* [OpenMP](http://openmp.org/wp/) (OpenSubdiv)
* [TBB](https://www.threadingbuildingblocks.org/) (OpenSubdiv)


### Build instructions
Tested on Linux (gcc 4.8) and OS X (clang 5.0), not tested on Windows.

__Clone the repository:__

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

### Usage

We provide scripts to run the mesh generation algorithm and contours
extraction code on a sequence of RIB files. We provide three test
sequences on the
[project web page](http://www.labri.fr/perso/pbenard/publications/contours.html);
you can export other shots using RenderMan or 3Delight for Maya, or
[3Dlight/Blender](http://mattebb.com/3delightblender/).

The settings are specified in `build/scripts/settings.py`. You need to
provide the local paths to the input and output shots directories on
your machine through the variables `mainTestShotFolder` and
`mainOutputFolder` (mandatory). The `shot` parameter allows you to
choose the sequence that you want to process; it needs to match
the name of the subdirectory containing the RIB files.

Then, to run the code:

````
# from the "build" directory
cd scripts
python npr.py
````

The output PLY meshes and curves (EPS, PDF and, on OS X, PNG files)
are saved in a subdirectory of `mainOutputFolder`. 
