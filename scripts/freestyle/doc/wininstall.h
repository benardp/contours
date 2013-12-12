/*! \page wininstall

\htmlonly
<div align="center">    
<h1  class="brighttitle"><font class="alt">w</font>indows
<font class="alt">i</font>nstallation <font class="alt">i</font>nstructions
</h1>
</div>

<div class="corpus">
<ul>
<li><a href="#exe">&raquo;Running the precompiled executable</a></li>
<li><a href="#vc">&raquo;MS Visual C++ 2003/2005</a></li>
<li><a href="#cygwin">&raquo;Cygwin</a></li>
</ul>


<hr/>
<a name="vc"></a>
<h2 class="brighttitle"><font class="alt">R</font>unning the precompiled executable</h2>

<ul>
  <li>Unzip the package somewhere </li>
  <li>Run <code>vcredist_x86.exe</code>.
  <br>This installs the libraries required for binaries built by visual C++ 2005. <a href="http://www.microsoft.com/downloads/details.aspx?familyid=200B2FD9-AE1A-4A14-984D-389C36F85647&displaylang=en">More information...</a></li>
  <li>Run <code>Freestyle.exe</code></li>
</ul>
  
<hr/>
<a name="vc"></a>
<h2 class="brighttitle"><font class="alt">V</font>isual
C++ 2003 or 2005</h2>

<h3>Requirements</h3>

don't be affraid...
<ul>
  <li><b>MS Visual C++ 2003 or 2005</b> [free] <br />
  (<a href="http://msdn.microsoft.com/vstudio/express/downloads/">Visual 2005 Express Edition is free</a>. Make sure to follow all instructions and to also install the Microsoft Platform SDK as instructed.)</li>
<li><b>OpenGL</b><br/> [free]
 This should be part of the standard VC++ installation.</li>
<li><b><a href="http://www.xmission.com/~nate/glut.html">GLUT</a></b> [free]<br/>
 The OpenGL Utility Toolkit (GLUT).       
</li>
  <li><b>QT 4.2.3</b><br/>
  QT is an open source multi-platform GUI and is free for open source projects.
  The trolltech win32 distribution targets the MinGW compiler but you can patch
  Qt4 sources to make it work with visual studio, as <a href="http://qtnode.net/wiki/Qt4_with_Visual_Studio">detailed here</a>.
</li>
  <li><a href="http://artis.imag.fr/Members/Gilles.Debunne/QGLViewer/installWindows.html"><b>libQGLViewer 2.2.5-1</b></a> [free]<br/>
  QGLViewer is a trackball-based 3D viewer including many useful features. It is packed as a QT widget and
distributed as a free open source library.
</li>
<li><a href="http://lib3ds.sourceforge.net/"><b>lib3ds 1.2</b></a> [free]<br/>
The free open source library allowing to load and save 3DS files.
</li>
<li><b><a href="http://www.swig.org/download.html">swig 1.3.31</a></b> [free]<br/>
 SWIG is an interface compiler that connects programs written in C and C++ with scripting languages
such as Perl, Python, Ruby, and Tcl. Freestyle uses it to generate the binding between
its C++ API and the style scripting language Python.
Precompiled binaries for windows are provided under the name swigwin-X.X.X.
</li>
<li><b><a href="http://www.python.org/download/">Python 2.5</a></b> [free]<br/>
Python is an interpreted, interactive, object-oriented programming language. We chose
it as our style scripting language. The python distribution also includes precompiled
binaries for windows.</li>
</ul>


<h3>Compilation</h3>

<ul>
<li>1) You must set a <code>FREESTYLE_DIR</code> environment variable
containing the freestyle directory location.</li>
<li>2) In <code>$FREESTYLE_DIR\src</code> edit the <code>libconfig.pri</code>.
This file contains the locations of the various libraries headers and files.
Modify the initial values facing the <code>win32</code> tags to match your own configuration.</li>
<li>3) In <code>$FREESTYLE_DIR\src</code>, run <code>makedsp.vcnet.debug.bat</code> (or <code>makedsp.vcnet.release.bat</code> if you specified <code>release</code> in the <code>src\Config.pri</code> file. This
will generate the VC++ <code>vcproj</code> project files from the <code>pro</code> files
in each subdirectory of freestyle.</li>
<li>4) In <code>$FREESTYLE_DIR\src</code>, open <code>Freestyle-vc8-debug.sln</code> (or <code>Freestyle-vc8-release.sln</code> if you're working in release mode. For visual c++ 2003, use the files containing vc7 instead of vc8 in their names).
This will open the Freestyle working space in VC++.</li>
<li>5) Set the proper active configuration (<code>debug</code> or <code>release</code>)
  for Freestyle (depending on the config specified
  in the <code>src\Config.pri</code> file) in the 'Build' > 'Configuration Manager' menu action and
compile the whole project using 'Build Solution' in the 'build' menu.
This will build Freestyle.exe as well as all its libraries in the <code>$FREESTYLE_DIR\build\win32\debug</code>
or <code>release</code> directory depending on your configuration.
Only the swig library remains to build as it
is currently not integrated in the QT process.
</li>
<li>5) In <code>$FREESTYLE_DIR\src\swig</code>, open the <code>FreestyleWrapper.vc8.vcproj</code> (or vc7),
set the proper configuration through 'Build' > 'Set Active Configuration' and
build the library using 'Build' > 'Build _Freestyle.dll'.
(Don't worry about the zillion warnings).
The <code>PYTHON_INCLUDE</code> and <code>PYTHON_LIB</code> environment variables must be defined
(For instance <code>PYTHON_INCLUDE: C:\Python25\include</code>
and <code>PYTHON_LIB: C:\Python24\libs\python25.lib</code>).
This step builds <code>_Freestyle.dll</code> and <code>Freestyle.py</code>, i.e. the python API for style description,
in the <code>$FREESTYLE_DIR\build\win32\release\python</code> directory (or <code>debug\python</code>).
</li>
<li>6) Run Freestyle, open the 'Windows' > 'Options Window', select the 'Directories tab'
and add to the 'Python path' the location of the directory where the library of the previous
was built (e.g. <code>$FREESTYLE_DIR\build\win32\release\python</code>).
Save and close the dialog window
(you might need to reset the Python Interpreter through 'Tools' > 'Reset Interpreter').
</li>
<li>7) Write great style modules and generate beautiful images. (Start perhaps by the
<a href="Tutorial.html">tutorial</a>).</li>
</ul>


<hr/>
<a name="cygwin"></a>
<h2 class="brighttitle"><font class="alt">C</font>ygwin</h2>

<h3>Requirements</h3>

<ul>
  <li><b>gcc 4.0 or 4.1 </b> [included in cygwin]</li>
<li><b>OpenGL</b> and <b>GLUT</b> [included in cygwin]</li>
  <li><b>QT 4.2.3</b> [free]<br/>
  QT is a free open source multi-platform GUI.
  The distribution provided by trolltech for MinGW should compile fine on cygwin
(Opengl must be enabled).
</li>
  <li><a href="http://artis.imag.fr/Members/Gilles.Debunne/QGLViewer/installWindows.html"><b>libQGLViewer 2.2.5-1</b></a> [free]<br/>
  QGLViewer is a trackball-based 3D viewer including many useful features. It is packed as a QT widget and
distributed as a free open source library. 
</li>
<li><a href="http://lib3ds.sourceforge.net/"><b>lib3ds 1.2</b></a><br/>
The free open source library allowing to load and save 3DS files.
</li>
<li><b><a href="http://www.swig.org/download.html">swig 1.3.31</a></b> [included in cygwin]<br/>
 SWIG is an interface compiler that connects programs written in C and C++ with scripting languages
such as Perl, Python, Ruby, and Tcl. Freestyle uses it to generate the binding between
its C++ API and the style scripting language Python.
</li>
<li><b><a href="http://www.python.org/download/">Python 2.5</a></b> [included in cygwin]<br/>
Python is an interpreted, interactive, object-oriented programming language. We chose
it as our style scripting language.</li>
</ul>

<h3>Compilation</h3>

<ul>
<li>1) You must set a <code>FREESTYLE_DIR</code> environment variable
containing the freestyle directory location.</li>
<li>2) In <code>$FREESTYLE_DIR/src</code> edit the <code>libconfig.pri</code>.
This file contains the locations of the various libraries headers and files.
Modify the initial values facing the <code>cygwin-g++</code> tags to match your own configuration.</li>
<li>3) Build Freestyle:<br/>
             <code> cd $FREESTYLE_DIR/src<br/>
              qmake<br/>
              make<br/>
       </code>
This will build Freestyle.exe as well as all its libraries in the <code>$FREESTYLE_DIR\build\cywin-g++\release</code>
or <code>debug</code> directory depending on your configuration.
Only the swig library remains to build as it
is currently not integrated in the QT process.
</li>
<li>4) Build the Python API:<br/>
<code> cd $FREESTYLE_DIR/src/swig<br/>
              make -f Makefile.cygwin<br/>
              make -f Makefile.cygwin install<br/>  
       </code>
This step builds <code>_Freestyle.dll</code> and <code>Freestyle.py</code>, i.e. the python API for style description,
in the <code>$FREESTYLE_DIR/build/cygwin-g++/release/python</code> directory (or <code>debug/python</code>).
</li>
<li>5) Run Freestyle:
<code>
cd $FREESTYLE_DIR/build/cygwin-g++/release/<br/>
              ./Freestyle<br/>
       </code>
Open the 'Windows' > 'Options Window', select the 'Directories tab'
and add to the 'Python path' the location of the directory where the library of the previous
was built (e.g. <code>$FREESTYLE_DIR/build/cygwin-g++/release/python</code>).
Save and close the dialog window
(you might need to reset the Python Interpreter through 'Tools' > 'Reset Interpreter').
</li>
<li>7) Write great style modules and generate beautiful images. (Start perhaps by the
<a href="Tutorial.html">tutorial</a>).</li>
</ul>
</div>
\endhtmlonly
*/
