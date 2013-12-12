/*! \page linuxinstall

\htmlonly

<div align="center">    
<h1  class="brighttitle"><font class="alt">l</font>inux
<font class="alt">c</font>ompilation <font class="alt">i</font>nstructions
</h1>
</div>

<div class="corpus">
<h3>Requirements</h3>

<ul>
  <li><b>gcc 4.0 or 4.1 </b></li>
<li><b>OpenGL</b> and <b>GLUT</b></li>
  <li><b><a href="http://www.trolltech.com/download/qt/x11.html">QT 4.2.3</a></b><br/>
  Qt is a multi-platform GUI toolkit.
Trolltech Qt is free and open source for Linux.
</li>
  <li><a href="http://artis.imag.fr/Members/Gilles.Debunne/QGLViewer/installUnix.html#linux"><b>libQGLViewer 2.2.5-1</b></a><br/>
  QGLViewer is a trackball-based 3D viewer including many useful features. It is packed as a QT widget and
distributed as a free open source library. 
</li>
<li><a href="http://lib3ds.sourceforge.net/"><b>lib3ds 1.2</b></a><br/>
The free open source library allowing to load and save 3DS files.
</li>
<li><b><a href="http://www.swig.org/download.html">swig 1.3.31</a></b><br/>
 SWIG is an interface compiler that connects programs written in C and C++ with scripting languages
such as Perl, Python, Ruby, and Tcl. Freestyle uses it to generate the binding between
its C++ API and the style scripting language Python.
</li>
<li><b><a href="http://www.python.org/download/">Python 2.5</a></b><br/>
Python is an interpreted, interactive, object-oriented programming language. We chose
it as our style scripting language.</li>
</ul>

<h3>Compilation</h3>

<ul>
<li>1) You must set a <code>FREESTYLE_DIR</code> environment variable
containing the freestyle directory location.</li>
<li>2) In <code>$FREESTYLE_DIR/src</code> edit the <code>libconfig.pri</code>.
This file contains the locations of the various libraries headers and files.
Modify the initial values facing the <code>linux-g++</code> tags to match your own configuration.</li>
<li>3) Build Freestyle:<br/>
             <code> cd $FREESTYLE_DIR/src<br/>
              qmake<br/>
              make<br/>
       </code>
This will build Freestyle.exe as well as all its libraries in the <code>$FREESTYLE_DIR\build\linux-g++\release</code>
or <code>debug</code> directory depending on your configuration.
Only the swig library remains to build as it
is currently not integrated in the QT process.
</li>
<li>4) Build the Python API:<br/>
<code> cd $FREESTYLE_DIR/src/swig<br/>
              make -f Makefile.linux<br/>
              make -f Makefile.linux install<br/>  
       </code>
This step builds <code>_Freestyle.so</code> and <code>Freestyle.py</code>, i.e. the python API for style description,
in the <code>$FREESTYLE_DIR/build/linux-g++/release/python</code> directory (or <code>debug/python</code>).
</li>
<li>5) Run Freestyle:
<code>
set LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib<br/>
cd $FREESTYLE_DIR/build/linux-g++/release/<br/>
              ./Freestyle<br/>
       </code>
Open the 'Windows' > 'Options Window', select the 'Directories tab'
and add to the 'Python path' the location of the directory where the library of the previous
was built (e.g. <code>$FREESTYLE_DIR/build/linux-g++/release/python</code>).
Save and close the dialog window
(you might need to reset the Python Interpreter through 'Tools' > 'Reset Interpreter').
</li>
<li>7) Write great style modules and generate beautiful images. (Start perhaps by the
<a href="Tutorial.html">tutorial</a>).</li>
</ul>

</div>
\endhtmlonly
*/
