/*! \page Tutorial Tutorial 
  
\section s1 Configuring Freestyle

All the python examples are located under the 'style_modules'
directory. If you define functions, predicates, shaders etc.
in files that are located in other directories than the
standard ones, you will need to complete the python path
through the "Windows > Options Window" dialog.

Warning: any modification made outside a style module itself
will not be taken into account automatically: you need to reset
the interpreter, through the "Tools > Reset Interpreter" menu item,
before redrawing the strokes.

\section s2 Before the Style
If we want to properly see the style we're about to
write, we need to open a 3D model, to choose
a proper viewpoint and to compute the ViewMap
of the scene.
You can open a 3D model through the File > Open
menu.
We'll work with the "simpleScene.3DS" model
in this tutorial.
Once it has been loaded, you can use
the trackball to choose the viewpoint you want.
Then, compute the ViewMap using the Tools > "Compute View Map"
menu item.
You have several useful display options. Here are some:
- e: show/hide the ViewMap
- 1: show the 3D model as solid
- 2: show the 3D model in wireframe
- 3: hide the 3D model

(All display options are accessible through "Help">"Control Bindings"
Thus you can see the ViewMap only by pressing
\a e and \a 3.
\n
\htmlonly
<table border=0 cellpadding=0 cellspacing=0>
  <tr>
    <td align="center" width="50%"><img src="tutorial_3D.jpg" width="100%"></td>
    <td align="center" width="50%"><img src="tutorial_ViewMap.jpg" width="100%"></td>
  </tr>
  <tr>
    <td align="center">the 3D scene</td>
    <td align="center">the ViewMap</td>
  </tr>
</table>
\endhtmlonly
\n
The ViewMap is now ready, we can write our Style Module.
Let us create an empty file named "StyleModule0.py" in which
we'll write our style module.
Then, display the Style Modeler window using the
Windows > "Style Modeler Window" menu item, and press
the '+' button that opens a file browsing window.
Load then the "StyleModule0.py" file.
A line with the StyleModule0.py is now visible
in the Style Modeler window.
Double-click on this line to make the Editor appear.
We'll write our Style Module using this editor.
Any change made in the file can be saved using the
"save" button.

\section s3 My first style module
Let us edit the file "StyleModule0.py" and
try to write a style module that paints each visible line
from red to green.
The first line we need to write is this one:
\n
\code
from Freestyle import *
\endcode
\n
It is the "include" instruction needed to use
any Freestyle element in python.
Any object that would be defined outside our StyleModule0.py
file would require such an import command to be used
in our style module.
Through this import we can access all the Operators of the pipeline
(see Operators) as well as any C++ predefined Function, Predicate
or Shader.\n
We are now going to write the selection part of the style module:
We said we want to apply our style only to visible lines, we therefore
need to use the select operator together with a predicate that
answers true for any visible line and false
in every other cases.
The QuantitativeInvisibilityUP1D built-in predicate exactly
matches our need: It takes an integer as argument and answers true
for any Interface1D whose quantitative invisibility equals this number.
Here is how our selection line must look like:
\n
\code
Operators.select(QuantitativeInvisibilityUP1D(0))
\endcode
\n
After that call, our active set of elements contains all the visible ViewEdge
of the ViewMap. We can already build strokes from these ViewEdge
(without chaining) and apply some shading.
We said we wanted the color of our Strokes to eventually vary from
red to green. The IncreasingColorShader StrokeShader is designed for this kind
of painting.
We therefore add this StrokeShader to the list of shaders that
will be used to assign visual attributes to strokes:
\n
\code
shaders_list = [IncreasingColorShader(1,0,0,1, 0,1,0,1)] 
\endcode
\n
The first four numbers passed to the IncreasingColorShader constructor
are the R,G,B and Alpha components of the first color (here, a solid
red) and the four next ones are the R,G,B and Alpha components of the second
color (here, a solid green).
We just need to actually create the strokes from our visible ViewEdge
and our shaders list by calling the Operators::create() operator:
\n
\code
Operators.create(TrueUP1D(), shaders_list)
\endcode
\n
The TrueUP1D predicate passed as the first argument of this
operator returns true for any Interface1D. It just means
we want to build a Stroke for every ViewEdge.
The second argument is the list of Shaders we just built.
Here is how the whole Style Module now looks like:
\n
\code
from Freestyle import *

Operators.select(QuantitativeInvisibilityUP1D(0))
shaders_list = 	[IncreasingColorShader(1,0,0,1, 0,1,0,1)]
Operators.create(TrueUP1D(), shaders_list)
\endcode
\n
We can compute the strokes from the ViewMap and this Style Module
by selecting the Tools > "Compute Strokes" in the main window.
Here is the result you should get:
\htmlonly
<table align="center" width="50%" border=0 cellpadding=0 cellspacing=0>
  <tr>
    <td align="center" width="100%"><img src="tutorial_ViewEdges.jpg" width="100%"></td>
  </tr>
  <tr>
    <td align="center">The strokes built using StyleModule0.py</td>
  </tr>
</table>
\endhtmlonly
\n
We can increase a little bit the thickness of our strokes
using the ConstantThicknessShader shader.
Let us add this shader into our shaders list:
\n
\code
shaders_list = 	[IncreasingColorShader(1,0,0,1, 0,1,0,1),
                 ConstantThicknessShader(4)]
\endcode
\n
Here is our new result:
\htmlonly
<table align="center" width="50%" border=0 cellpadding=0 cellspacing=0>
  <tr>
    <td align="center" width="100%"><img src="tutorial_ViewEdges2.jpg" width="100%"></td>
  </tr>
  <tr>
    <td align="center">The strokes built using StyleModule0.py including the ConstantThicknessShader</td>
  </tr>
</table>
\endhtmlonly
\n
Because no chaining was specified, each stroke
corresponds to a single ViewEdge, which leads
to short strokes.
Let us use a chaining based on the ChainSilhouetteIterator iterator, which
chains together all connected visible ViewEdges
of same nature. We choose a bidirectional chaining algorithm in order
to build the longest possible strokes. We also
want our chains to stop whenever we reach a non visible
ViewEdge.
Here is the corresponding chaining code:
\n
\code
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
\endcode
\n
This operation comes directly after the selection. Thus, we start
a new Chain for every ViewEdge of the selection that has not already been
processed.
Here is how the whole Style Module now looks like:
\n
\code
from Freestyle import *

Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[IncreasingColorShader(1,0,0,1, 0,1,0,1)]
Operators.create(TrueUP1D(), shaders_list)
\endcode
\n
And here is our new result:
\htmlonly
<table align="center" width="50%" border=0 cellpadding=0 cellspacing=0>
  <tr>
    <td align="center" width="100%"><img src="tutorial_Strokes.jpg" width="100%"></td>
  </tr>
  <tr>
    <td align="center">The strokes built using the chaining operator in StyleModule0.py</td>
  </tr>
</table>
\endhtmlonly
\n

\section s4 A more complex example [to come]
- Using Splitting
- Using sorting + density
- Organising a Style into several layers (StyleModules)

\section s5 Implementing our own objects.
So far we only used C++ predefined objects. However, our
operators are programmable in the sense that you can
program your own predicates, iterators, shaders, etc. and
use them instead of the ones that were seen above.
For instance, assuming
that you want a shader that assigns a thickness proportionally
to the inverse of the depth, you need to code
this shader as well as any needed function yourself.
The idea is that all the basic programming components required to write
powerful style rules are provided that should.
For instance, all information access is provided so that 
you can define any new information-based Function, Predicate or Shader,
and as our style description language is Python, these ones
can be of arbitrary complexity.
\n
For modularity issues, we recommend separating
the Functions, Predicates and Shaders definitions
from the Style Module files.
Indeed, a shader might be used in more than one Style Module.
Therefore, we suppose here that any Function, Predicate or Shader
will be written in a new file.


Implementing your own functions often implies the use
of many of the functions provided to access the information
(eg, 3D or 2D coordinates, depth discontinuity, line nature, etc.)
It is therefore essential to have a basic idea of how
these mechanisms work: Information can be accessed from
two high-level types of elements: points (OD elements) or
lines (1D elements). In practice, 0D elements can be 
arbitrary points along a ViewEdge, ViewVertex or StrokeVertex
(those are not exclusive types) and 1D elements can be
either ViewEdge, Chain or Stroke. In the general case,
you don't need to distinguish between one type or another within
a same category: Elements that are manipulated are therefore
either 0D elements or 1D elements (it is possible to retrieve
the specialized object from them when necessary).
The OD elements are manipulated through the Interface0DIterator type,
an iterator over
the points of a 1D element (Iterators instead of simple points,
as they encapsulate a 1D context that might be needed in the context of many
information queries), and the 1D elements are manipulated through
the Interface1D type.

\section s6 Implementing your own Functions
Among the functions provided to retrieve information
from points, we find the
GetProjectedZF0D functor that returns the depth value (in camera space)
for any Interface0DIterator. This function returns a value between 0 and 1.
Let us implement directly in python a simple function that returns the
"inverse depth", 1-z.
\n

First, as for any python file that would need objects
defined in Freestyle, we need to import the Freestyle module:
\n
\code
from Freestyle import *
\endcode
\n
We choose to call our Functor pyGetInverseProjectedZF0D. The "py"
prefixe indicating that the functor was defined in Python and the
"F0D" suffixe indicates that we're dealing with a Unary Function working
in 0D (on points).
As any functor that acts on Interface0DIterator, we must inherit
from UnaryFunction0D. In Python, there are different UnaryFunction0D
depending on the Function return value type.
Here, we will return a double, therefore, we inherit from
UnaryFunction0DDouble.
Here is the declaration line for our functor:
\n
\code
class pyGetInverseProjectedZF0D(UnaryFunction0DDouble):
\endcode
\n
We must now overload the "()" operator. In python the
corresponding method is named \a __call__.
Thus, our functor skeleton looks like this:
\n
\code
class pyGetInverseProjectedZF0D(UnaryFunction0DDouble):
	def __call__(self, inter):
                ## add the code here
                return 0
\endcode
\n
If needed, the constructor might be overloaded
through the \a __init__ method:
\n
\code
class pyGetInverseProjectedZF0D(UnaryFunction0DDouble):
        def __init__(self):
                ## add initialization code here
                
	def __call__(self, inter):
                ## add the code here
                return 0
\endcode
\n
To compute the inverse projected Z we use the GetProjectedZF0D functor:
\n
\code
class pyGetInverseProjectedZF0D(UnaryFunction0DDouble):
        def __init__(self):
                ## add initialization code here
                
	def __call__(self, inter):
                func = GetProjectedZF0D()
                return 1.0-func(inter)
\endcode
\n
We must recall here that our functions are in reality functors and
must therefore be instanciated (as objects) before use.
\n
We're done! Any Style Module or python file that wants to
use this function must import the module in which it is defined.
For example, we can use this Functor in the recursive split
operator in order to split the Chains at the max depth value:
\n
\code
func = pyGetInverseProjectedZF0D()
Operators.recursiveSplit(func, NotUP1D(pyHigherLengthUP1D(5)), 5)
\endcode
\n
This code evaluates pyGetInverseProjectedZF0D every 5 units along each
Chain and splits in two at the point realizing the min value.
We stop splitting when Chains length are less than 5 units.

\section s7 Implementing your own Predicates
In the previous example we needed  a predicate working on Interface1D and
telling whether its length was higher than a certain value or not.
Let us code this Predicate in python.
First, as for any python file that would need objects
defined in Freestyle, we need to import the Freestyle module:
\n
\code
from Freestyle import *
\endcode
\n
We call this predicate pyHigherLengthUP1D. The "py" prefix indicating
that we're dealing with a predicate defined in python, the "UP1D" suffix
indicating that it is a Unary Predicate working on 1D elements.
As any unary predicate of this kind, it must inherit from
UnaryPredicateUP1D.
Therefore, the declaration looks like this:
\n
\code
class pyHigherLengthUP1D(UnaryPredicate1D):
\endcode
\n
We want the user to have control over the threshold length value.
It must therefore be a parameter given at the predicate construction:
\n
\code
class pyHigherLengthUP1D(UnaryPredicate1D):
	def __init__(self,l):
		UnaryPredicate1D.__init__(self)
		self._l = l
\endcode
\n
The constructor (\a __init__) takes an argument \a l and
uses it to initialize a data member called \a _l.
As for Functors, the work must be done in the () operator,
i.e. the \a __call__ method in python:
\n
\code
class pyHigherLengthUP1D(UnaryPredicate1D):
	def __init__(self,l):
		UnaryPredicate1D.__init__(self)
                self._GetLength2D = GetLength2DF1D()
		self._l = l
                
	def __call__(self, inter):
		return (self._GetLength2D(inter) > self._l)
\endcode
\n
The implementation uses the GetLength2DF1D() access function provided
for any Interface1D to get its 2D length.
\n
Here is a code example of selection using this Predicate:
\n
\code
Operators.select(pyHigherLengthUP1D(20))
\endcode
\n
This operation selects the active Interface1D whose length
is higher than 20 units.

\section s8 Implementing your own Shaders
The objects that need to be redefined the most often are shaders.
Indeed, they're responsible for a great part of the style
and must often produce very specific visual effects.
Implementing a Shader might take a lot of time and be very complex
depending on the shading you want to achieve.
We'll illustrate this task on a simple example.
Suppose we want to write a Shader that assigns to the vertices
of a Stroke a thickness that linearly goes from a thickness t1 to a thickness
t2 between the begining and the end of the Stroke.
Once again, we first need to import the Freestyle module:
\n
\code
from Freestyle import *
\endcode
\n
We chose to call this Shader, pyVaryingThicknessShader.
"py" for python, "Shader" for ... shader.
It must inherit from the Shader base class, StrokeShader:
\n
\code
class pyVaryingThicknessShader(StrokeShader):
\endcode
\n
This shader must offer to the user the control over
the two thicknesses that will be used, therefore we
must redefine the constructor:
\n
\code
class pyVaryingThicknessShader(StrokeShader):
         def __init__(self, t1, t2):
		StrokeShader.__init__(self)
		self._t1 = t1
		self._t2 = t2
\endcode
\n
The two thicknesses are stored in the data members \a _t1 and \a _t2.
The shading implementation occurs in the \a shade method, that
we must implement. This method takes as argument the
Stroke that must be shaded (see StrokeShader::shade()).
The basic way to achieve any shading operation consists
in iterating over the StrokeVertices of the Stroke
and to modify each one's StrokeAttribute.
Here is the python code for the skeleton of such an iteration:
\n
\code
it = ioStroke.strokeVerticesBegin()
while it.isEnd() == 0:
        att = it.getObject().attribute()
        ## perform here any attribute modification
        it.increment()
\endcode
\n
And here is our complete shader:
\n
\code
class pyVaryingThicknessShader(StrokeShader):
         def __init__(self, t1, t2):
		StrokeShader.__init__(self)
		self._t1 = t1
		self._t2 = t2
                
	def shade(self, stroke):
		n = stroke.strokeVerticesSize()
		i = 0
		it = stroke.strokeVerticesBegin()
		it_end = stroke.strokeVerticesEnd()
		while it.isEnd() == 0:
			att = it.getObject().attribute()
			c = float(i)/float(n)
			t = (1.0 - c)*self._t1 + c * self._t2
			att.setThickness(t/2.0, t/2.0)
			i = i+1
			it.increment()
\endcode
\n
This shader can then be used as any other shader in a Style Module:
\n
\code
shaders_list = [ConstantColorShader(1,0,0,1),
                pyVaryingThicknessShader(2, 10)]
Operators.create(TrueUP1D(), shaders_list)                
\endcode
\n
This will build red strokes whose thickness varies linearly from
2 to 10 units.

*/
