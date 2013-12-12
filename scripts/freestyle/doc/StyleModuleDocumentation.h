/*! \page StyleModuleDocumentation Style Module Documentation 
  
\section smds1 What is a style module ?

A Style Module is the piece of code responsible for the
stylization of a part of the drawing. Basically, it applies
a style selectively to the lines of the drawing.
The input of a Style Module is the ViewMap.
The output is a set of strokes.
A Style Module is structured as a pipeline
of identified operations that allow to build
strokes from the edges of the ViewMap.
There are 5 kinds of operations:
  - Selection (Operators::select())
  - Chaining  (Operators::chain(), Operators::bidirectionalChain())
  - Splitting (Operators::sequentialSplit(), Operators::recursiveSplit())
  - Sorting (Operators::sort())
  - Creation  (Operators::create())

All these operations act on a set of 1D active elements.
The different elements that can be active at some point
of the pipeline are:
  - ViewEdge
  - Chain
  - Stroke

Initially, the set of active elements is the set
of every ViewEdge of the ViewMap.

\subsection ss11 Selection
The selection operator parse every elements of the
active set and keeps only the ones satisfying a certain
predicate.
The Operators::select() operator takes as argument
a unary predicate that works on any Interface1D.
For example:
\code
Operators.select(QuantitativeInvisibilityUP1D(0))
\endcode
uses the QuantitativeInvisibilityUP1D predicate
to select only the visible ViewEdge. 
The selection operator is designed to apply
the style, that will be defined afterwards in the style
module, selectively.
\n

\subsection ss12 Chaining
The chaining operators act on the active set of ViewEdge and
allow the user to specify the topology of the future strokes.
The idea is to implement an iterator to traverse the ViewMap graph
marching along ViewEdges. This iterator basically needs to
tell which next ViewEdge to follow when at a given vertex.
(see ViewEdgeIterator).
Several such iterators are provided by the
system (see ChainPredicateIterator, ChainSilhouetteIterator).
The chaining operator also takes as input a UnaryPredicate
working on Interface1D as a stopping criterion. If a ViewEdge
satisfying this predicate is reached during the march along
the graph, the chaining stops.
The chaining can be unidirectional (Operators::chain()) or bidirectional
(Operators::bidirectionalChain()). In the second case, the chain
will propagate in the two directions from the starting edge.
Here is a code example of a bidirectional chaining call:
\code
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
\endcode
Here we use the ChainSilhouetteIterator as our chaining rule and we tell
the chaining to stop as soon as a non visible ViewEdge is reached.
The chaining operator processes the set of active ViewEdge in order
(this set can previously be sorted using Operators::sort()).
It starts a chain with the first ViewEdge of the set. Every ViewEdge
involved in the chain we're building can be marked (In the previous example,
the time stamp of each ViewEdge is modified by default), in order
not to process two times the same ViewEdge. Once the chaining
reaches a ViewEdge that satisfies the stopping predicate, the chain
ends. The algorithm then examines the second ViewEdge of the list and
checks its time stamp in order to know whether it already has
been processed or not. If not, we start a new chain starting
from this ViewEdge. This operation is repeted until the last
ViewEdge of the active set was processed.
Here is the chaining algorithm:
\verbatim
for ve in the set of active ViewEdge:
  if !processed(ve)
    chain = BuildChain(ve, stoppingPredicate)
    add chain to the set of active chains
\endverbatim
At the end of the chaining operation, the active set contains
all the Chains (see Chain) that were just built.

\n

\subsection ss13 Splitting

The Splitting operation can be used to refine the topology
of each Chain. Two kinds of splitting are available.
The first one is the sequential splitting (Operators::sequentialSplit()).
In its basic version, it parses the Chain at a given arbitrary resolution and evaluates
a unary predicate (working on points) at each point along
the Chain. Each time the predicate is satisfied, the
chain is split into two chains.
At the end of the sequential split operation, the active
set of chains contain the new chains.
\code
Operators.sequentialSplit(TrueUP0D(), 2)
\endcode
In this example, the chain is split every 2 units.
A more elaborated version uses two predicates instead
of one: One to determine the starting point of the
new chain and the other to determine its ending point.
This second version can lead to a set of Chains that
are disjoint or that overlap if the two predicates
are different.
(see Operators::sequentialSplit() for more details).
\n

The second kind of splitting is the recursive split (Operators::recursiveSplit()).
For each Chain, this operator evaluates a Function
on the points along the Chain at a given resolution
looking for the point that realizes the maximum
value for the Function. The Chain is then split into
two. The process is then repeted on each of the
two new Chains recursively. The algorithm stops
when the input Chain satisfy a user-specified condition.
\code
func = Curvature2DAngleF0D()
Operators.recursiveSplit(func, NotUP1D(HigherLengthUP1D(5)), 5)
\endcode
In this code example, we recursively split the Chain at points
of highest 2D curvature. The curvature is evaluated for every
points long the Chain at a resolution of 5 units. A Chain
whose length is not higher than 5 units won't be split.
\n

\subsection ss14 Sorting

The sorting operator (Operators::sort()) allows the sorting
of the active set of Interface1D. It takes as input
a binary predicate used as an "<" operator to
order two Interface1D.
\code
Operators.sort(Length2DBP1D())
\endcode
In this code example, the sorting uses the Length2DBP1D binary predicate
to sort the Interface1D by incresing 2D lengths.
\n
The sorting is particularly useful when combined
with causal density. Indeed, the causal density
evaluates the density of the resulting image as it is
modified. If we wish to use such a tool to decide
to remove strokes whenever the local density is too high,
it is important to control the order in which the strokes
are drawn. In this case, we would use the sorting operator
to insure that the most "important" lines are drawn first.
\n

\subsection ss15 Creation

Finally, the creation operator (Operators::create()), takes
the active set of Chains as input and build Strokes.
It takes two arguments: The first one is a unary predicate that
works on Interface1D that is designed to make
a last selection on the set of chains. A Chain that doesn't
satisfy the condition won't lead to a Stroke.
The second input is a list of Shaders that will
be responsible for the shading of each built stroke.
\code
shaders_list = 	[
		StrokeTextureShader("smoothAlpha.bmp", Stroke.OPAQUE_MEDIUM, 0),
		ConstantThicknessShader(2), 
		SamplingShader(5.0),
		ConstantColorShader(0.2,0.2,0.2,1), 
		]
Operators.create(DensityUP1D(8,0.1, MEAN), shaders_list)
\endcode
In this example, we use the DensityUP1D predicate to remove any Chain
whose mean density is higher than 0.1.
For each stroke, we set the texture "smoothAlpha.bmp" with an OPAQUE_MEDIUM blending
mode, we define a constant thickness of 2 units, we resample the stroke
so that it has a point every 5 units and eventually assigns a dark grey constant
color.


\section smds2 What control do we have over this pipeline ?
[ In construction ]. 
The Style Module has a fixed pipeline structure.
However, it offers a lot of controls through,
first, the sequencing of the different pipeline
control structures, and, second, through the
definition of objects that are passed as argument
all along the pipeline.

- Sequence the different pipeline control structures
  - select, chain, split, sort, create
- Possibly implement
  - Functions
  - Predicates
  - Shaders


*/
