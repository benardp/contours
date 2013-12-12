from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *


#aaron's attempt at a QI-based color shader.  doesn't seem to work.

#Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), FalseUP1D())
shaders_list = 	[
		StrokeTextureShader("smoothAlpha.bmp", 
				     Stroke.OPAQUE_MEDIUM, 
				     0),

#		BezierCurveShader(3),
		pySamplingShader(2),
		pyQIColorShader(),
#		IncreasingColorShader(0.2,0.2,0.2,1,0.5,0.5,0.5,1),
		pyNonLinearVaryingThicknessShader(2,8,0.6), 
#		pyMaterialColorShader(0)
		]
Operators.create(TrueUP1D(), shaders_list)
