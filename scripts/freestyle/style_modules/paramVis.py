from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *

Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[
#		BezierCurveShader(3),
#		pySamplingShader(2),
#		ConstantColorShader(1,1,1),
		IncreasingColorShader(1,1,0,1,0,0,1,1),
                pyConstantThicknessShader(4)
#                pyIncreasingThicknessShader(5,10),
#pyNonLinearVaryingThicknessShader(2,8,0.6), 
#		pyMaterialColorShader(0)
		]
Operators.create(TrueUP1D(), shaders_list)
