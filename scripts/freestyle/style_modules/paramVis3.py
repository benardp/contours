from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *

seed = 4
N = 2


Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[
#		BezierCurveShader(3),
#		pySamplingShader(2),
#		ConstantColorShader(1,1,1),
		pyIncreasingRandomColorShader(seed,N),
#                pyConstantThicknessShader(4),
#                IncreasingThicknessShader(0.5,2),   # cow
#                IncreasingThicknessShader(2,6),   # red
                IncreasingThicknessShader(4,12),   # redposes
#                pyNonLinearVaryingThicknessShader(2,10,0.6),    #balloons
#		pyMaterialColorShader(0)
		]
Operators.create(TrueUP1D(), shaders_list)
