from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *


seed = 1
N = 2

Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(True),NotUP1D(QuantitativeInvisibilityUP1D(0)))
#Operators.bidirectionalChain(ChainPredicateIterator(False))
shaders_list = 	[
		pyRandomColorShader2(seed,N),
                pyConstantThicknessShader(4)
		]
Operators.create(TrueUP1D(), shaders_list)
