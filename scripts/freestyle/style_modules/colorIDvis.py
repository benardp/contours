from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *

Operators.select(QuantitativeInvisibilityUP1D(0))
#Operators.bidirectionalChain(ChainSilhouetteIterator(True),NotUP1D(QuantitativeInvisibilityUP1D(0)))
#Operators.bidirectionalChain(ChainPredicateIterator(False))
shaders_list = 	[
		pyColorIDShader(),
                pyConstantThicknessShader(4)
		]
Operators.create(TrueUP1D(), shaders_list)
