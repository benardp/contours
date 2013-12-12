###############################################################################
#                                                                             #
# This file is part of a style sheet example from the Freestyle application   #
# Copyright (C) 2001-2005  Stephane Grabli                                    #
#                                                                             #
# http://freestyle.sourceforge.net                                            #
#                                                                             #
###############################################################################
from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from shaders import *
Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
Operators.select(pyHigherLengthUP1D(20))
shaders_list = [
		pyBackboneStretcherShader(20),
		pyPerlinNoise1DShader(0.1, 10, 8),
		SmoothingShader(100, 0.1, 0, 0.2, 0, 0, 0, 1),
#		TextureAssignerShader(4),
		IncreasingColorShader(0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.2)
		]
Operators.create(TrueUP1D(), shaders_list)
