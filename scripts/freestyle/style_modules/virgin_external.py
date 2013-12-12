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
from PredicatesU1D import *
from shaders import *
dir = Vec2f(0, 1)
upred = AndUP1D(QuantitativeInvisibilityUP1D(0), ExternalContourUP1D() )
bpred = TrueBP1D()
Operators.select(upred)
Operators.bidirectionalChain(ChainPredicateIterator(upred,bpred))
shaders_list = 	[
		SamplingShader(2),
		TextureAssignerShader(2),
		pyPerlinNoise1DShader(0.01, 8, 8),
		SmoothingShader(100, 0.1, 0, 0.2, 0, 0, 0, 1),
		IncreasingThicknessShader(2, 8),
		IncreasingColorShader(0.4,0.0,0.1, 0.6, 0.2, 0.0, 0.0, 0.1),
		pyModulateAlphaShader(0.0, 0.7)
		]
Operators.create(TrueUP1D(), shaders_list)
