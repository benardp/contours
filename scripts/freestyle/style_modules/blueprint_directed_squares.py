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
upred = AndUP1D(QuantitativeInvisibilityUP1D(0), ContourUP1D())
bpred = SameShapeIdBP1D()
Operators.select(upred)
Operators.bidirectionalChain(ChainPredicateIterator(upred,bpred))
Operators.select(pyHigherLengthUP1D(200))
shaders_list = [
		pyBluePrintDirectedSquaresShader(4, 20, 1.4),
		pyPerlinNoise1DShader(0.08, 15, 8),
		TextureAssignerShader(4),
		IncreasingColorShader(0.3, 0.3, 0.3, 0.1, 0.3, 0.4, 0.4, 0.02)
               ]
Operators.create(TrueUP1D(), shaders_list)
