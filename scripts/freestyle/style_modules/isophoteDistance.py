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
Operators.bidirectionalChain(ChainSilhouetteIterator(),NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[
		ConstantColorShader(1,1,1),
                pyIsophoteDistanceShader(isovalue=.3, minThickness=0.5, maxThickness=2, scaling=1),  # red
#                pyIsophoteDistanceShader(isovalue=.4, minThickness=4, maxThickness=8, scaling=1),  # dug
		]
Operators.create(TrueUP1D(), shaders_list)
