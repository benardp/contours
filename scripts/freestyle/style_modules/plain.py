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
                pyConstantThicknessShader(4),
		]
Operators.create(TrueUP1D(), shaders_list)
