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
from random import *



seed = 2
N = 2



Operators.bidirectionalChain(ChainSilhouetteIterator()) #,NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[
		pyRandomColorShader2(seed,N),
		]
Operators.create(TrueUP1D(), shaders_list)
