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
from PredicatesU1D import *
from PredicatesU0D import *
from PredicatesB1D import *
from Functions0D import *
from shaders import *





Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
func = pyInverseCurvature2DAngleF0D()
Operators.recursiveSplit(func,  pyParameterUP0D(0.2,0.8), NotUP1D(pyHigherLengthUP1D(40)), 2)
Operators.select(pyHigherLengthUP1D(15))
shaders_list = 	[
		SamplingShader(3),
		pyGuidingLineShader(),
		TextureAssignerShader(4),
		IncreasingThicknessShader(8,18),

		IncreasingColorShader(0.4,0.4,0.4,0.5,0.65,0.65,0.65,0.5),
		pyLengthDependingBackboneStretcherShader(0.2)
		]
Operators.create(TrueUP1D(), shaders_list)
