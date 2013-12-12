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
shaders_list = 	[
#		StrokeTextureShader("smoothAlpha.bmp", 
#				     Stroke.OPAQUE_MEDIUM, 
#				     0),

		BezierCurveShader(3),
#		pySamplingShader(2),
		ConstantColorShader(1,1,1),
		pyNonLinearVaryingThicknessShader(2,4,0.6), 
		]
Operators.create(TrueUP1D(), shaders_list)
