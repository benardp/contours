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

shaders_list = 	[
		ConstantColorShader(1,1,1),
		pyNonLinearVaryingThicknessShader(1,4,0.6), 
		]
Operators.create(TrueUP1D(), shaders_list)
