###############################################################################

#                                                                             #

# This file is part of a style sheet example from the Freestyle application   #

# Copyright (C) 2001-2005  Stephane Grabli                                    #

#                                                                             #

# http://freestyle.sourceforge.net                                            #

#                                                                             #

###############################################################################




###############################################################################

#                                                                             #

# This file is part of a style sheet example of the the Freestyle application #

# Copyright (C) 2001-2004  Stephane Grabli (Stephane.Grabli@imag.fr)          #

#                                                                             #

# http://artis.imag.fr/Software/Freestyle                                     #

#                                                                             #

###############################################################################



from Freestyle import *
from logical_operators import *
from PredicatesB1D import *
from PredicatesU1D import *
from shaders import *


upred = QuantitativeInvisibilityUP1D(0)

Operators.select(upred)

Operators.select(pyHigherLengthUP1D(5))

Operators.bidirectionalChain(pySketchyChainingIterator())

shaders_list = 	[

		SamplingShader(4),

		SpatialNoiseShader(4, 100, 2, 1, 1), 

		IncreasingThicknessShader(4, 10), 

		IncreasingColorShader(0.2,0.2,0.2,1,0.5,0.5,0.5,1),

		TextureAssignerShader(4)

		]

Operators.create(TrueUP1D(), shaders_list)


