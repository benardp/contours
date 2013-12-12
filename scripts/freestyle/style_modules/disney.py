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

class pyMaterialColorShader(StrokeShader):
	def __init__(self, threshold=50):
		StrokeShader.__init__(self)
		self._threshold = threshold

	def getName(self):
		return "pyMaterialColorShader"

	def shade(self, stroke):
		it = stroke.strokeVerticesBegin()

		it_end = stroke.strokeVerticesEnd()
		func = MaterialF0D()
		xn = 0.312713
		yn = 0.329016
		Yn = 1.0
		un = 4.* xn/ ( -2.*xn + 12.*yn + 3. )
		vn= 9.* yn/ ( -2.*xn + 12.*yn +3. )	
		while it.isEnd() == 0:
			toto = it.castToInterface0DIterator()

			mat = func(toto)
			
			r = mat.diffuseR()
			g = mat.diffuseG()
			b = mat.diffuseB()

			X = 0.412453*r + 0.35758 *g + 0.180423*b
			Y = 0.212671*r + 0.71516 *g + 0.072169*b
			Z = 0.019334*r + 0.119193*g + 0.950227*b


			if((X == 0) and (Y == 0) and (Z == 0)):
				X = 0.01
				Y = 0.01
				Z = 0.01
			u = 4.*X / (X + 15.*Y + 3.*Z)
			v = 9.*Y / (X + 15.*Y + 3.*Z)
			
			L= 116. * math.pow((Y/Yn),(1./3.)) -16
			U = 13. * L * (u - un)

			V = 13. * L * (v - vn)
			
			if (L > self._threshold):
				L = L/1.28
				U = U+10
			else:
				L = L +2.5*(100-L)/5.
				U = U/3.0
				V = V/3.0				

			u = U / (13. * L) + un
			v = V / (13. * L) + vn
			
			Y = Yn * math.pow( ((L+16.)/116.), 3.)
			X = -9. * Y * u / ((u - 4.)* v - u * v)
			Z = (9. * Y - 15*v*Y - v*X) /( 3. * v)
			
			r = 3.240479 * X - 1.53715 * Y - 0.498535 * Z
			g = -0.969256 * X + 1.875991 * Y + 0.041556 * Z
			b = 0.055648 * X - 0.204043 * Y + 1.057311 * Z


			att = it.getObject().attribute()
			att.setColor(r, g, b)
			it.increment()

Operators.select(QuantitativeInvisibilityUP1D(0))
Operators.bidirectionalChain(ChainSilhouetteIterator(), NotUP1D(QuantitativeInvisibilityUP1D(0)))
shaders_list = 	[
		StrokeTextureShader("smoothAlpha.bmp", 
				     Stroke.OPAQUE_MEDIUM, 
				     0),

		BezierCurveShader(3),
		pySamplingShader(2),
		pyNonLinearVaryingThicknessShader(2,8,0.6), 
		pyMaterialColorShader(80)
		]
Operators.create(TrueUP1D(), shaders_list)
