#
#
# RI Filter that extracts all camera information from a RIB and pickles it to a file
#
#

import os, sys
import math

#if os.uname()[0] == 'Linux':
#    import pixver
#    rmanLocation = pixver.PixverGetPackageLocation('rman')
#    if rmanLocation:
#        sys.path.append(rmanLocation + '/bin')
#else
sys.path = sys.path + [os.environ['RMANTREE']+'/bin']
    
import prman

# hasn't been tested
#def matMult(m1, m2):
#    m = tuple([None]*16)  # initial 4x4 matrix
#    for i in range(0,4):
#        for j in range(0,4):
#            m(j+4*i) = 0
#            for k in range(0:4):
#                # m(i,j) = sum_k m1(i,k) m2(k,j)
#                m(j+4*i) = m(j+4*i) + m1(k+4*i) * m2(j+4*k)
#    return m

class CameraRif(prman.Rif): 
    def __init__(self, ri):
        prman.Rif.__init__(self, ri) 
#        Identity(self)  # set the matrix stack to be an identity matrix
        self._cameraData = {'main_cam':None}
        self._foundMainCam = False
        self._firstConcatTransform = True
        self._transformDepth = 0
        self._focalLength = -1
        self._cameraData['aspect'] = -1

    def Ignore(*args):
        pass
        
    Attribute = MotionEnd = AttributeEnd = AttributeBegin = MotionBegin = Color = SubdivisionMesh = HierarchicalSubdivisionMesh = Surface = Patch = ArchiveBegin = ArchiveEnd = ReadArchive = Option = Clipping = Shutter = TransformBegin = TransformEnd = Transform = Orientation = Opacity = Sides = IfBegin = Procedural = IfEnd = Else = ElseIf = Display = Translate = Rotate = PointsPolygons = Scale = ScopedCoordinateSystem = CoordinateSystem = Basis = Ignore
    #filtering out NuPatch seems to cause it to crash

    def Transform(self, *matrix):
        if (self._firstConcatTransform and self._transformDepth == 0):
            self._cameraData['worldTransform'] = matrix
            self._firstConcatTransform = False
        
    def ConcatTransform(self, *matrix):
#        if (self._foundMainCam):
#            self._cameraData['main_cam'] = matrix
#            self._foundMainCam = False
                
        if (self._firstConcatTransform and self._transformDepth == 0):
            self._cameraData['worldTransform'] = matrix
            self._firstConcatTransform = False

    def TransformBegin(self):
        self._transformDepth = self._transformDepth + 1

    def TransformEnd(self):
        self._transformDepth = self._transformDepth - 1
            
#    def Attribute(self, nm,tokens):
#        if (nm == 'identifier'):
#            self._foundMainCam = ('main_cam' == tokens['name'][0][-8:])

    def Clipping(self, near, far):
        self._cameraData['near'] = near
        self._cameraData['far'] = far

    def Format(self, xres, yres, pixelaspect):
        self._cameraData['xres'] = xres
        self._cameraData['yres'] = yres
        self._cameraData['pixelaspect'] = pixelaspect

    def FrameAspectRatio(self, aspect):
        self._cameraData['aspect'] = aspect

    def ScreenWindow(self, left, right, bottom, top):
        self._cameraData['left'] = left
        self._cameraData['right'] = right
        self._cameraData['bottom'] = bottom
        self._cameraData['top'] = top

    def Projection(self, proj, args):
        self._cameraData['projection'] = proj
        if args.has_key('fov'):
            fov = args['fov'][0]
        else:
            fov = 90.0
        self._cameraData['focalLength'] = 1/math.tan((fov/2.0)*(math.pi/180))


def runFilter(RIBFiles):
    prman.Init(["-catrib","/dev/null",""]) #"-woff",woff])
    ri = prman.Ri()
    rif = CameraRif(ri)
    prman.RifInit([rif])
    ri.Begin(ri.RENDER)
    for rib in RIBFiles:
        print('parsing for camera parameters: '+rib)
        prman.ParseFile(rib)
    ri.End()
    return rif._cameraData
