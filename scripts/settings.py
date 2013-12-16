# input and output shots paths
mainTestShotFolder = 'your path here'
mainOutputFolder   = 'your path here'

# processing steps to perform
forceRunRIF              = True # Force re-running RIB->PLY processing
renderStrokes            = True # Render strokes with Freestyle.
runFreestyleInteractive  = True # Keep Freestyle window open

startFrame = 'FIRST'
endFrame   = 'LAST'

# freestyle thresholds for topology simplification
graftThreshold = 0 
cuspTrimThreshold = 10

smoothSilhouettes = False  # if True, use interpolated NdotV (for comparison purposes)

meshSmoothing = 0.75       # smoothing to apply to triangle meshes when initializing surface

wiggleFactor = 0           # wiggling of the vertices in freestyle to avoid degenerated cases

# when computing mesh contours and ray-tests in Freestyle, take the consistency flags into account.
# ignored when refinement == 'None'
useConsistency = True

frameRanges = {'torus':(1,100), 'bunnyQuads':(1,400), 'walk':(1,120)}

# active shot

#shot = 'bunnyQuads'
shot = 'torus'
#shot = 'walk'

# ----- rib2mesh conversion options -------
subdivisionLevel = 1

# which kind of refinement steps to apply 

#refinement = 'None'    
#refinement = 'ContourOnly'
#refinement = 'ContourInconsistent'
#refinement = 'Full'
#refinement = 'Optimize'
refinement = 'Radial'
allowShifts = True   # allow "shifting" methods in refinement

# Stop radial algorithm early

#lastStep = 'PreProcess'
#lastStep = 'DetectCusp'
#lastStep = 'InsertContour'
#lastStep = 'InsertRadial'
#lastStep = 'FlipRadial'
#lastStep = 'ExtendRadial'
#lastStep = 'FlipEdge'
#lastStep = 'WigglingParam'
#lastStep = 'SplitEdge'
lastStep = 'Everything';

cullBackFaces = False

# -------------------- freestyle NPR style ---------------------------------------------

saveLayers = False # if True, export each selected style as an independant layer
#styleBasenames = ['paramVis3.py','plain.py']

#styleBasenames = ['plain.py']           
#styleBasenames = ['bsplineTaper.py']
#styleBasenames = ['bsplineTaperThin.py']
#styleBasenames = ['japanese_bigbrush.py']
#styleBasenames = ['sketchy3.py']
#styleBasenames = ['isophoteDistance.py']
#styleBasenames = ['taper.py']
#styleBasenames = ['taperTiff.py']

#styleBasenames = ['paramVis.py']
#styleBasenames = ['paramVis2.py']
styleBasenames = ['paramVis3.py']
#styleBasenames = ['colorIDvis.py']       # show each visible curve with the same color IDs from the viewer
#styleBasenames = ['colorID.py']          # show each curve with the same color IDs from the viewer
#styleBasenames = ['silsOnly.py','SIsonly.py','boundariesOnly.py','POregionOnly.py'] # color-code visible curves according to type
#styleBasenames = ['randomcolorvis.py']   #randomly color curves after visibility
#styleBasenames = ['randomcolor.py']      #randomly color all curves, and don't do visibility


#styleBasenames = ['bezierTaper.py']      #stylized. can produce artifacts in fitting step
#styleBasenames = ['bezierTaperThin.py']  #stylized, thinner strokes. can produce artifacts

# draw just the mesh boundary
#styleBasenames = ['SIsonly.py','boundariesOnly.py'] # color-code visible curves according to type


# -------------------- maximum output image size for display purposes ------------------

maxDisplayWidth = 640 
maxDisplayHeight = 480

# -------- which objects to extract or exclude from the RIBs ----------------------

objects = None   
exclusionObjects = None
