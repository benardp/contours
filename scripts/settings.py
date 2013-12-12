# TODO: create a class for each shot that encapsulates specific details like first and last frame, shotName, etc.


# processing steps to perform
forceRunRIF              = True # Force re-running RIB->PLY processing, optionally optimizing consistency
renderStrokes            = True # Render strokes with Freestyle. Will run RIF if PLY doesn't exist already.
runFreestyleInteractive  = True
matchStrokes             = False #True

startFrame = 'FIRST'
endFrame   = 'LAST' #'ALL'

graftThreshold = 0 # 0.25  # 1.5 for bunny (or maybe less?), 0.5 for sanjay, 0.25 for sanjay2 (?)
cuspTrimThreshold = 10 # walk = 15 / bunny = 20 / angelaFace = 10

smoothSilhouettes = False#True   # use interpolated NdotV (for comparison purposes)

meshSmoothing = 0.75       # smoothing to apply to triangle meshes when initializing surface

wiggleFactor = 0 # 0.001   # 0.001 to get frames 229-234 of sanjay1 to work

numRefinements = 1 # number of extra refinements (for triangle meshes only in Optimization mode)

# bunnySmall frame 25 has "iron cross" problem

useConsistency = True #False

frameRanges = {'bunny':(1,72), 'cow':(1,72), 'dev_NewLooks_1':(14,27),'devresearch_RedPoses':(14,27),'devresearch_sanjay_1':(1,362),'devresearch_sanjay_2':(1,215), 'devresearch_NewLooks_4':(1,40), 'devresearch_NewLooks_7':(330,450), 'torus':(1,100), 'torus_sliced':(1,100), 'spheres':(1,48), 'coneBoy' :(1,1), 'angela':(1000,1170), 'bunnyQuads':(1,400), 'angelaFace':(1,150), 'sintel':(1,1), 'darwin':(1,1), 'teapot':(1,1), 'walk':(1,120), 'horse':(1,200), 'lezard':(1,1), 'bimba':(1,1), 'fertility':(1,1), 'pig':(1,1), 'dazTest':(100,100)}


#shot = 'btest'

#shot = 'balloons'
#shot = 'bunny'   # stanford bunny  frames 1-72
#shot = 'cow'    # cow   frames 1-72
#shot = 'bunnySmall'

#shot = 'devresearch_sanjay_1'  # frames 1-347
#shot = 'devresearch_sanjay_2'  # frames 1-215
#shot = 'devresearch_RedPoses'      #  frames 14-27
#shot = 'devresearch_NewLooks_4'  # walk.   frames 1-40
#shot = 'devresearch_NewLooks_7'  # Red+Blue.   frames 330-450
#shot = 'dev_NewLooks_1'   # original version of Red poses.  frames 14-27. weird face, don't use this one

#shot = 'u340_30'  # dug: "my master made me this collar. squirrel."  frames 1-177

#shot = 'u140_26'  # hanging russell
#shot = 'pillows'
#shot = 'origIntersectingPillows'
#shot = 'intersectingPillows'
#shot = 'singleBalloon'

#shot = 'coneBoy'
#shot = 'angelaFace'
#shot = 'angela'
#shot = 'bunnyQuads'
shot = 'torus'
#shot = 'torus_sliced'
#shot = 'spheres'
#shot = 'teapot'

#shot = 'sintel'
#shot = 'darwin'
#shot = 'walk'
#shot = 'horse'
#shot = 'lezard'
#shot = 'bimba'
#shot = 'fertility'
#shot = 'pig'

#shot = 'dazTest'

#shot = 'rtest_19'   # Rat calisthenics test


# ----- ribto3ds conversion options -------
subdivisionLevel = 1

# which kind of refinement steps to apply 
#refinement = 'None'    
#refinement = 'ContourOnly'
#refinement = 'ContourInconsistent'
#refinement = 'Full'
refinement = 'Radial'
#refinement = 'Dualtess'
##refinement = 'Optimize'
allowShifts = True   # allow "shifting" methods in refinement

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


# when computing mesh contours and ray-tests in Freestyle, take the consistency flags into account.
# ignored when refinement == 'None'
#useConsistency = True


forceExtractShot            = False # shot->RIB via RIT. (Linux Only). will happen automtically if RIB is missing.


# -------------------- NPR style -------------------------------------------------

saveLayers = False #True
#styleBasenames = []
#styleBasenames = ['paramVis3.py','plain.py']

#styleBasenames = ['plain.py']           # vanilla white curves w/ visibility
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
#styleBasenames = ['colorIDvis.py']        # show each visible curve with the same color IDs from the viewer
#styleBasenames = ['colorID.py']        # show each curve with the same color IDs from the viewer
#styleBasenames = ['silsOnly.py','SIsonly.py','boundariesOnly.py','POregionOnly.py'] # color-code visible curves according to type
#styleBasenames = ['randomcolorvis.py']   #randomly color curves after visibility
#styleBasenames = ['randomcolor.py']      #randomly color all curves, and don't do visibility


#styleBasenames = ['bezierTaper.py']           #stylized. can produce artifacts in fitting step
#styleBasenames = ['bezierTaperThin.py']           #stylized, thinner strokes. can produce artifacts

# draw smooth contours

# draw just the mesh boundary
#styleBasenames = ['SIsonly.py','boundariesOnly.py'] # color-code visible curves according to type


mainTestShotFolder = '/home/benard/Projects/Toronto/contours/code/freestyleTestShots/'
mainOutputFolder   = '/home/benard/Projects/Toronto/contours/code/freestyleOutput/'




# -------------------- maximum output image size for display purposes ------------------

maxDisplayWidth = 640 
maxDisplayHeight = 480
#maxDisplayWidth = 2048 #  smaller than the window dimensions for desktop with 30" display
#maxDisplayHeight = 1280


# -------------------- QuickTime settings ------------------------------------------

QTScale = 100   # image downsampling
QTFrameStep = 1    # for rendering on 2s, but behaves oddly, so don't use it
makeQuickTime               = False # seems to be hella slow and memory-hogging



# -------- which objects to extract from the RIBs ----------------------

objects = None   
exclusionObjects = None

if shot == 'u340_30':
    exclusionObjects = ['sets','sky','props','Russell','Cornea']  # the whole scene
#    exclusionObjects = ['sets','Russell','Cornea','Carl']   # just Dug
            #    objects = ['REye']
#    objects = ['Dug']
#    objects = ['Dug','Carl']
#    objects = ['Scalp']
#    objects = ['Rock']
    
if shot == 'u140_26':
    exclusionObjects = ['Sails','Badge']
#    objects = ['Hat']
#    objects = ['Strap']
#    objects = ['Button']
#    objects = ['RussellGroup', 'Harness', 'Sheet']
#    objects = ['RussellGroup']
#    objects = ['Neckerchief']
#    objects = ['RussellShirt','Sash','Neckerchief']
#    objects = ['RussellShirt','Sash'] #,'Shorts']
#    objects = ['RussellShirt']
#    objects = ['HatButton','Strap','Dome']

#    objects = ['Strap']
#    objects = ['Shoe']
#    objects = ['Sash']
#    objects = ['Skin']
#    objects = ['Shoe','Harness','Skin']
#    objects = ['RussellShirt','Sash','Shorts','Belt']


if shot in ['devresearch_RedPoses']:
    exclusionObjects = ['Mouth']
#    objects = ['LHand']
    
if shot in ['devresearch_sanjay_1']:
    exclusionObjects = ['sets','props']
#    objects = ['LSole']
#    objects = ['Body']
#    objects = ['LHand']

if shot in ['devresearch_sanjay_2']:
    exclusionObjects = ['sets','props','sky', 'Mouth']
#    objects = ['LSole']
#    objects = ['Body']
#    objects = ['LHand']

if shot == 'dev_NewLooks_1':
#    objects = ['RSole']
    
    pass
#   objects = ['Face','Body'] 
#    pass
#    exclusionObjects = None
#    exclusionObjects = ['Mouth']

#    objects = ['MuffStrap']
#    exclusionObjects = ['Sky','Ground','Mouth','Eye']
#    objects = ['LowerGum']

#    objects = ['BodySuit']
#    objects = ['BodySuit','Muff']

#    objects = ['LThread']

#    objects = ['RHand']
#    objects = ['LHand']
#    objects = ['RSkate']
#    objects = ['RSole']
#    objects = ['RShoe']
#    objects = ['Eye']
#    objects = ['RPupil']

if shot == 'sintel':
    objects = ['GEO-body']
    #objects = ['GEO-cornea']
    #objects = ['GEO-boots']
    #objects = ['GEO-eyeballs']
    #objects = ['GEO-gloves']
    #objects = ['GEO-pants']
    #objects = ['GEO-shirt']
    #exclusionObjects = ['GEO-body_mouth']

if shot == 'darwin':
    pass
    #objects = ['Alice.003']
    #objects = ['Alice']
    #objects = ['Shirt']
    #objects = ['Shoes']
