import os
import sys
import string
import cPickle
import math

try:
    #requires Renderman python API
    import prman
    import cameraRif
    prmanAvailable = True
except ImportError, e:
    prmanAvailable = False
    pass

############################ OPTIONS AND SETTINGS
freestyleDir = os.getcwd() + '/freestyle'

if os.uname()[0] == 'Linux':
    buildName = 'linux'
else:
    buildName = 'macosx'

freestyleLibPath = '../freestyle/lib/python'
sys.path.append(freestyleLibPath)

############################ GENERATE FREESTYLE SETTINGS FILES ############################

xmlDir = os.path.expanduser('~/.Freestyle')
    
try:
    os.mkdir(xmlDir)
except:
    pass

optstr = '<FreestyleOptions> \n <default_path> \n <models path="" /> \n <patterns path="%s/textures/variation_patterns/" /> \n <brushes path="%s/textures/brushes/" /> \n <python path="%s/style_modules" /> \n  <help index="%s/doc/html/index.html" /> \n </default_path> \n <default_browser cmd="mozilla %%s" /> \n <default_viewmap_format float_vectors="0" no_occluders="0" compute_steerable="0" /> \n <default_visibility exhaustive_computation="1" /> \n  <default_drawing_buffers back_buffer="1" front_buffer="0" /> \n <papers nb="0" /> \n <default_ridges enable="0" sphere_radius="1" /> \n <default_suggestive_contours enable="0" dkr_epsilon="0" /> </FreestyleOptions>'%(freestyleDir,freestyleDir,freestyleDir,freestyleDir)

optionsFile = open(xmlDir + '/options.xml','w')
optionsFile.write(optstr)
optionsFile.close()

curdirstr = '<FreestyleCurrentDirs> \n <models dir="%s/models" /> \n <modules dir="%s/style_modules" /> \n </FreestyleCurrentDirs>\n'%(freestyleDir,freestyleDir)

curdirFile = open(xmlDir + '/current_dirs.xml','w')
curdirFile.write(curdirstr)
curdirFile.close()


print('Importing Freestyle')
import Freestyle

visAlgorithm = 'rayCasting'

def fileExists(name):
    try:
        f = open(name)
        f.close()
        return True
    except IOError:
        return False


def runStrokes(s):  # s is the settings module/class
# note: this doesn't notice if the folder couldn't be created for some reason... should fix this.
# it fails when the parent directory doesn't exist (e.g., "mkdir a/b" if there is no "a")

    useConsistencyLocal = s.useConsistency
    refinementLocal = s.refinement

    # for testing smooth silhouettes:
    meshSilhouettes = not s.smoothSilhouettes
    if not meshSilhouettes:
        useConsistencyLocal = False
        refinementLocal = 'None'

    invertNormals = (s.shot == 'cow')   # flip orientation of all triangles 

    ### Generate list of frame numbers
    if s.startFrame == 'FIRST':
        s.startFrame = s.frameRanges[s.shot][0]
    if s.runFreestyleInteractive:
        endFrame = s.startFrame
    elif s.endFrame == 'LAST':
        endFrame = s.frameRanges[s.shot][1]
    elif s.endFrame == 'ALL':      
        s.startFrame = s.frameRanges[s.shot][0]
        endFrame = s.frameRanges[s.shot][1]
    else:
        endFrame = s.endFrame
    fileNums = range(s.startFrame,endFrame+1)    


    # the RiOrientation field is used in the UP shots and Sanjay's; ignored in the other ones. don't know why...
    useOrientation = (s.shot[0] in ['u','U']) or s.shot[:18] == 'devresearch_sanjay' or s.shot == 'angela'


    styleFilenames = map(lambda x: freestyleDir +'/style_modules/'+x, s.styleBasenames)

    print(styleFilenames)

    # --------------------- Which RIBs to process ------------------------------------------
    if s.objects == None:
        pattern = '.'
    else:
        pattern = '('+string.join(s.objects,'|')+')'

    if s.exclusionObjects == None:
        exclusionPattern = None
    else:
        exclusionPattern = '('+string.join(s.exclusionObjects,'|')+')'

    # All other test shots
    shotFolder = s.mainTestShotFolder + s.shot + '/'
    outputFolder = s.mainOutputFolder + s.shot + '/'

    shotName = s.shot

    headerPattern = shotFolder + shotName + '.%04d.rib'
        
    cameraFilePattern = shotFolder + 'camera.%d.pickle'
    
    inputHeaders = [headerPattern%i for i in fileNums]
    inputRIBs = ['/dev/null'] * len(fileNums)
    cameraFiles = [cameraFilePattern%i for i in fileNums]

    # append the names of the objects to the folder name
    if s.objects == None:
        outputFolder = outputFolder + 'All'
    else:
        outputFolder = outputFolder + string.join(s.objects,'')
    if s.exclusionObjects != None:
        outputFolder = outputFolder + '-' + string.join(s.exclusionObjects,'')
    
    # -------------------- output filenames --------------------------------------------

    refstrs = {'None':'N','ContourOnly':'C','ContourInconsistent':'CI','Full':'F','Optimize':'Opt','Radial':'R'}
    refstr = refstrs[refinementLocal]
    if not meshSilhouettes:
        refstr = refstr + '_Sm'

    meshBasename = 'geom-subd'+str(s.subdivisionLevel)+'-smooth'+str(s.meshSmoothing) + '_' + refstr

    if s.cullBackFaces:
        meshBasename = meshBasename+'_CB'

    if s.saveLayers == True:
        snapshotBasename = s.shot+'_#_'+refstr
    else:
        snapshotBasename = s.shot+'_'+s.styleBasenames[0][:-3]+'_'+refstr

    snapshotBasename = snapshotBasename + '_ts%d'%s.cuspTrimThreshold
    
    if s.runFreestyleInteractive:
        snapshotBasename = snapshotBasename+'_int'

    try:
        os.mkdir(outputFolder)
    except:
        pass

    ################################## MAIN RENDERING LOOP ########################################

    for i in range(0,len(fileNums)):
        print('Processing image #%d (range is #%d-#%d)'%(fileNums[i], fileNums[0], fileNums[-1]))

        headerFilename = inputHeaders[i]
        RIBfilename = inputRIBs[i]
        geomFilename = outputFolder+'/'+meshBasename+'.'+str(fileNums[i])+'.ply'
        snapshotFilename = outputFolder+'/'+snapshotBasename + '.%03d.tiff'%fileNums[i]
        EPSFilenamePolyline = outputFolder+'/'+snapshotBasename + '_polyline.%03d.eps'%fileNums[i]
        EPSFilenameThick = outputFolder+'/'+snapshotBasename + '.%03d.eps'%fileNums[i]
        PDFFilenamePolyline = outputFolder+'/'+snapshotBasename + '_polyline.%03d.pdf'%fileNums[i]
        PDFFilenameThick = outputFolder+'/'+snapshotBasename + '.%03d.pdf'%fileNums[i]
        PNGFilenamePolyline = outputFolder+'/'+snapshotBasename+'_polyline.%03d.png'%fileNums[i]
        PNGFilenameThick = outputFolder+'/'+snapshotBasename+'.%03d.png'%fileNums[i]
        
        cameraFilename = cameraFiles[i]
    
        ################### USE A RIF TO EXTRACT CAMERA TRANSFORM FROM RIB ########################

        if not fileExists(cameraFilename):
            if prmanAvailable:
                
                print('*** Extracting camera data from RIB file ***')
                cameraData = cameraRif.runFilter([headerFilename])
                
                cameraFile = open(cameraFilename,'wb')
                cPickle.dump(cameraData,cameraFile)
                cameraFile.close()
            else:
                print('*** Error camera data not available ***')
                return
        else:
            print('Loading pickled camera data from '+cameraFilename)
            cameraFile = open(cameraFilename,'r')
            cameraData = cPickle.load(cameraFile)
            cameraFile.close()
        
        #print(cameraData)

        ################### USE THE RIF TO EXTRACT GEOMETRY FROM RIB AND PROCESS IT ##################

        runRIF = False
        if (s.forceRunRIF or (s.renderStrokes and (not fileExists(geomFilename)))):
            
            print('*** Running RI Filter ***')
            runRIF = True
        
            #create the output folder if necessary
            if not os.path.exists(outputFolder):
                os.makedirs(outputFolder)

            if buildName == 'linux':
                dso='../tess_RifFilter/lib/libtess_RifFilter.so'
            else:
                dso='../tess_RifFilter/lib/libtess_RifFilter.dylib'

            rifargs = string.join(['-rifargs',geomFilename,'-pattern','"'+pattern+'"',
                                   '-refinement',refinementLocal,
                                   '-allowShifts',str(s.allowShifts),
                                   '-cullBackFaces',str(s.cullBackFaces),
                                   '-subdivLevel',str(s.subdivisionLevel),
                                   '-meshSmoothing',str(s.meshSmoothing),
                                   '-maxDisplayWidth',str(s.maxDisplayWidth), 
                                   '-maxDisplayHeight',str(s.maxDisplayHeight),
                                   '-meshSilhouettes',str(meshSilhouettes),
                                   '-runFreestyle',str(s.renderStrokes),
                                   '-runFreestyleInteractive',str(s.runFreestyleInteractive),
                                   '-freestyleLibPath',freestyleLibPath,
                                   '-cuspTrimThreshold',str(s.cuspTrimThreshold),
                                   '-graftThreshold',str(s.graftThreshold),
                                   '-wiggleFactor',str(s.wiggleFactor),
                                   '-useOrientation',str(useOrientation),
                                   '-useConsistency',str(useConsistencyLocal),
                                   '-outputImage',snapshotFilename,
                                   '-outputEPSPolyline',EPSFilenamePolyline,
                                   '-outputEPSThick',EPSFilenameThick,
                                   '-invertNormals',str(invertNormals),
                                   '-lastStep',s.lastStep])

            rifargs = rifargs + ' -beginStyleModules '+string.join(styleFilenames) + ' -endStyleModules'

            if exclusionPattern != None:
                rifargs = rifargs +' -exclude "'+exclusionPattern+'"'

            rifargs = rifargs + ' -rifend'
        
            cmd = 'renderdl -rif %s %s  -catrib %s'%(dso,rifargs,headerFilename)
        
            print('Executing '+cmd)
            result = os.system(cmd)

            if (result != 0):
                print('PRMAN ERROR: '+str(result))
                raise Exception() 
        
        # ----- count the number of faces in the geometry files (to be able to tell if it's empty -----
        if s.renderStrokes:
            f = open(geomFilename)
            numFaces = -1
            for line in f:
                if line[0:12] == 'element face':
                    numFaces = int(line[13:])
                    break
            f.close()

        ############################ RENDER STROKES IMAGE USING FREESTYLE #########################

        if s.renderStrokes and not runRIF:
            print('*** Starting Freestyle ***')

            # tell Freestyle which rendering styles to use
            Freestyle.clearStylesFS()
            for style in styleFilenames:
                Freestyle.addStyleFS(style)

            def arrayToMatrix(data):
                return Freestyle.Matrix4x4(data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],
                                           data[8],data[9],data[10],data[11],data[12],data[13],data[14],data[15])
        
            vas = {'rayCasting':0,'regionBased':1,'punchOut':2}

            if not s.runFreestyleInteractive or (s.maxDisplayWidth > cameraData['xres'] and s.maxDisplayHeight > cameraData['yres']):
                displayWidth = cameraData['xres']
                displayHeight = cameraData['yres']
            else:
                scaleW = float(s.maxDisplayWidth) / cameraData['xres']
                scaleH = float(s.maxDisplayHeight) / cameraData['yres']
                if scaleW < scaleH:
                    scale = scaleW
                else:
                    scale = scaleH
                displayWidth = int(math.floor(scale * cameraData['xres']))
                displayHeight = int(math.floor(scale * cameraData['yres']))

            if cameraData['focalLength'] <= 0:  # make sure we got a perspective projection specified
                print('bad focal length')
                sys.exit(1)

            if numFaces > 0:
                Freestyle.run(geomFilename,snapshotFilename, EPSFilenamePolyline, EPSFilenameThick,
                              arrayToMatrix(cameraData['worldTransform']),
                              cameraData['left'], cameraData['right'], cameraData['bottom'], cameraData['top'],
                              cameraData['pixelaspect'], cameraData['aspect'],
                              cameraData['near'],cameraData['far'], cameraData['focalLength'],
                              #output image dimensions (ignored in interactive mode):
                              cameraData['xres'], cameraData['yres'],      
                              displayWidth, displayHeight, 
                              vas[visAlgorithm], useConsistencyLocal, s.runFreestyleInteractive, s.cuspTrimThreshold, s.graftThreshold, s.wiggleFactor, "", s.saveLayers)
            else:
                print('Empty geometry. Not running Freestyle')


        ############################ GENERATE PDF ########################

        if s.renderStrokes and buildName == 'linux':
            os.system('ps2pdf %s %s\n'%(EPSFilenamePolyline, PDFFilenamePolyline))
            os.system('ps2pdf %s %s\n'%(EPSFilenameThick, PDFFilenameThick))
            print('\tevince %s\n'%(PDFFilenamePolyline))
            print('\tevince %s\n'%(PDFFilenameThick))

        if s.renderStrokes and buildName == 'macosx':
            if s.saveLayers == True:
                for styleName in s.styleBasenames:
                    os.system('pstopdf %s -o %s\n'%(EPSFilenamePolyline.replace('#',styleName[:-3]), PDFFilenamePolyline.replace('#',styleName[:-3])))
                    os.system('pstopdf %s -o %s\n'%(EPSFilenameThick.replace('#',styleName[:-3]), PDFFilenameThick.replace('#',styleName[:-3])))
            else:
                os.system('pstopdf %s -o %s\n'%(EPSFilenamePolyline, PDFFilenamePolyline))
                os.system('pstopdf %s -o %s\n'%(EPSFilenameThick, PDFFilenameThick))
                print('\topen %s\n'%(PDFFilenamePolyline))
                print('\topen %s\n'%(PDFFilenameThick))

                # create PNGs from the EPS output
                if s.renderStrokes and numFaces > 0:
                    os.system('gs -q -dEPSCrop -dNOPAUSE -dBATCH -sDEVICE=png16m -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sOutputFile=%s %s'%(PNGFilenameThick,EPSFilenameThick))
                    os.system('gs -q -dEPSCrop -dNOPAUSE -dBATCH -sDEVICE=png16m -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sOutputFile=%s %s'%(PNGFilenamePolyline,EPSFilenamePolyline))
                    print('\tsos %s\n'%(PNGFilenamePolyline))
                    print('\tsos %s\n'%(PNGFilenameThick))
        
if __name__ == "__main__":
    import settings

    runStrokes(settings)
