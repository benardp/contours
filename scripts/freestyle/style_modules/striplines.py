#!/pixar/d2/sets/tools-36/bin/python


# script to remove extra blank lines from style files from the Freestyle webpage

import os, sys

inputFileName = sys.argv[1]
if len(sys.argv) > 2:
    outputFileName = sys.argv[2]
else:
    outputFileName = inputFileName

f = open(inputFileName, 'r')
lines = f.readlines()
f.close()

of = open(outputFileName,'w')
for x in lines:
    if len(x) > 1:
        of.write(x)

of.close()
