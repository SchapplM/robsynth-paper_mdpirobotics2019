#!/bin/bash
# https://unix.stackexchange.com/questions/93959/how-to-convert-a-color-pdf-to-black-white
gs \
 -sOutputFile=pkmmdl_paper_greyscale.pdf \
 -sDEVICE=pdfwrite \
 -sColorConversionStrategy=Gray \
 -dProcessColorModel=/DeviceGray \
 -dCompatibilityLevel=1.4 \
 -dNOPAUSE \
 -dBATCH \
 pkmmdl_paper.pdf

