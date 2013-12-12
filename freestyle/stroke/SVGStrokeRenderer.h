#ifndef  SVGSTROKERENDERER_H
# define SVGSTROKERENDERER_H

#include <stdio.h>

# include "../system/FreestyleConfig.h"
# include "StrokeRenderer.h"
# include "StrokeRep.h"


class LIB_RENDERING_EXPORT SVGStrokeRenderer : public StrokeRenderer
{
public:
  SVGStrokeRenderer(const char * filename, int outputWidth, int outputHeight, bool polylineOutput, int polylineWidth);
  virtual ~SVGStrokeRenderer();

  /*! Renders a stroke rep */
  virtual void RenderStrokeRep(StrokeRep *iStrokeRep) const;
  virtual void RenderStrokeRepBasic(StrokeRep *iStrokeRep) const;

protected:
  FILE * _outputFile;
  int _width, _height;
  bool _polylineOutput;
  int _polylineWidth;
  //void renderNoTexture(StrokeRep *iStrokeRep) const;
};

#endif // SVGSTROKERENDERER_H
