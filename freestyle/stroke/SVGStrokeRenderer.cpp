#include "SVGStrokeRenderer.h"
#include "Stroke.h"
#include "StrokeAdvancedIterators.h"

SVGStrokeRenderer::SVGStrokeRenderer(const char * filename, int width, int height, bool polylineOutput, int polylineWidth)
{
  //  _textureManager = NULL;
  _outputFile = fopen(filename, "wt");
  if (_outputFile == NULL)
    {
      printf("UNABLE TO OPEN SVG OUTPUT FILE %s\n", filename);
      return;
    }
  _width = width;
  _height = height;

  fprintf(_outputFile, "<?xml version=\"1.0\" standalone=\"no\"?>\n\n<svg width=\"%dpx\" height=\"%dpx\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">\n\n",
	  width, height);

  _polylineOutput = polylineOutput;
  _polylineWidth = polylineWidth;
}


SVGStrokeRenderer::~SVGStrokeRenderer()
{
  //  if (_textureManager != NULL)
  //    delete _textureManager;

  if (_outputFile == NULL)
    return;
  
  fprintf(_outputFile, "</svg>\n");
  fclose(_outputFile);
}

void SVGStrokeRenderer::RenderStrokeRep(StrokeRep *iStrokeRep) const
{
  if (_polylineOutput)
    {
      // output a polyline
      
      Stroke * stroke = iStrokeRep->getStroke();
      
      fprintf(_outputFile, "<path d=\"");
      
      bool first = true;
      Vec3f color;
      for(Stroke::const_vertex_iterator v = stroke->vertices_begin(); v != stroke->vertices_end(); v++)
	{
	  const StrokeVertex * vert = (*v);

	  if (first)
	    {
	      const StrokeAttribute & attrib = vert->attribute();
	      color = attrib.getColorRGB();
	    }

	  fprintf(_outputFile, "%c %f %f ", first ? 'M' : 'L', vert->x(), _height-vert->y()-1);
	  
	  first = false;
	}  
      
      if (color[0] == 1 && color[1] == 1 && color[2] == 1)
	color = Vec3r (0,0,0);

      //  fprintf(_outputFile, "\" stroke=\"black\"/>\n");
      fprintf(_outputFile, "\" fill=\"none\" stroke=\"#%02X%02X%02X\" stroke-width=\"%d\"/>\n",int(color[0]*255), int(color[1]*255), int(color[2]*255), _polylineWidth);
    }
  else
    {
      vector<Strip*>& strips = iStrokeRep->getStrips();
      for(vector<Strip*>::iterator s=strips.begin(); s!=strips.end(); ++s)
	{
	  Strip::vertex_container& vertices = (*s)->vertices();
	  
	  // output all the odd points, then all the even points in reverse
	  
	  Vec3f color = vertices[0]->color();

	  if (color == Vec3f(1,1,1))
	    color = Vec3r (0,0,0);

	  fprintf(_outputFile, "<polygon opacity=\"1\" fill=\"rgb(%f,%f,%f)\" points=\"", color[0], color[1], color[2]);

	  for(int i=0;i<vertices.size(); i+=2)
	    {
	      StrokeVertexRep * svRep = vertices[i];
	      fprintf(_outputFile, "%f,%f ", svRep->point2d()[0], _height-svRep->point2d()[1]-1);
	    }
	  
	  for(int i=vertices.size()-1;i>=0;i-=2)
	    {
	      StrokeVertexRep * svRep = vertices[i];
	      fprintf(_outputFile, "%f,%f ", svRep->point2d()[0], _height-svRep->point2d()[1]-1);
	    }
	  
	  fprintf(_outputFile, "\"/>\n");
	}  
    }
}

void SVGStrokeRenderer::RenderStrokeRepBasic(StrokeRep *iStrokeRep) const
{
  RenderStrokeRep(iStrokeRep);
}
