
//
//  Copyright (C) : Please refer to the COPYRIGHT file distributed 
//   with this source distribution. 
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////

# include "PSStrokeRenderer.h"
# include "Canvas.h"
#include "StrokeAdvancedIterators.h"


PSStrokeRenderer::PSStrokeRenderer(const char* iFileName, int outputWidth, int outputHeight,bool polylineOutput, int polylineWidth)
    :StrokeRenderer(){
    if(!iFileName)
        iFileName = "freestyle.ps";
    // open the stream:
    _ofstream.open(iFileName, ios::out);
    if(!_ofstream.is_open()){
        cerr << "couldn't open the output file " << iFileName << endl;
    }
    _ofstream << "%!PS-Adobe-2.0 EPSF-2.0" << endl;
    _ofstream << "%%Creator: Freestyle (http://artis.imag.fr/Software/Freestyle)" << endl;
    _ofstream << "%%BoundingBox: " << 0 << " "<< 0 << " " << outputWidth << " " << outputHeight << endl;
    //  _ofstream << "%%BoundingBox: " << 0 << " "<< 0 << " " << Canvas::getInstance()->width() << " " << Canvas::getInstance()->height() << endl;
    _ofstream << "%%EndComments" << endl;

    _outputHeight = outputHeight;
    _outputWidth = outputWidth;
    _polylineOutput = polylineOutput;
    _polylineWidth = polylineWidth;
}

PSStrokeRenderer::~PSStrokeRenderer(){
    Close();
}

void PSStrokeRenderer::RenderStrokeRep(StrokeRep *iStrokeRep) const{
    RenderStrokeRepBasic(iStrokeRep);
}

void PSStrokeRenderer::RenderStrokeRepBasic(StrokeRep *iStrokeRep) const
{
    if (_polylineOutput)
    {
        Stroke * stroke = iStrokeRep->getStroke();

        _ofstream << "newpath" << endl;
        _ofstream << _polylineWidth << " setlinewidth\n";
        _ofstream << "1 setlinejoin\n";  // select round line joins.  default is miter, which creates protrustions at high-curvature areas

        bool first = true;

        for(Stroke::const_vertex_iterator v = stroke->vertices_begin(); v != stroke->vertices_end(); v++)
        {
            const StrokeVertex * vert = (*v);

            if (first)
            {
                const StrokeAttribute & attrib = vert->attribute();
                Vec3f color = attrib.getColorRGB();
                if (color[0] == 1 && color[1] == 1 && color[2] == 1)
                    color = Vec3r (0,0,0);
                _ofstream << (color)[0] << " " << (color)[1] << " " << (color)[2] << " setrgbcolor" <<endl;
            }

            _ofstream << (double) vert->x() << " " <<  (double) vert->y() ;
            if (first)
                _ofstream << " moveto\n";
            else
                _ofstream << " lineto\n";

            real z = vert->z();
            _ofstream << "%% "<< (double) z <<" depth\n";

            first = false;
        }

        _ofstream << "stroke" << endl;
        //      _ofstream << "closepath" << endl;
        //      _ofstream << "fill" << endl;
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

            _ofstream << "newpath" << endl;
            _ofstream << (color)[0] << " " << (color)[1] << " " << (color)[2] << " setrgbcolor" <<endl;

            bool first = true;

            for(int i=0;i<vertices.size(); i+=2)
            {
                StrokeVertexRep * svRep = vertices[i];
                _ofstream << (double) svRep->point2d()[0] << " " << (double) svRep->point2d()[1];
                if (first)
                    _ofstream << " moveto\n";
                else
                    _ofstream << " lineto\n";
                first = false;

            }

            for(int i=vertices.size()-1;i>=0;i-=2)
            {
                StrokeVertexRep * svRep = vertices[i];
                _ofstream << (double) svRep->point2d()[0] << " " << (double) svRep->point2d()[1] << " lineto\n";
            }

            _ofstream << "closepath" << endl;
            _ofstream << "fill" << endl;
        }
    }
}
/*
  else
    {
      vector<Strip*>& strips = iStrokeRep->getStrips();
      Strip::vertex_container::iterator v[3];
      StrokeVertexRep *svRep[3];
      Vec3r color[3];
      for(vector<Strip*>::iterator s=strips.begin(), send=strips.end();
      s!=send;
      ++s){
    Strip::vertex_container& vertices = (*s)->vertices();
    v[0] = vertices.begin();
    v[1] = v[0];++(v[1]);
    v[2] = v[1]; ++(v[2]);

    while(v[2]!=vertices.end()){
      svRep[0] = *(v[0]);
      svRep[1] = *(v[1]);
      svRep[2] = *(v[2]);

      color[0] = svRep[0]->color();

      if (color[0][0] == 1 && color[0][1] == 1 && color[0][2] == 1)
        color[0] = Vec3r(0,0,0);


      //color[1] = svRep[1]->color();
      //color[2] = svRep[2]->color();

      _ofstream << "newpath" << endl;
      _ofstream << (color[0])[0] << " " << (color[0])[1] << " " << (color[0])[2] << " setrgbcolor" <<endl;
      _ofstream << svRep[0]->point2d()[0] << " " <<svRep[0]->point2d()[1] << " moveto" << endl;
      _ofstream << svRep[1]->point2d()[0] << " " <<svRep[1]->point2d()[1] << " lineto" << endl;
      _ofstream << svRep[2]->point2d()[0] << " " <<svRep[2]->point2d()[1] << " lineto" << endl;
      _ofstream << "closepath" << endl;
      _ofstream << "fill" << endl;

      ++v[0];
      ++v[1];
      ++v[2];
    }
      }
    }
}
*/

void PSStrokeRenderer::Close(){
    if(_ofstream.is_open())
        _ofstream.close();
}

