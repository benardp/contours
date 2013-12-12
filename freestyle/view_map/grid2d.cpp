#include "grid2d.h"

inline int min(int a, int b) { return (a < b ? a : b); }
inline int max(int a, int b) { return (a > b ? a : b); }

Grid2D::Grid2D(int dim, Vec2r minLoc, Vec2r maxLoc)
{
  // ------------ allocate the grid ------------------
  _dim = dim;
  _grid = new Cell2D*[_dim];
  for(int i=0;i<_dim;i++)
    _grid[i] = new Cell2D[_dim];

  // -------------- determine the bounding box --------

  // expand the grid slightly
  Vec2r center = (minLoc+maxLoc)/2;
  _max = center + (maxLoc - center)*1.05;
  _min = center + (minLoc - center)*1.05;

  // Use square grid cells, for simplicity.
  // Thus we need to stretch one of the grid dimensions
  Vec2r diag = _max - _min;
  if (diag.y() > diag.x())
    {
      real diff = diag.y() - diag.x();
      _min.x() -= diff/2;
      _max.x() += diff/2;
    }
  else
    {
      real diff = diag.x() - diag.y();
      _min.y() -= diff/2;
      _max.y() += diff/2;
    }
}


Grid2D::~Grid2D()
{
  for(int i=0;i<_dim;i++)
    delete [] _grid[i];

  delete [] _grid;
}

Grid2D::iterator::iterator(Vec2r start, Vec2r end, Grid2D * grid)
{
  _grid = grid;
  
  _pos = _grid->pt2grid(start);

  Vec2r d = end-start;
  d.normalizeSafe();

  _dir = d * grid->_dim;
  _dir.x() = _dir.x()/(grid->_max[0] - grid->_min[0]);
  _dir.y() = _dir.y()/(grid->_max[1] - grid->_min[1]);

  _idir.x() = (_dir.x() < 0 ? -1 : (_dir.x() > 0 ? 1 : 0));
  _idir.y() = (_dir.y() < 0 ? -1 : (_dir.y() > 0 ? 1 : 0));

  // starting cell number

  _ix = min(max(int(floor(_pos.x())),0),_grid->_dim-1);
  _iy = min(max(int(floor(_pos.y())),0),_grid->_dim-1);

  // compute total edge length
  Vec2r dx2 = (_grid->pt2grid(end)-grid->pt2grid(start));
  _edgeLength = dx2.norm();

  // total length converted so far
  _length = 0;

  _isEnd = false;
  /*
  printf("New iterator: (%f,%f)-> (%f,%f); ", start.x(), start.y(), end.x(), end.y());
  printf("   grid indices: start [%f,%f], end [%f,%f], dir: [%f,%f], edgelength: %f\n", _pos.x(), _pos.y(), 
	 _grid->pt2grid(end).x(), _grid->pt2grid(end).y(), _dir.x(), _dir.y(), _edgeLength);
  printf("   norm: %f, dx: (%f, %f), edgelength2: %f\n", dx2.norm(),  dx2.x(), dx2.y(), 
	 dx2.squareNorm());

  real d1 = dx2.squareNorm();
  float d2 = d1;
  float d3 = sqrt(d2);
  real d4 = d3;

  printf("ds: %f %f %f %f\n", d1,d2,d3,d4);

  assert(!isnan(_edgeLength));*/
}

void 
Grid2D::iterator::operator++()
{
  if (_isEnd)
    return;

  // compute next location
  real tx= FLT_MAX, ty=FLT_MAX;
  Vec2r relstart(_pos.x()-_ix,_pos.y()-_iy);
  
  if (_idir.x() == 1)
    tx = (1-relstart.x())/_dir.x();
  else
    if (_idir.x() == -1)
      tx = -relstart.x()/_dir.x();

  if (tx < 0)
    {
      cerr << "warning: tx < 0\n";
      _isEnd = true;
      return;
    }
  
  if (_idir.y() == 1)
    ty = (1-relstart.y())/_dir.y();
  else
    if (_idir.y() == -1)
      ty = -relstart.y()/_dir.y();

  if (ty < 0)
    {
      cerr << "warning: ty < 0\n";
      _isEnd = true;
      return;
    }
  
  real dist = min(tx,ty);
  
  _pos += dist*_dir;
  _length += (dist*_dir).norm();
  
  if (tx <= ty)
    _ix += _idir.x();
  else
    _iy += _idir.y();

  if (_length >= _edgeLength)
    _isEnd = true;
}

void
Grid2D::grid2pt(real i,real j,real & x,real & y)
{
  x =   i*(_max.x()-_min.x())/_dim + _min.x();
  y =   j*(_max.y()-_min.y())/_dim + _min.y();
}

Vec2r
Grid2D::grid2pt(Vec2r gpt)
{
  real x,y;

  grid2pt(gpt.x(),gpt.y(),x,y);

  return Vec2r(x,y);
}

void
Grid2D::pt2grid(real x,real y,real & i,real & j)
{
  i = (x-_min.x())*_dim/(_max.x() - _min.x());
  j = (y-_min.y())*_dim/(_max.y() - _min.y());
}

Vec2r
Grid2D::pt2grid(Vec2r pt)
{
  real i,j;
  pt2grid(pt.x(),pt.y(),i,j);
  return Vec2r(i,j);
}


void
Grid2D::insertEdge(FEdge * fe)
{
  Vec2r start = fe->vertexA()->getPoint2D();
  Vec2r end = fe->vertexB()->getPoint2D();

  for(iterator it = begin(start,end); !it.isEnd(); ++it)
    (*it).edges.push_back(fe);
}
