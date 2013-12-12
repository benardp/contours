#include <vector>
#include <assert.h>

#include "Silhouette.h"


using namespace std;

struct Cell2D
{
  vector<FEdge*> edges;
};
  

class Grid2D
{
public:
  Grid2D(int dim, Vec2r minLoc, Vec2r maxLoc);
  ~Grid2D();

  // return cell at particular index location
  Cell2D & cell(int i,int j) { 
    assert( i < _dim);  assert( j < _dim); 
    assert( i >= 0);  assert( j >= 0); 
    return _grid[i][j]; };
  Cell2D & cell(Vec2i pt) { return cell(pt.x(), pt.y()); }

  // return cell at particular image coordinates
  Cell2D & cellLoc(real x,real y) { return cell(pt2grid(Vec2r(x,y))); }
  Cell2D & cellLoc(Vec2r loc) { return cell(pt2grid(loc)); }
  
  // return the image (x,y) location for a given grid index (real-valued)
  void grid2pt(real i,real j,real & x,real & y);
  Vec2r grid2pt(Vec2r gpt);
  void pt2grid(real x,real y,real& i,real& j);
  Vec2r pt2grid(Vec2r pt);

  void insertEdge(FEdge * edge);

  /*  int dim() const { return _dim; }
  Vec2r minL() const { return _min; }
  Vec2r maxL() const { return _max; }
  */

  // class for iterating over all cells covered by a given line segment
  class iterator
  {
  public:
    iterator(Vec2r start, Vec2r end, Grid2D * grid);
    iterator() { _isEnd = true; } // constructor for "end"
    Cell2D & operator *() { return _grid->cell(_ix,_iy); }
    bool isEnd() { return _isEnd; }
    void operator ++();
  private:
    Vec2r _pos;
    Vec2r _dir;
    Vec2r _end;
    Vec2i _idir;
    real _edgeLength;
    real _length;
    Grid2D * _grid;
    int _ix, _iy;
    bool _isEnd;
  };

  iterator begin(Vec2r start, Vec2r end) { return iterator(start,end,this); }
  iterator end() { return iterator(); }

  friend class iterator;

private:
  Cell2D** _grid;
  Vec2r _min,_max;
  int _dim;
};
