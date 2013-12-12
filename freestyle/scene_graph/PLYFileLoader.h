#ifndef  PLY_FILE_LOADER_H
# define PLY_FILE_LOADER_H

# include "NodeGroup.h"



class LIB_SCENE_GRAPH_EXPORT PLYFileLoader
{
public:
  /*! Builds a PLYFileLoader */
  //  PLYFileLoader();

  explicit PLYFileLoader(const char *iFileName);
  virtual ~PLYFileLoader();

  /*! Sets the name of the 3dsMax file to load */
  //  void SetFileName(const char *iFileName);

  /*! Loads the 3D scene and returns 
   *  a pointer to the scene root node
   */
  NodeGroup * Load();
  //void Load(const char *iFileName);

  /*! Gets the number of read faces */
  inline unsigned int numFacesRead() {return _numFacesRead;}

  /*! Gets the smallest edge size read */
  inline real minEdgeSize() {return _minEdgeSize;}

protected:
  char *_FileName;
  NodeGroup* _Scene;
  unsigned _numFacesRead;
  real _minEdgeSize;
};

#endif // PLY_FILE_LOADER_H
