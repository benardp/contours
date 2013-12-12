#include <assert.h>

#include "Interface1D.h"



int Interface1D::_livingRefs =0; 
int Interface1D::_totalRefs =0;
set<Interface1D*> Interface1D::_allI1Ds;

bool Interface1D::_erasingAllI1Ds = false;

Interface1D::~Interface1D()
{
  _livingRefs --;
  if (!_erasingAllI1Ds)
    {
      set<Interface1D*>::iterator it = _allI1Ds.find(this);
      assert(it != _allI1Ds.end());
      _allI1Ds.erase(it);
    }
}      

void Interface1D::eraseAllI1Ds()
{
  _erasingAllI1Ds = true;

  for(set<Interface1D*>::iterator it=_allI1Ds.begin(); it!=_allI1Ds.end(); ++it)
    {
      Interface1D * ptr = *it;
      //      printf("Deleting %08X\n", (long int)ptr);
      delete ptr;
    }
  _allI1Ds.clear(); 

  _erasingAllI1Ds = false;
}
