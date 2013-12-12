
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
#include "ChainingIterators.h"
#include "../system/TimeStamp.h"

ViewEdge* AdjacencyIterator::operator*() {
    return (*_internalIterator).first;
}
bool AdjacencyIterator::isIncoming() const{
    return (*_internalIterator).second;
}

void AdjacencyIterator::increment(){
    ++_internalIterator;
    while((!_internalIterator.isEnd()) && (!isValid((*_internalIterator).first)))
        ++_internalIterator;
}

bool AdjacencyIterator::isValid(ViewEdge* edge){
    if(_restrictToSelection)
        if(edge->getTimeStamp() != TimeStamp::instance()->getTimeStamp())
            return false;
    if(_restrictToUnvisited)
        if(edge->getChainingTimeStamp() > TimeStamp::instance()->getTimeStamp())
            return false;
    return true;
}

void ChainingIterator::increment() {
    _increment = true;
    ViewVertex * vertex = getVertex();
    if(!vertex){
        _edge = 0;
        return;
    }
    AdjacencyIterator it = AdjacencyIterator(vertex, _restrictToSelection, _restrictToUnvisited);
    if(it.isEnd())
        _edge = 0;
    else
        _edge = traverse(it);
    if(_edge == 0)
        return;
    if(_edge->A() == vertex)
        _orientation = true;
    else
        _orientation = false;
}

void ChainingIterator::decrement() {
    _increment = false;
    ViewVertex * vertex = getVertex();
    if(!vertex){
        _edge = 0;
        return;
    }
    AdjacencyIterator it = AdjacencyIterator(vertex, _restrictToSelection, _restrictToUnvisited);
    if(it.isEnd())
        _edge = 0;
    else
        _edge = traverse(it);
    if(_edge == 0)
        return;
    if(_edge->B() == vertex)
        _orientation = true;
    else
        _orientation = false;
}

//
// ChainSilhouetteIterators
//
///////////////////////////////////////////////////////////

void printEdge(char *name, ViewEdge *e)
{
    printf("%s: %08X.   N: %d, QI: %d, ID: %d-%d\n",
           name, e, e->getNature(), e->qi(), e->getId().getFirst(), e->getId().getSecond());
}

ViewEdge * ChainSilhouetteIterator::traverse(const AdjacencyIterator& ait){
    AdjacencyIterator it(ait);
    ViewVertex* nextVertex = getVertex();
    // we can't get a NULL nextVertex here, it was intercepted
    // before
    if(nextVertex->getNature() & Nature::T_VERTEX){
        TVertex * tvertex = (TVertex*)nextVertex;
        ViewEdge *mate = tvertex->mate(getCurrentEdge());

        ViewEdge * start = !it.isEnd() ? *it : NULL;

        /*
    //    if (tvertex->sameFace() && start != NULL)
        if (!tvertex->sameFace())
          {
    printf("------------------\n");
    printf("tvertex: %08X\n", tvertex);
    printf("sameFace: %s\n", tvertex->sameFace() ? "true" : "false");
    printEdge("current",getCurrentEdge());
    printEdge("start",start);
        if (mate != NULL)
            printEdge("mate",mate);
    else
      printf("mate: NULL\n");
    printf("tvertex: front: %08X, %08X, back: %08X, %08X\n",
           tvertex->frontEdgeA().first, tvertex->frontEdgeB().first,
           tvertex->backEdgeA().first, tvertex->backEdgeB().first);
    printf("iterator edges:\n");
    AdjacencyIterator it2(ait);

    while (!it2.isEnd())
      {
        printEdge("\t",*it2);
        ++it2;
      }
    printf("_sortedEdges:\n");
    for(ViewVertexInternal::orientedViewEdgeIterator eit = tvertex->edgesBegin();
        eit != tvertex->edgesEnd(); ++eit)
      printEdge("\t",(*eit).first);
          }
    */

        // if the intersection comes from the same face, we might want to continue along another edge
        if (tvertex->sameFace() && start != NULL)
            return start;

        // find mate and count outgoing edges
        int numOutgoing = 0;
        while(!it.isEnd())
        {
            ViewEdge *ve = *it;
            if(ve == mate)
            {
                assert(mate != NULL);
                //	    printf("returning %08X\n", mate);
                return ve;
            }
            ++it;
            ++ numOutgoing;
        }

        //    printf("numOutgoing = %d\n", numOutgoing);

        // only chain through the rear edge of a T-junction if there's only one other edge
        if (numOutgoing == 1)
        {
            //	printf("returning %08X\n", start);
            return start;
        }
        else
        {
            //	printf("returning NULL\n");
            return NULL;
        }

        // just return any valid continuation
        //    return start;

        //    return 0;
    }
    if(nextVertex->getNature() & Nature::NON_T_VERTEX){
        NonTVertex * nontvertex = (NonTVertex*)nextVertex;
        ViewEdge * newEdge(0);

        ViewEdge * start = !it.isEnd() ? *it : NULL;


        return start;
        /*
    // we'll try to chain the edges by keeping the same nature...
    // the preseance order is : SILHOUETTE, BORDER, CREASE, SUGGESTIVE, VALLEY, RIDGE
    Nature::EdgeNature natures[8] = {Nature::SILHOUETTE, Nature::ALL_INTERSECTION, Nature::BORDER, Nature::CREASE, Nature::SUGGESTIVE_CONTOUR, Nature::VALLEY, Nature::RIDGE };
    // note: the two kinds of intersections are treated as the same

    for(unsigned i=0; i<8; ++i)
      {
    if(getCurrentEdge()->getNature() & natures[i]){
      int n = 0;
      while(!it.isEnd()){
        ViewEdge *ve = *it;
        if(ve->getNature() & natures[i]){
          ++n;
          newEdge = ve;
        }
        ++it;
      }

      if (n==1)
        return newEdge;
      else
        return NULL;
    }
      }
    return NULL;
    */
    }
    assert(0);
}

ViewEdge * ChainPredicateIterator::traverse(const AdjacencyIterator& ait){
    AdjacencyIterator it(ait);
    // Iterates over next edges to see if one of them
    // respects the predicate:
    while(!it.isEnd()) {
        ViewEdge *ve = *it;
        if(((*_unary_predicate)(*ve)) && ((*_binary_predicate)(*(getCurrentEdge()),*(ve))))
            return ve;
        ++it;
    }
    return 0;
} 
