#include <map>
#include <deque>
#include <set>
#include <iostream>
#include <sstream>
#include <float.h>
#include <math.h>

#include "refineContour.h"

typedef enum {RF_CUSP, RF_DUAL_CONTOUR, RF_RADIAL_INT} RootFindMode;

real radialPlaneIntersect(vec3 & pos, vec3 & normal, const vec3 & vPos, const vec3 & cameraCenter)
{
    vec3 viewVec = cameraCenter - pos;
    vec3 planeNormal = viewVec ^ normal;
    planeNormal.normalize();
    return planeNormal * (vPos - pos);
}

bool SameSign(const real values[3])
{
    return (values[0]>0.0 && values[1]>0.0 && values[2]>0.0) ||
            (values[0]<0.0 && values[1]<0.0 && values[2]<0.0);
}

ParamPointCC RootFindContour(RootFindMode mode,
                             ParamPointCC p[3],
                             real r[3],
                             real ndotv[3],
                             vec3 limitPositions[3],
                             vec3 limitNormals[3],
                             vec3 * vPos, vec3 * vNormal,
                             const vec3 & cameraCenter,
                             int depth = 0)
{
    if(SameSign(ndotv) || SameSign(r)){
        return ParamPointCC();
    }

    std::vector<ParamPointCC> params;
    params.push_back(p[0]);
    params.push_back(p[1]);
    params.push_back(p[2]);
    std::vector<vec2> abs;
    abs.resize(3);
    if(!ParamPointCC::FindChart(params,abs)){
#ifdef VERBOSE
        printf("\nCAN'T ROOT FIND IN THIS TRIANGLE\n");
#endif
        return ParamPointCC();
    }

    real lengths[3];

    for(int i=0; i<3; i++){
        lengths[i] = sqrtl((abs[i][0] - abs[(i+1)%3][0])*(abs[i][0] - abs[(i+1)%3][0]) +
                           (abs[i][1] - abs[(i+1)%3][1])*(abs[i][1] - abs[(i+1)%3][1]));
    }

    real halfp = real(0.5) * (lengths[0] + lengths[1] + lengths[2]);
    real area = sqrtl(halfp*(halfp-lengths[0])*(halfp-lengths[1])*(halfp-lengths[2]));

    const real threshold_r = mode == RF_CUSP ? 1e-6 : 0.00001;
    const real threshold_ndotv = mode != RF_RADIAL_INT ? CONTOUR_THRESHOLD : threshold_r;

    if(fabsl(ndotv[0]) <= threshold_ndotv &&
            fabsl(ndotv[1]) <= threshold_ndotv &&
            fabsl(ndotv[2]) <= threshold_ndotv &&
            fabsl(r[0]) <= threshold_r &&
            fabsl(r[1]) <= threshold_r &&
            fabsl(r[2]) <= threshold_r ){
        ParamPointCC res =  ParamPointCC::Interpolate(ParamPointCC::Interpolate(p[0], p[1], 0.5), p[2], 1.0/3);
        return res;
    }
    if(area < CONTOUR_THRESHOLD*CONTOUR_THRESHOLD){
        ParamPointCC res =  ParamPointCC::Interpolate(ParamPointCC::Interpolate(p[0], p[1], 0.5), p[2], 1.0/3);
        return res;
    }

    //Split along longest edge
    int idx;
    for(idx=0; idx<3; idx++){
        if(lengths[idx]>=lengths[(idx+1)%3] && lengths[idx]>=lengths[(idx+2)%3])
            break;
    }
    assert(idx<3);

    ParamPointCC midPosParam = ParamPointCC::Interpolate(p[idx], p[(idx+1)%3], 0.5);

    if(!midPosParam.IsExactlyEvaluable())
        return ParamPointCC();

    vec3 midPos, midNormal;
    midPosParam.Evaluate(midPos,midNormal);

    real mid_r;
    if(mode == RF_CUSP){
        if(!midPosParam.RadialCurvature(cameraCenter,mid_r)){
#ifdef VERBOSE
            printf("\nRADIAL CURVATURE UNDEFINED\n");
#endif
            return ParamPointCC();
        }
    }else if(mode == RF_DUAL_CONTOUR){
        mid_r = radialPlaneIntersect(midPos,midNormal,vPos[0],cameraCenter);
    }else if(mode == RF_RADIAL_INT){
        mid_r = radialPlaneIntersect(vPos[0],vNormal[0],midPos,cameraCenter);
    }
    vec3 viewVec = midPos - cameraCenter;
    real mid_ndotv;
    if(mode == RF_RADIAL_INT)
        mid_ndotv = radialPlaneIntersect(vPos[1],vNormal[1],midPos,cameraCenter);
    else
        Facing(viewVec, midNormal, CONTOUR_THRESHOLD, &mid_ndotv);

    ParamPointCC new_p[3] = { p[idx], midPosParam, p[(idx+2)%3] };
    real new_r[3] = { r[idx], mid_r, r[(idx+2)%3]};
    real new_ndotv[3] = { ndotv[idx], mid_ndotv, ndotv[(idx+2)%3] };
    vec3 new_limitPos[3] = { limitPositions[idx], midPos, limitPositions[(idx+2)%3] };
    vec3 new_limitN[3] = { limitNormals[idx], midNormal, limitNormals[(idx+2)%3] };

    ParamPointCC test;
    test = RootFindContour(mode, new_p, new_r, new_ndotv, new_limitPos, new_limitN, vPos, vNormal, cameraCenter,depth+1);
    if(!test.IsNull())
        return test;

    new_p[0] = p[(idx+1)%3];
    new_r[0] = r[(idx+1)%3];
    new_ndotv[0] = ndotv[(idx+1)%3];
    new_limitPos[0] = limitPositions[(idx+1)%3];
    new_limitN[0] = limitNormals[(idx+1)%3];

    test= RootFindContour(mode, new_p, new_r, new_ndotv, new_limitPos, new_limitN, vPos, vNormal, cameraCenter, depth+1);
    if(!test.IsNull())
        return test;

    return ParamPointCC();
}

bool TestPlaneEdgeIntersect(const vec3 &planePoint, const vec3 &planeNormal, const vec3 & pos1, const vec3 & pos2){
    real d1 = planeNormal * (pos1 - planePoint);
    real d2 = planeNormal * (pos2 - planePoint);
    if((d1<0.0 && d2<0.0) || (d1>0.0 && d2>0.0)){
        return false;
    }
    return true;
}

bool TestPlaneEdgeIntersect(const vec3 &planePoint, const vec3 &planeNormal, MeshVertex* v1,MeshVertex* v2){
    return TestPlaneEdgeIntersect(planePoint,planeNormal,v1->GetData().pos,v2->GetData().pos);
}

ParamPointCC PlaneEdgeIntersection(const vec3 &planePoint, const vec3 &planeNormal,
                                   ParamPointCC param1, ParamPointCC param2)
{
    vec3 pos1, normal1;
    param1.Evaluate(pos1,normal1);
    vec3 pos2, normal2;
    param2.Evaluate(pos2,normal2);

    real d1 = planeNormal * (pos1 - planePoint);
    real d2 = planeNormal * (pos2 - planePoint);
    if((d1<0.0 && d2<0.0) || (d1>0.0 && d2>0.0)){
        return ParamPointCC();
    }

    ParamPointCC p1, p2;

    if(d1>0){
        real dtmp = d2;
        p2 = param1;
        d2 = d1;
        p1 = param2;
        d1 = dtmp;
    }else{
        p1 = param1;
        p2 = param2;
    }

    const int NUM_SAMPLES = 40;
    vec3 pos, normal;

    // Find intersection with the radial plan by root-finding
    ParamPointCC intersection;
    for(int s=0; s<NUM_SAMPLES; s++){
        intersection = ParamPointCC::Interpolate(p1,p2,0.5);
        if(intersection.IsNull() || !intersection.IsEvaluable())
            return ParamPointCC();
        intersection.Evaluate(pos,normal);
        real d = planeNormal * (pos - planePoint);
        if(d<-CONTOUR_THRESHOLD){
            p1 = intersection;
            d1 = d;
        }else if(d>CONTOUR_THRESHOLD){
            p2 = intersection;
            d2 = d;
        }else{
            return intersection;
        }
    }
    return ParamPointCC();
}

ParamPointCC PlaneEdgeIntersection(const vec3 &planePoint, const vec3 &planeNormal, MeshEdge* e){
    ParamPointCC p1 = e->GetOrgVertex()->GetData().sourceLoc;
    ParamPointCC p2 = e->GetDestVertex()->GetData().sourceLoc;
    real d1 = planeNormal * (e->GetOrgVertex()->GetData().pos - planePoint);
    real d2 = planeNormal * (e->GetDestVertex()->GetData().pos - planePoint);
    if((d1<0.0 && d2<0.0) || (d1>0.0 && d2>0.0)){
        return ParamPointCC();
    }

    if(d1>0){
        ParamPointCC tmp = p2;
        real dtmp = d2;
        p2 = p1;
        d2 = d1;
        p1 = tmp;
        d1 = dtmp;
    }

    const int NUM_SAMPLES = 40;
    vec3 pos, normal;

    // Find intersection with the radial plan by root-finding
    ParamPointCC intersection;
    for(int s=0; s<NUM_SAMPLES; s++){
        intersection = ParamPointCC::Interpolate(p1,p2,0.5);
        intersection.Evaluate(pos,normal);
        real d = planeNormal * (pos - planePoint);
        if(d<-CONTOUR_THRESHOLD){
            p1 = intersection;
            d1 = d;
        }else if(d>CONTOUR_THRESHOLD){
            p2 = intersection;
            d2 = d;
        }else{
            break;
        }
    }
    return intersection;
}

bool ExtendRadialEdge(MeshFace *face, Mesh *mesh, const vec3 & cameraCenter,
                      PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    assert(!IsConsistent(face,cameraCenter));

    for(int i=0; i<3; i++){
        MeshVertex *v0 = face->GetVertex(i);

        if(!v0->GetData().Radial() || (v0->GetData().Radial() && v0->GetData().radialOrg[0]->GetData().cusp))
            continue;

        MeshVertex *v1 = face->GetVertex((i+1)%3);
        MeshVertex *v2 = face->GetVertex((i+2)%3);

        MeshFace* adjacentFace = GetOppFace(face,i);
        MeshVertex* v3 = adjacentFace ? GetOtherVertex(adjacentFace,v1,v2) : NULL;

        int adjConsistent = 0;
        std::set<MeshFace*> oneRing;
        GetOneRing(v0,oneRing);
        for(std::set<MeshFace*>::iterator it=oneRing.begin(); it!=oneRing.end(); it++){
            if(IsConsistent((*it),cameraCenter))
                adjConsistent++;
        }
        if(adjacentFace && IsConsistent(adjacentFace,cameraCenter))
            adjConsistent++;

        if(v1->GetData().facing==CONTOUR || v2->GetData().facing==CONTOUR || (v3 && v3->GetData().facing==CONTOUR))
            continue;

        for(int k=0; k<v0->GetData().NumRadialOrg(); k++){
            MeshVertex* cpt = v0->GetData().radialOrg[k];

            if(cpt==v1 || cpt==v2 || cpt==v3)
                continue;

            vec3 viewVec = cameraCenter - cpt->GetData().pos;
            vec3 planeNormal = viewVec ^ cpt->GetData().normal;

            if(!TestPlaneEdgeIntersect(cpt->GetData().pos,planeNormal,v1->GetData().pos,v2->GetData().pos))
                continue;

            int max_numConsistent = 0;
            real bestMinQuality = 0.0;
            vec3 oldPos = v0->GetData().pos;
            vec3 bestPos = oldPos;
            ParamPointCC bestLoc;

            const int numSamples = 40;
            for(int s=1; s<numSamples; ++s){
                // Sample the 2 triangles to find the point that maximize the number of consistent tris
                ParamPointCC leftPosParam, rightPosParam;

                int sample = s;

                if(s<=numSamples/2){
                    leftPosParam  = ParamPointCC::Interpolate(cpt->GetData().sourceLoc,
                                                              v1->GetData().sourceLoc,
                                                              double(s)/double(numSamples/2));
                    rightPosParam = ParamPointCC::Interpolate(cpt->GetData().sourceLoc,
                                                              v2->GetData().sourceLoc,
                                                              double(s)/double(numSamples/2));
                }else if(v3){
                    sample = s-numSamples/2;
                    leftPosParam  = ParamPointCC::Interpolate(v3->GetData().sourceLoc,
                                                              v1->GetData().sourceLoc,
                                                              double(sample)/double(numSamples/2));
                    rightPosParam = ParamPointCC::Interpolate(v3->GetData().sourceLoc,
                                                              v2->GetData().sourceLoc,
                                                              double(sample)/double(numSamples/2));
                }else{
                    break;
                }
                if(leftPosParam.IsNull() || rightPosParam.IsNull()){
#ifdef VERBOSE
                    printf("CAN'T INTEPOLATE\n");
#endif
                    continue;
                }

                ParamPointCC newPointParam = PlaneEdgeIntersection(cpt->GetData().pos,planeNormal,
                                                                   leftPosParam,rightPosParam);
                if(newPointParam.IsNull() || !newPointParam.IsEvaluable())
                    continue;

                vec3 newPos, newNormal;
                newPointParam.Evaluate(newPos,newNormal);

                real lambda,mu;
                if(Outside<VertexDataCatmark>(face->GetVertex(0)->GetData().pos,face->GetVertex(1)->GetData().pos,face->GetVertex(2)->GetData().pos,newPos,lambda,mu)){
                    continue;
                }else{
                    bool validMove = true;
                    std::set<MeshFace*> oneRing;
                    GetOneRing(v0,oneRing);
                    std::set<MeshVertex*> oneRingVertices;
                    GetOneRingVertices(v0,oneRingVertices);
                    for(std::set<MeshFace*>::iterator fit=oneRing.begin(); fit!=oneRing.end() && validMove; fit++){
                        MeshFace* f = (*fit);
                        int idx = GetVertexIndex(f,v0);
                        for(std::set<MeshVertex*>::iterator it=oneRingVertices.begin(); it!=oneRingVertices.end() && validMove; it++){
                            MeshVertex* v = (*it);
                            if(v==f->GetVertex(0) || v==f->GetVertex(1) || v==f->GetVertex(2))
                                continue;
                            if(!Outside<VertexDataCatmark>(f->GetVertex((idx+1)%3)->GetData().pos,f->GetVertex((idx+2)%3)->GetData().pos,newPos,v->GetData().pos,lambda,mu)){
                                validMove = false;
                                break;
                            }
                        }
                    }
                    if(!validMove)
                        continue;
                }

                v0->GetData().pos = newPos;

                int numConsistent = 0;
                std::vector<real> quality;
                oneRing.clear();
                GetOneRing(v0,oneRing);
                quality.reserve(oneRing.size()+1);
                for(std::set<MeshFace*>::iterator it=oneRing.begin(); it!=oneRing.end(); it++){
                    MeshFace* currFace = (*it);
                    if(adjacentFace && currFace == face){
                        MeshVertex* newFaces[2][3] = {{v0,v3,v1},{v0,v3,v2}};
                        for(int f=0;f<2;f++){
                            if(IsConsistent<VertexDataCatmark>(newFaces[f],cameraCenter))
                                numConsistent++;
                            quality.push_back(TriangleQuality(newFaces[f]));
                        }
                    }else{
                        if(IsConsistent(currFace,cameraCenter))
                            numConsistent++;
                        quality.push_back(TriangleQuality(currFace));
                    }
                }
                std::vector<real>::iterator minQuality = std::min_element(quality.begin(),quality.end());

                if((numConsistent > max_numConsistent) || (numConsistent == max_numConsistent && (*minQuality) > bestMinQuality)){
                    bestPos = newPos;
                    bestLoc = newPointParam;
                    max_numConsistent = numConsistent;
                    bestMinQuality = (*minQuality);
                }
                v0->GetData().pos = oldPos;
            }

            if(bestPos==oldPos){
                wiggleQueue.Remove(face);
                splitQueue.Remove(face);
                return false;
            }

            bool shiftable = EnsureShiftable(v0, bestLoc, cameraCenter, mesh, &wiggleQueue, &splitQueue, true);

            if(max_numConsistent >= adjConsistent && shiftable){

                wiggleQueue.Remove(face);
                splitQueue.Remove(face);
                mesh->DeleteFace(face);
                if(adjacentFace){
                    wiggleQueue.Remove(adjacentFace);
                    splitQueue.Remove(adjacentFace);
                    mesh->DeleteFace(adjacentFace);
                }

                v0->GetData().isShifted = true;
                if(v0->GetData().origLoc.IsNull())
                    v0->GetData().origLoc = v0->GetData().sourceLoc;
                SetupVertex(v0->GetData(),bestLoc,cameraCenter);

                if(v3 && !v0->GetEdge(v3) && !v3->GetEdge(v0)){
                    if(v1->GetEdge(v0) || v0->GetEdge(v2) || v3->GetEdge(v1) || v2->GetEdge(v3)){
                        if(v1->GetEdge(v3) || v3->GetEdge(v0) || v0->GetEdge(v1) || v2->GetEdge(v0) || v0->GetEdge(v3) || v3->GetEdge(v2)){
#ifdef LINK_FREESTYLE
                            char str[200];
                            sprintf(str, "BUG");
                            addRIFDebugPoint(-1,double(v0->GetData().pos[0]),double(v0->GetData().pos[1]),double(v0->GetData().pos[2]),str,0);
#endif
#ifdef VERBOSE
                            printf("\nBUG\n");
#endif
                            return false;
                        }
                        NewFaceDebug(mesh, v1, v3, v0);
                        NewFaceDebug(mesh, v2, v0, v3);
                    }else{
#ifdef VERBOSE
                        printf("\nELSE\n");
#endif
                        NewFaceDebug(mesh, v1, v0, v3);
                        NewFaceDebug(mesh, v2, v3, v0);
                    }
                }else if(v3){
#ifdef LINK_FREESTYLE
                    char str[200];
                    sprintf(str, "v3");
                    addRIFDebugPoint(-1,double(v0->GetData().pos[0]),double(v0->GetData().pos[1]),double(v0->GetData().pos[2]),str,0);
#endif
                    printf("\n  \n");
                    assert(!v0->GetEdge(v1));
                    assert(!v2->GetEdge(v0));
                    if(v1->GetEdge(v0)){
                        face = v1->GetEdge(v0)->GetLeftFace();
                        if(GetOtherVertex(face,v0,v1) != v3){
                            face = v0->GetEdge(v2)->GetLeftFace();
                            wiggleQueue.Remove(face);
                            splitQueue.Remove(face);
                            mesh->DeleteFace(face);
                            NewFaceDebug(mesh, v0, v1, v3);
                        }else{
                            wiggleQueue.Remove(face);
                            splitQueue.Remove(face);
                            mesh->DeleteFace(face);
                            NewFaceDebug(mesh, v0, v3, v2);
                        }
                    }else if(v0->GetEdge(v2)){
                        face = v0->GetEdge(v2)->GetLeftFace();
                        if(GetOtherVertex(face,v0,v2) != v3){
                            wiggleQueue.Remove(face);
                            splitQueue.Remove(face);
                            mesh->DeleteFace(face);
                            NewFaceDebug(mesh, v0, v3, v2);
                        }else{
                            wiggleQueue.Remove(face);
                            splitQueue.Remove(face);
                            mesh->DeleteFace(face);
                            NewFaceDebug(mesh, v0, v1, v3);
                        }
                    }else{
#ifdef VERBOSE
                        printf("BAD ELSE?\n");
#endif
                    }
                }else{
#ifdef VERBOSE
                    printf("BAD ELSE 2?\n");
#endif
                }

                EnsureShiftable(v0, bestLoc, cameraCenter, mesh, &wiggleQueue, &splitQueue, false);

                return true;
            }
        }
    }
    return false;
}

bool FlipEdge(MeshFace * face, int oppVertex, Mesh * mesh, PriorityQueueCatmark & nonRadialFaces,
              PriorityQueueCatmark & wiggleQueue, bool force=false)
{
    int eOpp;
    MeshFace * oppFace = GetOppFace(face, oppVertex, eOpp);

    if (oppFace == NULL)
        return false;

    MeshVertex * v0 = face->GetVertex(oppVertex);
    MeshVertex * v1 = face->GetVertex( (oppVertex+1)%3 );
    MeshVertex * v2 = face->GetVertex( (oppVertex+2)%3 );
    MeshVertex * v3 = oppFace->GetVertex(eOpp);

    if(!force && !ParamPointCC::ConvexInChart(v1->GetData().sourceLoc,v0->GetData().sourceLoc,
                                              v2->GetData().sourceLoc,v3->GetData().sourceLoc)){
        return false;
    }

    nonRadialFaces.Remove(face);
    nonRadialFaces.Remove(oppFace);
    wiggleQueue.Remove(face);
    wiggleQueue.Remove(oppFace);

    mesh->DeleteFace(face);
    mesh->DeleteFace(oppFace);

    MeshFace * f1 = NewFace(mesh, v0, v1, v3);
    if(!IsStandardRadialFace(f1))
        nonRadialFaces.Insert(f1);
    MeshFace * f2 = NewFace(mesh, v0, v3, v2);
    if(!IsStandardRadialFace(f2))
        nonRadialFaces.Insert(f2);
    return true;
}

bool FlipAllowed(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2, MeshVertex* v3,
                 MeshFace* currentFace, MeshFace* adjacentFace, Mesh* mesh, MeshVertex* &vSkip,
                 PriorityQueueCatmark & nonRadialFaces, PriorityQueueCatmark & wiggleQueue)
{
    MeshVertex* f[3] = {v0,v1,v3};
    real lambda, mu;
    if(Outside(f,v2,lambda,mu))
        return true;

    if(v2->GetData().facing == CONTOUR)
        return false;

    MeshEdge* e1 = v0->GetEdge(v2) ? v0->GetEdge(v2) : v2->GetEdge(v0);
    MeshFace* adj1 = e1->GetLeftFace() != currentFace ? e1->GetLeftFace() : e1->GetRightFace();
    MeshEdge* e2 = v2->GetEdge(v3) ? v2->GetEdge(v3) : v3->GetEdge(v2);
    MeshFace* adj2 = e2->GetLeftFace() != adjacentFace ? e2->GetLeftFace() : e2->GetRightFace();
    if(adj1 && adj2){ // not on boundary
        bool flip = true;
        if(adj1!=adj2){
            while(flip){
                int oppIdx = GetVertexIndex(adj1,v0);
                flip = FlipEdge(adj1,oppIdx,mesh,nonRadialFaces,wiggleQueue);
                e1 = v0->GetEdge(v2) ? v0->GetEdge(v2) : v2->GetEdge(v0);
                adj1 = e1->GetLeftFace() != currentFace ? e1->GetLeftFace() : e1->GetRightFace();
                if(GetOtherVertex(adj1,v0,v2)==v3){
                    adj2=adj1;
                    break;
                }
            }
            flip = (adj1!=adj2);
            while(flip){
                int oppIdx = GetVertexIndex(adj2,v3);
                flip = FlipEdge(adj2,oppIdx,mesh,nonRadialFaces,wiggleQueue);
                e2 = v2->GetEdge(v3) ? v2->GetEdge(v3) : v3->GetEdge(v2);
                adj2 = e2->GetLeftFace() != adjacentFace ? e2->GetLeftFace() : e2->GetRightFace();
                if(GetOtherVertex(adj2,v3,v2)==v0){
                    adj1=adj2;
                    break;
                }
            }
        }
        if(adj1==adj2 && flip){
#ifdef LINK_FREESTYLE
            char str[200];
            sprintf(str, "v2");
            addRIFDebugPoint(-1, double(v2->GetData().pos[0]), double(v2->GetData().pos[1]), double(v2->GetData().pos[2]), str, NULL);
            sprintf(str, "v0");
            addRIFDebugPoint(-1, double(v0->GetData().pos[0]), double(v0->GetData().pos[1]), double(v0->GetData().pos[2]), str, NULL);
            sprintf(str, "v3");
            addRIFDebugPoint(-1, double(v3->GetData().pos[0]), double(v3->GetData().pos[1]), double(v3->GetData().pos[2]), str, NULL);
#endif
            nonRadialFaces.Remove(adj1);
            wiggleQueue.Remove(adj1);
            mesh->DeleteFace(adj1);
            vSkip = v2;
            return true;
        }
        return false;
    }
    return false;
}

bool ImproveRadialConsistency(MeshVertex *cpt, Mesh *mesh, int & numFlip, PriorityQueueCatmark & wiggleQueue)
{
    std::set<MeshFace*> oneRing;
    GetOneRing(cpt,oneRing);
    PriorityQueueCatmark nonRadialFaces;
    for(std::set<MeshFace*>::iterator fit = oneRing.begin(); fit!=oneRing.end(); ++fit){
        assert( (*fit) != NULL);
        if(!IsStandardRadialFace(*fit))
            nonRadialFaces.Insert(*fit);
    }

    int iter = nonRadialFaces.Size() * 3;
    while(nonRadialFaces.Size() != 0 && iter > 0){
        MeshFace* currentFace = nonRadialFaces.PopFront();
        bool flipped = false;
        for(int i=0; i<3 && !flipped; i++){
            MeshVertex *v0 = currentFace->GetVertex(i);
            MeshVertex *v1 = currentFace->GetVertex((i+1)%3);
            MeshVertex *v2 = currentFace->GetVertex((i+2)%3);

            if(v1->GetData().HasRadialOrg(v2) || v2->GetData().HasRadialOrg(v1))
                continue;

            if(cpt->GetData().cusp &&
                    ( !(v0->GetData().Radial() || v1->GetData().Radial() || v2->GetData().Radial()) ||
                      (v0 !=cpt && v0->GetData().facing==CONTOUR) ||
                      (v1 !=cpt && v1->GetData().facing==CONTOUR) ||
                      (v2 !=cpt && v2->GetData().facing==CONTOUR))){
                continue;
            }

            MeshEdge* adjacentEdge = v1->GetEdge(v2) ? v1->GetEdge(v2) : v2->GetEdge(v1);
            MeshFace* adjacentFace = adjacentEdge->GetLeftFace() != currentFace ?
                        adjacentEdge->GetLeftFace() : adjacentEdge->GetRightFace();
            if(!adjacentFace) // boundary
                continue;

            assert(currentFace != NULL && adjacentFace != NULL);
            if(cpt->GetData().cusp && IsStandardRadialFace(currentFace) && IsStandardRadialFace(adjacentFace))
                continue;

            if(!nonRadialFaces.HasFace(adjacentFace))
                continue;

            MeshVertex* v3 = GetOtherVertex(adjacentFace,v1,v2);

            if(cpt->GetData().cusp && v3->GetData().facing == CONTOUR)
                continue;

            if((IsStandardRadialFace(v0,v1,v3) || IsStandardRadialFace(v0,v3,v2))){

                MeshVertex *vSkip = NULL;
                // If v1 (resp. v2) is "inside" v0v2v3 (resp. v0v1v3), don't flip (it'd produce folds)
                if(!FlipAllowed(v0,v1,v2,v3,currentFace,adjacentFace,mesh,vSkip,nonRadialFaces,wiggleQueue)){
                    continue;
                }else if(vSkip==NULL && !FlipAllowed(v0,v2,v1,v3,currentFace,adjacentFace,mesh,vSkip,nonRadialFaces,wiggleQueue)){
                    continue;
                }else if(vSkip==NULL && (v0->GetEdge(v3)!=NULL || v3->GetEdge(v0)!=NULL)){
                    MeshEdge* oppEdge = v0->GetEdge(v3) ? v0->GetEdge(v3) : v3->GetEdge(v0);
                    MeshFace* oppFaces[2] = {oppEdge->GetLeftFace(),oppEdge->GetRightFace()};
                    for(int j=0; j<2; j++){
                        if(oppFaces[j]){
                            MeshVertex* otherVertex = GetOtherVertex(oppFaces[j],v0,v3);
                            if(otherVertex==v1 || otherVertex==v2){
                                nonRadialFaces.Remove(oppFaces[j]);
                                wiggleQueue.Remove(oppFaces[j]);
                                mesh->DeleteFace(oppFaces[j]);
                                vSkip=otherVertex;
                                break;
                            }
                        }
                    }
                    if(v0->GetEdge(v3)!=NULL || v3->GetEdge(v0)!=NULL){
#ifdef LINK_FREESTYLE
                        char str[200];
                        sprintf(str, "v0");
                        addRIFDebugPoint(-1, double(v0->GetData().pos[0]), double(v0->GetData().pos[1]), double(v0->GetData().pos[2]), str, 0);
                        sprintf(str, "v3");
                        addRIFDebugPoint(-1, double(v3->GetData().pos[0]), double(v3->GetData().pos[1]), double(v3->GetData().pos[2]), str, 0);
#endif
                        continue;
                    }
                }

                nonRadialFaces.Remove(adjacentFace);
                flipped = true;
                numFlip++;

                wiggleQueue.Remove(currentFace);
                wiggleQueue.Remove(adjacentFace);
                mesh->DeleteFace(currentFace);
                mesh->DeleteFace(adjacentFace);

                assert(vSkip!=NULL || (v0->GetEdge(v3)==NULL && v3->GetEdge(v0)==NULL));
                assert(vSkip==NULL || vSkip==v1 || vSkip==v2);

                MeshFace* newFace = NULL;
                if(!vSkip || (vSkip && vSkip!=v1)){
                    newFace = NewFaceDebug(mesh,v0,v1,v3);
                    assert(newFace != NULL);
                    if(!IsStandardRadialFace(newFace))
                        nonRadialFaces.Insert(newFace);
                }
                if(!vSkip || (vSkip && vSkip!=v2)){
                    newFace = NewFaceDebug(mesh,v0,v3,v2);
                    assert(newFace != NULL);
                    if(!IsStandardRadialFace(newFace))
                        nonRadialFaces.Insert(newFace);
                }
            }
        }
        if(!flipped)
            nonRadialFaces.Insert(currentFace);
        iter--;
    }

    nonRadialFaces.Clear();
    oneRing.clear();
    GetOneRing(cpt,oneRing);
    for(std::set<MeshFace*>::iterator fit = oneRing.begin(); fit!=oneRing.end(); ++fit){
        assert( (*fit) != NULL);
        if(!IsStandardRadialFace(*fit))
            nonRadialFaces.Insert(*fit);
    }
    if(nonRadialFaces.Size()>0){
        return false;
    }
    return true;
}


MeshVertex* SplitFace(MeshFace * face, Mesh * mesh,
                      ParamPointCC & splitPointParam, const vec3 & cameraCenter,
                      PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                      MeshVertex* radialOrg=NULL, bool enqueueNewFace=true)
{
    assert(face != NULL);

    MeshVertex* v[3];
    for(int i=0; i<3; i++){
        v[i] = face->GetVertex(i);
        for(int j=0; j<v[i]->GetData().NumRadialOrg(); j++)
            v[i]->GetData().radialOrg[j] = NULL;
    }

    MeshVertex* newV = mesh->NewVertex();
    SetupVertex(newV->GetData(),splitPointParam,cameraCenter);
    if(radialOrg)
        newV->GetData().AddRadialOrg(radialOrg);

    std::list<MeshFace*> f;
    bool skip[3] = {false, false, false};

    FlipAndInsert(v,newV,face,mesh,f,wiggleQueue,splitQueue,skip);

    wiggleQueue.Remove(face);
    splitQueue.Remove(face);
    mesh->DeleteFace(face);

    if(!skip[0])
        f.push_back(NewFaceDebug(mesh, newV, v[0], v[1]));
    if(!skip[1])
        f.push_back(NewFaceDebug(mesh, newV, v[1], v[2]));
    if(!skip[2])
        f.push_back(NewFaceDebug(mesh, newV, v[2], v[0]));

    if(enqueueNewFace){
        for(std::list<MeshFace*>::iterator it=f.begin(); it!=f.end();it++)
            wiggleQueue.Insert(*it);
    }

    return newV;
}

typedef std::pair<ParamPointCC,ParamPointCC> ParamPointPair;



ParamPointPair FindContourPointIntersectBruteForce(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2,
                                                   MeshVertex* v3, const vec3 & cameraCenter,
                                                   MeshFace* oppFace1, MeshFace* oppFace2)
{
    MeshVertex* oppVertex1=NULL, * oppVertex2=NULL;
    for(int i=0; i<3; ++i){
        if(oppFace1->GetVertex(i) != v0 && oppFace1->GetVertex(i) != v1)
            oppVertex1 = oppFace1->GetVertex(i);
        if(oppFace2->GetVertex(i) != v0 && oppFace2->GetVertex(i) != v2)
            oppVertex2 = oppFace2->GetVertex(i);
    }

    const int NUM_SAMPLES = 40;

    for(int i=1; i<NUM_SAMPLES; i++){   //Sample v0v1
        ParamPointCC pointParam = ParamPointCC::Interpolate(v0->GetData().sourceLoc,
                                                            v1->GetData().sourceLoc,
                                                            real(i)/real(NUM_SAMPLES));
        FacingType pointFacing = Facing(pointParam, cameraCenter, CONTOUR_THRESHOLD);
        ParamPointCC newPt;
        if(pointFacing!=CONTOUR && pointFacing != v3->GetData().facing)
            FindContour(pointParam, v3->GetData().sourceLoc, cameraCenter, newPt);
        else if(pointFacing!=CONTOUR && pointFacing != oppVertex1->GetData().facing)
            FindContour(pointParam, oppVertex1->GetData().sourceLoc, cameraCenter, newPt);
        else{
            assert(pointFacing==CONTOUR);
            newPt = pointParam;
        }
        assert(!newPt.IsNull() && newPt.IsEvaluable());

        vec3 newPos, newNormal;
        newPt.Evaluate(newPos,newNormal);
        vec3 viewVec = cameraCenter - newPos;
        vec3 planeNormal = viewVec ^ newNormal;
        planeNormal.normalize();

        for(int j=1; j<NUM_SAMPLES; j++){   //Sample v0v2
            ParamPointCC pointParam2 = ParamPointCC::Interpolate(v0->GetData().sourceLoc,
                                                                 v2->GetData().sourceLoc,
                                                                 real(j)/real(NUM_SAMPLES));
            FacingType pointFacing2 = Facing(pointParam2, cameraCenter, CONTOUR_THRESHOLD);
            ParamPointCC newPt2;
            if(pointFacing2 != CONTOUR && pointFacing2 != v3->GetData().facing)
                FindContour(pointParam2, v3->GetData().sourceLoc, cameraCenter, newPt2);
            else if(pointFacing2!=CONTOUR && pointFacing2 != oppVertex2->GetData().facing)
                FindContour(pointParam2, oppVertex2->GetData().sourceLoc, cameraCenter, newPt2);
            else{
                assert(pointFacing2==CONTOUR);
                newPt2 = pointParam2;
            }
            assert(!newPt2.IsNull() && newPt2.IsEvaluable());

            vec3 newPos2, newNormal2;
            newPt2.Evaluate(newPos2,newNormal2);
            vec3 viewVec2 = cameraCenter - newPos2;
            vec3 planeNormal2 = viewVec2 ^ newNormal2;
            planeNormal2.normalize();

            bool test1 = TestPlaneEdgeIntersect(newPos,planeNormal,newPos2,v3->GetData().pos);
            bool test2 = TestPlaneEdgeIntersect(newPos2,planeNormal2,newPos,v3->GetData().pos);
            bool test3 = TestPlaneEdgeIntersect(newPos,planeNormal,newPos2,v0->GetData().pos);
            bool test4 = TestPlaneEdgeIntersect(newPos2,planeNormal2,newPos,v0->GetData().pos);

            if((test1 && test2) || (test3 && test4)){
                printf("\ni=%d j=%d\n",i,j);
                return ParamPointPair(newPt,newPt2);
            }
        }
    }
    return ParamPointPair();
}

ParamPointCC FindContourPointIntersect(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2, MeshVertex* v3,
                                       const vec3 & cameraCenter, MeshFace* oppFace)
{
    vec3 viewVec = cameraCenter - v2->GetData().pos;
    vec3 planeNormal = viewVec ^ v2->GetData().normal;
    planeNormal.normalize();

    MeshVertex* oppVertex;
    for(int i=0; i<3; ++i){
        oppVertex = oppFace->GetVertex(i);
        if(oppVertex != v0 && oppVertex != v1)
            break;
    }
    if(oppVertex->GetData().facing == v3->GetData().facing || oppVertex->GetData().facing == CONTOUR){
        printf("\nWEIRD CONFIGURATION\n");
        return ParamPointCC();
    }

    ParamPointCC top = v1->GetData().sourceLoc;
    ParamPointCC bottom = v0->GetData().sourceLoc;

    std::list<ParamPointPair> intervals;
    intervals.push_back(ParamPointPair(top,bottom));

    const int NUM_SAMPLES = 200;
    int s = 0;
    while(!intervals.empty() && s<NUM_SAMPLES){
        top = intervals.front().first;
        bottom = intervals.front().second;
        intervals.pop_front();
        ParamPointCC midPointParam = ParamPointCC::Interpolate(top,bottom,0.5);
        FacingType midFacing = Facing(midPointParam, cameraCenter, CONTOUR_THRESHOLD);
        ParamPointCC newPt;
        if(midFacing!=CONTOUR && midFacing != v3->GetData().facing)
            FindContour(midPointParam, v3->GetData().sourceLoc, cameraCenter, newPt);
        else if(midFacing!=CONTOUR && midFacing != oppVertex->GetData().facing)
            FindContour(midPointParam, oppVertex->GetData().sourceLoc, cameraCenter, newPt);
        else{
            assert(midFacing==CONTOUR);
            newPt = midPointParam;
        }

        assert(!newPt.IsNull() && newPt.IsEvaluable());

        vec3 midPos, midNormal;
        newPt.Evaluate(midPos,midNormal);

        vec3 viewVec2 = cameraCenter - midPos;
        vec3 planeNormal2 = viewVec2 ^ midNormal;
        planeNormal2.normalize();
        bool test1 = TestPlaneEdgeIntersect(midPos,planeNormal2,v2,v3);
        bool test2 = TestPlaneEdgeIntersect(v2->GetData().pos,planeNormal,midPos,v3->GetData().pos);
        bool test3 = TestPlaneEdgeIntersect(midPos,planeNormal2,v0,v2);
        bool test4 = TestPlaneEdgeIntersect(v2->GetData().pos,planeNormal,midPos,v0->GetData().pos);

        if((test1 && test2) || (test3 && test4))
            return newPt;
        if((!test1 && !test3) || (!test2 && !test4)){
            intervals.push_back(ParamPointPair(top,newPt));
            intervals.push_back(ParamPointPair(newPt,bottom));
        }else if(!test2 || !test3){
            intervals.push_back(ParamPointPair(newPt,bottom));
        }else{
            intervals.push_back(ParamPointPair(top,newPt));
        }
        s++;
    }
    return ParamPointCC();
}

MeshFace* FlipEdge(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2, MeshVertex* v3,
                   MeshFace * face, MeshFace * adjacentFace, Mesh * mesh,
                   PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    wiggleQueue.Remove(adjacentFace);
    splitQueue.Remove(adjacentFace);
    mesh->DeleteFace(adjacentFace);
    wiggleQueue.Remove(face);
    splitQueue.Remove(face);
    mesh->DeleteFace(face);

    MeshFace* oldFace;
    if(v3->GetEdge(v1) || v2->GetEdge(v3) || v1->GetEdge(v0) || v3->GetEdge(v0)){
        oldFace = NewFaceDebug(mesh,v1,v3,v0);
        NewFaceDebug(mesh,v3,v2,v0);
    }else{
        oldFace = NewFaceDebug(mesh,v3,v1,v0);
        NewFaceDebug(mesh,v2,v3,v0);
    }
    return oldFace;
}


ParamPointCC SampleEdgeFromCenter(MeshVertex* v0, MeshVertex* v1, const vec3 & planeNormal)
{
    ParamPointCC posParam;
    vec3 limitPos, limitN;
    real d_sup = planeNormal * (v1->GetData().pos - v0->GetData().pos);
    real d;
    const int NUM_SAMPLES = 40;
    for(int i=0; i<NUM_SAMPLES; i++){
        if(i%2==0)
            posParam = ParamPointCC::Interpolate(v0->GetData().sourceLoc,
                                                 v1->GetData().sourceLoc,0.5+0.5*real(i)/real(NUM_SAMPLES));
        else
            posParam = ParamPointCC::Interpolate(v0->GetData().sourceLoc,
                                                 v1->GetData().sourceLoc,0.5-0.5*real(i-1)/real(NUM_SAMPLES));
        if(posParam.IsNull() || !posParam.IsEvaluable())
            continue;
        posParam.Evaluate(limitPos, limitN);
        d = planeNormal * (limitPos - v0->GetData().pos);
        if((d>0.0 && d_sup>0) || (d<0.0 && d_sup<0.0)){
            return posParam;
        }
    }

    return v1->GetData().sourceLoc;
}

bool InsertRadialEdgeFaceCenter(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2, const vec3 & planeNormal,
                                MeshFace * face, Mesh * mesh, const vec3 & cameraCenter, bool allowShifts,
                                PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    if(!TestPlaneEdgeIntersect(v0->GetData().pos, planeNormal, v1, v2))
        return false;

    // Intersection with the radial plane found

    ParamPointCC leftPosParam = SampleEdgeFromCenter(v0,v1,planeNormal);
    ParamPointCC rightPosParam = SampleEdgeFromCenter(v0,v2,planeNormal);

    ParamPointCC splitPointParam = PlaneEdgeIntersection(v0->GetData().pos, planeNormal, leftPosParam, rightPosParam);
    if(splitPointParam.IsNull() || !splitPointParam.IsEvaluable())
        return false;

    FacingType ft = Facing(splitPointParam, cameraCenter, CONTOUR_THRESHOLD);


    if(ft!=v1->GetData().facing && ft!=v2->GetData().facing){

        const int NUM_SAMPLES = 201;

        for(int i=1; i<NUM_SAMPLES; i++){
            ParamPointCC param = ParamPointCC::Interpolate(v0->GetData().sourceLoc,v1->GetData().sourceLoc,real(i)/real(NUM_SAMPLES));
            ParamPointCC cptParam = PlaneEdgeIntersection(v0->GetData().pos,planeNormal,param,v2->GetData().sourceLoc);
            if(cptParam.IsNull())
                continue;
            vec3 limitPos, limitN;
            cptParam.Evaluate(limitPos, limitN);

            if(Facing(cptParam,cameraCenter)!=ft){
                splitPointParam = cptParam;
                break;
            }
        }
    }


    if(splitPointParam.IsNull() || !splitPointParam.IsEvaluable()){
#ifdef VERBOSE
        printf("\nCAN'T INSERT RADIAL EDGE\n");
#endif
        return false;
    }

    SplitFace(face,mesh,splitPointParam,cameraCenter,wiggleQueue,splitQueue,v0,false);
    return true;
}

bool InsertRadialEdge(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2, const vec3 & planeNormal,
                      MeshFace * face, Mesh * mesh, const vec3 & cameraCenter, bool allowShifts,
                      PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                      std::set<MeshVertex*> & contourPoints)
{
    contourPoints.insert(v1);
    if(v2->GetData().facing == CONTOUR){
        return false;
    }

    vec3 viewVec2 = cameraCenter - v1->GetData().pos;
    vec3 planeNormal2 = viewVec2 ^ v1->GetData().normal;
    planeNormal2.normalize();

    bool intersect_v0 = TestPlaneEdgeIntersect(v0->GetData().pos,planeNormal,v1,v2);
    bool intersect_v1 = TestPlaneEdgeIntersect(v1->GetData().pos,planeNormal2,v0,v2);
    if(intersect_v0 && intersect_v1){
        // find the intersection point of the 2 radial planes inside the triangle
        // => rule 3B
#ifdef VERBOSE
        printf("RADIAL EDGES INTERSECT IN TRIS\n");
#endif

        ParamPointCC p[3];
        real r1[3];
        real r2[3];
        vec3 limitPos[3];
        vec3 limitNormal[3];
        for(int k=0; k<3; k++){
            VertexDataCatmark & data = face->GetVertex(k)->GetData();
            p[k] = data.sourceLoc;
            r1[k] = radialPlaneIntersect(v0->GetData().pos,v0->GetData().normal,data.pos,cameraCenter);
            r2[k] = radialPlaneIntersect(v1->GetData().pos,v1->GetData().normal,data.pos,cameraCenter);
            limitPos[k] = data.pos ;
            limitNormal[k] = data.normal;
        }

        vec3 vPos[2], vNormal[2];
        vPos[0] = v0->GetData().pos;
        vPos[1] = v1->GetData().pos;
        vNormal[0] = v0->GetData().normal;
        vNormal[1] = v1->GetData().normal;
        ParamPointCC newPosParam = RootFindContour(RF_RADIAL_INT, p, r1, r2, limitPos, limitNormal,
                                                   vPos, vNormal, cameraCenter);

        if(!newPosParam.IsNull() && newPosParam.IsEvaluable()){
            printf("FOUND\n");
            MeshVertex* newV = SplitFace(face,mesh,newPosParam,cameraCenter,wiggleQueue,splitQueue,v0,false);
            newV->GetData().AddRadialOrg(v1);
            return true;
        }else{
            printf("NOT FOUND\n");
            return InsertRadialEdgeFaceCenter(v0,v1,v2,planeNormal,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue);
        }
        return false;
    }
    if(intersect_v0){
        //        if(RadialEdgesIntersectOnEdge(v0,v1,v2,face,mesh,cameraCenter,wiggleQueue,splitQueue))
        //            return true;

        return InsertRadialEdgeFaceCenter(v0,v1,v2,planeNormal,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue);
    }
    if(intersect_v1){
        //        if(RadialEdgesIntersectOnEdge(v1,v0,v2,face,mesh,cameraCenter,wiggleQueue,splitQueue))
        //            return true;

        return InsertRadialEdgeFaceCenter(v1,v0,v2,planeNormal2,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue);
    }
    return false;
}

bool InsertRadialEdges(MeshFace * face, Mesh * mesh, const vec3 & cameraCenter, bool allowShifts,
                       PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                       std::set<MeshVertex*> & contourPoints)
{
    for(int i=0; i<3; i++){
        MeshVertex* v0 = face->GetVertex(i);

        if(v0->GetData().facing != CONTOUR)
            continue;

        contourPoints.insert(v0);

        vec3 viewVec = cameraCenter - v0->GetData().pos;
        vec3 planeNormal = viewVec ^ v0->GetData().normal;
        planeNormal.normalize();

        MeshVertex* v1 = face->GetVertex((i+1)%3);
        MeshVertex* v2 = face->GetVertex((i+2)%3);

        if(v1->GetData().facing == CONTOUR){
            return InsertRadialEdge(v0,v1,v2,planeNormal,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue,contourPoints);
        }
        if(v2->GetData().facing == CONTOUR){
            return InsertRadialEdge(v0,v2,v1,planeNormal,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue,contourPoints);
        }

        // v0 is the only contour point => rule 3A
        return InsertRadialEdgeFaceCenter(v0,v1,v2,planeNormal,face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue);
    }
    return false;
}

bool FixFold(int cusp_idx, MeshFace* face, Mesh* mesh, const vec3 & cameraCenter)
{
    if(!IsConsistent(face,cameraCenter)){
        MeshVertex* cusp = face->GetVertex(cusp_idx);
        MeshVertex* v0 = face->GetVertex((cusp_idx+1)%3);
        MeshVertex* v1 = face->GetVertex((cusp_idx+2)%3);
        MeshEdge* adjacentEdge = v0->GetEdge(v1) ? v0->GetEdge(v1) : v1->GetEdge(v0);
        MeshFace* adjacentFace = adjacentEdge->GetLeftFace() != face ? adjacentEdge->GetLeftFace() : adjacentEdge->GetRightFace();
        MeshVertex* oppV = NULL;
        for(int i=0; i<3; i++)
            if(adjacentFace->GetVertex(i)!=v0 && adjacentFace->GetVertex(i)!=v1){
                oppV = adjacentFace->GetVertex(i);
                break;
            }
        if(!oppV || oppV->GetData().facing==CONTOUR)
            return false;

        MeshVertex* f0[3] = {cusp,v0,oppV};
        MeshVertex* f1[3] = {cusp,oppV,v1};

        printf("\nFLIP\n");

        mesh->DeleteFace(face);
        mesh->DeleteFace(adjacentFace);
        NewFaceDebug(mesh,f0[0],f0[1],f0[2]);
        NewFaceDebug(mesh,f1[0],f1[1],f1[2]);
    }
    return true;
}

MeshVertex* InsertCusp(ParamPointCC & cuspParam, MeshVertex* cPts[2], MeshVertex* oppPts[2],
                       MeshFace* faces[2], Mesh* mesh, const vec3 & cameraCenter,
                       PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    MeshVertex* newV = mesh->NewVertex();
    SetupVertex(newV->GetData(),cuspParam,cameraCenter);
    newV->GetData().facing = CONTOUR;
    newV->GetData().cusp = true;

    std::list<MeshFace*> newFaces;
    bool skip0[3] = {false,false,false};
    bool skip1[3] = {false,false,false};

    real lambda0, mu0;
    real lambda1, mu1;
    MeshVertex* vertices0[3] = {oppPts[0],cPts[0],cPts[1]};
    MeshVertex* vertices1[3] = {oppPts[1],cPts[0],cPts[1]};
    if(Outside(vertices0,newV,lambda0,mu0) && Outside(vertices1,newV,lambda1,mu1)){
        if(lambda0<0 || mu0<0){
            FlipAndInsert(vertices0,newV,faces[0],mesh,newFaces,wiggleQueue,splitQueue,skip0);
        }else if(lambda1<0 || mu1<0){
            FlipAndInsert(vertices1,newV,faces[1],mesh,newFaces,wiggleQueue,splitQueue,skip1);
        }else{
            printf("CUSP OUTSIDE: CAN'T FLIP (%.10f,%.10f,%.10f,%.10f)\n",double(lambda0),double(mu0),double(lambda1),double(mu1));
        }
    }

    wiggleQueue.Remove(faces[0]);
    splitQueue.Remove(faces[0]);
    mesh->DeleteFace(faces[0]);
    wiggleQueue.Remove(faces[1]);
    splitQueue.Remove(faces[1]);
    mesh->DeleteFace(faces[1]);

    vec3 viewVec = newV->GetData().pos - cameraCenter;
    vec3 planeNormal = viewVec ^ newV->GetData().normal;
    planeNormal.normalize();

    if(cPts[0]->GetEdge(oppPts[0]) || oppPts[0]->GetEdge(cPts[1])
            || oppPts[1]->GetEdge(cPts[0]) ||cPts[1]->GetEdge(oppPts[1])){
        if(!skip0[0])
            newFaces.push_back(NewFaceDebug(mesh, cPts[0], newV, oppPts[0]));
        if(!skip0[2])
            newFaces.push_back(NewFaceDebug(mesh, cPts[1], oppPts[0], newV));
        if(!skip1[0])
            newFaces.push_back(NewFaceDebug(mesh, cPts[0], oppPts[1], newV));
        if(!skip1[2])
            newFaces.push_back(NewFaceDebug(mesh, cPts[1], newV, oppPts[1]));

        //        if(!TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[0],oppPts[0]) &&
        //                !TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[1],oppPts[0]))
        //            FixFold(1,f0,mesh,cameraCenter);
        //        if(!TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[0],oppPts[0]) &&
        //                !TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[1],oppPts[0]))
        //            FixFold(2,f1,mesh,cameraCenter);
    }else{
        if(!skip0[0])
            newFaces.push_back(NewFaceDebug(mesh, cPts[0], oppPts[0], newV));
        if(!skip0[2])
            newFaces.push_back(NewFaceDebug(mesh, cPts[1], newV, oppPts[0]));
        if(!skip1[0])
            newFaces.push_back(NewFaceDebug(mesh, cPts[0], newV, oppPts[1]));
        if(!skip1[2])
            newFaces.push_back(NewFaceDebug(mesh, cPts[1], oppPts[1], newV));

        //        if(!TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[0],oppPts[0]) &&
        //                !TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[1],oppPts[0]))
        //            FixFold(2,f0,mesh,cameraCenter);
        //        if(!TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[0],oppPts[0]) &&
        //                !TestPlaneEdgeIntersect(newV->GetData().pos,planeNormal,cPts[1],oppPts[0]))
        //            FixFold(1,f1,mesh,cameraCenter);
    }

    for(std::list<MeshFace*>::iterator it=newFaces.begin(); it!=newFaces.end();it++)
        wiggleQueue.Insert(*it);

    return newV;
}

bool InsertSmoothCusp(MeshFace * face, Mesh * mesh,const vec3 & cameraCenter, bool allowShifts,
                      PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{

    std::vector<MeshVertex*> contourPts;
    MeshVertex* v=NULL;
    for(int i=0; i<3; i++)
        if(face->GetVertex(i)->GetData().facing == CONTOUR)
            contourPts.push_back(face->GetVertex(i));
        else
            v = face->GetVertex(i);

    if(contourPts.size() == 2){

        MeshEdge* e = contourPts[0]->GetEdge(contourPts[1]) ? contourPts[0]->GetEdge(contourPts[1]) : contourPts[1]->GetEdge(contourPts[0]);
        MeshFace* adj = e->GetLeftFace() != face ? e->GetLeftFace() : e->GetRightFace();
        if(adj){
            wiggleQueue.Remove(adj);
            MeshVertex * opp=NULL;
            for(int j=0; j<3; j++)
                if(adj->GetVertex(j) != contourPts[0] && adj->GetVertex(j) != contourPts[1])
                    opp = adj->GetVertex(j);
            assert(opp!=NULL);
            real radCurv[4];
            v->GetData().sourceLoc.RadialCurvature(cameraCenter,radCurv[0]);
            opp->GetData().sourceLoc.RadialCurvature(cameraCenter,radCurv[1]);
            contourPts[0]->GetData().sourceLoc.RadialCurvature(cameraCenter,radCurv[2]);
            contourPts[1]->GetData().sourceLoc.RadialCurvature(cameraCenter,radCurv[3]);
            int numPos = 0;
            int numNeg = 0;
            for(int j=0; j<4; j++)
                if(radCurv[j]>0.0)
                    numPos++;
                else if(radCurv[j]<0.0)
                    numNeg++;
            //assert((numPos+numNeg)==4);
            if((numPos+numNeg)!=4)
                return false;
            if(numPos==numNeg ||
                    (fabs(numPos-numNeg)<=2 &&
                     ((radCurv[2]>0.0 && radCurv[3]<0.0) || (radCurv[2]<0.0 && radCurv[3]>0.0)))){

                MeshFace* faces[2] = {face, adj};
                ParamPointCC cuspParam;

                for(int j=0; j<2; j++){
                    ParamPointCC p[3];
                    real r[3];
                    real ndotv[3];
                    vec3 limitPos[3];
                    vec3 limitNormal[3];
                    for(int i=0; i<3; i++){
                        VertexDataCatmark & data = faces[j]->GetVertex(i)->GetData();
                        real radCurv;
                        data.sourceLoc.RadialCurvature(cameraCenter,radCurv);
                        r[i] = radCurv;
                        p[i] = data.sourceLoc;
                        ndotv[i] = data.facing == CONTOUR ? 0.0 : data.ndotv;
                        limitPos[i] = data.pos ;
                        limitNormal[i] = data.normal;
                    }
                    cuspParam = RootFindContour(RF_CUSP, p, r, ndotv, limitPos, limitNormal,
                                                NULL, NULL, cameraCenter);

                    if(!cuspParam.IsNull() && cuspParam.IsEvaluable()){
                        real r;
                        bool success = cuspParam.RadialCurvature(cameraCenter,r);
                        if(Facing(cuspParam,cameraCenter,CONTOUR_THRESHOLD)!=CONTOUR || !success || fabsl(r)>0.01)
                            return false;

                        MeshVertex* cPts[2]  = {NULL, NULL};
                        MeshVertex* oppPts[2] = {NULL, NULL};
                        int cPts_idx = 0;
                        for(int k=0; k<3; k++)
                            if(faces[j]->GetVertex(k)->GetData().facing == CONTOUR){
                                cPts[cPts_idx] = faces[j]->GetVertex(k);
                                cPts_idx++;
                            }else{
                                if(oppPts[0]!=NULL)
                                    return false;
                                oppPts[0] = faces[j]->GetVertex(k);
                            }
                        assert(cPts_idx == 2);
                        assert(oppPts[0]);
                        for(int k=0; k<3; k++)
                            if(faces[(j+1)%2]->GetVertex(k) != cPts[0] &&
                                    faces[(j+1)%2]->GetVertex(k) != cPts[1]){
                                oppPts[1] = faces[(j+1)%2]->GetVertex(k);
                                break;
                            }
                        assert(oppPts[1]);

                        MeshVertex* newV = InsertCusp(cuspParam,cPts,oppPts,faces,mesh,cameraCenter,wiggleQueue,splitQueue);
#if LINK_FREESTYLE
                        char str[200];
                        real ndotv;
                        success = newV->GetData().sourceLoc.RadialCurvature(cameraCenter,r);
                        Facing(newV->GetData().sourceLoc,cameraCenter,CONTOUR_THRESHOLD,&ndotv);
                        sprintf(str, "SMOOTH CUSP POST (n.v=%.10f, r=%.10f, success=%d)",double(ndotv),double(r),success);
                        addRIFDebugPoint(3, double(newV->GetData().pos[0]), double(newV->GetData().pos[1]), double(newV->GetData().pos[2]), str, 0);
#endif
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/*
bool CheckAdjacentFaces(MeshFace* face, Mesh* mesh, const vec3 & cameraCenter,
                        PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    std::vector<MeshVertex*> contourPts;
    MeshVertex* v;
    for(int i=0; i<3; i++)
        if(face->GetVertex(i)->GetData().facing == CONTOUR)
            contourPts.push_back(face->GetVertex(i));
        else
            v = face->GetVertex(i);
    if(contourPts.size() == 1){
        MeshVertex* cpt = contourPts[0];
        //bool rootFindNeeded = false;
        for(int i=0; i<3; i++){
            if(face->GetVertex(i)!=cpt){
                MeshVertex* cur = face->GetVertex(i);
                ParamPointCC p = ParamPointCC::Interpolate(cpt->GetData().sourceLoc,cur->GetData().sourceLoc,0.01);
                vec3 limitPos, limitN;
                p.Evaluate(limitPos, limitN);
                real ndotv;
                vec3 viewVec = limitPos - cameraCenter;
                FacingType ft = Facing(viewVec, limitN, CONTOUR_THRESHOLD, &ndotv);
                if(ft != cur->GetData().facing){
                    real t = FindZeroCrossingBySampling(cpt->GetData().sourceLoc,
                                                              cur->GetData().sourceLoc,
                                                              cur->GetData().facing, cameraCenter);
                    if(t!=-1){
                        ParamPointCC newLoc = ParamPointCC::Interpolate(cpt->GetData().sourceLoc,
                                                                        cur->GetData().sourceLoc, t);
                        assert(!newLoc.IsNull() && newLoc.IsEvaluable());
                        printf("CONTOUR POINT\n");

                        MeshEdge* cEdge = cpt->GetEdge(cur) ? cpt->GetEdge(cur) : cur->GetEdge(cpt);
                        assert(cEdge);
                        MeshFace* oppFace = cEdge->GetLeftFace() != face ? cEdge->GetLeftFace() : cEdge->GetRightFace();
                        MeshVertex* oppPts[2];
                        oppPts[0] = face->GetVertex((i+1)%3) != cpt ? face->GetVertex((i+1)%3) : face->GetVertex((i+2)%3);
                        for(int j=0; j<3; j++)
                            if(oppFace->GetVertex(j)->GetData().facing == CONTOUR &&
                                    oppFace->GetVertex(j) != cpt){
                                oppPts[1] = oppFace->GetVertex(j);
                                break;
                            }

                        wiggleQueue.Remove(face);
                        splitQueue.Remove(face);
                        wiggleQueue.Remove(oppFace);
                        splitQueue.Remove(oppFace);
                        mesh->DeleteFace(face);
                        mesh->DeleteFace(oppFace);

                        MeshVertex* newV = mesh->NewVertex();
                        SetupVertex(newV->GetData(),newLoc,cameraCenter);
                        newV->GetData().facing = CONTOUR;

                        MeshFace *f0, *f1, *f2, *f3;
                        if(cpt->GetEdge(oppPts[0]) || oppPts[0]->GetEdge(cur)
                                || oppPts[1]->GetEdge(cpt) || cur->GetEdge(oppPts[1])){
                            f0 = NewFaceDebug(mesh, cpt, newV, oppPts[0]);
                            f1 = NewFaceDebug(mesh, cur, oppPts[0], newV);
                            f2 = NewFaceDebug(mesh, cpt, oppPts[1], newV);
                            f3 = NewFaceDebug(mesh, cur, newV, oppPts[1]);
                        }else{
                            f0 = NewFaceDebug(mesh, cpt, oppPts[0], newV);
                            f1 = NewFaceDebug(mesh, cur, newV, oppPts[0]);
                            f2 = NewFaceDebug(mesh, cpt, newV, oppPts[1]);
                            f3 = NewFaceDebug(mesh, cur, oppPts[1], newV);
                        }
                        //wiggleQueue.Insert(f2);

                        MeshFace* oppFace2 = oppPts[1]->GetEdge(cpt)->GetLeftFace() != f2 ?
                                    oppPts[1]->GetEdge(cpt)->GetLeftFace() :
                                    oppPts[1]->GetEdge(cpt)->GetRightFace();

                        MeshVertex* oppVertex;
                        for(int j=0; j<3; j++)
                            if(oppFace2->GetVertex(j) != cpt && oppFace2->GetVertex(j) != oppPts[1]){
                                oppVertex = oppFace2->GetVertex(j);
                                break;
                            }

                        mesh->DeleteFace(f2);
                        mesh->DeleteFace(oppFace2);

                        if(cpt->GetEdge(oppVertex) || oppVertex->GetEdge(newV)
                                || oppPts[1]->GetEdge(newV) || oppVertex->GetEdge(oppPts[1])){
                            f0 = NewFaceDebug(mesh, cpt, newV, oppVertex);
                            f1 = NewFaceDebug(mesh, oppPts[1], oppVertex, newV);
                        }else{
                            f0 = NewFaceDebug(mesh, cpt, oppVertex, newV);
                            f1 = NewFaceDebug(mesh, oppPts[1], newV, oppVertex);
                        }

                        return true;
                    }
                }
            }
        }
        return false;
    }
    if(contourPts.size() == 3){
        printf("\n3 CPTS\n");
    }
    return false;
}*/

bool TestTangentInTriangle(ParamPointCC & newLoc1, ParamPointCC & newLoc2,
                           MeshVertex* v, const vec3 & cameraCenter)
{
    ParamPointCC nearPoint1 = ParamPointCC::Interpolate(newLoc1,v->GetData().sourceLoc,0.01);
    assert(!nearPoint1.IsNull() && nearPoint1.IsEvaluable());
    vec3 limitPos, limitN, viewVec;
    nearPoint1.Evaluate(limitPos, limitN);
    viewVec = limitPos - cameraCenter;
    FacingType nf1 = Facing(viewVec, limitN, CONTOUR_THRESHOLD);
    ParamPointCC nearPoint2 = ParamPointCC::Interpolate(newLoc2,v->GetData().sourceLoc,0.01);
    assert(!nearPoint2.IsNull() && nearPoint2.IsEvaluable());
    nearPoint2.Evaluate(limitPos, limitN);
    viewVec = limitPos - cameraCenter;
    FacingType nf2 = Facing(viewVec, limitN, CONTOUR_THRESHOLD);
    return (nf1==v->GetData().facing && nf2 == v->GetData().facing);
}

bool BuildContour(MeshFace* face, Mesh * mesh, const vec3 & cameraCenter, bool allowShifts,
                  PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    for(int e=0; e<3; ++e){
        MeshVertex * v0 = face->GetVertex(e);
        MeshVertex * v1 = face->GetVertex((e+1)%3);
        MeshVertex * v2 = face->GetVertex((e+2)%3);
        FacingType f0 = v0->GetData().facing;
        FacingType f1 = v1->GetData().facing;
        FacingType f2 = v2->GetData().facing;

        if(f0 != CONTOUR && f1 != CONTOUR && f2 != CONTOUR && f0 != f1 && f0 != f2){
            // Find the two contour points by root-finding
            ParamPointCC newLoc1, newLoc2;
            MeshVertex * extSrc = NULL;
            bool rootFindingFailed = !FindContour(v0,v1,cameraCenter,newLoc1,extSrc);
            assert(!rootFindingFailed && !newLoc1.IsNull() && newLoc1.IsEvaluable());

            rootFindingFailed = !FindContour(v0,v2,cameraCenter,newLoc2,extSrc);
            assert(!rootFindingFailed && !newLoc2.IsNull() && newLoc2.IsEvaluable());

            //Find the correct way of spliting the face

            //Case 1
            if(TestTangentInTriangle(newLoc1, newLoc2, v1, cameraCenter)){

                return true;
            }

            //Case 2
            if(TestTangentInTriangle(newLoc1, newLoc2, v2, cameraCenter)){

                return true;
            }

            printf("\n CASE 3\n");
            return true;
        }
    }
    return false;
}

bool InsertSmoothCuspFirst(MeshFace* face, Mesh* mesh, const vec3 & cameraCenter, bool allowShifts,
                           PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                           bool insert, std::set<std::pair<MeshVertex*,MeshVertex*> >& cuspEdges){
    ParamPointCC p[3];
    real r[3];
    real ndotv[3];
    vec3 limitPos[3];
    vec3 limitNormal[3];
    bool curvatureError = false;
    for(int i=0; i<3; i++){
        VertexDataCatmark & data = face->GetVertex(i)->GetData();
        real radCurv;
        if(!data.sourceLoc.RadialCurvature(cameraCenter,radCurv)){
            curvatureError = true;
        }
        r[i] = radCurv;
        p[i] = data.sourceLoc;
        ndotv[i] = data.facing == CONTOUR ? 0.0 : data.ndotv;
        limitPos[i] = data.pos ;
        limitNormal[i] = data.normal;
    }

    if(curvatureError){
        if(!SameSign(ndotv))
            printf("\nRADIAL CURVATURE UNDEFINED: CUSP MIGHT BE MISSING\n");
        return false;
    }

    ParamPointCC cuspParam = RootFindContour(RF_CUSP, p, r, ndotv, limitPos, limitNormal,
                                             NULL, NULL, cameraCenter);

    if(!cuspParam.IsNull()){
        real r;
        bool success = cuspParam.RadialCurvature(cameraCenter,r);
        if(Facing(cuspParam,cameraCenter,CONTOUR_THRESHOLD)!=CONTOUR || !success || fabsl(r)>0.01){
            printf("\nSPURIOUS CUSP\n");
#if LINK_FREESTYLE
            char str[200];
            cuspParam.Evaluate(limitPos[0],limitNormal[0]);
            real r, ndotv;
            bool success = cuspParam.RadialCurvature(cameraCenter,r);
            Facing(cuspParam,cameraCenter,CONTOUR_THRESHOLD,&ndotv);
            sprintf(str, "SPURIOUS CUSP (n.v=%.10f, r=%.10f, success=%d)",double(ndotv),double(r),success);
            addRIFDebugPoint(-1, double(limitPos[0][0]), double(limitPos[0][1]), double(limitPos[0][2]), str, 0);
#endif
            return false;
        }

        if(insert){
            MeshVertex* v = SplitFace(face,mesh,cuspParam,cameraCenter,wiggleQueue,splitQueue,NULL,false);
            v->GetData().cusp = true;
            v->GetData().facing = CONTOUR;
            return true;
        }else{
            int oppVertexIdx;
            for(oppVertexIdx=0; oppVertexIdx<3; oppVertexIdx++){
                if(face->GetVertex(oppVertexIdx)->GetData().facing!=face->GetVertex((oppVertexIdx+1)%3)->GetData().facing &&
                        face->GetVertex(oppVertexIdx)->GetData().facing!=face->GetVertex((oppVertexIdx+2)%3)->GetData().facing /*&&
                                face->GetVertex(oppVertexIdx)->GetData().facing!=CONTOUR*/)
                    break;
            }
            MeshVertex* oppV = face->GetVertex(oppVertexIdx);
            MeshVertex* v0 = face->GetVertex((oppVertexIdx+1)%3);
            MeshVertex* v1 = face->GetVertex((oppVertexIdx+2)%3);
            std::vector<ParamPointCC> pts;
            pts.push_back(cuspParam);
            pts.push_back(oppV->GetData().sourceLoc);
            pts.push_back(v0->GetData().sourceLoc);
            pts.push_back(v1->GetData().sourceLoc);
            std::vector<vec2> abs;
            ChartCC * chart = ParamPointCC::FindChart(pts,abs);
            if(chart == NULL){
                printf("CHART NOT FOUND\n");
                return false;
            }

            real raydir_a = abs[1][0] - abs[0][0];
            real raydir_b = abs[1][1] - abs[0][1];

            // line segment direction
            real linedir_a = abs[3][0] - abs[2][0];
            real linedir_b = abs[3][1] - abs[2][1];

            real denom = linedir_b * raydir_a - linedir_a * raydir_b;
            real u_a = (linedir_a * (abs[0][1] - abs[2][1])
                              - linedir_b * (abs[0][0] - abs[2][0])) / denom;

            ParamPointCC edgeIntersectionParam = chart->ABtoParam(abs[0][0] + u_a * raydir_a, abs[0][1] + u_a * raydir_b);

            if(edgeIntersectionParam.IsNull()){
                printf("AB to Param conversion failed\n");
                return false;
            }

            if(oppV->GetData().facing == CONTOUR || Facing(edgeIntersectionParam,cameraCenter,CONTOUR_THRESHOLD)==CONTOUR){
//#if LINK_FREESTYLE
//                char str[200];
//                sprintf(str, "FACING WRONG");
//                limitPos[0] = oppV->GetData().pos;
//                addRIFDebugPoint(-1, double(limitPos[0][0]), double(limitPos[0][1]), double(limitPos[0][2]), str, NULL);
//#endif
                printf("SKIP: FACING WRONG\n");
                return false;
            }

            MeshVertex* newV=NULL;

            if(allowShifts)
                newV = ShiftVertex(v0,v1,edgeIntersectionParam,cameraCenter,mesh,&wiggleQueue,&splitQueue,false,false);
            if(newV){
                MeshEdge* adjacentEdge;
                if(newV==v0)
                    adjacentEdge = v0->GetEdge(oppV) ? v0->GetEdge(oppV) : oppV->GetEdge(v0);
                else
                    adjacentEdge = v1->GetEdge(oppV) ? v1->GetEdge(oppV) : oppV->GetEdge(v1);
                MeshFace* adjacentFace = adjacentEdge->GetLeftFace() != face ? adjacentEdge->GetLeftFace() : adjacentEdge->GetRightFace();

                wiggleQueue.Remove(adjacentFace);
                //printf("SHIFT\n");
            }else{
                newV = mesh->NewVertex();
                SetupVertex(newV->GetData(),edgeIntersectionParam,cameraCenter);
                //            printf("\n%d %d %d\n",face->GetVertex(oppVertexIdx)->GetData().facing,face->GetVertex((oppVertexIdx+1)%3)->GetData().facing,face->GetVertex((oppVertexIdx+2)%3)->GetData().facing);
                //            edgeIntersectionParam.Print();
                //            cuspParam.Print();
                if(newV->GetData().facing == oppV->GetData().facing){
                    printf("SKIP2: FACING WRONG\n");
                    mesh->DeleteVertex(newV);
                    return false;
                }
                InsertVertex<VertexDataCatmark>(face,oppVertexIdx,newV,mesh,wiggleQueue,splitQueue,false,false);
            }
            cuspEdges.insert(std::pair<MeshVertex*,MeshVertex*>(oppV,newV));
#if LINK_FREESTYLE
            char str[200];
            cuspParam.Evaluate(limitPos[0],limitNormal[0]);
            real r, ndotv;
            bool success = cuspParam.RadialCurvature(cameraCenter,r);
            Facing(cuspParam,cameraCenter,CONTOUR_THRESHOLD,&ndotv);
            sprintf(str, "SMOOTH CUSP PRE (n.v=%.10f, r=%.10f, success=%d)",double(ndotv),double(r),success);
            addRIFDebugPoint(3, double(limitPos[0][0]), double(limitPos[0][1]), double(limitPos[0][2]), str, 0);
#endif
        }
        return true;
    }
    return false;
}

bool CheckZeroCrossingBySampling(MeshVertex* v0, MeshVertex* v1, int v2Idx,
                                 MeshFace* face, Mesh* mesh, const vec3 & cameraCenter,
                                 PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
  int id = std::max(v0->GetData().id,v1->GetData().id)+1;
  if(id>5)
    return false;

    bool found = false;
    ParamPointCC pt;

    const int NUM_SAMPLES = 10;
    for(int j=0; j<NUM_SAMPLES; j++){
        real t = real(j+1.0)/real(NUM_SAMPLES + 1);
        pt = ParamPointCC::Interpolate(v0->GetData().sourceLoc, v1->GetData().sourceLoc, t);
        if(pt.IsNull() || !pt.IsEvaluable())
            continue;
        FacingType ft = Facing(pt,cameraCenter);
        if(ft!=v0->GetData().facing){
            found = true;
            break;
        }
    }
    if(found){
        MeshVertex* newV = mesh->NewVertex();
        SetupVertex(newV->GetData(),pt,cameraCenter);
    newV->GetData().id = id;

        InsertVertex<VertexDataCatmark>(face,v2Idx,newV,mesh,wiggleQueue,splitQueue);
        return true;
    }
    return false;
}

void RefineContourRadial(Mesh * mesh, const vec3 & cameraCenter,
                         const bool allowShifts,
                         const RefineRadialStep lastStep)
{
    PriorityQueueCatmark wiggleQueue;
    PriorityQueueCatmark splitQueue;

    bool localAllowShifts = allowShifts;

    // find all inconsistent faces and put them in the queue
    std::list<MeshFace*> faces;
    mesh->GetFaces(std::back_inserter(faces));

    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    printf("Num faces: %d\n", mesh->GetNumFaces());
    printf("Initial queue: %d\n", (int)wiggleQueue.Size());

    std::set<std::pair<MeshVertex*,MeshVertex*> > badEdges;

    unsigned int numCUSPs = 0;
    int numZCs = 0;
    int numREs = 0;
    int numFlips = 0;
    int numSplits = 0;
    int numExtend = 0;

    //---------------- MAIN LOOP: iterate over the faces until everything is consistent --------

    printf("Refining contour\n");

#if 1
    // Check zero-crossing by sampling
    while (wiggleQueue.Size() != 0) {
        fflush(stdout);
        MeshFace * face = wiggleQueue.PopFront();

        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);
        for(int i=0; i<3; i++){
            MeshVertex* v0 = face->GetVertex(i);
            MeshVertex* v1 = face->GetVertex((i+1)%3);

            if(v0->GetData().facing != v1->GetData().facing)
                continue;

            if(CheckZeroCrossingBySampling(v0,v1,(i+2)%3,face,mesh,cameraCenter,wiggleQueue,splitQueue)){
                numSplits++;
                break;
            }
        }
    }

    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);
#endif

    if(lastStep == PREPROCESS)
        return;

#if 1
    // If insert==true: insert cusps before contour points
    // Else: detect cusps but don't insert them. Instead, create an edge passing through the cusp.

    std::set<std::pair<MeshVertex*,MeshVertex*> > cuspEdges;

    while (wiggleQueue.Size() != 0) {
        fflush(stdout);
        MeshFace * face = wiggleQueue.PopFront();

        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);
        if(InsertSmoothCuspFirst(face,mesh,cameraCenter, allowShifts, wiggleQueue, splitQueue,false,cuspEdges))
            numCUSPs++;
    }

    //assert(cuspEdges.size()==numCUSPs);

    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    splitQueue.Clear();
#endif

    if(lastStep == DETECT_CUSP)
        return;

#if 1
    // Insert contour points
    while (wiggleQueue.Size() != 0)
    {
        fflush(stdout);
        MeshFace * face = wiggleQueue.PopFront();
        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        if (SplitZeroCrossingFace(face, mesh, cameraCenter, localAllowShifts, wiggleQueue, splitQueue, badEdges, cuspEdges)){
            numZCs++;

            while (splitQueue.Size() != 0)
            {
                // re-check cusp in new triangles
                face = splitQueue.PopFront();

                int numCpt = 0;
                for(int i=0; i<3; i++){
                    if(face->GetVertex(i)->GetData().facing == CONTOUR)
                        numCpt++;
                }
                bool searchCusp = true;
                for(std::set<std::pair<MeshVertex*,MeshVertex*> >::iterator it=cuspEdges.begin(); it!=cuspEdges.end() && searchCusp; it++){
                    for(int i=0; i<3; i++){
                        if((*it).first==face->GetVertex(i) || (*it).second==face->GetVertex(i)){
                            searchCusp = false;
                            break;
                        }
                    }
                }
                if(searchCusp)
                    if((numCpt<2 && InsertSmoothCuspFirst(face,mesh,cameraCenter,allowShifts,wiggleQueue,splitQueue,false,cuspEdges)) ||
                            (numCpt==2 && InsertSmoothCusp(face,mesh,cameraCenter,false,wiggleQueue,splitQueue)))
                        numCUSPs++;
            }

            continue;
        }
    }
    //assert(cuspEdges.size()==numCUSPs);
    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    splitQueue.Clear();
#endif

#if 1
    // Insert contour points
    while (wiggleQueue.Size() != 0)
    {
        fflush(stdout);
        MeshFace * face = wiggleQueue.PopFront();
        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        if (SplitZeroCrossingFace(face, mesh, cameraCenter, localAllowShifts, wiggleQueue, splitQueue, badEdges, cuspEdges)){
            numZCs++;
        }
    }
#endif

    if(lastStep == INSERT_CONTOUR)
        return;

#if 1
    badEdges.clear();
    // Effectively insert cusps
    for(std::set<std::pair<MeshVertex*,MeshVertex*> >::iterator it=cuspEdges.begin(); it!=cuspEdges.end(); it++){
        fflush(stdout);
        MeshEdge * edge = (*it).first->GetEdge((*it).second) ? (*it).first->GetEdge((*it).second) : (*it).second->GetEdge((*it).first);
        if(!edge || (*it).first->GetData().facing == (*it).second->GetData().facing){
            continue;
        }
        MeshFace * face = edge->GetLeftFace() ? edge->GetLeftFace() : edge->GetRightFace();
        if(!face)
            continue;
        int oppIdx = -1;
        for(int i=0; i<3; i++){
            if(face->GetVertex(i)!=(*it).first && face->GetVertex(i)!=(*it).second){
                oppIdx=i;
                break;
            }
        }
        printf("queue size: %d+%d / %d. CUSPS: %d. ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        std::pair<MeshVertex*,MeshVertex*> badEdge;
        SplitZeroCrossingEdge(face, oppIdx, mesh, cameraCenter, false, wiggleQueue, splitQueue, badEdge, cuspEdges, true);
        numZCs++;

        if (badEdge.first == NULL){
            continue;
        }else{
            printf("FAILURE TO INSERT CUSP\n");
#if LINK_FREESTYLE
            char str[200];
            sprintf(str, "FAILURE TO INSERT CUSP");
            addRIFDebugPoint(-1, double((*it).first->GetData().pos[0]), double((*it).first->GetData().pos[1]), double((*it).first->GetData().pos[2]), str, 0);
            addRIFDebugPoint(-1, double((*it).second->GetData().pos[0]), double((*it).second->GetData().pos[1]), double((*it).second->GetData().pos[2]), str, 0);
#endif
            numCUSPs--;
        }
    }
#endif

    if(lastStep == INSERT_CUSP)
        return;

    std::set<MeshVertex*> contourPoints;

    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it){
        MeshFace * face = (*it);
        int numCpt = 0;
        for(int i=0; i<3; i++)
            if(face->GetVertex(i)->GetData().facing == CONTOUR)
                numCpt++;
#if LINK_FREESTYLE
        if(numCpt==3){
            char str[200];
            sprintf(str, "BUG.");
            addRIFDebugPoint(-1, double(face->GetVertex(0)->GetData().pos[0]), double(face->GetVertex(0)->GetData().pos[1]), double(face->GetVertex(0)->GetData().pos[2]), str, 0);
            addRIFDebugPoint(-1, double(face->GetVertex(1)->GetData().pos[0]), double(face->GetVertex(1)->GetData().pos[1]), double(face->GetVertex(1)->GetData().pos[2]), str, 0);
            addRIFDebugPoint(-1, double(face->GetVertex(2)->GetData().pos[0]), double(face->GetVertex(2)->GetData().pos[1]), double(face->GetVertex(2)->GetData().pos[2]), str, 0);
        }
#endif
    }

#if 1
    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    // Insert radial edges
    while (wiggleQueue.Size() != 0)
    {
        fflush(stdout);
        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        MeshFace * face = wiggleQueue.PopFront();

        if(InsertRadialEdges(face, mesh, cameraCenter, localAllowShifts, wiggleQueue, splitQueue, contourPoints)){
            numREs++;
            continue;
        }
    }
#endif

    if(lastStep == INSERT_RADIAL)
        return;

#if 1
    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    // Flip edges to increase the number of radial triangles along the contours
    bool allRadial = true;
    int iter = 2;
    do{
        while (wiggleQueue.Size() != 0)
        {
            MeshFace * face = wiggleQueue.PopFront();
            for(int i=0; i<3; i++)
                if(face->GetVertex(i)->GetData().facing==CONTOUR){
                    fflush(stdout);
                    printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
                           wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
                           numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);
                    bool ir = ImproveRadialConsistency(face->GetVertex(i),mesh,numFlips,wiggleQueue);
                    allRadial = ir && allRadial;
                    break;
                }
        }
        iter--;
    }while(!allRadial && iter>0);

    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);
#endif

    if(lastStep == FLIP_RADIAL)
        return;

#if 1
    // Extend radial edges to decrease the number of inconsistent triangles
    while (wiggleQueue.Size() != 0)
    {
        MeshFace * face = wiggleQueue.PopFront();

        if(IsConsistent<VertexDataCatmark>(face,cameraCenter))
            continue;

        fflush(stdout);
        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        if(ExtendRadialEdge(face,mesh,cameraCenter,wiggleQueue,splitQueue)){
            numExtend++;
        }
    }
#endif

    if(lastStep == EXTEND_RADIAL)
        return;

#if 1
    // Try to improve consistency by fliping edges of inconsistent triangles
    faces.resize(0);
    mesh->GetFaces(std::back_inserter(faces));
    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        wiggleQueue.Insert(*it);

    while (wiggleQueue.Size() != 0)
    {
        MeshFace * face = wiggleQueue.PopFront();

        if(IsConsistent<VertexDataCatmark>(face,cameraCenter))
            continue;

        fflush(stdout);
        printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
               numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

        if (FlipFace(face, mesh, cameraCenter, wiggleQueue, splitQueue))
        {
            numFlips ++;
            continue;
        }
    }
#endif

    if(lastStep == FLIP_EDGE)
        return;

    WiggleInParamSpace(mesh,cameraCenter);

    if(lastStep == WIGGLING_PARAM)
        return;

    for(int i=0; i<3; i++){

        int currSplit = numSplits;

        // Try to improve consistency by spliting edges of inconsistent triangles
        faces.resize(0);
        mesh->GetFaces(std::back_inserter(faces));
        splitQueue.Clear();
        for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
            if(!IsConsistent<VertexDataCatmark>(*it,cameraCenter))
                splitQueue.Insert(*it);
        badEdges.clear();

        while (splitQueue.Size() != 0)
        {
            MeshFace * face = splitQueue.PopFront();

            if(IsConsistent<VertexDataCatmark>(face,cameraCenter))
                continue;

            fflush(stdout);
            printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \r",
                   wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
                   numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);

            if (FindBestSplitPointFace(face, localAllowShifts, cameraCenter, mesh, badEdges, wiggleQueue, splitQueue))
            {
                numSplits ++;
                continue;
            }
        }

        if(currSplit != numSplits)
            WiggleInParamSpace(mesh,cameraCenter);
        else
            break;
    }

    if(lastStep == SPLIT_EDGE)
        return;

    WiggleAllVertices(mesh, cameraCenter);


    // Remove disconnected vertices
    std::list<MeshVertex*> vertices;
    mesh->GetVertices(std::back_inserter(vertices));
    for(std::list<MeshVertex*>::iterator vit = vertices.begin(); vit != vertices.end(); ++vit){
        if(!(*vit)->IsConnected()) {
            mesh->DeleteVertex(*vit);
        }
    }

    printf("queue size: %d+%d / %d. CUSPS: %d.  ZC: %d. radial: %d.  flips: %d. splits: %d. extend: %d   \n",
           wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),
           numCUSPs, numZCs, numREs, numFlips, numSplits, numExtend);
}

