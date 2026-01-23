// Created on: 1995-11-10
// Created by: Yves FRICAUD
// Copyright (c) 1995-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_Tool.hxx>
#include <BRep_TVertex.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <fstream>
#include <BRepTools.hxx>
#include <iomanip>
#include <BRepAlgo_FaceRestrictor.hxx>
#include <BRepAlgo_Loop.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomLib.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Ax2.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_DataMapIteratorOfDataMapOfShapeShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopTools_SequenceOfShape.hxx>

#include <stdio.h>
// #define OCCT_DEBUG_ALGO
// #define DRAW
#ifdef DRAW
  #include <DBRep.hxx>
  #pragma comment(lib, "TKDraw")
#endif
#ifdef OCCT_DEBUG_ALGO
Standard_Boolean AffichLoop = Standard_True;
Standard_Integer NbLoops    = 0;
Standard_Integer NbWires    = 1;
static char*     name       = new char[100];
#endif

//=================================================================================================

BRepAlgo_Loop::BRepAlgo_Loop()
    : myTolConf(0.001)
{
}

//=================================================================================================

void BRepAlgo_Loop::Init(const TopoDS_Face& F)
{
  myConstEdges.Clear();
  myEdges.Clear();
  myVerOnEdges.Clear();
  myNewWires.Clear();
  myNewFaces.Clear();
  myProblematicWires.Clear();
  myCutEdges.Clear();
  myFace = F;
}

//=======================================================================
// function : Bubble
// purpose  : Orders the sequence of vertices by increasing parameter.
//=======================================================================

static void Bubble(const TopoDS_Edge& E, TopTools_SequenceOfShape& Seq)
{
  // Remove duplicates
  for (Standard_Integer i = 1; i < Seq.Length(); i++)
    for (Standard_Integer j = i + 1; j <= Seq.Length(); j++)
      if (Seq(i) == Seq(j))
      {
        Seq.Remove(j);
        j--;
      }

  Standard_Boolean Invert   = Standard_True;
  Standard_Integer NbPoints = Seq.Length();
  Standard_Real    U1, U2;
  TopoDS_Vertex    V1, V2;

  while (Invert)
  {
    Invert = Standard_False;
    for (Standard_Integer i = 1; i < NbPoints; i++)
    {
      TopoDS_Shape aLocalV = Seq.Value(i).Oriented(TopAbs_INTERNAL);
      V1                   = TopoDS::Vertex(aLocalV);
      aLocalV              = Seq.Value(i + 1).Oriented(TopAbs_INTERNAL);
      V2                   = TopoDS::Vertex(aLocalV);
      //      V1 = TopoDS::Vertex(Seq.Value(i)  .Oriented(TopAbs_INTERNAL));
      //      V2 = TopoDS::Vertex(Seq.Value(i+1).Oriented(TopAbs_INTERNAL));

      U1 = BRep_Tool::Parameter(V1, E);
      U2 = BRep_Tool::Parameter(V2, E);
      if (U2 < U1)
      {
        Seq.Exchange(i, i + 1);
        Invert = Standard_True;
      }
    }
  }
}

//=================================================================================================

void BRepAlgo_Loop::AddEdge(TopoDS_Edge& E, const TopTools_ListOfShape& LV)
{
  myEdges.Append(E);
  myVerOnEdges.Bind(E, LV);
}

//=================================================================================================

void BRepAlgo_Loop::AddConstEdge(const TopoDS_Edge& E)
{
  myConstEdges.Append(E);
}

//=================================================================================================

void BRepAlgo_Loop::AddConstEdges(const TopTools_ListOfShape& LE)
{
  TopTools_ListIteratorOfListOfShape itl(LE);
  for (; itl.More(); itl.Next())
  {
    myConstEdges.Append(itl.Value());
  }
}

//=================================================================================================

void BRepAlgo_Loop::SetImageVV(const BRepAlgo_Image& theImageVV)
{
  myImageVV = theImageVV;
}

//=======================================================================
// function : UpdateClosedEdge
// purpose  : If the first or the last vertex of intersection
//           coincides with the closing vertex, it is removed from SV.
//           it will be added at the beginning and the end of SV by the caller.
//=======================================================================

static TopoDS_Vertex UpdateClosedEdge(const TopoDS_Edge& E, TopTools_SequenceOfShape& SV)
{
  TopoDS_Vertex    VB[2], V1, V2, VRes;
  gp_Pnt           P, PC;
  Standard_Boolean OnStart = 0, OnEnd = 0;
  //// modified by jgv, 13.04.04 for OCC5634 ////
  TopExp::Vertices(E, V1, V2);
  Standard_Real Tol = BRep_Tool::Tolerance(V1);
  ///////////////////////////////////////////////

  if (SV.IsEmpty())
    return VRes;

  VB[0] = TopoDS::Vertex(SV.First());
  VB[1] = TopoDS::Vertex(SV.Last());
  PC    = BRep_Tool::Pnt(V1);

  for (Standard_Integer i = 0; i < 2; i++)
  {
    P = BRep_Tool::Pnt(VB[i]);
    if (P.IsEqual(PC, Tol))
    {
      VRes = VB[i];
      if (i == 0)
        OnStart = Standard_True;
      else
        OnEnd = Standard_True;
    }
  }
  if (OnStart && OnEnd)
  {
    if (!VB[0].IsSame(VB[1]))
    {
#ifdef OCCT_DEBUG_ALGO
      if (AffichLoop)
        std::cout << "Two different vertices on the closing vertex" << std::endl;
#endif
    }
    else
    {
      SV.Remove(1);
      if (!SV.IsEmpty())
        SV.Remove(SV.Length());
    }
  }
  else if (OnStart)
    SV.Remove(1);
  else if (OnEnd)
    SV.Remove(SV.Length());

  return VRes;
}

//=================================================================================================

static void RemovePendingEdges(TopTools_IndexedDataMapOfShapeListOfShape& MVE)
{
  //--------------------------------
  // Remove hanging edges.
  //--------------------------------
  TopTools_ListOfShape               ToRemove;
  TopTools_ListIteratorOfListOfShape itl;
  Standard_Boolean                   YaSupress = Standard_True;
  TopoDS_Vertex                      V1, V2;
  // std::ofstream logFile("d:/debug_brepoffset.log", std::ios::app);

  while (YaSupress)
  {
    YaSupress = Standard_False;
    TopTools_ListOfShape VToRemove;
    TopTools_MapOfShape  EToRemove;

    for (Standard_Integer iV = 1; iV <= MVE.Extent(); iV++)
    {
      const TopoDS_Shape&         aVertex = MVE.FindKey(iV);
      const TopTools_ListOfShape& anEdges = MVE(iV);
      if (anEdges.IsEmpty())
      {
        VToRemove.Append(aVertex);
      }
      if (anEdges.Extent() == 1)
      {
        const TopoDS_Edge& E = TopoDS::Edge(anEdges.First());
        // if (E.Closed()) continue; // Assuming CheckClosed is similar logic
        // V1, V2; TopExp::Vertices(E, V1, V2);
        // if (!V1.IsSame(V2)) {
           // EToRemove.Add(E);
           // logFile << "  RemovePendingEdges: Mark edge " << (void*)&E << " for removal (one connected vertex only)" << std::endl;
        // }
      }
    }

    if (!VToRemove.IsEmpty())
    {
      YaSupress = Standard_True;
      for (itl.Initialize(VToRemove); itl.More(); itl.Next())
      {
        MVE.RemoveKey(itl.Value());
      }
      if (!EToRemove.IsEmpty())
      {
        for (Standard_Integer iV = 1; iV <= MVE.Extent(); iV++)
        {
          TopTools_ListOfShape& LE = MVE.ChangeFromIndex(iV);
          itl.Initialize(LE);
          while (itl.More())
          {
            if (EToRemove.Contains(itl.Value()))
            {
              // logFile << "  RemovePendingEdges: Removing edge " << (void*)&itl.Value() << " from vertex " << iV << std::endl;
              // itl.Value().Reverse(); // Just to do something non-destructive? No, just skip removal.
              // LE.Remove(itl); // DISABLE REMOVAL FOR DIAGNOSIS
              itl.Next();
            }
            else
              itl.Next();
          }
        }
      }
    }
    // YaSupress = Standard_False; // Force loop to stop after one pass since we disabled removal
  }
}

//=================================================================================================

static Standard_Boolean SamePnt2d(const TopoDS_Vertex& V,
                                  const TopoDS_Edge&   E1,
                                  const TopoDS_Edge&   E2,
                                  const TopoDS_Face&   F)
{
  Standard_Real f1, f2, l1, l2;
  gp_Pnt2d      P1, P2;
  TopoDS_Face   FF = F;
  FF.Orientation(TopAbs_FORWARD);
  
  Handle(Geom2d_Curve) C1 = BRep_Tool::CurveOnSurface(E1, FF, f1, l1);
  Handle(Geom2d_Curve) C2 = BRep_Tool::CurveOnSurface(E2, FF, f2, l2);
  if (C1.IsNull() || C2.IsNull()) return Standard_False;
  
  P1 = (E1.Orientation() == TopAbs_FORWARD) ? C1->Value(f1) : C1->Value(l1);
  P2 = (E2.Orientation() == TopAbs_FORWARD) ? C2->Value(l2) : C2->Value(f2);
  
  Standard_Real Tol  = 100 * BRep_Tool::Tolerance(V);
  if (Tol < Precision::Confusion()) Tol = Precision::Confusion();
  Standard_Real Dist = P1.Distance(P2);
  
  if (Dist < Tol) return Standard_True;

  TopLoc_Location Loc;
  Handle(Geom_Surface) S = BRep_Tool::Surface(F, Loc);
  if (S.IsNull()) return Standard_False;
  
  if (S->IsUPeriodic()) {
      Standard_Real UP = S->UPeriod();
      if (std::abs(P1.X() - P2.X() - UP) < Tol || std::abs(P1.X() - P2.X() + UP) < Tol)
          if (std::abs(P1.Y() - P2.Y()) < Tol) return Standard_True;
  }
  if (S->IsVPeriodic()) {
      Standard_Real VP = S->VPeriod();
      if (std::abs(P1.Y() - P2.Y() - VP) < Tol || std::abs(P1.Y() - P2.Y() + VP) < Tol)
          if (std::abs(P1.X() - P2.X()) < Tol) return Standard_True;
  }
  
  return Standard_False;
}

//=======================================================================
// function : SelectEdge
// purpose  : Find edge <NE> connected to <CE> by vertex <CV> in the
//           list <LE>. <NE> is removed from the list. If <CE> is
//           also in the list <LE> with the same orientation, it is
//           removed from the list.
//=======================================================================

static Standard_Boolean SelectEdge(const TopoDS_Face&    F,
                                   const TopoDS_Edge&    CE,
                                   const TopoDS_Vertex&  CV,
                                   TopoDS_Edge&          NE,
                                   TopTools_ListOfShape& LE,
                                   const Standard_Real   theTolConf)
{
  Standard_Boolean isCandidate = Standard_False;
  Standard_Boolean use2D       = Standard_False;
  Standard_Real    dist2Min    = Precision::Infinite();
  Standard_Real    maxScore    = -Precision::Infinite();

  TopTools_ListIteratorOfListOfShape itl;
  std::ofstream logFile("d:/debug_brepoffset.log", std::ios::app);
  logFile << "--- SelectEdge at vertex (" << BRep_Tool::Pnt(CV).X() << "," << BRep_Tool::Pnt(CV).Y() << "," << BRep_Tool::Pnt(CV).Z() << ") LE count: " << LE.Extent() << std::endl;
  for (itl.Initialize(LE); itl.More(); itl.Next()) {
      const TopoDS_Edge& E = TopoDS::Edge(itl.Value());
      logFile << "  Pool edge: " << (void*)&E;
      TopoDS_Vertex V1, V2; TopExp::Vertices(E, V1, V2);
      if (!V1.IsNull()) logFile << " V1=(" << BRep_Tool::Pnt(V1).X() << "," << BRep_Tool::Pnt(V1).Y() << "," << BRep_Tool::Pnt(V1).Z() << ")";
      if (!V2.IsNull()) logFile << " V2=(" << BRep_Tool::Pnt(V2).X() << "," << BRep_Tool::Pnt(V2).Y() << "," << BRep_Tool::Pnt(V2).Z() << ")";
      logFile << std::endl;
  }

  NE.Nullify();

  // 1. Remove exact same edge (with same orientation)
  for (itl.Initialize(LE); itl.More(); itl.Next())
  {
    if (itl.Value().IsEqual(CE))
    {
      LE.Remove(itl);
      break;
    }
  }

  if (LE.IsEmpty()) return Standard_False;

  // 2. Prepare UV info for current edge end
  TopLoc_Location Loc;
  Handle(Geom_Surface) S = BRep_Tool::Surface(F, Loc);
  Standard_Real UP = (S->IsUPeriodic()) ? S->UPeriod() : 0.0;
  Standard_Real VP = (S->IsVPeriodic()) ? S->VPeriod() : 0.0;

  Standard_Real f, l, uCE;
  TopoDS_Face FF = F; FF.Orientation(TopAbs_FORWARD);
  
  Handle(Geom2d_Curve) CCE = BRep_Tool::CurveOnSurface(CE, FF, f, l);
  if (CCE.IsNull()) return Standard_False;
  
  uCE = (CE.Orientation() == TopAbs_FORWARD) ? l : f;
  gp_Pnt2d PCE = CCE->Value(uCE);
  gp_Vec2d VCE; gp_Pnt2d Pdummy;
  CCE->D1(uCE, Pdummy, VCE);
  if (CE.Orientation() == TopAbs_REVERSED) VCE.Reverse();

  // 3. Search for best candidate
  Standard_Real bestScore = -100.0; // Lower baseline
  TopTools_ListIteratorOfListOfShape itBest;
  Standard_Real searchTol = std::max(theTolConf, 100 * BRep_Tool::Tolerance(CV));
  
  // Prepare 3D info for CE if needed
  gp_Pnt P3D_CE; gp_Vec V3D_CE;
  BRepAdaptor_Curve BC_CE(CE);
  Standard_Real pCE = (CE.Orientation() == TopAbs_FORWARD) ? BC_CE.LastParameter() : BC_CE.FirstParameter();
  BC_CE.D1(pCE, P3D_CE, V3D_CE);
  if (CE.Orientation() == TopAbs_REVERSED) V3D_CE.Reverse();

  for (itl.Initialize(LE); itl.More(); itl.Next())
  {
    const TopoDS_Edge& CE2 = TopoDS::Edge(itl.Value());
    if (CE2.IsSame(CE))
      continue;

    Standard_Boolean foundV = Standard_False;
    TopExp_Explorer  Exp;
    for (Exp.Init(CE2.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); Exp.More(); Exp.Next())
    {
      const TopoDS_Vertex& V = TopoDS::Vertex(Exp.Current());
      if (V.IsSame(CV))
      {
        foundV = Standard_True;
        break;
      }
    }

    logFile << "  Checking edge: " << (void*)&CE2;
    if (foundV) {
        logFile << " [TOPOLOGICALLY CONNECTED]" << std::endl;
    } else {
        // Check 3D distance
        Standard_Real minDV = Precision::Infinite();
        for (Exp.Init(CE2, TopAbs_VERTEX); Exp.More(); Exp.Next()) {
            minDV = std::min(minDV, BRep_Tool::Pnt(CV).Distance(BRep_Tool::Pnt(TopoDS::Vertex(Exp.Current()))));
        }
        logFile << " [3D Dist: " << minDV << "]" << std::endl;
    }
    
    // Avoid picking the reverse of the same edge if possible
    if (CE2.IsSame(CE) && CE2.Orientation() != CE.Orientation()) continue;

    isCandidate = Standard_False;
    use2D = Standard_False;
    
    // Check 2D
    Handle(Geom2d_Curve) CE2_2d = BRep_Tool::CurveOnSurface(CE2, FF, f, l);
    Standard_Real uE = 0.0;
    gp_Pnt2d PE;
    
    if (!CE2_2d.IsNull()) {
        uE = (CE2.Orientation() == TopAbs_FORWARD) ? f : l;
        PE = CE2_2d->Value(uE);
        
        Standard_Real dist = PCE.Distance(PE);
        Standard_Boolean isP = (dist < searchTol);
        if (!isP && UP > 0) isP = (std::abs(PCE.X()-PE.X()-UP)<searchTol || std::abs(PCE.X()-PE.X()+UP)<searchTol) && std::abs(PCE.Y()-PE.Y())<searchTol;
        if (!isP && VP > 0) isP = (std::abs(PCE.Y()-PE.Y()-VP)<searchTol || std::abs(PCE.Y()-PE.Y()+VP)<searchTol) && std::abs(PCE.X()-PE.X())<searchTol;
        
        if (isP) {
            isCandidate = Standard_True;
            use2D = Standard_True;
        }
    }
    
    // Check 3D (Fallback if 2D failed)
    if (!isCandidate) {
        BRepAdaptor_Curve BC_E(CE2);
        Standard_Real pE = (CE2.Orientation() == TopAbs_FORWARD) ? BC_E.FirstParameter() : BC_E.LastParameter();
        gp_Pnt P3D_E = BC_E.Value(pE);
        if (P3D_CE.Distance(P3D_E) < searchTol) {
            isCandidate = Standard_True;
            // use2D remains False
        }
    }

    if (isCandidate)
    {
      Standard_Real score = -2.0;
      
      if (use2D) {
          gp_Vec2d VE; gp_Pnt2d PdummyLocal;
          CE2_2d->D1(uE, PdummyLocal, VE);
          if (CE2.Orientation() == TopAbs_REVERSED) VE.Reverse();
          
          if (VCE.SquareMagnitude() > 1e-20 && VE.SquareMagnitude() > 1e-20) {
              Standard_Real dot = VCE.Normalized().Dot(VE.Normalized());
              Standard_Real cross = VCE.Normalized().Crossed(VE.Normalized());
              score = dot + cross * 0.01; 
          } else score = 1.0;
      } else {
          // 3D Score
          BRepAdaptor_Curve BC_E(CE2);
          Standard_Real pE = (CE2.Orientation() == TopAbs_FORWARD) ? BC_E.FirstParameter() : BC_E.LastParameter();
          gp_Pnt P3D_E; gp_Vec V3D_E;
          BC_E.D1(pE, P3D_E, V3D_E);
          
          if (CE2.Orientation() == TopAbs_REVERSED) V3D_E.Reverse();
          
          if (V3D_CE.SquareMagnitude() > 1e-20 && V3D_E.SquareMagnitude() > 1e-20) {
              gp_Vec dirCE = V3D_CE.Normalized();
              gp_Vec dirE = V3D_E.Normalized();
              Standard_Real dot = dirCE.Dot(dirE);
              
              // Calculate "Left" turn bias using Surface Normal at Junction
              // Note: We use CE's endpoint (uCE) to estimate normal at the vertex
              gp_Vec D1U, D1V; gp_Pnt Psurf;
              S->D1(PCE.X(), PCE.Y(), Psurf, D1U, D1V);
              gp_Vec Normal = D1U.Crossed(D1V);
              if (Normal.SquareMagnitude() > 1e-20) {
                 Normal.Normalize();
                 if (F.Orientation() == TopAbs_REVERSED) Normal.Reverse();
                 
                 gp_Vec CrossVec = dirCE.Crossed(dirE);
                 Standard_Real cross = CrossVec.Dot(Normal);
                 score = dot + cross * 0.01;
              } else {
                 score = dot;
              }
          } else score = 1.0;
      }

      if (score > bestScore)
      {
        bestScore = score;
        itBest    = itl;
      }
    }
  }

  if (bestScore > -100.0)
  {
    NE = TopoDS::Edge(itBest.Value());
    LE.Remove(itBest);
    return Standard_True;
  }
  
  return Standard_False;
}

//=================================================================================================

static void PurgeNewEdges(TopTools_DataMapOfShapeListOfShape& NewEdges,
                          const TopTools_MapOfShape&          UsedEdges)
{
  TopTools_DataMapIteratorOfDataMapOfShapeListOfShape it(NewEdges);
  for (; it.More(); it.Next())
  {
    TopTools_ListOfShape&              LNE = NewEdges.ChangeFind(it.Key());
    TopTools_ListIteratorOfListOfShape itL(LNE);
    while (itL.More())
    {
      const TopoDS_Shape& NE = itL.Value();
      if (!UsedEdges.Contains(NE))
      {
        LNE.Remove(itL);
      }
      else
      {
        itL.Next();
      }
    }
  }
}

//=================================================================================================

static void StoreInMVE(const TopoDS_Face&                         F,
                       TopoDS_Edge&                               E,
                       TopTools_IndexedDataMapOfShapeListOfShape& MVE,
                       Standard_Boolean&                          YaCouture,
                       TopTools_DataMapOfShapeShape&              VerticesForSubstitute,
                       const Standard_Real                        theTolConf)
{
  TopoDS_Vertex        V1, V2, V;
  TopTools_ListOfShape Empty;

  gp_Pnt       P1, P;
  BRep_Builder BB;
  for (Standard_Integer iV = 1; iV <= MVE.Extent(); iV++)
  {
    V = TopoDS::Vertex(MVE.FindKey(iV));
    P = BRep_Tool::Pnt(V);
    TopTools_ListOfShape VList;
    TopoDS_Iterator      VerExp(E);
    for (; VerExp.More(); VerExp.Next())
      VList.Append(VerExp.Value());
    TopTools_ListIteratorOfListOfShape itl(VList);
    for (; itl.More(); itl.Next())
    {
      V1 = TopoDS::Vertex(itl.Value());
      P1 = BRep_Tool::Pnt(V1);
      if (P.IsEqual(P1, theTolConf) && !V.IsSame(V1))
      {
        V.Orientation(V1.Orientation());
        Standard_Real U = BRep_Tool::Parameter(V1, E);

        if (VerticesForSubstitute.IsBound(V1))
        {
          TopoDS_Shape OldNewV = VerticesForSubstitute(V1);
          if (!OldNewV.IsSame(V))
          {
            VerticesForSubstitute.Bind(OldNewV, V);
            VerticesForSubstitute(V1) = V;
          }
        }
        else
        {
          if (VerticesForSubstitute.IsBound(V))
          {
            TopoDS_Shape NewNewV = VerticesForSubstitute(V);
            if (!NewNewV.IsSame(V1))
              VerticesForSubstitute.Bind(V1, NewNewV);
          }
          else
          {
            VerticesForSubstitute.Bind(V1, V);
            TopTools_DataMapIteratorOfDataMapOfShapeShape mapit(VerticesForSubstitute);
            for (; mapit.More(); mapit.Next())
              if (mapit.Value().IsSame(V1))
                VerticesForSubstitute(mapit.Key()) = V;
          }
        }
        E.Free(Standard_True);
        BB.Remove(E, V1);
        BB.Add(E, V);
        BB.UpdateVertex(V, U, E, theTolConf);
      }
    }
  }

  TopExp::Vertices(E, V1, V2);
  if (V1.IsNull() && V2.IsNull())
  {
    YaCouture = Standard_False;
    return;
  }
  if (!MVE.Contains(V1))
  {
    MVE.Add(V1, Empty);
  }
  MVE.ChangeFromKey(V1).Append(E);
  if (!V1.IsSame(V2))
  {
    if (!MVE.Contains(V2))
    {
      MVE.Add(V2, Empty);
    }
    MVE.ChangeFromKey(V2).Append(E);
  }
  TopLoc_Location      L;
  Handle(Geom_Surface) S = BRep_Tool::Surface(F, L);
  if (BRep_Tool::IsClosed(E, S, L))
  {
    MVE.ChangeFromKey(V2).Append(E.Reversed());
    if (!V1.IsSame(V2))
    {
      MVE.ChangeFromKey(V1).Append(E.Reversed());
    }
    YaCouture = Standard_True;
  }
}

//=================================================================================================

void BRepAlgo_Loop::Perform()
{
  PerformNew();
}

//=================================================================================================

#include <Toolkit_VertexClusterer.hxx>
#include <Precision.hxx>
#include <GeomLib.hxx>
#include <TColgp_Array1OfPnt.hxx>

void BRepAlgo_Loop::PerformNew()
{
  myNewWires.Clear();
  myNewFaces.Clear();
  myProblematicWires.Clear();

  // 1. Ensure all edges are cut
  TopTools_ListIteratorOfListOfShape it;
  for (it.Initialize(myEdges); it.More(); it.Next())
  {
    const TopoDS_Edge& anEdge = TopoDS::Edge(it.Value());
    if (!myCutEdges.IsBound(anEdge))
    {
      const TopTools_ListOfShape* pVertices = myVerOnEdges.Seek(anEdge);
      TopTools_ListOfShape aLCE;
      if (pVertices && !pVertices->IsEmpty()) CutEdge(anEdge, *pVertices, aLCE);
      else aLCE.Append(anEdge);
      myCutEdges.Bind(anEdge, aLCE);
    }
  }

  // 2. Cluster vertices (Spatial)
  // Seed with Constant Edges first to preserve their vertices
  Toolkit_VertexClusterer aClusterer(myTolConf);
  aClusterer.AddEdges(myConstEdges);

  TopTools_ListOfShape aCutEdgesList;
  for (it.Initialize(myEdges); it.More(); it.Next())
  {
    const TopTools_ListOfShape& aL = myCutEdges.Find(it.Value());
    for (TopTools_ListIteratorOfListOfShape itL(aL); itL.More(); itL.Next()) aCutEdgesList.Append(itL.Value());
  }
  aClusterer.AddEdges(aCutEdgesList);

  TopTools_ListOfShape allEdges;
  allEdges.Append(myConstEdges);
  allEdges.Append(aCutEdgesList);
  if (allEdges.IsEmpty()) return;

  // 2.1 Calculate new geometry for spatial clusters
  BRep_Builder BB;
  const TopTools_DataMapOfShapeListOfShape& aClusters = aClusterer.Clusters();
  for (TopTools_DataMapIteratorOfDataMapOfShapeListOfShape itC(aClusters); itC.More(); itC.Next())
  {
    const TopTools_ListOfShape& aVlist = itC.Value();
    if (aVlist.Extent() == 1) continue;

    Standard_Real aMaxTol = 0.;
    TColgp_Array1OfPnt Points(1, aVlist.Extent());
    TopTools_ListIteratorOfListOfShape itV(aVlist);
    Standard_Integer jj = 0;
    for (; itV.More(); itV.Next())
    {
      const TopoDS_Vertex& aV = TopoDS::Vertex(itV.Value());
      aMaxTol = std::max(aMaxTol, BRep_Tool::Tolerance(aV));
      Points(++jj) = BRep_Tool::Pnt(aV);
    }

    gp_Ax2 anAxis; Standard_Boolean IsSingular;
    GeomLib::AxeOfInertia(Points, anAxis, IsSingular);
    gp_Pnt aCentre = anAxis.Location();
    Standard_Real aMaxDist = 0.;
    for (jj = 1; jj <= Points.Upper(); jj++)
      aMaxDist = std::max(aMaxDist, std::sqrt(aCentre.SquareDistance(Points(jj))));
    aMaxTol = std::max(aMaxTol, aMaxDist);
    BB.UpdateVertex(TopoDS::Vertex(itC.Key()), aCentre, aMaxTol);
  }

  // 3. Build connectivity map (MVE) with master replacement
  TopTools_IndexedDataMapOfShapeListOfShape MVE;
  TopTools_MapOfShape aDejaVu;
  TopLoc_Location Loc; Handle(Geom_Surface) Surf = BRep_Tool::Surface(myFace, Loc);

  for (it.Initialize(allEdges); it.More(); it.Next())
  {
    TopoDS_Edge E = TopoDS::Edge(it.Value());
    if (!aDejaVu.Add(E)) continue;

    TopTools_ListOfShape vHandles;
    for (TopoDS_Iterator itV(E); itV.More(); itV.Next()) vHandles.Append(itV.Value());

    for (TopTools_ListIteratorOfListOfShape itVH(vHandles); itVH.More(); itVH.Next())
    {
      TopoDS_Vertex VOrig = TopoDS::Vertex(itVH.Value());
      TopoDS_Vertex VMaster = aClusterer.GetMaster(VOrig);
      if (!VOrig.IsSame(VMaster))
      {
        if (myVerticesForSubstitute.IsBound(VOrig)) {
            TopoDS_Shape OldNewV = myVerticesForSubstitute(VOrig);
            if (!OldNewV.IsSame(VMaster)) { myVerticesForSubstitute.Bind(OldNewV, VMaster); myVerticesForSubstitute(VOrig) = VMaster; }
        } else {
            if (myVerticesForSubstitute.IsBound(VMaster)) {
                TopoDS_Shape NewNewV = myVerticesForSubstitute(VMaster);
                if (!NewNewV.IsSame(VOrig)) myVerticesForSubstitute.Bind(VOrig, NewNewV);
            } else {
                myVerticesForSubstitute.Bind(VOrig, VMaster);
                for (TopTools_DataMapIteratorOfDataMapOfShapeShape mapit(myVerticesForSubstitute); mapit.More(); mapit.Next())
                    if (mapit.Value().IsSame(VOrig)) mapit.ChangeValue() = VMaster;
            }
        }
        Standard_Real U = BRep_Tool::Parameter(VOrig, E);
        E.Free(Standard_True); BB.Remove(E, VOrig); BB.Add(E, VMaster.Oriented(VOrig.Orientation()));
        BB.UpdateVertex(VMaster, U, E, myTolConf);
      }
    }

    TopoDS_Vertex V1, V2; TopExp::Vertices(E, V1, V2);
    if (!V1.IsNull()) { if (!MVE.Contains(V1)) MVE.Add(V1, TopTools_ListOfShape()); MVE.ChangeFromKey(V1).Append(E); }
    if (!V2.IsNull() && !V2.IsSame(V1)) { if (!MVE.Contains(V2)) MVE.Add(V2, TopTools_ListOfShape()); MVE.ChangeFromKey(V2).Append(E); }
    
    if (!Surf.IsNull() && BRep_Tool::IsClosed(E, Surf, Loc))
    {
      if (!V2.IsNull()) MVE.ChangeFromKey(V2).Append(E.Reversed());
      if (!V1.IsNull() && !V1.IsSame(V2)) MVE.ChangeFromKey(V1).Append(E.Reversed());
    }
  }

  // 4. Identity-based merging (Topological / myImageVV)
  UpdateVEmap(MVE);
  
  // 4.1 Remove hanging edges ONCE before traversal
  RemovePendingEdges(MVE);

  // 5. Traversal Logic with orientation prioritization
  TopTools_MapOfShape UsedEdges;
  while (MVE.Extent() > 0)
  {
    if (MVE.Extent() == 0) break;
    
    TopoDS_Wire NW; BB.MakeWire(NW);
        TopoDS_Edge EF, CE, NE;
        
        // Find best starting vertex and edge (Prefer Forward)
        Standard_Integer iStart = 1;
        Standard_Boolean foundStart = Standard_False;
        
        for (Standard_Integer iMap = 1; iMap <= MVE.Extent(); iMap++) {
            const TopoDS_Vertex& V = TopoDS::Vertex(MVE.FindKey(iMap));
            const TopTools_ListOfShape& aL = MVE(iMap);
            if (aL.IsEmpty()) continue;
            
            for (TopTools_ListIteratorOfListOfShape itL(aL); itL.More(); itL.Next()) {
                const TopoDS_Edge& E = TopoDS::Edge(itL.Value());
                TopoDS_Vertex V1, V2; TopExp::Vertices(E, V1, V2);
                if ((E.Orientation() == TopAbs_FORWARD && V.IsSame(V1)) ||
                    (E.Orientation() == TopAbs_REVERSED && V.IsSame(V2))) {
                    CE = E; iStart = iMap; foundStart = Standard_True; break;
                }
            }
            if (foundStart) break;
        }
        if (!foundStart) {
            // Fallback: pick any edge from the first non-empty list
             for (Standard_Integer iMap = 1; iMap <= MVE.Extent(); iMap++) {
                if (!MVE(iMap).IsEmpty()) {
                    CE = TopoDS::Edge(MVE(iMap).First());
                    iStart = iMap;
                    foundStart = Standard_True;
                    break;
                }
             }

             if (!foundStart) {
                 // Should not happen if MVE.Extent() > 0, but check to be safe
                 // Check if map contains only empty lists (which shouldn't technically happen if RemovePendingEdges works right, but safety first)
                 MVE.Clear();
                 break;
             }
        }

        EF = CE;
        TopoDS_Vertex VF = TopoDS::Vertex(MVE.FindKey(iStart));
        TopoDS_Vertex CV = VF;

        TopTools_ListOfShape& aStartList = MVE.ChangeFromIndex(iStart);
        for (TopTools_ListIteratorOfListOfShape itL(aStartList); itL.More(); ) {
          if (itL.Value().IsEqual(CE)) { aStartList.Remove(itL); break; }
          else itL.Next();
        }
        if (aStartList.IsEmpty()) MVE.RemoveKey(VF);

        Standard_Boolean End = Standard_False;
        while (!End) {
          TopoDS_Vertex V1, V2; TopExp::Vertices(CE, V1, V2); 
          CV = CV.IsSame(V1) ? V2 : V1;
          BB.Add(NW, CE); UsedEdges.Add(CE);
          if (!MVE.Contains(CV) || MVE.FindFromKey(CV).IsEmpty()) End = Standard_True;
          else {
            End = !SelectEdge(myFace, CE, CV, NE, MVE.ChangeFromKey(CV), myTolConf);
            if (!End) { CE = NE; if (MVE.FindFromKey(CV).IsEmpty()) MVE.RemoveKey(CV); }
          }
        }
      
      // Finalizing the wire
      if (VF.IsSame(CV)) {
        if (SamePnt2d(VF, EF, CE, myFace)) { NW.Closed(Standard_True); myNewWires.Append(NW); }
        else if (BRep_Tool::Tolerance(VF) < myTolConf) {
          BB.UpdateVertex(VF, myTolConf);
          if (SamePnt2d(VF, EF, CE, myFace)) { NW.Closed(Standard_True); myNewWires.Append(NW); }
          else myProblematicWires.Append(NW);
        } else myProblematicWires.Append(NW);
      } else myProblematicWires.Append(NW);
    }
  PurgeNewEdges(myCutEdges, UsedEdges);
}

//=================================================================================================

//=================================================================================================

const TopTools_ListOfShape& BRepAlgo_Loop::ProblematicWires() const
{
  return myProblematicWires;
}

//=================================================================================================

void BRepAlgo_Loop::CutEdge(const TopoDS_Edge&          E,
                            const TopTools_ListOfShape& VOnE,
                            TopTools_ListOfShape&       NE) const
{
  TopoDS_Shape aLocalE = E.Oriented(TopAbs_FORWARD);
  TopoDS_Edge  WE      = TopoDS::Edge(aLocalE);

  Standard_Real                      U1, U2;
  TopoDS_Vertex                      V1, V2;
  TopTools_SequenceOfShape           SV;
  TopTools_ListIteratorOfListOfShape it(VOnE);
  BRep_Builder                       B;

  for (; it.More(); it.Next())
  {
    SV.Append(it.Value());
  }
  //--------------------------------
  // Parse vertices on the edge.
  //--------------------------------
  Bubble(WE, SV);

  Standard_Integer NbVer = SV.Length();
  //----------------------------------------------------------------
  // Construction of new edges.
  // Note :  vertices at the extremities of edges are not
  //         onligatorily in the list of vertices
  //----------------------------------------------------------------
  if (SV.IsEmpty())
  {
    NE.Append(E);
    return;
  }
  TopoDS_Vertex VF, VL;
  Standard_Real f, l;
  BRep_Tool::Range(WE, f, l);
  TopExp::Vertices(WE, VF, VL);

  if (NbVer == 2)
  {
    if (SV(1).IsEqual(VF) && SV(2).IsEqual(VL))
    {
      NE.Append(E);
#ifdef DRAW
      if (AffichLoop)
      {
        DBRep::Set("ECOpied", E);
      }
#endif
      return;
    }
  }
  //----------------------------------------------------
  // Processing of closed edges
  // If a vertex of intersection is on the common vertex
  // it should appear at the beginning and end of SV.
  //----------------------------------------------------
  TopoDS_Vertex VCEI;
  if (!VF.IsNull() && VF.IsSame(VL))
  {
    VCEI = UpdateClosedEdge(WE, SV);
    if (!VCEI.IsNull())
    {
      TopoDS_Shape aLocalV = VCEI.Oriented(TopAbs_FORWARD);
      VF                   = TopoDS::Vertex(aLocalV);
      aLocalV              = VCEI.Oriented(TopAbs_REVERSED);
      VL                   = TopoDS::Vertex(aLocalV);
    }
    SV.Prepend(VF);
    SV.Append(VL);
  }
  else
  {
    //-----------------------------------------
    // Eventually all extremities of the edge.
    //-----------------------------------------
    if (!VF.IsNull() && !VF.IsSame(SV.First()))
      SV.Prepend(VF);
    if (!VL.IsNull() && !VL.IsSame(SV.Last()))
      SV.Append(VL);
  }

  while (!SV.IsEmpty())
  {
    while (!SV.IsEmpty() && SV.First().Orientation() != TopAbs_FORWARD)
    {
      SV.Remove(1);
    }
    if (SV.IsEmpty())
      break;
    V1 = TopoDS::Vertex(SV.First());
    SV.Remove(1);
    if (SV.IsEmpty())
      break;
    if (SV.First().Orientation() == TopAbs_REVERSED)
    {
      V2 = TopoDS::Vertex(SV.First());
      SV.Remove(1);
      //-------------------------------------------
      // Copy the edge and restriction by V1 V2.
      //-------------------------------------------
      TopoDS_Shape NewEdge    = WE.EmptyCopied();
      TopoDS_Shape aLocalEdge = V1.Oriented(TopAbs_FORWARD);
      B.Add(NewEdge, aLocalEdge);
      aLocalEdge = V2.Oriented(TopAbs_REVERSED);
      B.Add(TopoDS::Edge(NewEdge), aLocalEdge);
      if (V1.IsSame(VF))
        U1 = f;
      else
      {
        TopoDS_Shape aLocalV = V1.Oriented(TopAbs_INTERNAL);
        U1                   = BRep_Tool::Parameter(TopoDS::Vertex(aLocalV), WE);
      }
      if (V2.IsSame(VL))
        U2 = l;
      else
      {
        TopoDS_Shape aLocalV = V2.Oriented(TopAbs_INTERNAL);
        U2                   = BRep_Tool::Parameter(TopoDS::Vertex(aLocalV), WE);
      }
      B.Range(TopoDS::Edge(NewEdge), U1, U2);
#ifdef DRAW
      if (AffichLoop)
      {
        DBRep::Set("Cut", NewEdge);
      }
#endif
      NE.Append(NewEdge.Oriented(E.Orientation()));
    }
  }

  // Remove edges with size <= tolerance
  Standard_Real Tol = 0.001; // 5.e-05; //5.e-07;
  it.Initialize(NE);
  while (it.More())
  {
    // skl : I change "E" to "EE"
    TopoDS_Edge   EE = TopoDS::Edge(it.Value());
    Standard_Real fpar, lpar;
    BRep_Tool::Range(EE, fpar, lpar);
    if (lpar - fpar <= Precision::Confusion())
      NE.Remove(it);
    else
    {
      gp_Pnt2d pf, pl;
      BRep_Tool::UVPoints(EE, myFace, pf, pl);
      if (pf.Distance(pl) <= Tol && !BRep_Tool::IsClosed(EE))
        NE.Remove(it);
      else
        it.Next();
    }
  }
}

//=================================================================================================

const TopTools_ListOfShape& BRepAlgo_Loop::NewWires() const
{
  return myNewWires;
}

//=================================================================================================

const TopTools_ListOfShape& BRepAlgo_Loop::NewFaces() const
{
  return myNewFaces;
}

//=================================================================================================

void BRepAlgo_Loop::WiresToFaces()
{
  if (!myNewWires.IsEmpty())
  {
    BRepAlgo_FaceRestrictor FR;
    TopoDS_Shape            aLocalS = myFace.Oriented(TopAbs_FORWARD);
    FR.Init(TopoDS::Face(aLocalS), Standard_False);
    //    FR.Init (TopoDS::Face(myFace.Oriented(TopAbs_FORWARD)),
    //	     Standard_False);
    TopTools_ListIteratorOfListOfShape it(myNewWires);
    for (; it.More(); it.Next())
    {
      FR.Add(TopoDS::Wire(it.ChangeValue()));
    }

    FR.Perform();

    if (FR.IsDone())
    {
      TopAbs_Orientation OriF = myFace.Orientation();
      for (; FR.More(); FR.Next())
      {
        myNewFaces.Append(FR.Current().Oriented(OriF));
      }
    }
  }
}

//=================================================================================================

const TopTools_ListOfShape& BRepAlgo_Loop::NewEdges(const TopoDS_Edge& E) const
{
  return myCutEdges(E);
}

//=================================================================================================

void BRepAlgo_Loop::GetVerticesForSubstitute(TopTools_DataMapOfShapeShape& VerVerMap) const
{
  VerVerMap = myVerticesForSubstitute;
}

//=================================================================================================

void BRepAlgo_Loop::VerticesForSubstitute(TopTools_DataMapOfShapeShape& VerVerMap)
{
  myVerticesForSubstitute = VerVerMap;
}

//=================================================================================================

void BRepAlgo_Loop::UpdateVEmap(TopTools_IndexedDataMapOfShapeListOfShape& theVEmap)
{
  TopTools_IndexedDataMapOfShapeListOfShape VerLver;

  for (Standard_Integer ii = 1; ii <= theVEmap.Extent(); ii++)
  {
    const TopoDS_Vertex&        aVertex = TopoDS::Vertex(theVEmap.FindKey(ii));
    if (myImageVV.IsImage(aVertex))
    {
      const TopoDS_Vertex& aProVertex = TopoDS::Vertex(myImageVV.ImageFrom(aVertex));
      if (VerLver.Contains(aProVertex))
      {
        TopTools_ListOfShape& aVlist = VerLver.ChangeFromKey(aProVertex);
        aVlist.Append(aVertex.Oriented(TopAbs_FORWARD));
      }
      else
      {
        TopTools_ListOfShape aVlist;
        aVlist.Append(aVertex.Oriented(TopAbs_FORWARD));
        VerLver.Add(aProVertex, aVlist);
      }
    }
  }

  // Perform geometric clustering for vertices not already clustered via myImageVV
  TopTools_IndexedMapOfShape allVertices;
  for (Standard_Integer ii = 1; ii <= theVEmap.Extent(); ii++)
  {
    allVertices.Add(theVEmap.FindKey(ii));
  }

  std::ofstream logFile("d:/debug_brepoffset.log", std::ios::app);
  logFile << "--- UpdateVEmap: Starting vertex analysis (" << allVertices.Extent() << " vertices)" << std::endl;
  logFile << std::fixed << std::setprecision(10);
  for (Standard_Integer i = 1; i <= allVertices.Extent(); i++) {
     gp_Pnt P = BRep_Tool::Pnt(TopoDS::Vertex(allVertices(i)));
     logFile << "  v" << i << "=(" << P.X() << "," << P.Y() << "," << P.Z() << ")" << std::endl;
  }
  
  for (Standard_Integer i = 1; i <= allVertices.Extent(); i++) {
     for (Standard_Integer j = i + 1; j <= allVertices.Extent(); j++) {
        Standard_Real d = BRep_Tool::Pnt(TopoDS::Vertex(allVertices(i))).Distance(BRep_Tool::Pnt(TopoDS::Vertex(allVertices(j))));
        if (d < 1.0) {
            logFile << "  Dist v" << i << " to v" << j << ": " << d << std::endl;
        }
     }
  }

  // 1. Initialize representative map with all vertices pointing to themselves
  TopTools_DataMapOfShapeShape repMap;
  for (Standard_Integer i = 1; i <= allVertices.Extent(); i++)
      repMap.Bind(allVertices(i), allVertices(i));

  // 2. Initial merging based on existing image-based clusters (VerLver)
  for (Standard_Integer ii = 1; ii <= VerLver.Extent(); ii++) {
      const TopoDS_Shape& r = VerLver.FindKey(ii);
      const TopTools_ListOfShape& list = VerLver.FindFromIndex(ii);
      for (TopTools_ListIteratorOfListOfShape it(list); it.More(); it.Next())
          repMap.Bind(it.Value(), r);
  }

  // 3. Geometric merging (0.05 tolerance) for ALL pairs
  for (Standard_Integer i = 1; i <= allVertices.Extent(); i++) {
      const TopoDS_Shape& v1 = allVertices(i);
      gp_Pnt P1 = BRep_Tool::Pnt(TopoDS::Vertex(v1));
      for (Standard_Integer j = i + 1; j <= allVertices.Extent(); j++) {
          const TopoDS_Shape& v2 = allVertices(j);
          gp_Pnt P2 = BRep_Tool::Pnt(TopoDS::Vertex(v2));
          if (P1.Distance(P2) < 0.05) {
              const TopoDS_Shape& r1 = repMap(v1);
              const TopoDS_Shape& r2 = repMap(v2);
              if (!r1.IsSame(r2)) {
                   // Union-Find: Merge cluster r2 into cluster r1
                   for (Standard_Integer k = 1; k <= allVertices.Extent(); k++) {
                       if (repMap(allVertices(k)).IsSame(r2))
                           repMap(allVertices(k)) = r1;
                   }
                   logFile << "--- UpdateVEmap: GEOMETRICALLY MERGED cluster of v" << i << " and v" << j << " (dist: " << P1.Distance(P2) << ")" << std::endl;
              }
          }
      }
  }

  // 4. Rebuild VerLver based on final repMap
  VerLver.Clear();
  for (Standard_Integer ii = 1; ii <= allVertices.Extent(); ii++) {
      const TopoDS_Shape& v = allVertices(ii);
      const TopoDS_Shape& r = repMap(v);
      if (!VerLver.Contains(r)) {
          TopTools_ListOfShape L; VerLver.Add(r, L);
      }
      VerLver.ChangeFromKey(r).Append(v);
  }

  // 5. Build substitution vertices as before
  BRep_Builder aBB;
  TopTools_DataMapOfShapeShape VSubstitution;
  for (Standard_Integer ii = 1; ii <= VerLver.Extent(); ii++)
  {
    const TopTools_ListOfShape& aVlist = VerLver.FindFromIndex(ii);
    if (aVlist.Extent() == 1)
      continue;

    Standard_Real      aMaxTol = 0.;
    TColgp_Array1OfPnt Points(1, aVlist.Extent());

    TopTools_ListIteratorOfListOfShape itl(aVlist);
    Standard_Integer                   jj = 0;
    for (; itl.More(); itl.Next())
    {
      const TopoDS_Vertex& aVertex = TopoDS::Vertex(itl.Value());
      Standard_Real        aTol    = BRep_Tool::Tolerance(aVertex);
      aMaxTol                      = std::max(aMaxTol, aTol);
      gp_Pnt aPnt                  = BRep_Tool::Pnt(aVertex);
      Points(++jj)                 = aPnt;
    }

    gp_Ax2           anAxis;
    Standard_Boolean IsSingular;
    GeomLib::AxeOfInertia(Points, anAxis, IsSingular);
    gp_Pnt           aCentre = anAxis.Location();
    Standard_Real    aMaxDist = 0.;
    for (Standard_Integer k = 1; k <= Points.Upper(); k++)
      aMaxDist = std::max(aMaxDist, std::sqrt(aCentre.SquareDistance(Points(k))));
    
    // Final tolerance must cover the distance to all points plus their original tolerances
    aMaxTol = std::max(aMaxTol, aMaxDist);

    // Find constant vertex
    TopoDS_Vertex aConstVertex;
    for (itl.Initialize(aVlist); itl.More(); itl.Next())
    {
      const TopoDS_Vertex&               aVertex = TopoDS::Vertex(itl.Value());
      const TopTools_ListOfShape&        aElist  = theVEmap.FindFromKey(aVertex);
      if (aElist.IsEmpty()) continue;
      const TopoDS_Shape&                anEdge  = aElist.First();
      TopTools_ListIteratorOfListOfShape itcedges(myConstEdges);
      for (; itcedges.More(); itcedges.Next())
        if (anEdge.IsSame(itcedges.Value()))
        {
          aConstVertex = aVertex;
          break;
        }
      if (!aConstVertex.IsNull())
        break;
    }
    if (aConstVertex.IsNull())
      aConstVertex = TopoDS::Vertex(aVlist.First());
    aBB.UpdateVertex(aConstVertex, aCentre, aMaxTol);

    for (itl.Initialize(aVlist); itl.More(); itl.Next())
    {
      const TopoDS_Vertex& aVertex = TopoDS::Vertex(itl.Value());
      if (aVertex.IsSame(aConstVertex))
        continue;

      VSubstitution.Bind(aVertex, aConstVertex);
      
      // Update myVerticesForSubstitute
      if (myVerticesForSubstitute.IsBound(aVertex)) {
          TopoDS_Shape OldNewV = myVerticesForSubstitute(aVertex);
          if (!OldNewV.IsSame(aConstVertex)) { myVerticesForSubstitute.Bind(OldNewV, aConstVertex); myVerticesForSubstitute(aVertex) = aConstVertex; }
      } else {
          myVerticesForSubstitute.Bind(aVertex, aConstVertex);
          for (TopTools_DataMapIteratorOfDataMapOfShapeShape mapit(myVerticesForSubstitute); mapit.More(); mapit.Next())
              if (mapit.Value().IsSame(aVertex)) mapit.ChangeValue() = aConstVertex;
      }

      const TopTools_ListOfShape& aElist = theVEmap.FindFromKey(aVertex);
      TopTools_ListIteratorOfListOfShape itE(aElist);
      for (; itE.More(); itE.Next()) {
          TopoDS_Edge anEdge = TopoDS::Edge(itE.Value());
          TopoDS_Edge anEdgeF = anEdge; anEdgeF.Orientation(TopAbs_FORWARD);
          
          TopTools_ListOfShape aVToRemove;
          for (TopoDS_Iterator itVInE(anEdgeF); itVInE.More(); itVInE.Next())
            if (itVInE.Value().IsSame(aVertex)) aVToRemove.Append(itVInE.Value());
          
          if (!aVToRemove.IsEmpty()) {
            anEdgeF.Free(Standard_True);
            for (TopTools_ListIteratorOfListOfShape itTR(aVToRemove); itTR.More(); itTR.Next()) {
              Standard_Real U = BRep_Tool::Parameter(TopoDS::Vertex(itTR.Value()), anEdgeF);
              aBB.Remove(anEdgeF, itTR.Value());
              aBB.Add(anEdgeF, aConstVertex.Oriented(itTR.Value().Orientation()));
              aBB.UpdateVertex(aConstVertex, U, anEdgeF, aMaxTol);
            }
          }
      }
    }
  }

  // 5. FINAL REBUILD OF theVEmap FROM SCRATCH
  // This ensures perfect consistency between edges and their vertex lists.
  TopTools_MapOfShape processedTS;
  TopTools_ListOfShape allEdges;
  for (Standard_Integer i = 1; i <= theVEmap.Extent(); i++) {
      const TopTools_ListOfShape& LE = theVEmap(i);
      for (TopTools_ListIteratorOfListOfShape itL(LE); itL.More(); itL.Next()) {
          if (processedTS.Add(itL.Value())) allEdges.Append(itL.Value());
      }
  }

  theVEmap.Clear();
  TopLoc_Location Loc; Handle(Geom_Surface) Surf = BRep_Tool::Surface(myFace, Loc);
  for (TopTools_ListIteratorOfListOfShape itE(allEdges); itE.More(); itE.Next())
  {
      TopoDS_Edge E = TopoDS::Edge(itE.Value());
      TopTools_IndexedMapOfShape VInE;
      for (TopoDS_Iterator itV(E); itV.More(); itV.Next()) VInE.Add(itV.Value());

      for (Standard_Integer iv = 1; iv <= VInE.Extent(); iv++) {
          const TopoDS_Shape& V = VInE(iv);
          if (!theVEmap.Contains(V)) theVEmap.Add(V, TopTools_ListOfShape());
          theVEmap.ChangeFromKey(V).Append(E);
      }
      
      // Handle closed edges (seam) - mirroring PerformNew logic
      if (!Surf.IsNull() && BRep_Tool::IsClosed(E, Surf, Loc)) {
          TopoDS_Edge ER = TopoDS::Edge(E.Reversed());
          TopTools_IndexedMapOfShape VInER;
          for (TopoDS_Iterator itV(ER); itV.More(); itV.Next()) VInER.Add(itV.Value());
          for (Standard_Integer iv = 1; iv <= VInER.Extent(); iv++) {
              const TopoDS_Shape& V = VInER(iv);
              if (!theVEmap.Contains(V)) theVEmap.Add(V, TopTools_ListOfShape());
              theVEmap.ChangeFromKey(V).Append(ER);
          }
      }
  }
}
