#include <BRepAlgo_Loop.hxx>
#include <BRepAlgo_LoopSolver.hxx>
#include <BRepAlgo_FaceRestrictor.hxx>

#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_Tool.hxx>
#include <BRep_TVertex.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomLib.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
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
#include <algorithm>

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
  
  TopExp::Vertices(E, V1, V2);
  Standard_Real Tol = BRep_Tool::Tolerance(V1);

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

void BRepAlgo_Loop::Perform()
{
  TopTools_ListIteratorOfListOfShape itl, itl1;

#ifdef OCCT_DEBUG_ALGO
  if (AffichLoop)
  {
    std::cout << "NewLoop" << std::endl;
    NbLoops++;
  #ifdef DRAW
    Sprintf(name, "FLoop_%d", NbLoops);
    DBRep::Set(name, myFace);
    Standard_Integer NbEdges = 1;
  #endif
    for (itl.Initialize(myEdges); itl.More(); itl.Next())
    {
      const TopoDS_Edge& E = TopoDS::Edge(itl.Value());
  #ifdef DRAW
      Sprintf(name, "EEE_%d_%d", NbLoops, NbEdges++);
      DBRep::Set(name, E);
  #endif
    }
    for (itl.Initialize(myConstEdges); itl.More(); itl.Next())
    {
      const TopoDS_Edge& E = TopoDS::Edge(itl.Value());
  #ifdef DRAW
      Sprintf(name, "EEE_%d_%d", NbLoops, NbEdges++);
      DBRep::Set(name, E);
  #endif
    }
  }
#endif

  //------------------------------------------------
  // 1. Cut edges based on intersection vertices
  //------------------------------------------------
  for (itl.Initialize(myEdges); itl.More(); itl.Next())
  {
    const TopoDS_Edge&          anEdge = TopoDS::Edge(itl.Value());
    TopTools_ListOfShape        LCE;
    const TopTools_ListOfShape* pVertices = myVerOnEdges.Seek(anEdge);
    if (pVertices)
    {
      CutEdge(anEdge, *pVertices, LCE);
      myCutEdges.Bind(anEdge, LCE);
    }
  }

  //------------------------------------------------
  // 2. Collect all potential edges for the loop solver
  //------------------------------------------------
  TopTools_ListOfShape AllEdges;
  
  // Add sliced parts of the new edges
  for (itl.Initialize(myEdges); itl.More(); itl.Next())
  {
    const TopTools_ListOfShape* pLCE = myCutEdges.Seek(itl.Value());
    if (pLCE)
    {
       for (itl1.Initialize(*pLCE); itl1.More(); itl1.Next())
       {
          AllEdges.Append(itl1.Value());
       }
    }
  }
  
  // Add constant edges (boundaries, seams, internal features)
  for (itl.Initialize(myConstEdges); itl.More(); itl.Next())
  {
      AllEdges.Append(itl.Value());
  }
  
  //------------------------------------------------
  // 3. Invoke BRepAlgo_LoopSolver
  //------------------------------------------------
  BRepAlgo_LoopSolver Solver;
  Solver.SetTolerance(myTolConf); // Critical: Pass tolerance
  Solver.Init(myFace, AllEdges);
  Solver.Perform();
  
  //------------------------------------------------
  // 4. Retrieve results
  //------------------------------------------------
  const TopTools_ListOfShape& NewWires = Solver.GetWires();
  myNewWires.Assign(NewWires);
  
  // Clear faces, they will be built by WiresToFaces
  myNewFaces.Clear();
  
#ifdef DRAW
    if (AffichLoop)
    {
      Standard_Integer iW = 1;
      for (itl.Initialize(myNewWires); itl.More(); itl.Next()) {
        Sprintf(name, "NW_%d_%d", NbLoops, iW++);
        DBRep::Set(name, itl.Value());
      }
    }
#endif

  //------------------------------------------------
  // 5. Cleanup unused cut edges
  //------------------------------------------------
  TopTools_MapOfShape UsedEdges;
  for(itl.Initialize(myNewWires); itl.More(); itl.Next())
  {
       TopoDS_Iterator itW(itl.Value());
       for(; itW.More(); itW.Next()) UsedEdges.Add(itW.Value());
  }
  
  PurgeNewEdges(myCutEdges, UsedEdges);
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
      
      // CRITICAL FIX: Transfer geometry from original edge to new split edge
      B.Transfert(WE, TopoDS::Edge(NewEdge));

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
  Standard_Real Tol = myTolConf; // Use the tolerance member
  it.Initialize(NE);
  while (it.More())
  {
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
  if (!myNewFaces.IsEmpty()) return;
  
  if (!myNewWires.IsEmpty())
  {
    BRepAlgo_FaceRestrictor FR;
    TopoDS_Shape            aLocalS = myFace.Oriented(TopAbs_FORWARD);
    FR.Init(TopoDS::Face(aLocalS), Standard_False);

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
    const TopTools_ListOfShape& aElist  = theVEmap(ii);
    if (aElist.Extent() == 1 && myImageVV.IsImage(aVertex))
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

  if (VerLver.IsEmpty())
    return;

  BRep_Builder aBB;
  for (Standard_Integer ii = 1; ii <= VerLver.Extent(); ii++)
  {
    const TopTools_ListOfShape& aVlist = VerLver(ii);
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
    gp_Pnt        aCentre  = anAxis.Location();
    Standard_Real aMaxDist = 0.;
    for (jj = 1; jj <= Points.Upper(); jj++)
    {
      Standard_Real aSqDist = aCentre.SquareDistance(Points(jj));
      aMaxDist              = std::max(aMaxDist, aSqDist);
    }
    aMaxDist = std::sqrt(aMaxDist);
    aMaxTol  = std::max(aMaxTol, aMaxDist);

    // Find constant vertex
    TopoDS_Vertex aConstVertex;
    for (itl.Initialize(aVlist); itl.More(); itl.Next())
    {
      const TopoDS_Vertex&               aVertex = TopoDS::Vertex(itl.Value());
      const TopTools_ListOfShape&        aElist  = theVEmap.FindFromKey(aVertex);
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

      const TopTools_ListOfShape& aElist = theVEmap.FindFromKey(aVertex);
      TopoDS_Edge                 anEdge = TopoDS::Edge(aElist.First());
      anEdge.Orientation(TopAbs_FORWARD);
      TopoDS_Vertex aV1, aV2;
      TopExp::Vertices(anEdge, aV1, aV2);
      TopoDS_Vertex aVertexToRemove = (aV1.IsSame(aVertex)) ? aV1 : aV2;
      anEdge.Free(Standard_True);
      aBB.Remove(anEdge, aVertexToRemove);
      aBB.Add(anEdge, aConstVertex.Oriented(aVertexToRemove.Orientation()));
    }
  }

  TopTools_IndexedMapOfShape Emap;
  for (Standard_Integer ii = 1; ii <= theVEmap.Extent(); ii++)
  {
    const TopTools_ListOfShape&        aElist = theVEmap(ii);
    TopTools_ListIteratorOfListOfShape itl(aElist);
    for (; itl.More(); itl.Next())
      Emap.Add(itl.Value());
  }

  theVEmap.Clear();
  for (Standard_Integer ii = 1; ii <= Emap.Extent(); ii++)
    TopExp::MapShapesAndAncestors(Emap(ii), TopAbs_VERTEX, TopAbs_EDGE, theVEmap);
}
