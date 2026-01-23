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

#include <Toolkit_VertexClusterer.hxx>

#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <NCollection_CellFilter.hxx>
#include <TopTools_DataMapOfShapeListOfShape.hxx>
#include <TopTools_DataMapIteratorOfDataMapOfShapeListOfShape.hxx>

//=================================================================================================

//! Internal Inspector for CellFilter
struct Toolkit_VertexInspector
{
  typedef Standard_Integer Target;
  typedef gp_Pnt           Point;
  enum { Dimension = 3 };

  Toolkit_VertexInspector(Standard_Real theTol, const TopTools_IndexedMapOfShape& theMap, const gp_Pnt& thePnt)
      : myTol(theTol),
        myMap(theMap),
        myPnt(thePnt),
        myResult(-1)
  {
  }

  static Standard_Real Coord(int i, const Point& thePnt) { return thePnt.Coord(i + 1); }

  static Standard_Boolean IsEqual(const Target& theT1, const Target& theT2) { return theT1 == theT2; }

  NCollection_CellFilter_Action Inspect(const Target& theTarget)
  {
    const TopoDS_Vertex& aV = TopoDS::Vertex(myMap(theTarget));
    const gp_Pnt         aP = BRep_Tool::Pnt(aV);
    if (myPnt.IsEqual(aP, myTol))
    {
      myResult = theTarget;
    }
    return CellFilter_Keep;
  }

  Standard_Real                     myTol;
  const TopTools_IndexedMapOfShape& myMap;
  gp_Pnt                            myPnt;
  Standard_Integer                  myResult;
};

typedef NCollection_CellFilter<Toolkit_VertexInspector> Toolkit_VertexFilter;

//=================================================================================================

Toolkit_VertexClusterer::Toolkit_VertexClusterer(const Standard_Real theTolerance)
    : myTolerance(theTolerance)
{
  myFilter = (Standard_Address)new Toolkit_VertexFilter(myTolerance);
}

//=================================================================================================

Toolkit_VertexClusterer::~Toolkit_VertexClusterer()
{
  if (myFilter != NULL)
  {
    delete (Toolkit_VertexFilter*)myFilter;
  }
}

//=================================================================================================

void Toolkit_VertexClusterer::AddEdges(const TopTools_ListOfShape& theEdges)
{
  TopTools_ListIteratorOfListOfShape it(theEdges);
  for (; it.More(); it.Next())
  {
    TopExp_Explorer anExp(it.Value(), TopAbs_VERTEX);
    for (; anExp.More(); anExp.Next())
    {
      AddVertex(TopoDS::Vertex(anExp.Current()));
    }
  }
}

//=================================================================================================

void Toolkit_VertexClusterer::AddVertex(const TopoDS_Vertex& theVertex)
{
  if (myVertexMap.IsBound(theVertex))
    return;

  gp_Pnt aPnt = BRep_Tool::Pnt(theVertex);
  
  Toolkit_VertexInspector anInspector(myTolerance, myMasterVertices, aPnt);
  gp_Pnt aMinPnt(aPnt.X() - myTolerance, aPnt.Y() - myTolerance, aPnt.Z() - myTolerance);
  gp_Pnt aMaxPnt(aPnt.X() + myTolerance, aPnt.Y() + myTolerance, aPnt.Z() + myTolerance);
  
  Toolkit_VertexFilter* pFilter = (Toolkit_VertexFilter*)myFilter;
  pFilter->Inspect(aMinPnt, aMaxPnt, anInspector);

  if (anInspector.myResult != -1)
  {
    // Found an existing cluster
    const TopoDS_Vertex& aMaster = TopoDS::Vertex(myMasterVertices(anInspector.myResult));
    myVertexMap.Bind(theVertex, aMaster);
    myClusters.ChangeFind(aMaster).Append(theVertex);
  }
  else
  {
    // New cluster
    Standard_Integer anIdx = myMasterVertices.Add(theVertex);
    pFilter->Add(anIdx, aPnt);
    myVertexMap.Bind(theVertex, theVertex);
    
    TopTools_ListOfShape aL;
    aL.Append(theVertex);
    myClusters.Bind(theVertex, aL);
  }
}

//=================================================================================================

TopoDS_Vertex Toolkit_VertexClusterer::GetMaster(const TopoDS_Vertex& theVertex) const
{
  if (myVertexMap.IsBound(theVertex))
  {
    return TopoDS::Vertex(myVertexMap(theVertex));
  }
  return theVertex;
}

//=================================================================================================

void Toolkit_VertexClusterer::Bind(const TopoDS_Vertex& theVertex, const TopoDS_Vertex& theMaster)
{
  if (theVertex.IsSame(theMaster)) return;

  // 1. Ensure theMaster is correctly rooted in our system
  if (!myClusters.IsBound(theMaster))
  {
    TopTools_ListOfShape aL;
    aL.Append(theMaster);
    myClusters.Bind(theMaster, aL);
    myVertexMap.Bind(theMaster, theMaster);
  }
  else
  {
      // theMaster is already a member (or master) of some cluster.
      // We should use ITS current master as the ultimate target to avoid chaining.
      TopoDS_Vertex anActualMaster = GetMaster(theMaster);
      if (theVertex.IsSame(anActualMaster)) return;
      
      // If we are here, we are merging theVertex into anActualMaster's cluster.
      // Recurse once with the actual master to keep the tree flat.
      if (!anActualMaster.IsSame(theMaster)) {
          Bind(theVertex, anActualMaster);
          return;
      }
  }

  // 2. Identify the current cluster of theVertex
  TopoDS_Vertex anOldMaster = GetMaster(theVertex);

  if (!anOldMaster.IsSame(theMaster))
  {
    if (myClusters.IsBound(anOldMaster))
    {
      // Move all members of the old cluster to the new one
      TopTools_ListOfShape& aVList = myClusters.ChangeFind(anOldMaster);
      TopTools_ListOfShape& aNewList = myClusters.ChangeFind(theMaster);
      for (TopTools_ListIteratorOfListOfShape it(aVList); it.More(); it.Next())
      {
        const TopoDS_Shape& aV = it.Value();
        if (aV.IsSame(theMaster)) continue;
        myVertexMap.Bind(aV, theMaster);
        aNewList.Append(aV);
      }
      myClusters.UnBind(anOldMaster);
    }
    else
    {
      // theVertex was free, just attach it
      myVertexMap.Bind(theVertex, theMaster);
      myClusters.ChangeFind(theMaster).Append(theVertex);
    }
  }
}
