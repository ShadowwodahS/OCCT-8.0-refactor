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

#ifndef _Toolkit_VertexClusterer_HeaderFile
#define _Toolkit_VertexClusterer_HeaderFile

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Real.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_DataMapOfShapeShape.hxx>
#include <TopTools_DataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

//! Toolkit_VertexClusterer clusters vertices that are within a specified tolerance.
//! It uses a spatial grid (NCollection_CellFilter) to ensure O(N log N) performance.
class Toolkit_VertexClusterer
{
public:
  DEFINE_STANDARD_ALLOC

  //! Constructor
  Standard_EXPORT Toolkit_VertexClusterer(const Standard_Real theTolerance);

  //! Destructor
  Standard_EXPORT virtual ~Toolkit_VertexClusterer();

  //! Adds a list of edges and clusters their vertices.
  Standard_EXPORT void AddEdges(const TopTools_ListOfShape& theEdges);

  //! Adds a single vertex to the clusterer.
  Standard_EXPORT void AddVertex(const TopoDS_Vertex& theVertex);

  //! Returns the master vertex for a given vertex.
  Standard_EXPORT TopoDS_Vertex GetMaster(const TopoDS_Vertex& theVertex) const;

  //! Manually binds a vertex to a master (forces merging).
  Standard_EXPORT void Bind(const TopoDS_Vertex& theVertex, const TopoDS_Vertex& theMaster);

  //! Returns the map of clusters (Master -> List of vertices).
  const TopTools_DataMapOfShapeListOfShape& Clusters() const { return myClusters; }

private:
  Standard_Real                      myTolerance;
  TopTools_DataMapOfShapeShape       myVertexMap;
  TopTools_IndexedMapOfShape         myMasterVertices;
  TopTools_DataMapOfShapeListOfShape myClusters;
  Standard_Address                   myFilter; // Opaque pointer to Toolkit_VertexFilter
};

#endif // _Toolkit_VertexClusterer_HeaderFile
