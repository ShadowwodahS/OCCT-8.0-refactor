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

#ifndef _BRepAlgo_LoopBuildResult_HeaderFile
#define _BRepAlgo_LoopBuildResult_HeaderFile

#include <BRepAlgo_LoopBuildStatus.hxx>
#include <TopTools_ListOfShape.hxx>
#include <Standard_Boolean.hxx>

//! Represents the result of a loop building operation.
class BRepAlgo_LoopBuildResult
{
public:
  //! Constructor
  BRepAlgo_LoopBuildResult() : myStatus(BRepAlgo_LoopBuildStatus_Failed_Unspecified) {}

  //! Returns the status of the operation
  BRepAlgo_LoopBuildStatus Status() const { return myStatus; }

  //! Returns true if at least one wire/face was built
  Standard_Boolean IsSuccess() const { return myStatus == BRepAlgo_LoopBuildStatus_Success || myStatus == BRepAlgo_LoopBuildStatus_PartialSuccess; }

  //! Returns the list of successfully built wires
  const TopTools_ListOfShape& Wires() const { return myWires; }

  //! Returns the list of successfully built faces
  const TopTools_ListOfShape& Faces() const { return myFaces; }

  //! Returns the list of problematic shapes (e.g. open wires)
  const TopTools_ListOfShape& ProblematicShapes() const { return myProblematicShapes; }

private:
  friend class BRepAlgo_LoopBuilder;

  BRepAlgo_LoopBuildStatus myStatus;
  TopTools_ListOfShape     myWires;
  TopTools_ListOfShape     myFaces;
  TopTools_ListOfShape     myProblematicShapes;
};

#endif // _BRepAlgo_LoopBuildResult_HeaderFile
