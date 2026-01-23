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

#ifndef _BRepAlgo_LoopBuildStatus_HeaderFile
#define _BRepAlgo_LoopBuildStatus_HeaderFile

//! Enumeration for loop building status
enum BRepAlgo_LoopBuildStatus
{
  BRepAlgo_LoopBuildStatus_Success,          //!< All wires were built successfully and are closed.
  BRepAlgo_LoopBuildStatus_PartialSuccess,   //!< Some wires were built, but some failed or are open.
  BRepAlgo_LoopBuildStatus_Failed_NoEdges,   //!< No edges were provided.
  BRepAlgo_LoopBuildStatus_Failed_OpenLoop,  //!< Failed to build any closed wires.
  BRepAlgo_LoopBuildStatus_Failed_NotOnFace, //!< Provided edges are not on the face.
  BRepAlgo_LoopBuildStatus_Failed_Unspecified //!< General failure.
};

#endif // _BRepAlgo_LoopBuildStatus_HeaderFile
