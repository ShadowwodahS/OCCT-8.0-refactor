#ifndef _BRepAlgo_LoopSolver_HeaderFile
#define _BRepAlgo_LoopSolver_HeaderFile

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Handle.hxx>

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>
#include <vector>

// Internal structures for graph traversal
// Defined in header to allow vector member in class
struct BRepAlgo_SolverEdge
{
  TopoDS_Edge Edge;
  Standard_Integer NextNodeIndex; // Target Node Index
  Standard_Boolean IsForward;     // Orientation relative to geometry
  Standard_Boolean Visited;       // Traversal flag
  Standard_Integer PCurveIndex;   // 1 or 2 (for Seams)
  gp_Vec2d         Tangent;       // Tangent at Start
  
  BRepAlgo_SolverEdge() 
    : NextNodeIndex(-1), IsForward(Standard_True), Visited(Standard_False), 
      PCurveIndex(1) {}
};

struct BRepAlgo_SolverNode
{
  gp_Pnt2d Point; // 2D Location
  std::vector<BRepAlgo_SolverEdge> OutgoingEdges;
  Standard_Integer ID;
  
  BRepAlgo_SolverNode() : ID(-1) {}
};

//! Robust solver for BRepAlgo_Loop using 2D Geometry-Based Graph.
class BRepAlgo_LoopSolver
{
public:
  DEFINE_STANDARD_ALLOC

  Standard_EXPORT BRepAlgo_LoopSolver();
  Standard_EXPORT ~BRepAlgo_LoopSolver();

  //! Initialize with Face and Edges
  Standard_EXPORT void Init(const TopoDS_Face& F, const TopTools_ListOfShape& Edges);

  //! Set the tolerance for 2D point merging
  void SetTolerance(Standard_Real theTol) { myTolerance = theTol; }

  //! Perform the loop reconstruction
  Standard_EXPORT void Perform();

  //! Returns the constructed wires
  Standard_EXPORT const TopTools_ListOfShape& GetWires() const;

private:
  
  //! Build graph based on 2D endpoints of PCurves
  void BuildGraph();

  //! Find loops using Left-Most Turn (CCW) logic
  void FindLoops();

private:
  TopoDS_Face          myFace;
  TopTools_ListOfShape myInputEdges;
  Standard_Real        myTolerance;

  // Output
  TopTools_ListOfShape myResultWires;
  
  // Graph data
  std::vector<BRepAlgo_SolverNode> myNodes;
};

#endif // _BRepAlgo_LoopSolver_HeaderFile
