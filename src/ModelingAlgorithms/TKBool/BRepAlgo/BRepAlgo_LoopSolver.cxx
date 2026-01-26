#include <BRepAlgo_LoopSolver.hxx>

#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_CurveOnSurface.hxx>
#include <BRep_CurveOnClosedSurface.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>

#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <gp_Vec2d.hxx>
#include <Precision.hxx>
#include <algorithm>
#include <cmath>
#include <vector>

//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------

//! Safe helper to get PCurve with Index (1 or 2) for Seam edges.
static Handle(Geom2d_Curve) GetPCurve(const TopoDS_Edge& E, const TopoDS_Face& F, Standard_Integer Index, Standard_Real& f, Standard_Real& l)
{
  TopLoc_Location L;
  const Handle(Geom_Surface)& S = BRep_Tool::Surface(F, L);
  
  Handle(BRep_TEdge) TE = Handle(BRep_TEdge)::DownCast(E.TShape());
  BRep_ListIteratorOfListOfCurveRepresentation it(TE->Curves());
  
  for (; it.More(); it.Next()) {
    const Handle(BRep_CurveRepresentation)& cr = it.Value();
    if (cr->IsCurveOnSurface(S, L)) {
      if (cr->IsCurveOnClosedSurface()) {
        Handle(BRep_CurveOnClosedSurface) clos = Handle(BRep_CurveOnClosedSurface)::DownCast(cr);
        clos->Range(f, l);
        if (Index == 1) return clos->PCurve();
        if (Index == 2) return clos->PCurve2();
      } else {
        Handle(BRep_CurveOnSurface) norm = Handle(BRep_CurveOnSurface)::DownCast(cr);
        norm->Range(f, l);
        return norm->PCurve();
      }
    }
  }
  return Handle(Geom2d_Curve)();
}

//-----------------------------------------------------------------------------
// Point Merger
//-----------------------------------------------------------------------------

// Helper to merge 2D points within tolerance
class BRepAlgo_PointMerger
{
public:
  BRepAlgo_PointMerger(Standard_Real tol) : myTol(tol) {}

  Standard_Integer AddPoint(const gp_Pnt2d& p)
  {
    Standard_Real tolSq = myTol * myTol;
    // Linear search is robust and sufficient for typical edge counts
    for (size_t i = 0; i < myPoints.size(); ++i)
    {
      if (myPoints[i].SquareDistance(p) < tolSq)
        return (Standard_Integer)i;
    }
    myPoints.push_back(p);
    return (Standard_Integer)(myPoints.size() - 1);
  }

  const std::vector<gp_Pnt2d>& Points() const { return myPoints; }

private:
  std::vector<gp_Pnt2d> myPoints;
  Standard_Real         myTol;
};

//-----------------------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------------------

BRepAlgo_LoopSolver::BRepAlgo_LoopSolver()
  : myTolerance(1.0e-5)
{
}

BRepAlgo_LoopSolver::~BRepAlgo_LoopSolver()
{
}

void BRepAlgo_LoopSolver::Init(const TopoDS_Face& F, const TopTools_ListOfShape& Edges)
{
  myFace = F;
  myInputEdges = Edges;
  myResultWires.Clear();
  myNodes.clear();
}

void BRepAlgo_LoopSolver::Perform()
{
  if (myFace.IsNull()) return;
  
  BuildGraph();
  FindLoops();
}

const TopTools_ListOfShape& BRepAlgo_LoopSolver::GetWires() const
{
  return myResultWires;
}

void BRepAlgo_LoopSolver::BuildGraph()
{
  // Use a minimum robustness tolerance for 2D gaps (1e-5).
  // This handles small gaps in P-Curves better than strict 1e-7.
  Standard_Real graphTol = std::max(myTolerance, 1.0e-5);
  BRepAlgo_PointMerger Merger(graphTol);

  TopTools_ListIteratorOfListOfShape it(myInputEdges);
  for (; it.More(); it.Next())
  {
    const TopoDS_Edge& EOriginal = TopoDS::Edge(it.Value());
    
    if (BRep_Tool::Degenerated(EOriginal)) continue;

    Standard_Boolean isSeam = BRep_Tool::IsClosed(EOriginal, myFace);
    int maxIdx = isSeam ? 2 : 1;

    for (int idx = 1; idx <= maxIdx; ++idx)
    {
      Standard_Real f, l;
      Handle(Geom2d_Curve) C = GetPCurve(EOriginal, myFace, idx, f, l);
      if (C.IsNull()) continue;
      
      // Skip micro edges in 2D
      if (std::abs(l - f) < Precision::PConfusion()) continue;

      gp_Pnt2d PFirst = C->Value(f);
      gp_Pnt2d PLast  = C->Value(l);
      
      // Calculate Tangents
      gp_Pnt2d tmp;
      gp_Vec2d TanFirst, TanLast;
      C->D1(f, tmp, TanFirst);
      C->D1(l, tmp, TanLast);
      
      if (TanFirst.Magnitude() > gp::Resolution()) TanFirst.Normalize();
      if (TanLast.Magnitude()  > gp::Resolution()) TanLast.Normalize();

      // Merge Nodes in 2D Space
      // This correctly keeps (0,0) and (2PI,0) separate on cylinders
      Standard_Integer id1 = Merger.AddPoint(PFirst);
      Standard_Integer id2 = Merger.AddPoint(PLast);

      // Add Forward Edge (id1 -> id2)
      {
        BRepAlgo_SolverEdge SE;
        SE.Edge = EOriginal;
        SE.NextNodeIndex = id2;
        SE.IsForward = Standard_True;
        SE.PCurveIndex = idx;
        SE.Tangent = TanFirst; 
        
        if (myNodes.size() <= (size_t)id1) myNodes.resize(id1 + 1);
        if (myNodes[id1].ID == -1) { // Init new node
             myNodes[id1].ID = id1;
             myNodes[id1].Point = Merger.Points()[id1];
        }
        myNodes[id1].OutgoingEdges.push_back(SE);
      }

      // Add Reversed Edge (id2 -> id1)
      {
        BRepAlgo_SolverEdge SE;
        SE.Edge = EOriginal;
        SE.NextNodeIndex = id1;
        SE.IsForward = Standard_False;
        SE.PCurveIndex = idx;
        SE.Tangent = -TanLast; 
        
        if (myNodes.size() <= (size_t)id2) myNodes.resize(id2 + 1);
        if (myNodes[id2].ID == -1) {
             myNodes[id2].ID = id2;
             myNodes[id2].Point = Merger.Points()[id2];
        }
        myNodes[id2].OutgoingEdges.push_back(SE);
      }
    }
  }
}

void BRepAlgo_LoopSolver::FindLoops()
{
  // Reset visited status
  for (size_t i = 0; i < myNodes.size(); ++i) {
    for (size_t j = 0; j < myNodes[i].OutgoingEdges.size(); ++j) {
      myNodes[i].OutgoingEdges[j].Visited = Standard_False;
    }
  }

  for (size_t i = 0; i < myNodes.size(); ++i)
  {
    // Skip uninitialized nodes (if any gaps in IDs)
    if (myNodes[i].ID == -1) continue;

    for (size_t j = 0; j < myNodes[i].OutgoingEdges.size(); ++j)
    {
      if (myNodes[i].OutgoingEdges[j].Visited) continue;

      BRepBuilderAPI_MakeWire MW;
      
      Standard_Integer startNodeIdx = (Standard_Integer)i;
      Standard_Integer currNodeIdx  = startNodeIdx;
      BRepAlgo_SolverEdge* currEdge = &myNodes[i].OutgoingEdges[j];
      
      bool loopClosed = false;
      int stepCount = 0;
      const int maxSteps = (int)myInputEdges.Extent() * 4;

      while (stepCount++ < maxSteps)
      {
        currEdge->Visited = true;
        
        TopoDS_Shape aLocalShape = currEdge->Edge.Oriented(currEdge->IsForward ? TopAbs_FORWARD : TopAbs_REVERSED);
        MW.Add(TopoDS::Edge(aLocalShape));

        currNodeIdx = currEdge->NextNodeIndex;

        if (currNodeIdx == startNodeIdx)
        {
          loopClosed = true;
          break;
        }

        // --- Find Best Next Edge (Left-Most Turn) ---
        
        // Incoming vector
        gp_Vec2d V_Incoming; 
        {
          Standard_Real f, l;
          Handle(Geom2d_Curve) C = GetPCurve(currEdge->Edge, myFace, currEdge->PCurveIndex, f, l);
          gp_Pnt2d p;
          if (currEdge->IsForward) C->D1(l, p, V_Incoming);
          else                     { C->D1(f, p, V_Incoming); V_Incoming.Reverse(); }
        }
        
        // V_Ref points BACK along the path we came from
        gp_Vec2d V_Ref = -V_Incoming;
        if (V_Ref.Magnitude() < gp::Resolution()) break;

        Standard_Real bestAngle = -1.0; 
        BRepAlgo_SolverEdge* nextEdge = nullptr;
        
        // Check outgoing edges from current node
        if (currNodeIdx < (int)myNodes.size()) 
        {
            BRepAlgo_SolverNode& Node = myNodes[currNodeIdx];
            for (size_t k = 0; k < Node.OutgoingEdges.size(); ++k)
            {
               BRepAlgo_SolverEdge& Cand = Node.OutgoingEdges[k];
               
               // Prevent immediate U-turn on same edge geometry
               // Exception: Allowed if switching between Seam(1) and Seam(2)
               if (Cand.Edge.IsSame(currEdge->Edge) && Cand.PCurveIndex == currEdge->PCurveIndex) continue; 

               Standard_Real ang = V_Ref.Angle(Cand.Tangent); // [-PI, PI]
               if (ang < 0) ang += 2.0 * M_PI; // [0, 2PI)
               
               // Maximize Angle -> Left Turn -> Material on Left
               if (ang > bestAngle)
               {
                 bestAngle = ang;
                 nextEdge = &Cand;
               }
            }
        }

        if (nextEdge) {
           currEdge = nextEdge;
        } else {
           break; 
        }
      }

      if (loopClosed && MW.IsDone())
      {
         TopoDS_Wire W = MW.Wire();
         W.Closed(Standard_True);
         myResultWires.Append(W);
      }
    }
  }
}
