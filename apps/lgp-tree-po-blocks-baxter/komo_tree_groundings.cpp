#include <komo_wrapper.h>

#include "komo_tree_groundings.h"

#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>
#include <Kin/TM_qLimits.h>

#include <observation_tasks.h>
#include <axis_alignment.h>

using namespace rai;
using W = mp::KomoWrapper;

constexpr bool activateObjectives = true;

double shapeSize(const KinematicWorld& K, const char* name, uint i=2);

void groundTreeInit( const mp::TreeBuilder& tb, KOMO_ext* komo, int verbose )
{

}

void groundTreePickUp(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreeUnStack(it, tb, facts, komo, verbose);
}

void groundTreeUnStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();

  /// new version more akin to Franka example
  mp::Interval all{{it.time.from, it.time.to}, it.edge};
  if(activateObjectives)  W(komo).addObjective(all, tb, new LimitsConstraint(0.05), OT_ineq, NoArr, 1.0, 0);

  // approach
  mp::Interval before{{it.time.to - 0.3, it.time.to - 0.3}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetPosition( eff, object, ARR( 0.0, 0.0, 0.1 ) ), OT_sos, NoArr, 1e2, 0 ); // coming from above

  // switch
  mp::Interval st{{it.time.to, it.time.to}, it.edge};
  Transformation rel{0};
  rel.rot.setRadZ(0.0);
  rel.rot.setRadY(-M_PI_2);
  rel.pos.set(0, 0, 0);
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, eff, object, komo->world, SWInit_zero, 0, rel));

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  if(komo->k_order > 1)
  {
    mp::Interval just_after{{it.time.to, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  /// former version:
//  // approach
//  //if(activateObjectives) W(komo).addObjective({{it.time.to-0.3, it.time.to}, it.edge}, tb, new TM_InsideBox(komo->world, eff, NoVector, object, 0.04), OT_ineq, NoArr, 1e2, 0); // inside object at grasp moment

//  // switch
//  mp::Interval st{{it.time.to, it.time.to}, it.edge};

//  Transformation rel{0};
//  rel.rot.setRadZ(0.5 * 3.1415);
//  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_quatBall, eff, object, komo->world, SWInit_zero, 0, rel));

//  // after (stay stable)
//  mp::Interval future{{it.time.to, it.time.to + 2.0}, it.edge}; // fix for at least the 2 nexts
//  mp::Interval end{{it.time.to, it.time.to}, it.edge};
//  if(activateObjectives)  W(komo).addObjective(future, tb, new TM_ZeroQVel(komo->world, object), OT_eq, NoArr, 3e1, 1, +1, -1);
//  if(komo->k_order > 1)
//  {
//    if(activateObjectives)  W(komo).addObjective(end, tb, new TM_LinAngVel(komo->world, object), OT_eq, NoArr, 1e1, 2, +0, +1);
//  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" <<  " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  mp::Interval all{{it.time.from, it.time.to}, it.edge};
  if(activateObjectives)  W(komo).addObjective(all, tb, new LimitsConstraint(0.05), OT_ineq, NoArr, 1.0, 0);

  /// Version based on franka example
  mp::Interval before{{it.time.to - 0.3, it.time.to - 0.3}, it.edge};
  // approach
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetPosition( object, place, ARR( 0.0, 0.0, 0.1 ) ), OT_sos, NoArr, 1e2, 0 ); // coming from above

  mp::Interval just_before{{it.time.to - 0.2, it.time.to - 0.001}, it.edge};
  //if(activateObjectives) W(komo).addObjective( just_before, tb, new AxisAlignment( object, ARR( 0, 0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2, 0 );

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  if(activateObjectives) W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

  // switch
  mp::Interval st{{it.time.to, it.time.to}, it.edge};
  Transformation rel{0};
  rel.rot.setRadZ( 0.0 );
  //rel.rot.setRadY(-M_PI_2);
  rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));

  if(komo->k_order > 1)
  {
    mp::Interval just_after{{it.time.to, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  /// Former version
//  // approach
//  mp::Interval end{{it.time.to, it.time.to}, it.edge};
//  if(activateObjectives) W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

//  // switch
//  mp::Interval st{{it.time.to, it.time.to}, it.edge};
//  Transformation rel{0};
//  rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
//  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_transXYPhi, place, object, komo->world, SWInit_zero, 0, rel));

//  // after (stay stable)
//  mp::Interval future{{it.time.to, -1.0}, it.edge}; // how to improve it? ground until the end sounds inefficient!
//  if(activateObjectives) W(komo).addObjective(future, tb, new TM_ZeroQVel(komo->world, object), OT_eq, NoArr, 3e1, 1, +1, -1);
//  if(komo->k_order > 1)
//  {
//    if(activateObjectives) W(komo).addObjective(end, tb, new TM_LinAngVel(komo->world, object), OT_eq, NoArr, 1e1, 2, +0, +1);
//  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  /// New objectives for Journal paper 2024
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge};
  if(activateObjectives)  W(komo, verbose).addObjective(all, tb, new LimitsConstraint(0.05), OT_ineq, NoArr, 1.0, 0);

  //mp::W(komo).addObjective(start, end, branch, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2,0 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  mp::Interval end{{it.time.to-0.2, it.time.to - 0.01}, it.edge};
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorAimAtObjectCenter( "head", facts[0].c_str(), ARR( -0.3, 0, -1.0 ) ), OT_eq, NoArr, 5.0e1, 0 );
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorAlignsWithPivot( "head", facts[0].c_str(), ARR( 0.05, 0.0, 0.0 ), 45.0 * 3.1415 / 180.0 ), OT_ineq, NoArr, 1e2, 0 );
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorDistanceToObject( "head", facts[0].c_str(), 0.5, 0.0 ), OT_sos, NoArr, 1e2, 0 );

  /// Former objective
  //if(activateObjectives)  W(komo).addObjective(end, tb, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -0.3, 0, -1.0 ), 0.65 ), OT_eq, NoArr, 1e2, 0 );

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(it, tb, facts, komo, verbose);
}
