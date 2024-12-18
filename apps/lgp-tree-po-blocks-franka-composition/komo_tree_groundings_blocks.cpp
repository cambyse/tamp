#include <komo_wrapper.h>

#include "komo_tree_groundings_blocks.h"
#include "constants.h"

#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>
#include <Kin/TM_qLimits.h>

#include <kin_equality_task.h>
#include <observation_tasks.h>
#include <axis_alignment.h>

using namespace rai;
using W = mp::KomoWrapper;

constexpr bool activateObjectives{true};

double shapeSize(const KinematicWorld& K, const char* name, uint i=2);

namespace blocks // refers to subproblem in paper
{
double BlockToPositionX(const std::string& block, const std::string& place)
{
  if(place.find("block") != std::string::npos)
  {
    return 0.0; // always stack centered onto another block
  }

  static std::map<std::string, double> blockToPosition{
    {"block_1", -0.2 },
    {"block_2", 0.2 },
    {"block_3", 0.0 },
  };

  const auto it = blockToPosition.find(block);

  CHECK(it != blockToPosition.end(), "");

  return it->second;
}

void groundTreeInit( const mp::TreeBuilder& tb, KOMO_ext* komo, int verbose )
{
}

void groundTreePickUp(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreeUnStack(it, tb, facts, komo, verbose);
}

void groundTreeUnStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge}; // needs to revise ors to update limits which are not conistent with scaling?
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, 1.0e2, 0);

  const auto& eff = "franka_hand";
  const auto& object = facts[0].c_str();

  const double t_switch {it.time.to};
  // approach
  mp::Interval before{{t_switch - 0.6, t_switch - 0.3}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetZPosition( eff, object, 0.15, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // coming from above

  if(activateObjectives) W(komo).addObjective( all, tb, new AxisAlignment( eff, ARR( 0.0, 0.0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 5.0e1, 0 ); // the Y axis (lon axis of the gripper with finger) is // to Z

//  mp::Interval all{{it.time.from, it.time.to}, it.edge};
//  if(activateObjectives) W(komo).addObjective( just_before, tb, new AxisOrthogonal( eff, ARR( 0.0, 1.0, 0.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2, 0 ); // the Y axis (lon axis of the gripper with finger) is orthogonal to Z
//  if(activateObjectives) W(komo).addObjective( just_before, tb, new BoxAxisAligned( eff, ARR( 1.0, 0.0, 0 ), object, ARR( -1.0, 0.0, 0 ) ), OT_sos, NoArr, 1e2, 0 ); // pick orthogonal to the sides

  // switch
  mp::Interval st{{t_switch, t_switch}, it.edge};
  Transformation rel{0};
  rel.rot.setRadZ(M_PI_2);
  rel.pos.set(0, 0, 0);
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, eff, object, komo->world, SWInit_zero, 0, rel));

  if(komo->k_order > 1)
  {
    mp::Interval just_after{{t_switch, it.time.to + 0.2}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroMovement( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" <<  " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge}; // needs to revise ors to update limits which are not conistent with scaling?
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, 5.0e2, 0);

  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();
  const auto& eff = "franka_hand";

  const double t_switch {it.time.to};

  // transfer
  if(activateObjectives) W(komo).addObjective( all, tb, new AxisAlignment( eff, ARR( 0.0, 0.0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e1, 0 ); // the Y axis (lon axis of the gripper with finger) is // to Z

  // approach
  mp::Interval before{{t_switch - 0.6, t_switch - 0.4}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetZPosition( object, place, 0.15, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // coming from above

//  mp::Interval just_before{{t_switch - 0.2, t_switch - 0.01}, it.edge};
//  if(activateObjectives) W(komo).addObjective( just_before, tb, new AxisAlignment( object, ARR( 0, 0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2, 0 );

//  mp::Interval end{{t_switch - 0.2, t_switch - 0.01}, it.edge};
//  if(activateObjectives) W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

  // switch
  mp::Interval st{{t_switch, t_switch}, it.edge};
  Transformation rel{0};
  rel.rot.setRadZ( 0.0 );
  rel.pos.set( BlockToPositionX(object, place), 0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
  W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));

  if(komo->k_order > 1)
  {
    mp::Interval just_before{{t_switch - 0.2, t_switch - 0.01}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_before, tb, new ZeroMovement( object ), OT_eq, NoArr, 1e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge}; // needs to revise ors to update limits which are not conistent with scaling?
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, 1.0e2, 0);
  //

  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();
  const auto& eff = "franka_hand";

  const double t_switch {it.time.to - 0.2};
  const double t_switch_2 {t_switch + 0.05};

  // transfer
  if(activateObjectives) W(komo).addObjective( all, tb, new AxisAlignment( eff, ARR( 0.0, 0.0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 5.0e1, 0 ); // the Y axis (lon axis of the gripper with finger) is // to Z

  // approach
  mp::Interval before{{t_switch - 0.6, t_switch - 0.4}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetZPosition( object, place, 0.15, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // coming from above

  //mp::Interval all{{it.time.from, it.time.to}, it.edge};
  //if(activateObjectives) W(komo).addObjective( all, tb, new AxisOrthogonal( eff, ARR( 0.0, 1.0, 0.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 1e2, 0 ); // the Y axis (lon axis of the gripper with finger) is orthogonal to Z

  // switch 1
  {
    mp::Interval st{{t_switch, t_switch}, it.edge};
    Transformation rel{0};
    rel.rot.setRadZ( 0.0 );
    rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
    W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));
  }

  // switch 2
  {
    mp::Interval st{{t_switch_2, t_switch_2}, it.edge};
    Transformation rel{0};
    rel.rot.setRadZ(M_PI_2);
    rel.pos.set(0, 0, 0);
    W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, eff, object, komo->world, SWInit_zero, 0, rel));
  }

  mp::Interval during_switches{{t_switch, t_switch_2}, it.edge};
  W(komo).addObjective(during_switches, tb, new mp::AgentKinEquality( 0, kFrankaCheckConfiguration, kFrankaCheckConfigurationMask ), OT_eq, NoArr, 0.5e2, 0 );

  if(komo->k_order > 1)
  {
    mp::Interval just_before{{t_switch - 0.1, t_switch}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_before, tb, new ZeroMovement( object ), OT_eq, NoArr, 0.5e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)

    mp::Interval just_after{{t_switch_2, t_switch_2 + 0.1}, it.edge};
    if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroMovement( object ), OT_eq, NoArr, 0.5e2, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(it, tb, facts, komo, verbose);
}
}
