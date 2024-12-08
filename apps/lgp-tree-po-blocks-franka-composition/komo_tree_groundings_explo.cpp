#include <komo_wrapper.h>

#include "komo_tree_groundings_explo.h"
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

namespace explo{

constexpr bool activateObjectives{true};
constexpr double JointBounds_scale{1.0e2};
constexpr double ZeroVelocity_scale{1.0e2};
constexpr double TargetPosition_scale{1.0e2};
constexpr double SensorAimAtObjectCenter_scale{1.0e2};
constexpr double SensorAlignsWithPivot_scale{1.0e2};
constexpr double SensorDistanceToObject_scale{5.0e1};

Quaternion SideToGrasp(const std::string& side, const bool flipped)
{
  std::cout << side << " " << flipped << std::endl;
  if(flipped)
  {
    int a = 0;
  }

  static std::map<std::pair<std::string, bool>, double> coloredSideToYaw{
    {std::make_pair("side_0", false), 0.0 },
    {std::make_pair("side_0", true), 0.0 },
    {std::make_pair("side_1", false), 3.1415 * 0.5 },
    {std::make_pair("side_1", true), 3.1415 * 0.5 },
    {std::make_pair("side_2", false), 3.1415 },
    {std::make_pair("side_2", true), 3.1415 },
    {std::make_pair("side_3", false), -3.1415 * 0.5 },
    {std::make_pair("side_3", true), -3.1415 * 0.5 },
    {std::make_pair("side_4", false), 3.1415 * 0.5 },
    {std::make_pair("side_4", true), 3.1415 * 0.5 },
    {std::make_pair("side_5", false), -3.1415 * 0.5 },
    {std::make_pair("side_5", true), -3.1415 * 0.5 },
  };

  static std::map<std::pair<std::string, bool>, double> coloredSideToPitch{
    {std::make_pair("side_0", false), 0.0 },
    {std::make_pair("side_0", true), 0.0 },
    {std::make_pair("side_1", false), 0.0 },
    {std::make_pair("side_1", true), 0.0 },
    {std::make_pair("side_2", false), 0.0 },
    {std::make_pair("side_2", true), 0.0 },
    {std::make_pair("side_3", false), 0.0 },
    {std::make_pair("side_3", true), 0.0 },
    {std::make_pair("side_4", false), 0.0 },
    {std::make_pair("side_4", true), -3.145 * 0.5 },
    {std::make_pair("side_5", false), 0.0 },
    {std::make_pair("side_5", true), 0.0 }
  };

  static std::map<std::pair<std::string, bool>, double> coloredSideToRoll{
    {std::make_pair("side_0", false), 0.0 },
    {std::make_pair("side_0", true), 0.0 },
    {std::make_pair("side_1", false), 0.0 },
    {std::make_pair("side_1", true), 0.0 },
    {std::make_pair("side_2", false), 0.0 },
    {std::make_pair("side_2", true), 0.0 },
    {std::make_pair("side_3", false), 0.0 },
    {std::make_pair("side_3", true), 0.0 },
    {std::make_pair("side_4", false), -3.1415 * 0.5 },
    {std::make_pair("side_4", true), -3.1415 * 0.5 },
    {std::make_pair("side_5", false), 3.1415 * 0.5 },
    {std::make_pair("side_5", true), 3.1415 * 0.5 },
  };

  Quaternion q;

  const double angle_x = coloredSideToRoll[std::make_pair(side, flipped)];
  const double angle_y = coloredSideToPitch[std::make_pair(side, flipped)];
  const double angle_z = coloredSideToYaw[std::make_pair(side, flipped)];

  q.setRpy(angle_x, angle_y, angle_z);

  return q;
}

Quaternion SideToRelease(const std::string& side) // orientation object - table
{
  static std::map<std::string, double> coloredSideToYaw{
    {"side_0", -3.1415 / 2.0 },
    {"side_1", 0.0 },
    {"side_2", 3.1415 * 0.5 },
    {"side_3", -3.1415 },
    {"side_4", 0.0 * 3.1415 * 0.5 },
    {"side_5", 0.0 }
  };

  static std::map<std::string, double> coloredSideToPitch{
    {"side_0", 0.0 },
    {"side_1", 0.0 },
    {"side_2", 0.0 },
    {"side_3", 0.0 },
    {"side_4", 0.0 * 3.1415 * 0.5 },
    {"side_5", 0.0 }
  };

  static std::map<std::string, double> coloredSideToRoll{
    {"side_0", 0.0 },
    {"side_1", 0.0 },
    {"side_2", 0.0 },
    {"side_3", 0.0 },
    {"side_4", -3.1415 * 0.5 },
    {"side_5", 3.1415 * 0.5 }
  };

  Quaternion q;

  q.setRpy(coloredSideToRoll[side], coloredSideToPitch[side], coloredSideToYaw[side]);

  return q;
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
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, JointBounds_scale, 0);

  const auto& eff = "franka_hand";
  const auto& object = facts[0].c_str();
  const bool flipped = (facts[2] == "TRUE");
  const bool final_action = (facts[4] == "TRUE");

  // never go under the table
  if(activateObjectives) W(komo).addObjective( all, tb, new TargetZPosition( eff, "tableC", 0.05, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // never go under the table

  // approach coming from above
  mp::Interval before{{it.time.from + 0.35, it.time.to - 0.35}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetZPosition( eff, "tableC", 0.125, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // come from above

  if(final_action)
  {
    mp::Interval final_rejoin{{it.time.to - 0.3, it.time.to - 0.001}, it.edge};
    W(komo).addObjective(final_rejoin, tb, new mp::AgentKinEquality( 0, kFrankaCheckConfiguration, kFrankaCheckConfigurationMask ), OT_eq, NoArr, 0.5e2, 0 );
  }
  else
  {
    // switch
    mp::Interval st{{it.time.to, it.time.to}, it.edge};
    Transformation rel{0};
    rel.rot = SideToGrasp(facts[3], flipped);
    rel.pos.set(0, 0, 0);
    W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, eff, object, komo->world, SWInit_zero, 0, rel));

    mp::Interval end{{it.time.to, it.time.to}, it.edge};
    if(komo->k_order > 1 && !final_action)
    {
      mp::Interval just_after{{it.time.to, it.time.to + 0.15}, it.edge};
      if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, ZeroVelocity_scale, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table!) MAYBE completely wrong?! does it really affect velocities?
    }
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" <<  " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge}; // needs to revise ors to update limits which are not conistent with scaling?
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, JointBounds_scale, 0);

  const auto& eff = "franka_hand";
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();
  const bool final_action = (facts[3] == "TRUE");

  // never go under the table
  if(activateObjectives) W(komo).addObjective( all, tb, new TargetZPosition( eff, place, 0.05, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // never go under the table

  // approach coming from above
  mp::Interval before{{it.time.from + 0.35, it.time.to - 0.35}, it.edge};
  if(activateObjectives) W(komo).addObjective( before, tb, new TargetZPosition( eff, place, 0.1, -1.0 ), OT_ineq, NoArr, 1e1, 0 ); // come from above

  //if(activateObjectives) W(komo).addObjective( all, tb, new AxisAlignment( eff, ARR( 0.0, 0.0, 1.0 ), ARR( 0, 0, 1.0 ) ), OT_sos, NoArr, 5.0e1, 0 ); // the Y axis (lon axis of the gripper with finger) is // to Z

  if(final_action)
  {
    mp::Interval final_rejoin{{it.time.to - 0.15, it.time.to - 0.001}, it.edge};
    W(komo).addObjective(final_rejoin, tb, new mp::AgentKinEquality( 0, kFrankaCheckConfiguration, kFrankaCheckConfigurationMask ), OT_eq, NoArr, 0.5e2, 0 );
  }
  else
  {
    // switch
    mp::Interval st{{it.time.to, it.time.to}, it.edge};
    Transformation rel{0};
    rel.rot = SideToRelease(facts[2]);
    rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
    W(komo).addSwitch(st, tb, new KinematicSwitch(SW_effJoint, JT_rigid, place, object, komo->world, SWInit_zero, 0, rel));

    if(komo->k_order > 1)
    {
      mp::Interval just_after{{it.time.to - 0.1, it.time.to}, it.edge};
      if(activateObjectives) W(komo).addObjective( just_after, tb, new ZeroVelocity( object ), OT_eq, NoArr, ZeroVelocity_scale, 1 ); // force the object not to move when starting to pick (mainly to force it not to go under the table)
    }
  }

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << "(" << it.edge.from << ")" << " -> " << it.time.to << "(" << it.edge.to << ")" << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreePutDownFlipped(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(it, tb, facts, komo, verbose);
}


void groundTreeCheck(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::Interval all{{it.time.from, it.time.to - 0.01}, it.edge}; // needs to revise ors to update limits which are not conistent with scaling?
  if(activateObjectives)  W(komo).addObjective(all, tb, new mp::AgentKinBounds(), OT_ineq, NoArr, JointBounds_scale, 0);
  //

  const auto& eff = "franka_hand";
  const auto& object = facts[0].c_str();
  const auto& side = facts[1].c_str();

  std::map<std::string, arr> sideToPivot{
    {"side_0", ARR( -0.05, 0.0, 0.0 )},
    {"side_1", ARR( 0.00, 0.05, 0.0 )},
    {"side_2", ARR( 0.05, 0.00, 0.0 )},
    {"side_3", ARR( 0.00, -0.05, 0.0 )},
    {"side_4", ARR( 0.00, 0.00, 0.05 )},
    {"side_5", ARR( 0.00, 0.00, -0.05 )}
  };

  // never go under the table
  if(activateObjectives) W(komo).addObjective( all, tb, new TargetZPosition( eff, object, 0.03, -1.0 ), OT_ineq, NoArr, 1e1, 0 );

  mp::Interval end{{it.time.to - 0.2, it.time.to}, it.edge};
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorAimAtObjectCenter( eff, object, ARR( 0, 0, -1 ) ), OT_eq, NoArr, SensorAimAtObjectCenter_scale, 0 );
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorAlignsWithPivot( eff, object, sideToPivot[side], 45.0 * 3.1415 / 180.0 ), OT_ineq, NoArr, SensorAlignsWithPivot_scale, 0 );
  if(activateObjectives) W(komo, verbose).addObjective(end, tb, new SensorDistanceToObject( eff, object, 0.2, 0.0 ), OT_sos, NoArr, SensorDistanceToObject_scale, 0 );

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
