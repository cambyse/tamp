#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
#include <Kin/kinViewer.h>

#include <graph_planner.h>
#include <mcts_planner.h>
#include <mcts_planner_bs.h>
#include <komo_sparse_planner.h>
#include <komo_planner.h>

#include <approx_shape_to_sphere.h>
#include <observation_tasks.h>
#include <object_manipulation_tamp_controller.h>

#include "komo_tree_groundings_explo.h"
#include "komo_tree_groundings_blocks.h"
#include "composed_policy_visualizer.h"
#include "constants.h"

#include <utility.h>

//===========================================================================

void plan(const double c0, const bool watch)
{
  // debug mp
//  {
//    mp::KOMOPlanner mp;

//    mp.setNSteps( 20 );
//    mp.setMinMarkovianCost( 0.00 );
//    mp.setMaxConstraint( 15.0 );
//    mp.addCostIrrelevantTask( "FixSwichedObjects" );

//    mp.setKin( "LGP-3-blocks-1-side-kin.g" );

//    // register symbols
//    mp.registerInit( groundTreeInit );
//    mp.registerTask( "pick-up"      , groundTreePickUp );
//    mp.registerTask( "put-down"     , groundTreePutDown );
//    mp.registerTask( "check"        , groundTreeCheck );
//    mp.registerTask( "stack"        , groundTreeStack );
//    mp.registerTask( "unstack"      , groundTreeUnStack );

//    Policy policy;
//    policy.load("policy-4-final");

//    auto po     = MotionPlanningParameters( policy.id() );
//    po.setParam( "type", "markovJointPath" );
//    mp.solveAndInform( po, policy );

//    { // evaluate and display markovian -> output in optimizationReportMarkovianPathTree.re
//      auto po     = MotionPlanningParameters( policy.id() );
//      po.setParam( "type", "EvaluateMarkovianCosts" );
//      mp.solveAndInform( po, policy, true );
//    }

//    return;
//  }
  //
  //srand(1);

  // build planner
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );
  mp.saveXVariable(true);
  // set problem 3 blocks
  {
    tp.setR0( -c0, 15.0 ); // -10.0
    tp.setNIterMinMax( 100000, 1000000 );
    tp.setRollOutMaxSteps( 50 );
    tp.setFol( "LGP-3-blocks-1-side-fol.g" );
    mp.setKin( "LGP-3-blocks-1-side-kin.g" );
  }

  // set problem 4 blocks
//  {
//    tp.setR0( -1.0, 5.0 );
//    tp.setNIterMinMax( 500000, 1000000 );
//    tp.setRollOutMaxSteps( 100 );
//    //  tp.setFol( "LGP-4-blocks-1-side-fol.g" );
//    //  mp.setKin( "LGP-4-blocks-1-side-kin.g" );
//  }


  // register symbols
  mp.registerInit( blocks::groundTreeInit );
  mp.registerTask( "pick-up"      , blocks::groundTreePickUp );
  mp.registerTask( "put-down"     , blocks::groundTreePutDown );
  mp.registerTask( "check"        , blocks::groundTreeCheck );
  mp.registerTask( "stack"        , blocks::groundTreeStack );
  mp.registerTask( "unstack"      , blocks::groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;
  tamp.plan(config);
}

void plan_explo(const double c0, const bool watch)
{
  //srand(1);

  // build planner
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "SensorDistanceToObject" );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );
  mp.setRejoinStartConfigurationAtPolicyLeaf_( true );
  mp.saveXVariable(true);
  // set problem
  // A
  {
    tp.setR0( -c0, 15.0 ); // -1.0, -10.0
    tp.setNIterMinMax( 100000, 1000000 );
    tp.setRollOutMaxSteps( 50 );
    tp.setFol( "LGP-1-block-6-sides-fol.g" );
    mp.setKin( "LGP-1-block-6-sides-kin.g", kFrankaCheckConfiguration );
  }

  // register symbols
  mp.registerInit( explo::groundTreeInit );
  mp.registerTask( "pick-up"      , explo::groundTreePickUp );
  mp.registerTask( "put-down"     , explo::groundTreePutDown );
  mp.registerTask( "put-down-flipped", explo::groundTreePutDownFlipped );
  mp.registerTask( "check"        , explo::groundTreeCheck );
  mp.registerTask( "stack"        , explo::groundTreeStack );
  mp.registerTask( "unstack"      , explo::groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;
//  std::ofstream params("results/params.data");
//  params << "c0: " << -1.0 << std::endl;
//  params.close();

  tamp.plan(config);
}

void play()
{
  // scenario A :
  // - pick up block_2; blue; side is facing table (5),
  //   - observe (long)
  // - pick up block_3; red; side is 1
  //   - observe (short)
  // - build

  // high index = 4
  // low index = 5; low_index = 1

  XVariable XHigh;
  XHigh.load("composition/xvariable-high");

  Policy policyHigh;
  policyHigh.load("composition/policy-high");

  XVariable Xlow;
  Xlow.load("composition/xvariable-low");

  Policy policyLow;
  policyLow.load("composition/policy-low");

  // create worlds for high, and their variations..
  rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > startKinematicsHigh;
  mp::KOMOPlanner mpHigh;
  mpHigh.setKin("composition/LGP-3-blocks-1-side-kin.g");
  startKinematicsHigh.append(mpHigh.startKinematics());
  {
    mp::KOMOPlanner mp;
    mp.setKin("composition/LGP-3-blocks-1-side-kin_1.g");
    startKinematicsHigh.append(mp.startKinematics());
  }
  {
    mp::KOMOPlanner mp;
    mp.setKin("composition/LGP-3-blocks-1-side-kin_2.g");
    startKinematicsHigh.append(mp.startKinematics());
  }
  {
    mp::KOMOPlanner mp;
    mp.setKin("composition/LGP-3-blocks-1-side-kin_3.g");
    startKinematicsHigh.append(mp.startKinematics());
  }

  rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > startKinematicsLow;
  mp::KOMOPlanner mpLow;
  mpLow.setKin("composition/LGP-1-block-6-sides-kin.g");
  startKinematicsLow.append(mpLow.startKinematics());
  {
    mp::KOMOPlanner mp;
    mp.setKin("composition/LGP-1-block-6-sides-kin_1.g");
    startKinematicsLow.append(mp.startKinematics());
  }
  {
    mp::KOMOPlanner mp;
    mp.setKin("composition/LGP-1-block-6-sides-kin_2.g");
    startKinematicsLow.append(mp.startKinematics());
  }

  // register symbols high
  mpHigh.registerInit( blocks::groundTreeInit );
  mpHigh.registerTask( "pick-up"      , blocks::groundTreePickUp );
  mpHigh.registerTask( "put-down"     , blocks::groundTreePutDown );
  mpHigh.registerTask( "check"        , blocks::groundTreeCheck );
  mpHigh.registerTask( "stack"        , blocks::groundTreeStack );
  mpHigh.registerTask( "unstack"      , blocks::groundTreeUnStack );

  // register symbols Low
  mpLow.registerInit( explo::groundTreeInit );
  mpLow.registerTask( "pick-up"      , explo::groundTreePickUp );
  mpLow.registerTask( "put-down"     , explo::groundTreePutDown );
  mpLow.registerTask( "put-down-flipped", explo::groundTreePutDownFlipped );
  mpLow.registerTask( "check"        , explo::groundTreeCheck );
  mpLow.registerTask( "stack"        , explo::groundTreeStack );
  mpLow.registerTask( "unstack"      , explo::groundTreeUnStack );


  mp::ComposedPolicyVisualizer visualizer{{}, {}};
  visualizer.visualizeComposedPolicy(startKinematicsHigh, startKinematicsLow,
                                     policyHigh, policyLow,
                                     XHigh, Xlow,
                                     mpHigh.komoFactory(),
                                     mpLow.komoFactory());
}

void show()
{
  mp::KOMOPlanner mp;
  mp.setKin("composition/LGP-3-blocks-1-side-kin.g");
  const_cast<rai::KinematicWorld*>(mp.startKinematics()(4).get())->watch(true);
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);
  auto args = parseArguments(argc, argv);

  rnd.clockSeed();

  const auto pb = value_or(args, "-pb", "A");
  const double c0 = std::stof(value_or(args, "-c0", "1.0"));
  const bool watch = std::stoi(value_or(args, "-display", "1")) != 0;

  if(pb == "C")
  {
    plan(c0, watch);
  }

  if(pb == "A")
  {
    plan_explo(c0, watch);
  }

  if(pb == "CA")
  {
    plan(c0, watch);
    plan_explo(c0, watch);
  }

  if(pb == "play")
  {
    play();
  }

  if(pb == "show")
  {
    show();
  }

  return 0;
}
