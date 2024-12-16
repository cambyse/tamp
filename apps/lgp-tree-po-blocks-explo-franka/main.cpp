#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <Kin/kinViewer.h>

#include <graph_planner.h>
#include <mcts_planner.h>
#include <mcts_planner_bs.h>

#include <komo_planner.h>
#include <approx_shape_to_sphere.h>
#include <observation_tasks.h>
#include <object_manipulation_tamp_controller.h>
#include <utility.h>

#include "komo_tree_groundings.h"

//===========================================================================

void plan_1_block(const double c0, const bool watch)
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

  // set problem
  // A
  {
    tp.setR0( -c0, 15.0 ); // -1.0, -10.0
    tp.setNIterMinMax( 100000, 1000000 );
    tp.setRollOutMaxSteps( 50 );
    tp.setFol( "LGP-1-block-6-sides-fol.g" );
    mp.setKin( "LGP-1-block-6-sides-kin.g" );
  }

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "put-down-flipped", groundTreePutDownFlipped );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;

  std::ofstream params("results/params.data");
  params << "c0: " << c0 << std::endl;
  params.close();

  tamp.plan(config);
}

void plan_2_blocks(const double c0, const bool watch)
{
  // debug task planning
//  {
//    matp::MCTSPlanner tp;

//    tp.setR0( -1.0, 1.0 );
//    tp.setNIterMinMax( 500, 100000 );
//    tp.setRollOutMaxSteps( 50 );
//    tp.setNumberRollOutPerSimulation( 1 );
//    tp.setVerbose( false );

//    tp.setFol( "LGP-2-blocks-6-sides-fol.g" );

//    tp.solve();
//    auto policy = tp.getPolicy();
//    savePolicyToFile( policy, "-candidate" );
//  }

  // debug motion planning
//  {
//    Policy policy;
//    policy.load("policy-0-candidate");

//    mp::KOMOPlanner mp;
//    mp.setNSteps( 20 );
//    mp.setMinMarkovianCost( 0.00 );
//    mp.setMaxConstraint( 15.0 );
//    mp.addCostIrrelevantTask( "FixSwichedObjects" );
//    mp.addCostIrrelevantTask( "SensorDistanceToObject" );
//    mp.addCostIrrelevantTask( "SensorAlignsWithPivot" );
//    mp.addCostIrrelevantTask( "SensorAimAtObjectCenter" );
//    mp.addCostIrrelevantTask( "ZeroVelocity" );

//    mp.setKin( "LGP-2-block-6-sides-kin.g" );

//    mp.registerInit( groundTreeInit );
//    mp.registerTask( "pick-up"      , groundTreePickUp );
//    mp.registerTask( "put-down"     , groundTreePutDown );
//    mp.registerTask( "put-down-flipped", groundTreePutDownFlipped );
//    mp.registerTask( "check"        , groundTreeCheck );
//    mp.registerTask( "stack"        , groundTreeStack );
//    mp.registerTask( "stack-flipped", groundTreeStack );
//    mp.registerTask( "unstack"      , groundTreeUnStack );

//    {
//      auto po     = MotionPlanningParameters( policy.id() );
//      po.setParam( "type", "markovJointPath" );
//      mp.solveAndInform( po, policy );
//    }
//    savePolicyToFile( policy, "-informed" );

//    { // evaluate and display markovian -> output in optimizationReportMarkovianPathTree.re
//      auto po     = MotionPlanningParameters( policy.id() );
//      po.setParam( "type", "EvaluateMarkovianCosts" );
//      mp.solveAndInform( po, policy, false );
//    }

//    /// DECOMPOSED SPARSE OPTIMIZATION
//    // adsm
//    {
//      auto po     = MotionPlanningParameters( policy.id() );
//      po.setParam( "type", "ADMMCompressed" ); //ADMMSparse, ADMMCompressed
//      po.setParam( "decompositionStrategy", "Identity" ); // SubTreesAfterFirstBranching, BranchGen, Identity
//      po.setParam( "nJobs", "8" );
//      mp.solveAndInform( po, policy, true );   // optimize and displays joint optimization -> output in optimizationReportAdmmCompressed.re
//    }
//  }
  //
  //srand(1);

  // build planner
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 1.0 );
  tp.setNIterMinMax( 5000, 100000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );
  mp.addCostIrrelevantTask( "SensorDistanceToObject" );
  mp.addCostIrrelevantTask( "SensorAlignsWithPivot" );
  mp.addCostIrrelevantTask( "SensorAimAtObjectCenter" );
  mp.addCostIrrelevantTask( "ZeroVelocity" );
  mp.addCostIrrelevantTask( "AgentKinBounds" );
  mp.addCostIrrelevantTask( "TargetZPosition" );


  // set problem
  tp.setFol( "LGP-2-blocks-6-sides-fol.g" );
  mp.setKin( "LGP-2-block-6-sides-kin.g" );

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "put-down-flipped", groundTreePutDownFlipped );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "stack-flipped", groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;
  tamp.plan(config);
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  auto args = parseArguments(argc, argv);

  rnd.clockSeed();

  const std::string pb = value_or(args, "-pb", "A");
  const double c0 = std::stof(value_or(args, "-c0", "1.0"));
  const bool watch = std::stoi(value_or(args, "-display", "1")) != 0;

  if(pb == "A")
  {
    plan_1_block(c0, watch); // algorithm requires r0
  }
  if(pb == "B")
  {
    plan_2_blocks(c0, watch);
  }

  //display_robot();
  //plan();
  //planMCTS();

  return 0;
}
