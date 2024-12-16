#include <functional>
#include <list>
#include <chrono>

#include <boost/filesystem.hpp>
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

void plan_4_blocks(const double c0, const bool watch)
{
  //srand(1);

  // build planner
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 5.0 );
  tp.setNIterMinMax( 500000, 1000000 );
  tp.setRollOutMaxSteps( 100 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );

  // set problem
  tp.setFol( "LGP-4-blocks-1-side-fol.g" );
  mp.setKin( "LGP-4-blocks-1-side-kin.g" );

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;
  tamp.plan(config);
}

void plan_3_blocks(const double c0, const bool watch)
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
  mp.addCostIrrelevantTask( "FixSwichedObjects" );

  // set problem
  // A
  {
    tp.setR0( -c0, 15.0 );
    tp.setNIterMinMax( 100000, 1000000 );
    tp.setRollOutMaxSteps( 50 );
    tp.setFol( "LGP-3-blocks-1-side-fol.g" );
    mp.setKin( "LGP-3-blocks-1-side-kin.g" );
  }


  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp( tp, mp );
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = false;
  config.watchJointOptimizationResults = watch;
  tamp.plan( config );
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  auto args = parseArguments(argc, argv);

  rnd.clockSeed();

  const std::string pb = value_or(args, "-pb", "C");
  const double c0 = std::stof(value_or(args, "-c0", "1.0"));
  const bool watch = std::stoi(value_or(args, "-display", "1")) != 0;

  if(pb == "C")
  {
    plan_3_blocks(c0, watch);
  }
  if(pb == "C4")
  {
    plan_4_blocks(c0, watch);
  }

  return 0;
}
