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

static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

static void savePolicyToFile( const Policy & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;

  namess << "policy-" << policy.id() << suffix;
  policy.save( namess.str() );

  namess << ".gv";
  policy.saveToGraphFile( namess.str() );
}

//===========================================================================

void display_robot()
{
    rai::KinematicWorld kin;

    kin.init( "LGP-1-block-6-sides-kin.g" );
    //kin.setJointState({0.0, 3.0, 0.0, 0.65, 0.0, -1.0, -0.78});

    std::cout << "q:" << kin.q << std::endl;

//    kin.init( "LGP-blocks-kin-unified-b6.g" );

////    const double zf = 1.47;
////    const double s = 0.55;
////    kin.gl().camera.setPosition(s * 10., s * 4.5, zf + s * ( 3.5 - zf ));

//    const double zf = 1.0;
//    const double s = 0.35;
//    kin.gl().camera.setPosition(s * 10., s * 0, zf + s * ( 1.5 - zf ));

//    kin.gl().camera.focus(0.5, 0, zf);
//    kin.gl().camera.upright();

    kin.watch(100);
//    kin.write( std::cout );

//    rai::wait( 300, true );
}

void plan_1_block(const double r0)
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
    tp.setR0( r0, 15.0 ); // -1.0, -10.0
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
  config.watchJointOptimizationResults = false;

  std::ofstream params("results/params.data");
  params << "c0: " << -r0 << std::endl;
  params.close();

  tamp.plan(config);
}

void plan_2_blocks()
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

  tp.setR0( -1.0, 1.0 );
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
  config.watchJointOptimizationResults = true;
  tamp.plan(config);
}

void planMCTS()
{
  srand(1);

  // content of this function is now merged with the main plan()
  matp::MCTSPlanner tp;
  //matp::MCTSPlannerBs tp;

  tp.setR0( -1.0, 1.0 );
  tp.setNIterMinMax( 500, 100000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  tp.setFol( "LGP-1-block-6-sides-fol.g" );
  //tp.setFol( "LGP-2-blocks-6-sides-fol.g" );

  // SOLVE
  auto start = std::chrono::high_resolution_clock::now();

  tp.solve();

  const auto elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << "planning time (ms): " << std::chrono::duration_cast< std::chrono::milliseconds >(elapsed).count() << std::endl;
  tp.saveMCTSGraphToFile( "decision_graph.gv" );
  //generatePngImage( "decision_graph.gv" );

  // BUILD POLICY
  auto policy = tp.getPolicy();

  savePolicyToFile( policy, "-candidate-2-blocks" );
}

//===========================================================================



int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  auto args = parseArguments(argc, argv);

  rnd.clockSeed();

  for (const auto& [key, value] : args)
  {
     std::cout << key << ": " << value << std::endl;
  }

  const uint n_blocks = std::stoi(value_or(args, "-n", "1"));

  if(n_blocks == 1)
  {
    const double c0 = std::stof(value_or(args, "-c0", "5.0"));

    plan_1_block(-c0); // algorithm requires r0
  }

  if(n_blocks == 2)
  {
    plan_2_blocks();
  }

  //display_robot();

  //plan();

  //planMCTS();

  return 0;
}
