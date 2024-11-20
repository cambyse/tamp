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

void plan()
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

  tp.setR0( -1.0, 5.0 );
  tp.setNIterMinMax( 500000, 1000000 );
  tp.setRollOutMaxSteps( 100 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setMaxConstraint( 15.0 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );

  // set problem 3 blocks
  {
    tp.setR0( -10.0, 15.0 ); // -10.0, -0.1
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
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  // build and run tamp controller
  ObjectManipulationTAMPController tamp(tp, mp);
  TAMPlanningConfiguration config;
  config.watchMarkovianOptimizationResults = true;
  config.watchJointOptimizationResults = true;
  tamp.plan(config);
}


//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan();

  return 0;
}
