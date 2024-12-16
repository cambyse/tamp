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

//===========================================================================

void plan_graph_search()
{
  ///
  std::ofstream candidate, results;
  candidate.open( "policy-candidates.data" );
  results.open( "policy-results.data" );
  double graph_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;

  namespace ba = boost::accumulators;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_length;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_acc_cost;

  for( auto i = 0; i < 1; ++i )
  {
    matp::GraphPlanner tp;
    mp::KOMOPlanner mp;

    // set planner specific parameters
    tp.setR0( -0.25 ); //-0.25//-0.1//-0.015 );
    tp.setMaxDepth( 20 );
    mp.setNSteps( 20 );
    mp.setMinMarkovianCost( 0.00 );
    ///
    // register symbols
    mp.registerInit( groundTreeInit );
    mp.registerTask( "pick-up"      , groundTreePickUp );
    mp.registerTask( "put-down"     , groundTreePutDown );
    mp.registerTask( "check"        , groundTreeCheck );
    mp.registerTask( "stack"        , groundTreeStack );
    mp.registerTask( "unstack"      , groundTreeUnStack );

    // set start configurations
    // D
    //tp.setFol( "LGP-blocks-fol-one-table-no-precondition.g" );
    //mp.setKin( "LGP-blocks-kin-one-table.g" );

    // C
    //tp.setFol( "LGP-blocks-fol-one-table.g" );
    //mp.setKin( "LGP-blocks-kin-one-table.g" );

    // checked, probably doesn't work with n steps = 5
    // B
    //tp.setFol( "LGP-blocks-fol-2w-one-table.g" );
    //tp.setFol( "LGP-blocks-fol-2w-one-table-no-precondition.g" );
    //mp.setKin( "LGP-blocks-kin-2w-one-table.g" );

    // A
    //tp.setFol( "LGP-blocks-fol-1w-one-table.g" );
    //tp.setFol( "LGP-blocks-fol-1w-one-table-no-precondition.g" );
    //mp.setKin( "LGP-blocks-kin-1w-one-table.g" );

    // 4 blocks linear
    //tp.setFol( "LGP-blocks-fol-4-blocks-1w-one-table.g" );
    //mp.setKin( "LGP-blocks-kin-4-blocks-1w-one-table.g" );

    // 4 blocks new version
    tp.setFol( "LGP-blocks-fol-4-blocks-24w-one-table.g" );
    mp.setKin( "LGP-blocks-kin-4-blocks-24w-one-table.g" );
    //tp.setFol( "LGP-blocks-fol-1w-unified-4-blocks-new.g" );
    //tp.setFol( "LGP-blocks-fol-2w-model-2-unified.g" );
    //mp.setKin( "LGP-blocks-kin-1w-unified-4-blocks-new.g" );

    // 5 blocks linear
    //tp.setFol( "LGP-blocks-fol-5-blocks-1w-one-table.g" );
    //mp.setKin( "LGP-blocks-kin-5-blocks-1w-one-table.g" );

    // 6 blocks linear
    //tp.setFol( "LGP-blocks-fol-6-blocks-1w-one-table.g" );
    //mp.setKin( "LGP-blocks-kin-6-blocks-1w-one-table.g" );

    {
      auto start = std::chrono::high_resolution_clock::now();
      /// GRAPH BUILDING
      tp.buildGraph(true);
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    Policy policy, lastPolicy;

    {
      auto start = std::chrono::high_resolution_clock::now();
      tp.solve();
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    policy = tp.getPolicy();

    uint nIt = 0;
    const uint maxIt = 1000;
    do
    {
      nIt++;
      ///
      savePolicyToFile( policy );
      candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
      ///

      lastPolicy = policy;

      {
        auto start = std::chrono::high_resolution_clock::now();
        /// MOTION PLANNING
        auto po     = MotionPlanningParameters( policy.id() );
        po.setParam( "type", "markovJointPath" );
        mp.solveAndInform( po, policy );
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      }
      ///
      //savePolicyToFile( policy, "-informed" );
      //results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
      ///

      {
        auto start = std::chrono::high_resolution_clock::now();
        /// TASK PLANNING
        tp.integrate( policy );
        tp.solve();
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
      }
      policy = tp.getPolicy();

    } while( lastPolicy != policy && nIt != maxIt );

    /////
    savePolicyToFile( policy, "-final" );
    candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
    results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;

    candidate.close();
    results.close();
    /////
    {
      auto start = std::chrono::high_resolution_clock::now();
      mp.display( policy, 3000 );
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    /////

//    // eval
//    auto eval = mp.evaluateLastSolution();
//    acc_length( eval.first );
//    acc_acc_cost( eval.second );
  }

  std::ofstream timings;
  timings.open("timings.data");
  timings << "graph_building_s="<< graph_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< graph_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();

  // evaluation
  std::cout << "LENGTH: [" <<  ba::min( acc_length ) << " " << ba::max( acc_length ) << "] mean:" << ba::mean( acc_length ) << " std_dev:" << sqrt( ba::variance( acc_length ) ) << std::endl;
  std::cout << "ACC COSTS: [" << ba::min( acc_acc_cost ) << " " << ba::max( acc_acc_cost ) << "] mean:" << ba::mean( acc_acc_cost ) << " std_dev:" << sqrt( ba::variance( acc_acc_cost ) ) << std::endl;
}

void display_robot()
{
  {
    rai::KinematicWorld kin;

    kin.init( "LGP-blocks-kin-2-blocks-one-table-for-display.g" ); // LGP-franka-kin-one-table.g
    //kin.setJointState({0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    //std::cout << "q:" << kin.q << std::endl;

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
}

void decision_tree()
{
  matp::MCTSPlanner tp;
  //matp::MCTSPlannerBs tp;

  srand(2);

  tp.setR0( -1.0, 50.0 );
  tp.setNIterMinMax( 500000, 1000000 );
  tp.setRollOutMaxSteps( 100 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  tp.setFol( "LGP-blocks-fol-one-table-no-precondition.g" );
  //tp.setFol( "LGP-blocks-fol-2w-one-table_for_display.g" ); / for paper

  // SOLVE
  auto start = std::chrono::high_resolution_clock::now();

  tp.solve();

  const auto elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << "planning time (ms): " << std::chrono::duration_cast< std::chrono::milliseconds >(elapsed).count() << std::endl;
  tp.saveMCTSGraphToFile( "decision_tree.gv" );
  generatePngImage( "decision_tree.gv" );

  // BUILD POLICY
  auto policy = tp.getPolicy();

  savePolicyToFile( policy, "-candidate-2-blocks" );
}

void komo_tree_dev()
{
  mp::KOMOPlanner mp;

  // set planner specific parameters
  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );
  mp.setExecutionPolicy(std::launch::async);

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );
  //mp.setKin( "LGP-blocks-kin-1w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-2w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-one-table.g" );
  mp.setKin( "LGP-blocks-kin-4-blocks-24w-one-table.g" );


  // load policy
  Policy policy;
  //policy.load("policy-0-1w");
  //policy.load("policy-0-2w");
  //policy.load("policy-0-6w");
  policy.load("policy-0-24w");

  // plan
  auto po     = MotionPlanningParameters( policy.id() );
  //po.setParam( "type", "jointSparse" );


  //po.setParam( "type", "ADMMSparse" );
//  {
    po.setParam( "type", "ADMMCompressed" );
    po.setParam( "decompositionStrategy", "SubTreesAfterFirstBranching" ); // trees only because otherwise no subproblems!
//    po.setParam( "decompositionStrategy", "BranchGen");
//    po.setParam( "decompositionStrategy", "LinearSplit" ); // NOT EFFICIENT
    po.setParam( "nJobs", "8" );
//  }

  //po.setParam( "type", "ADMMDecompose" ); // decompose hessian only

  mp.solveAndInform( po, policy );
  //mp.display(policy, 200);
}

void plan_3_methods()
{
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planner specific parameters
  tp.setR0( -0.25 ); //-0.25//-0.1//-0.015 );
  tp.setMaxDepth( 20 );
  mp.setNSteps( 20 );
  mp.setMinMarkovianCost( 0.00 );

  // register symbols
  mp.registerInit( groundTreeInit );
  mp.registerTask( "pick-up"      , groundTreePickUp );
  mp.registerTask( "put-down"     , groundTreePutDown );
  mp.registerTask( "check"        , groundTreeCheck );
  mp.registerTask( "stack"        , groundTreeStack );
  mp.registerTask( "unstack"      , groundTreeUnStack );

  tp.setFol( "LGP-blocks-fol-one-table.g" );
  mp.setKin( "LGP-blocks-kin-one-table.g" );

  /// GRAPH BUILDING
  tp.buildGraph(true);

  /// POLICY SEARCH
  Policy policy, lastPolicy;
  tp.solve();
  policy = tp.getPolicy();

  uint nIt = 0;
  const uint maxIt = 1000;
  do
  {
    nIt++;
    lastPolicy = policy;

    {
    /// PIECEWISE MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp.solveAndInform( po, policy );
    }
    {
    /// TASK PLANNING
    tp.integrate( policy );
    tp.solve();
    }
    policy = tp.getPolicy();

  } while( lastPolicy != policy && nIt != maxIt );

  std::cout << "Policy found after " << nIt << " iterations." << std::endl;

  /// JOINT OPTIMIZATION
  // default method
  //mp.display(policy, 200);

  // single joint optimization
  {
  auto po     = MotionPlanningParameters( policy.id() );
  po.setParam( "type", "jointSparse" );
  mp.solveAndInform( po, policy ); // it displays
  }

  // adsm
  {
  auto po     = MotionPlanningParameters( policy.id() );
  po.setParam( "type", "ADMMCompressed" );
  po.setParam( "decompositionStrategy", "Identity" ); // SubTreesAfterFirstBranching, BranchGen, Identity
  po.setParam( "nJobs", "8" );
  mp.solveAndInform( po, policy ); // it displays
  }
}

void ground_and_run(matp::MCTSPlanner& tp, mp::KOMOPlanner& mp, bool watchJointOptimizationResults)
{
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
  config.watchJointOptimizationResults = watchJointOptimizationResults;
  config.saveVideo = false;
  tamp.plan(config);
}

void plan_A(const double c0, bool watchJointOptimizationResults)
{
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 15.0 ); // -10.0, -0.1
  tp.setNIterMinMax( 50000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setFol( "LGP-blocks-fol-1w-one-table.g" );
  mp.setKin( "LGP-blocks-kin-1w-one-table.g" );

  mp.setNSteps( 20 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );
  mp.addCostIrrelevantTask( "LimitsConstraint" );

  ground_and_run(tp, mp, watchJointOptimizationResults);
}

void plan_B(const double c0, bool watchJointOptimizationResults)
{
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 15.0 ); // -10.0, -0.1
  tp.setNIterMinMax( 100000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setFol( "LGP-blocks-fol-2w-one-table.g" );
  mp.setKin( "LGP-blocks-kin-2w-one-table.g" );

  ground_and_run(tp, mp, watchJointOptimizationResults);
}

void plan_C(const double c0, bool watchJointOptimizationResults)
{
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 15.0 ); // -10.0, -0.1
  tp.setNIterMinMax( 100000, 1000000 );
  tp.setRollOutMaxSteps( 50 );

  tp.setFol( "LGP-blocks-fol-one-table.g" );
  mp.setKin( "LGP-blocks-kin-one-table.g" );

  ground_and_run(tp, mp, watchJointOptimizationResults);
}

void plan_D(const double c0, bool watchJointOptimizationResults)
{
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -c0, 40.0 ); // -10.0, -0.1
  tp.setNIterMinMax( 750000, 1000000 );
  tp.setRollOutMaxSteps( 75 );
  tp.setFol( "LGP-blocks-fol-one-table-no-precondition_easy.g" );
  mp.setKin( "LGP-blocks-kin-one-table.g" );

  ground_and_run(tp, mp, watchJointOptimizationResults);
}

void plan_Journal_2024()
{
  //srand(1);

  // build planner
  matp::MCTSPlanner tp;
  mp::KOMOPlanner mp;

  tp.setR0( -1.0, 15.0 );
  tp.setNIterMinMax( 100000, 1000000 );
  tp.setRollOutMaxSteps( 50 );
  tp.setNumberRollOutPerSimulation( 1 );
  tp.setVerbose( false );

  mp.setNSteps( 20 );
  mp.addCostIrrelevantTask( "FixSwichedObjects" );
  mp.addCostIrrelevantTask( "LimitsConstraint" );

  // set start configurations
  // D
//  {
//    tp.setR0( -0.1, 40.0 ); // -10.0, -0.1
//    tp.setNIterMinMax( 750000, 1000000 );
//    tp.setRollOutMaxSteps( 75 );
//    tp.setFol( "LGP-blocks-fol-one-table-no-precondition_easy.g" );
//    mp.setKin( "LGP-blocks-kin-one-table.g" );
//  }

  // E
//  {
//    tp.setR0( -0.1, 75.0 );
//    tp.setNIterMinMax( 2000000, 10000000 ); // 1000000 - by default
//    tp.setRollOutMaxSteps( 100 );
//    //tp.setFol( "LGP-blocks-fol-one-table-no-precondition.g" ); -> too pesimistic logic, use only for Phd to show limit of MCTS when buliding all the tree upfront
//    mp.setKin( "LGP-blocks-kin-one-table.g" );
//  }

  // C
  {
    tp.setR0( -10.0, 15.0 ); // -10.0, -0.1
    tp.setNIterMinMax( 100000, 1000000 );
    tp.setRollOutMaxSteps( 50 );

    tp.setFol( "LGP-blocks-fol-one-table.g" );
    mp.setKin( "LGP-blocks-kin-one-table.g" );
  }

  // checked, probably doesn't work with n steps = 5
  // B
//  {
//    tp.setR0( -10.0, 15.0 ); // -10.0, -0.1
//    tp.setNIterMinMax( 100000, 1000000 );
//    tp.setRollOutMaxSteps( 50 );
//    tp.setFol( "LGP-blocks-fol-2w-one-table.g" );
//    //tp.setFol( "LGP-blocks-fol-2w-one-table-no-precondition.g" );
//    mp.setKin( "LGP-blocks-kin-2w-one-table.g" );
//  }

  // A
//  {
//    tp.setR0( -10.0, 15.0 ); // -10.0, -0.1
//    tp.setNIterMinMax( 50000, 1000000 );
//    tp.setRollOutMaxSteps( 50 );
//    tp.setFol( "LGP-blocks-fol-1w-one-table.g" );
//    //tp.setFol( "LGP-blocks-fol-1w-one-table-no-precondition.g" );
//    mp.setKin( "LGP-blocks-kin-1w-one-table.g" );
//  }

  // 4 blocks linear
  //tp.setFol( "LGP-blocks-fol-4-blocks-1w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-4-blocks-1w-one-table.g" );

  // 4 blocks new version -> probably an issue with the logic -> check with franka if this is necessary to have this one working
  //tp.setFol( "LGP-blocks-fol-4-blocks-24w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-4-blocks-24w-one-table.g" );
  //tp.setFol( "LGP-blocks-fol-1w-unified-4-blocks-new.g" );
  //tp.setFol( "LGP-blocks-fol-2w-model-2-unified.g" );
  //mp.setKin( "LGP-blocks-kin-1w-unified-4-blocks-new.g" );

  // 5 blocks linear
  //tp.setFol( "LGP-blocks-fol-5-blocks-1w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-5-blocks-1w-one-table.g" );

  // 6 blocks linear
  //tp.setFol( "LGP-blocks-fol-6-blocks-1w-one-table.g" );
  //mp.setKin( "LGP-blocks-kin-6-blocks-1w-one-table.g" );

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
  config.watchJointOptimizationResults = false;
  config.saveVideo = true;
  tamp.plan(config);
}


//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  auto args = parseArguments(argc, argv);

  const std::string pb = value_or(args, "-pb", "B");
  const double c0 = std::stof(value_or(args, "-c0", "5.0"));
  const bool watch = std::stoi(value_or(args, "-display", "1")) != 0;

  if(pb == "A")
  {
    plan_A(c0, watch);
  }
  if(pb == "B")
  {
    plan_B(c0, watch);
  }
  if(pb == "C")
  {
    plan_C(c0, watch);
  }
  if(pb == "D")
  {
    plan_D(c0, watch);
  }

  //plan_Journal_2024();
  //komo_tree_dev();
  //plan_graph_search();
  //plan_3_methods();
  /// methods for creating materail for the thesis
  //display_robot();
  //decision_tree();

  return 0;
}
