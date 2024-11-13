#include <komo_planner.h>

#include <kin_equality_task.h>
#include <trajectory_tree_visualizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/TM_FixSwitchedObjects.h>

#include <belief_state.h>
#include <komo_planner_utils.h>
#include <komo_sparse_planner.h>
#include <komo_sub_problems_finder.h>
#include <komo_planner_utils.h>
#include <Core/util.h>

#include <unordered_set>
#include <thread>
#include <future>
#include <list>
#include <chrono>


namespace mp
{
static constexpr double eps = std::numeric_limits< double >::epsilon();

static double GetCost(const Graph& result, const StringA& filtered_tasks)
{
  double cost{0.0};

  for(const auto& node: result)
  {
    const auto& task_name = node->keys().front();

    const auto& attributes = node->getValue<Graph>();

    if(attributes) // no attributes indicate here that the node doesn't correspond to a task
    {
      const auto& type = attributes->get<rai::String>({ "type" });

      if(attributes && !isTaskCostIrrelevant(task_name, type, filtered_tasks))
      {
        cost += attributes->get<double>({ "sqrCosts" });
      }
    }
  }

  return cost;
}

static double GetConstraints(const Graph& result, const StringA& filtered_tasks)
{
  double constraint{0.0};

  for(const auto& node: result)
  {
    const auto& task_name = node->keys().front();

    const auto& attributes = node->getValue<Graph>();

    if(attributes) // no attributes indicate here that the node doesn't correspond to a task
    {
      const auto& type = attributes->get<rai::String>({ "type" });

      if(attributes && !isTaskConstraintIrrelevant(task_name, type, filtered_tasks))
      {
        constraint += attributes->get<double>({ "constraints" });
      }
    }
  }

  //  std::cout << result << std::endl;
  //  return result.get<double>( { "total", "constraints" } );

  return constraint;
}

//--------Motion Planner--------------//

//double KOMOPlanner::getCost(const std::shared_ptr<ExtensibleKOMO> & komo ) const
//{
//  double total_cost{0.0};

//  for(uint i=0; i<komo->objectives.N; i++)
//  {
//    Objective *task = komo->objectives(i);
//    WorldL Ktuple;
//    Ktuple.resize(task->map->order + 1);

//    if(isTaskIrrelevant(task->name, config_.taskIrrelevantForPolicyCost))
//    {
//       continue;
//    }

//    double task_cost{0.0};

//    for(uint t=0;t < std::min(task->vars.d0, komo->configurations.d0 - task->map->order - 1);t++)
//    {
//      for(uint s=0; s < task->map->order + 1; ++s)
//      {
//        const auto global = t + s;

//        CHECK(global >= 0 && global < komo->configurations.d0, "");

//        Ktuple(s) = komo->configurations(global);
//      }

//      uint d=task->map->__dim_phi(Ktuple);
//      arr y;
//      arr J;
//      task->map->__phi(y, J, Ktuple);
//      CHECK(y.d0 == d, "wrong tm dimensionality");

//      const double global_scale = task->map->scale.N ? task->map->scale(0) : 1.0;
//      const double scale = task->scales.d0 ? task->scales(t) * global_scale : global_scale;

//      if(task->type==OT_sos)
//      {
//        task_cost += scale * sumOfSqr(y);
//      }
//    }

//    std::cout << task->name << " : " << task_cost << std::endl;

//    total_cost += task_cost;
//  }

//  std::cout << "total_cost : " << total_cost << std::endl;

//  return total_cost;
//}

void KOMOPlanner::setKin( const std::string & kinDescription )
{
  Graph G = loadKin(kinDescription);

  if( G[ config_.beliefStateTag_ ] == nullptr )
  {
    auto kin = std::make_shared< rai::KinematicWorld >();
    kin->init( kinDescription.c_str() );
    computeMeshNormals( kin->frames );
    kin->calc_fwdPropagateFrames();
    //kin->watch(/*true*/);

    startKinematics_.append( kin );
  }
  else
  {
    const auto& bsGraph = &G.get<Graph>(config_.beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // build the different worlds
    for( uint w = 0; w < nWorlds; w++ )
    {
      Graph kinG = loadKin(kinDescription);

      // copy unobservable facts
      auto n = bsGraph->elem(w);

      for( const auto& nn : n->graph() )
      {
        nn->newClone( kinG );
      }

      const auto& bsNode = kinG.getNode( config_.beliefStateTag_ );
      kinG.removeValue(bsNode);

      auto kin = createKin(kinG);
      computeMeshNormals( kin->frames );
      kin->calc_fwdPropagateFrames();
      //
      //if( w == 5 )
      //kin->watch( true );
      //
      startKinematics_.append( kin );
    }
  }

  computeQMask();
}

std::vector< double > KOMOPlanner::drawRandomVector( const std::vector< double > & override )
{
  if( startKinematics_.size() == 0 )
  {
    return std::vector< double >();
  }

  if( override.size() > 0 )
  {
    randomVec_ = override;
    return randomVec_;
  }

  const auto& world = startKinematics_(0);

  // get size of random Vector
  uint randomVecSize = 0;
  for( const auto& f : world->frames )
  {
    if( f->ats["random_bounds"]  )
    {
      const auto& randomBounds = f->ats.get<arr>("random_bounds");

      for( const auto& b : randomBounds )
      {
        if( b > 0 )
        {
          randomVecSize++;
        }
      }
    }
  }

  // draw it
  randomVec_ = std::vector< double >( randomVecSize );

  for( auto i = 0; i < randomVecSize; ++i )
  {
    auto v = rnd.uni(-1.0, 1.0);
    randomVec_[i] = v;
  }

  return randomVec_;
}

void KOMOPlanner::solveAndInform( const MotionPlanningParameters & po, Policy & policy, bool watch )
{
  CHECK( startKinematics_.d0 == policy.N(), "consistency problem, the belief state size of the policy differs from the belief state size of the kinematics" );
  CHECK( po.policyId() == policy.id(), "id of the policy and the planning orders are not consistent" );

  //po.getParam( "type" );
  clearLastNonMarkovianResults();

  /// PATH OPTI
  if( po.getParam( "type" ) == "markovJointPath" )
  {
    optimizeMarkovianPath( policy );
    saveMarkovianPathOptimizationResults( policy );
  }
  else if( po.getParam( "type" ) == "jointPath" )
  {
    // solve on path level
    optimizePath( policy );

    // solve on joint path level
    if( policy.N() > 1 )
    {
      optimizeJointPath( policy );
    }

    saveJointPathOptimizationResults( policy );
  }
  else if( po.getParam( "type" ) == "jointSparse" )
  {
    JointPlanner planner(config_, komoFactory_);
    planner.optimize(policy, startKinematics_, watch);
  }
  else if( po.getParam( "type" ) == "ADMMSparse" )
  {
    ADMMSParsePlanner planner(config_, komoFactory_);
    planner.optimize(policy, startKinematics_, watch);
  }
  else if( po.getParam( "type" ) == "ADMMCompressed" )
  {
    ADMMCompressedPlanner planner(config_, komoFactory_, getMarkovianPathTreeVariableQDim(policy));
    planner.setDecompositionStrategy(po.getParam("decompositionStrategy"), po.getParam("nJobs"));
    planner.optimize(policy, startKinematics_, watch);
  }
  else if( po.getParam( "type" ) == "ADMMDecompose" )
  {
    KOMOSubProblemsFinder analyser(config_, komoFactory_);
    analyser.analyse(policy, startKinematics_);
  }
  else if( po.getParam( "type" ) == "EvaluateMarkovianCosts" )
  {
    EvaluationPlanner evaluation(config_, komoFactory_, getMarkovianPathTreeVariableQDim(policy), "results/optimizationReportMarkovianPathTree.re");
    evaluation.optimize(policy, startKinematics_, watch);
  }
  else
  {
    CHECK( false, "not implemented yet!" );
  }
  //CHECK( checkPolicyIntegrity( policy ), "Policy is corrupted" );
}

void KOMOPlanner::display( const Policy & policy, double sec )
{
  // deprecated
  Policy tmp( policy );
  MotionPlanningParameters po( policy.id() );

  po.setParam( "type", "jointPath" );
  // resolve since this planner doesn't store paths
  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  solveAndInform( po, tmp, true );

  //
  const auto elapsed = std::chrono::high_resolution_clock::now() - start;
  const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  std::cout << "motion planning time (ms):" << ms << std::endl;
  //

  // retrieve trajectories
  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > frames;

  const auto & kinFrames = jointPathKinFrames_.size() > 1 ? jointPathKinFrames_ : pathKinFrames_;
  for( const auto& leafWorldKinFramesPair : kinFrames )
  {
    frames.append( leafWorldKinFramesPair.second );
  }

  // display
  if( sec > 0 )
  {
    TrajectoryTreeVisualizer viz( frames, "policy", config_.microSteps_ / config_.secPerPhase_ * 10 );

    rai::wait( sec, true );
  }
}

void KOMOPlanner::displayMarkovianPaths( const Policy & policy, double sec ) const
{
  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > frames;
  frames.resize(1);

  const auto policy_leaves = policy.leaves();

  const auto get_leaf_for_world = [](const uint w, const std::list<Policy::GraphNodeTypePtr>& leaves ) -> Policy::GraphNodeTypePtr
  {
    const auto leaf_it = std::find_if(leaves.cbegin(), leaves.cend(), [&w](const auto& leaf) { return leaf->data().beliefState[w] > 0.0; } );

    CHECK(leaf_it != leaves.cend(), "policy doesn't seem to solve all belief states!");

    return *leaf_it;
  };

  const auto get_path_to_leaf = [](const Policy::GraphNodeTypePtr& leaf) -> std::list<uint>
  {
     std::list<uint> path;
     path.push_back(leaf->data().decisionGraphNodeId);

     auto parent = leaf->parent();
     while(parent)
     {
       if(parent->data().decisionGraphNodeId != 0)
          path.push_back(parent->data().decisionGraphNodeId);
       parent = parent->parent();
     }

     path.reverse();

     return path;
  };


  frames(0).resize(startKinematics_.d0);
  for(auto w{0}; w < startKinematics_.d0; ++w)
  {
    const auto leaf = get_leaf_for_world(w, policy_leaves);

    const auto path = get_path_to_leaf(leaf);

    for(const auto node_id: path)
    {
      const auto path_pieces_it = markovianPaths_.find(node_id);
      CHECK(path_pieces_it != markovianPaths_.cend(), "policy doesn't have planned paths!");

      const auto& x_witness_path_piece = path_pieces_it->second;

      const uint start_s = frames(0)(w).empty() ? 0 : markovian_path_k_order_; // don't append two times the prefixes!
      const uint end_s = (node_id == leaf->id()) ? x_witness_path_piece.d0 : x_witness_path_piece.d0 /*- 1*/;

      CHECK(start_s < x_witness_path_piece.d0, "path pieces for world should not be empty");
      CHECK(end_s <= x_witness_path_piece.d0, "path pieces for world should not be empty");

      for(auto s = start_s ; s < end_s; ++s)
      {
        const auto q = x_witness_path_piece(s).q;
        auto kin = x_witness_path_piece(s); // copy to get the good world
        kin.setJointState(q);
        frames(0)(w).append(kin);
      }
    }
  }

  if( sec > 0 )
  {
    TrajectoryTreeVisualizer viz( frames, "policy", config_.microSteps_ / config_.secPerPhase_ * 10 );

    rai::wait();
  }
}

//arr KOMOPlanner::getMarkovianPathTree( const Policy & policy ) const
//{
//  // This method assumes a constant qdim, which is not possible for more complex kinematic switches!
//  arr x;

//  const auto tree = buildTree(policy);

//  // build a map policy id -> decision graph id
//  std::unordered_map<uint, uint> nodeIdToDecisionGraphId = GetNodeIdToDecisionGraphIds(policy);

//  // go through policy and gather planned configurations
//  const auto q_dim = startKinematics_.front()->q.d0;
//  x.resize((tree.n_nodes() - 1) * config_.microSteps_ * q_dim);
//  std::unordered_set<uint> visited;

//  for(const auto& l: policy.sleaves())
//  {
//    auto q = l;
//    auto p = q->parent();

//    const auto branch= tree._get_branch(l->id());

//    while(p)
//    {
//      if(visited.find(q->id()) == visited.end())
//      {
//        double start = p->depth();
//        double end = q->depth();

//        const auto var_0 = tree.get_vars0(TimeInterval{start, end}, branch, config_.microSteps_);
//        const auto decision_graph_id = nodeIdToDecisionGraphId.at(q->id());
//        const auto& witness_path_piece = markovianPaths_.at(decision_graph_id);

//        for(uint s=0; s < witness_path_piece.d0 - markovian_path_k_order_; ++s)
//        {
//          const auto global = var_0(s);
//          const auto & kin = witness_path_piece(s + markovian_path_k_order_);

//          for(uint i=0; i < q_dim; ++i)
//          {
//            const auto global_i = global * q_dim + i;
//            CHECK(x(global_i) == 0.0, "overrides part of x already gathered, it is most likely a bug!");
//            x(global_i) = kin.q(i);
//          }
//        }

//        // double check connection integrity
//        if(p->id() != 0)
//        {
//          const auto parent_decision_graph_id = nodeIdToDecisionGraphId.at(p->id());
//          const auto& witness_parent_path_piece = markovianPaths_.at(parent_decision_graph_id);
////          std::cout << "check transition " << p->data().decisionGraphNodeId << "->" << q->data().decisionGraphNodeId << " (" << p->id() << "->" << q->id() << ")" << std::endl;
////          std::cout << witness_path_piece(0).q << " vs. " << witness_parent_path_piece(-2).q << std::endl;
////          std::cout << witness_path_piece(1).q << " vs. " << witness_parent_path_piece(-1).q << std::endl;

//          CHECK(witness_parent_path_piece(-2).q == witness_path_piece(0).q, "wrong order 2 connection!");
//        }
//        //

//        visited.insert(q->id());
//      }
//      q = p;
//      p = q->parent();
//    }
//  }

//  return x;
//}

XVariable KOMOPlanner::getMarkovianPathTreeVariableQDim( const Policy & policy ) const
{
  // build a map policy id -> decision graph id
  std::unordered_map<uint, uint> nodeIdToDecisionGraphId = GetNodeIdToDecisionGraphIds(policy);

  const auto tree = buildTree(policy);

  // go through policy and gather xdims
  intA stepToQDim;
  stepToQDim.resize((tree.n_nodes() - 1) * config_.microSteps_);

  ForEachEdge(policy, [&](const auto&p, const auto& q, const auto& branch)
  {
    const double start = p->depth();
    const double end = q->depth();

    const auto var_0 = tree.get_vars0(TimeInterval{start, end}, branch, config_.microSteps_);
    const auto decision_graph_id = nodeIdToDecisionGraphId.at(q->id());
    const auto& witness_path_piece = markovianPaths_.at(decision_graph_id);

    for(uint s=0; s < witness_path_piece.d0 - markovian_path_k_order_; ++s)
    {
      const auto global = var_0(s);
      const auto & kin = witness_path_piece(s + markovian_path_k_order_);

      CHECK(stepToQDim(global) == 0, "the mapping step to qdim should not be written twice, it is most likely a bug!");
      stepToQDim(global) = kin.q.d0;
    }
  });

  // compute cumulated x dimension
  intA stepTointegratedQDim; stepTointegratedQDim.resize(stepToQDim.d0); // mapping from step to the cumulated qdim up to there in the optimization variable
  uint integratedQDim{0};
  for(uint s{0}; s < stepToQDim.d0; ++s)
  {
    integratedQDim += stepToQDim(s);
    stepTointegratedQDim(s) = integratedQDim;
  }

  // go through policy and gather planned configurations
  XVariable X;
  X.stepToQDim = stepToQDim;
  X.stepTointegratedQDim = stepTointegratedQDim;
  X.x.resize(integratedQDim);

  ForEachEdge(policy, [&](const auto&p, const auto& q, const auto& branch)
  {
    const double start = p->depth();
    const double end = q->depth();

    const auto var_0 = tree.get_vars0(TimeInterval{start, end}, branch, config_.microSteps_);
    const auto decision_graph_id = nodeIdToDecisionGraphId.at(q->id());
    const auto& witness_path_piece = markovianPaths_.at(decision_graph_id);

    for(uint s=0; s < witness_path_piece.d0 - markovian_path_k_order_; ++s)
    {
      const auto global = var_0(s);
      const auto & kin = witness_path_piece(s + markovian_path_k_order_);
      const auto qdim = stepToQDim(global);
      const auto global_i_start = stepTointegratedQDim(global) - qdim;

      CHECK(kin.q.d0 == qdim, "inconsistent dimensions");

      for(uint i=0; i < qdim; ++i)
      {
        const auto global_i = global_i_start + i;
        CHECK(X.x(global_i) == 0.0, "overrides part of x already gathered, it is most likely a bug!");
        X.x(global_i) = kin.q(i);
      }
    }

    // double check connection integrity
    if(p->id() != 0)
    {
      const auto parent_decision_graph_id = nodeIdToDecisionGraphId.at(p->id());
      const auto& witness_parent_path_piece = markovianPaths_.at(parent_decision_graph_id);
//          std::cout << "check transition " << p->data().decisionGraphNodeId << "->" << q->data().decisionGraphNodeId << " (" << p->id() << "->" << q->id() << ")" << std::endl;
//          std::cout << witness_path_piece(0).q << " vs. " << witness_parent_path_piece(-2).q << std::endl;
//          std::cout << witness_path_piece(1).q << " vs. " << witness_parent_path_piece(-1).q << std::endl;

      CHECK(witness_parent_path_piece(-2).q == witness_path_piece(0).q, "wrong order 2 connection!");
    }
  });

  return X;
}

std::pair< double, double > KOMOPlanner::evaluateLastSolution()
{
  // retrieve trajectories
  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > frames;

  //CHECK( jointPathKinFrames_.size() == 0, "not supported yet if branching!" );

  const auto & kinFrames = jointPathKinFrames_.size() > 1 ? jointPathKinFrames_ : pathKinFrames_;
  for( const auto& leafWorldKinFramesPair : kinFrames )
  {
    frames.append( leafWorldKinFramesPair.second );
  }
  // evaluation
  for( auto k = 0; k < frames.N; ++k )
  {
    for( auto l = 0; l < frames.at(k).N; ++l )
    {
      auto eval = evaluate( frames.at(k).at(l), config_.secPerPhase_ / config_.microSteps_ );

      auto length = eval.first;
      auto acc_cost = eval.second;

      return std::make_pair( length, acc_cost );
    }
  }

  return {}; // should we end up here at all?
}

void KOMOPlanner::registerInit( const TreeInitGrounder & grounder )
{
  komoFactory_.registerInit( grounder );
}

void KOMOPlanner::registerTask( const std::string & type, const TreeSymbolGrounder & grounder )
{
  komoFactory_.registerTask( type, grounder );
}

void KOMOPlanner::computeQMask()
{
  qmask_ = extractAgentQMask( *startKinematics_( 0 ) );

  // sanity check
  for( uint w = 1; w < startKinematics_.size(); ++w )
  {
    CHECK( qmask_ == extractAgentQMask( *startKinematics_( 1 ) ), "corruption in agent joint definition" );
  }
}

///MARKOVIAN
// markovian path
void KOMOPlanner::optimizeMarkovianPath( Policy & policy )
{
  std::cout << "optimizing markovian paths.." << std::endl;

  optimizeMarkovianPathFrom( policy.root() );
}

void KOMOPlanner::optimizeMarkovianPathFrom( const Policy::GraphNodeTypePtr & node )
{
  std::cout << "optimizing markovian path to:" << node->id() << std::endl;

  bool feasible = true;

  if( markovianPathCosts_.find( node->data().decisionGraphNodeId ) == markovianPathCosts_.end() )
  {
    const auto N = node->data().beliefState.size();
    markovianPathCosts_      [ node->data().decisionGraphNodeId ] = 0;
    markovianPathConstraints_[ node->data().decisionGraphNodeId ] = 0;

    const auto w_it = std::find_if(node->data().beliefState.begin(), node->data().beliefState.end(), [](const double p) { return p > eps; });
    const auto w = std::distance(node->data().beliefState.begin(), w_it);

    if(!node->isRoot()) CHECK( effMarkovianPathKinematics_.find( node->parent()->data().decisionGraphNodeId ) != effMarkovianPathKinematics_.end(), "no parent effective kinematic!" );

    rai::KinematicWorld kin = node->isRoot() ? *( startKinematics_( w ) ) : ( effMarkovianPathKinematics_.find( node->parent()->data().decisionGraphNodeId )->second );

    // create komo
    auto komo = komoFactory_.createKomo();

    // set-up komo
    komo->setModel( kin, true/*, false, true, false, false*/ );
    komo->setTiming( 1.0, config_.microSteps_, config_.secPerPhase_, 2 );
    komo->setSquaredQAccelerations(); // include velocity and pose costs, based on config file rai.cfg (In binary folder)
    komo->setFixSwitchedObjects( -1., -1., 3e1 ); // This forces a zero velocity at the time where the kinematic switch happens

    komo->groundInit();

    ///
    if( node->id() == 3 )
    {
      int a{0};
    }
    int verbose = 1;
    komo->groundTasks( 0, node->data().leadingKomoArgs, verbose );
//    for(auto parent = node->parent(); parent; parent=parent->parent())
//    {
//      std::cout << "parent:" << parent->id() << std::endl;
//      const double phase = static_cast<double>(parent->depth()) - static_cast<double>(node->depth());
//      //komo->groundTasks( phase, parent->data().leadingKomoArgs );
//    }
    ///

    komo->reset();

    // apply correct prefix
    if(!node->isRoot())
    {
      //std::cout << node->parent()->data().decisionGraphNodeId << "->" << node->data().decisionGraphNodeId << std::endl;

      const auto& parent_path_piece = markovianPaths_.find( node->parent()->data().decisionGraphNodeId )->second;

      for(auto s = 0; s < komo->k_order; ++s)
      {
        const auto& kin = parent_path_piece( -int(komo->k_order) + s );
        //komo->configurations(s)->setJointState(kin.q);
        komo->configurations(s)->copy(kin);
      }
    }

    // run
    try {
      komo->verbose = 0;
      komo->run();
    } catch( const char* msg ){
      cout << "KOMO FAILED: " << msg <<endl;
    }

    if( node->id() == 20 )
    {
////      int a{0};
////          for(const auto& f: komo->world.frames)
////          {
////            std::cout << f->name << "--->---" << (f->parent ? f->parent->name : "" ) << std::endl;
////          }

//      komo->getReport(true);
//      komo->configurations.front()->watch(true);
//      komo->configurations.last()->watch(true);

//      komo->displayTrajectory();

//      komo->saveTrajectory( std::to_string( node->id() ) );
//      komo->plotVelocity( std::to_string( node->id() ) );
//      rai::wait();
    }

    const Graph result = komo->getReport();

    std::cout << "result:" << result << std::endl;

    const double cost = GetCost(result, config_.taskIrrelevantForPolicyCost);
    const double constraints = GetConstraints(result, config_.taskIrrelevantForPolicyCost);//result.get<double>( { "total", "constraints" } );

    markovianPathCosts_      [ node->data().decisionGraphNodeId ] += /*node->data().beliefState[ w ] **/ cost;
    markovianPathConstraints_[ node->data().decisionGraphNodeId ] += /*node->data().beliefState[ w ] **/ constraints;

    std::cout << "costs:" << cost << " constraints:" << constraints << std::endl;

    // what to do with the cost and constraints here??
    if( constraints >= config_.maxConstraint_ )
    {
      feasible = false;

      //komo->getReport(true);
      //komo->configurations.first()->watch(true);
      //komo->configurations.last()->watch(true);
      //komo->displayTrajectory();
    }

    // update effective kinematic
    effMarkovianPathKinematics_[ node->data().decisionGraphNodeId ] = *komo->configurations.last();

    // copy path
    for( auto s = 0; s < komo->configurations.N; ++s )
    {
      rai::KinematicWorld kin( *komo->configurations( s ) );
      markovianPaths_[ node->data().decisionGraphNodeId ].append( kin );
    }

    // update switch
    for( rai::KinematicSwitch * sw: komo->switches )
    {
      if( sw->timeOfApplication >=2 ) sw->apply( effMarkovianPathKinematics_[ node->data().decisionGraphNodeId ] );
    }
    effMarkovianPathKinematics_[ node->data().decisionGraphNodeId ].getJointState();

    // free
    freeKomo( komo );
  }
  // solve for next nodes if this one was feasible
  if( feasible )
  {
    for( const auto& c : node->children() )
    {
      optimizeMarkovianPathFrom( c );
    }
  }
}

void KOMOPlanner::saveMarkovianPathOptimizationResults( Policy & policy ) const
{
  std::list< Policy::GraphNodeTypePtr > fifo;
  fifo.push_back( policy.root() );

  while( ! fifo.empty()  )
  {
    auto node = fifo.back();
    fifo.pop_back();

    const auto kIt = markovianPathConstraints_.find(node->data().decisionGraphNodeId);
    const auto cIt = markovianPathCosts_.find(node->data().decisionGraphNodeId);
    CHECK(kIt != markovianPathConstraints_.end(), "map should contain optimization results");
    CHECK(cIt != markovianPathCosts_.end(), "map should contain optimization results");

    double constraint = kIt->second;
    double cost = cIt->second;

    if( constraint >= config_.maxConstraint_ ) // policy infeasible
    {
      std::cout << "Markovian Optimization failed on node " << node->id() << " constraint:" << constraint << std::endl;

      node->data().markovianReturn = std::numeric_limits< double >::lowest();
      //node->setValue( std::numeric_limits< double >::lowest() );
      //node->setStatus( PolicyNode::INFORMED );

      policy.setValue( std::numeric_limits< double >::lowest() );
    }
    else
    {
      node->data().markovianReturn =  -( config_.minMarkovianCost_ + cost );
      node->data().status = PolicyNodeData::INFORMED;
      // push children on list
      for( const auto& c : node->children() )
      {
        fifo.push_back( c );
      }
    }
  }

  /// UPDATE VALUES
  updateValues( policy );
  policy.setStatus( Policy::INFORMED );
}

///NON MARKOVIAN
void KOMOPlanner::clearLastNonMarkovianResults()
{
  // path
  for( auto& pair : pathKinFrames_ )
  {
    pair.second.clear();
  }
  pathKinFrames_.clear(); // maps each leaf to its path

  // joint path
  for( auto& pair : jointPathKinFrames_ )
  {
    pair.second.clear();
  }

  jointPathKinFrames_.clear(); // maps each leaf to its path
}
// path
void KOMOPlanner::optimizePath( Policy & policy )
{
  std::cout << "optimizing full path.." << std::endl;

  bsToLeafs_             = rai::Array< PolicyNodePtr > ( policy.N() );

  for( const auto& l : policy.leaves() )
  {
    optimizePathTo( l );
  }

//  std::list<std::future<void>> futures;
//  for( const auto& l : policy.leaves() )
//  {
//     auto future = std::async(config_.executionPolicy_,
//                              [&]{
//                                  optimizePathTo( l );
//                                });

//     futures.push_back(std::move(future));
//  }

//  for(auto &future: futures)
//  {
//    future.get();
//  }

  computePathQResult(policy);
}

void KOMOPlanner::optimizePathTo( const PolicyNodePtr & leaf )
{
  const auto N = leaf->data().beliefState.size();

  pathKinFrames_[ leaf->id() ] = rai::Array< rai::Array< rai::KinematicWorld > >( N );
  pathXSolution_[ leaf->id() ] = rai::Array< arr                               >( N );
  pathCostsPerPhase_[ leaf->id() ] = rai::Array< arr >( N );

  //-- collect 'path nodes'
  auto treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N; ++w )
  {
    if( leaf->data().beliefState[ w ] > eps )
    {
      // indicate this leaf as terminal for this node, this is used during the joint optimization..
      bsToLeafs_( w ) = leaf;

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      auto leafTime = leaf->depth();
      komo->setModel( *startKinematics_( w ), true/*, false, true, false, false*/ );
      komo->setTiming( leafTime, config_.microSteps_, config_.secPerPhase_, 2 );
      komo->setSquaredQAccelerations();
      komo->setFixSwitchedObjects( -1., -1., 3e1 ); // This forces a zero velocity at the time where the kinematic switch happens

      komo->groundInit();

      for( const auto& node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->depth(): 0. );     // get parent time
        komo->groundTasks( time, node->data().leadingKomoArgs ); // ground parent action (included in the initial state)
      }

      komo->applyRandomization( randomVec_ );
      komo->reset();
      //komo->verbose = 3;

      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

//      if(w == 5)
//      {
//        komo->getReport(true);
//        komo->displayTrajectory();
//      }
      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
//      if( leaf->id() == 10 )
//      {
//        //      komo->plotTrajectory();
//        komo->displayTrajectory( 0.02, true );
//        komo->saveTrajectory( "-j-" + std::to_string( w ) );
//          komo->plotVelocity();// "-j-"   + std::to_string( w ) );
//      }

      auto costs = komo->getCostsPerPhase();
      Graph result = komo->getReport();
      double cost        = result.get<double>( {"total","sqrCosts"} );
      double constraints = result.get<double>( {"total","constraints"} );

      pathCostsPerPhase_[ leaf->id() ]( w ) = costs;

      for( auto s = 0; s < komo->configurations.N; ++s )
      {
        rai::KinematicWorld kin( *komo->configurations( s ) );
        pathKinFrames_[ leaf->id() ]( w ).append( kin );
      }

      pathXSolution_[ leaf->id() ]( w ) = komo->x;

      // free
      freeKomo( komo );
    }
  }
}

void KOMOPlanner::computePathQResult( const Policy& policy )
{
  pathQResult_ = QResult( policy.N(), qmask_, config_.microSteps_ );
  for( uint w = 0; w < bsToLeafs_.size(); ++w )
  {
    const PolicyNodePtr leaf = bsToLeafs_.at(w);
    const auto& trajForW = pathKinFrames_.at(leaf->id()).at(w);
    const uint nSteps = trajForW.size();

    pathQResult_.createTrajectory(w, nSteps);

    for( uint s = 0; s < nSteps; ++s )
    {
      pathQResult_.setQ( w, s, trajForW.at(s).q );
    }
  }
}

void KOMOPlanner::optimizeJointPath( Policy & policy )
{
  std::cout << "optimizing full joint path.." << std::endl;

  std::list<std::future<void>> futures;
  auto leaves = policy.leaves();
  for( const auto& l : leaves )
  {
    auto future = std::async(config_.executionPolicy_,
                             [&]{
      optimizeJointPathTo( l );
    });
    futures.push_back(std::move(future));
  }

  for(auto &future: futures)
  {
    future.get();
  }

  computeJointPathQResult( policy );
}

void KOMOPlanner::optimizeJointPathTo( const PolicyNodePtr & leaf )
{
  const auto N = leaf->data().beliefState.size();

  jointPathKinFrames_  [ leaf->id() ] = rai::Array< rai::Array< rai::KinematicWorld > >( N );
  jointPathCostsPerPhase_[ leaf->id() ] = rai::Array< arr >( N );

  //-- collect 'path nodes'
  const auto treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N; ++w )
  {
    if( leaf->data().beliefState[ w ] > eps )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      auto leafTime = leaf->depth();
      komo->setModel( *startKinematics_( w ), true/*, false, true, false, false*/ );
      komo->setTiming( leafTime, config_.microSteps_, config_.secPerPhase_, 2 );
      komo->setSquaredQAccelerations();
      komo->setFixSwitchedObjects( -1., -1., 3e1 ); // This forces a zero velocity at the time where the kinematic switch happens

      komo->groundInit();

      for( const auto& node:treepath )
      {
        // set task
        auto start = ( node->parent() ? node->parent()->depth(): 0. );     // get parent time
        komo->groundTasks( start, node->data().leadingKomoArgs );          // ground parent action (included in the initial state)

        if( node->depth() > 0 )
        {
          for( auto s = 1; s < komo->stepsPerPhase; ++s )
          {
            uint stepsPerPhase = komo->stepsPerPhase; // get number of steps per phases
            uint nodeSlice = stepsPerPhase * node->depth() - s;
            arr q = zeros( pathKinFrames_[ leaf->id() ]( w )( nodeSlice ).q.N );

            // set constraints enforcing the path equality among worlds
            uint nSupport = 0;
            for( auto x = 0; x < N; ++x )
            {
              if( node->data().beliefState[ x ] > 0 )
              {
                CHECK( bsToLeafs_( x ) != nullptr, "no leaf for this state!!?" );

                const auto& terminalLeafx = bsToLeafs_( x );

                CHECK( pathKinFrames_[ terminalLeafx->id() ]( x ).N > 0, "one node along the solution path doesn't have a path solution already!" );

                const auto & pathLeafx     = pathKinFrames_[ terminalLeafx->id() ]( x );

                CHECK_EQ( q.N, pathLeafx( nodeSlice ).q.N, "wrong q dimensions!" );

                q += node->data().beliefState[ x ] * pathLeafx( nodeSlice ).q;

                nSupport++;
              }
            }

            if( nSupport > 1 )  // enforce kin equality between at least two worlds, useless with just one world!
            {
              AgentKinEquality * task = new AgentKinEquality( node->id(), q, qmask_ );  // tmp camille, think to delete it, or komo does it?
              double slice_t = node->depth() - double( s ) / stepsPerPhase;
              komo->addObjective( slice_t, slice_t, task, OT_eq, NoArr, config_.kinEqualityWeight_ );

              //
              //std::cout << "depth:" << node->depth() << " slice:" << slice_t << " has kin equality, q size = " << qmask.size() << std::endl;
              //
            }
          }
        }
      }

      komo->applyRandomization( randomVec_ );
      komo->set_x( pathXSolution_[ leaf->id() ]( w ) );
      komo->reset();

      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
//     if( leaf->id() == 7 )
//     {
//  //      komo->plotTrajectory();
//        komo->displayTrajectory( 0.02, true );
//        komo->saveTrajectory( "-j-" + std::to_string( w ) );
//        komo->plotVelocity( "-j-"   + std::to_string( w ) );
//     }

      auto costs = komo->getCostsPerPhase();
      const auto& result = komo->getReport();

      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      // store costs
      jointPathCostsPerPhase_[ leaf->id() ]( w ) = costs;

      // store result
      for( auto s = 0; s < komo->configurations.N; ++s )
      {
        const rai::KinematicWorld& kin( *komo->configurations( s ) );
        jointPathKinFrames_[ leaf->id() ]( w ).append( kin );
      }

      // free
      freeKomo( komo );
    }
  }
}

void KOMOPlanner::computeJointPathQResult( const Policy& policy )
{
  jointPathQResult_ = QResult( policy.N(), qmask_, config_.microSteps_ );
  for( uint w = 0; w < bsToLeafs_.size(); ++w )
  {
    const PolicyNodePtr leaf = bsToLeafs_.at(w);
    const auto& trajForW = jointPathKinFrames_.at(leaf->id()).at(w);
    const uint nSteps = trajForW.size(); //  .at(w)->size();

    jointPathQResult_.createTrajectory(w, nSteps);

    for( uint s = 0; s < nSteps; ++s )
    {
      jointPathQResult_.setQ( w, s, trajForW.at(s).q );
    }
  }
}

void KOMOPlanner::saveJointPathOptimizationResults( Policy & policy ) const
{
  /// INFORM POLICY NODES
  std::list< Policy::GraphNodeTypePtr > fifo;
  fifo.push_back( policy.root() );

  while( ! fifo.empty()  )
  {
    auto node = fifo.back();
    fifo.pop_back();

    uint phase = node->depth();

    double cost = 0;

    // get the right world
    auto & pathCostsPerPhase = jointPathKinFrames_.size() > 1 ? jointPathCostsPerPhase_ : pathCostsPerPhase_;
    for( auto w = 0; w < node->data().beliefState.size(); ++w )
    {
      if( node->data().beliefState[ w ] > 0 )
      {
        const auto& leaf = bsToLeafs_( w );
        auto tIt = pathCostsPerPhase.find(leaf->id());

        CHECK( pathCostsPerPhase.find( leaf->id() ) != pathCostsPerPhase.end(), "corruption in datastructure" );
        CHECK(tIt!=pathCostsPerPhase.end(), "optimization results should be in the map");

        const auto& trajCosts = tIt->second( w );
        const auto& wcost = trajCosts( phase - 1 );

        cost += node->data().beliefState[ w ] * wcost;
        //std::cout << "cost of phase:" << cost << " phase:" << phase << std::endl;
      }
    }

    // push children on list
    for( const auto& c : node->children() )
    {
      c->data().markovianReturn = - cost;

      fifo.push_back( c );
    }
  }

  /// UPDATE VALUES AND STATUS
  updateValues( policy );
  policy.setQResult(policy.N()>1 ? jointPathQResult_ : pathQResult_);
  policy.setStatus( Policy::INFORMED );
}

}
