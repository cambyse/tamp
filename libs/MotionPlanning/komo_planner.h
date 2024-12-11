#pragma once

#include <string>
#include <unordered_map>
#include <future>

#include <skeleton.h>
#include <motion_planner.h>

#include <komo_factory.h>
#include <path_evaluator.h>
#include <komo_planner_config.h>

namespace mp
{

class KOMOPlanner : public MotionPlanner
{
  using PolicyNodePtr = Policy::GraphNodeTypePtr;
  using PolicyNodeWeakPtr = Policy::GraphNodeTypeWeakPtr;

public:
  //double getCost(const std::shared_ptr<ExtensibleKOMO> & komo ) const;

  // modifiers
  void setKin( const std::string & kinDescription, const arr& q = NoArr ) override;
  std::vector< double > drawRandomVector( const std::vector< double > & override = std::vector< double >() );

  // informers
  void solveAndInform( const MotionPlanningParameters &, Policy &, bool watch = false, bool saveVideo = false ) override;

  // display
  void display( const Policy & policy, double sec ) override;
  void displayMarkovianPaths( const Policy & policy, double sec ) const;
  //arr getMarkovianPathTree( const Policy & policy ) const; // retired, was not supporting variable qdim!
  XVariable getMarkovianPathTreeVariableQDim( const Policy & policy ) const;

  // evaluation (paper with TMKit comparison)
  std::pair< double, double > evaluateLastSolution();

  // ground symbols
  void registerInit( const TreeInitGrounder & grounder );
  void registerTask( const std::string & type, const TreeSymbolGrounder & grounder );

  void setSecsPerPhase( double s ) { config_.secPerPhase_ = s; }
  void setNSteps( uint n ) { config_.microSteps_ = n; }
  void setMinMarkovianCost( double m ) { config_.minMarkovianCost_ = m; }
  void setExecutionPolicy(std::launch mode) { config_.executionPolicy_ = mode; }
  void setMaxConstraint( double maxConstraint ) { config_.maxConstraint_ = maxConstraint; }
  void setRejoinStartConfigurationAtPolicyLeaf_( bool rejoin ) { config_.rejoinStartConfigurationAtPolicyLeaf_ = rejoin; }
  void addCostIrrelevantTask(const rai::String& task_name) { config_.taskIrrelevantForPolicyCost.append(task_name); };
  void saveXVariable(bool save) { config_.saveXVariable = save; }

  // testonly accessor
  const KOMOPlannerConfig& config() const { return config_; }
  const KOMOFactory& komoFactory() const { return komoFactory_; }
  const rai::Array< std::shared_ptr< const rai::KinematicWorld > >& startKinematics() const { return startKinematics_; }

private:
  void computeQMask();

  /// MARKOVIAN
  // markovian path
  void optimizeMarkovianPath( Policy & );
  void optimizeMarkovianPathFrom( const PolicyNodePtr & );
  void saveMarkovianPathOptimizationResults( Policy & ) const;

  /// NON MARKOVIAN
  void clearLastNonMarkovianResults();
  // path
  void optimizePath( Policy & );
  void optimizePathTo( const PolicyNodePtr& );
  void computePathQResult( const Policy& policy );

  // joint path
  void optimizeJointPath( Policy & );
  void optimizeJointPathTo( const PolicyNodePtr& );
  void computeJointPathQResult( const Policy& policy );
  void saveJointPathOptimizationResults( Policy & ) const;

private:
  // state
  rai::Array< std::shared_ptr< const rai::KinematicWorld > > startKinematics_;
  arr qmask_; //1 for agent, 0 for non agent joint
  KOMOFactory komoFactory_;

  std::vector< double > randomVec_; // used to randomize the initial configuration

  // markovian path
  const uint markovian_path_k_order_{2};
  std::unordered_map< uint, rai::KinematicWorld > effMarkovianPathKinematics_;  // resulting final kinematic of path piece going to node id
  std::unordered_map< uint, rai::Array< rai::KinematicWorld > > markovianPaths_; // resulting path(s) of traj piece going to node id
  std::unordered_map< uint, double > markovianPathCosts_; // node id -> averaged cost
  std::unordered_map< uint, double > markovianPathConstraints_; // node id -> averaged constraints

  // path
  std::unordered_map< uint, rai::Array< rai::Array< rai::KinematicWorld > > > pathKinFrames_; // node(leaf) -> trajectory for each world
  std::unordered_map< uint, rai::Array< arr > > pathXSolution_; // node(leaf) -> x for each world
  std::unordered_map< uint, rai::Array< arr > > pathCostsPerPhase_;
  QResult pathQResult_;

  // joint path
  std::unordered_map< uint, rai::Array< rai::Array< rai::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  std::unordered_map< uint, rai::Array< arr > > jointPathCostsPerPhase_;
  rai::Array< PolicyNodePtr > bsToLeafs_; //indicates the leaf terminating for a given state
  QResult jointPathQResult_;

  // params
  KOMOPlannerConfig config_;
};

}
