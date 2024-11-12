#pragma once

#include <string>
#include <unordered_map>
#include <future>

#include <skeleton.h>

#include <komo_factory.h>
#include <komo_planner_config.h>
#include <komo_wrapper.h>
#include <subtree_generators.h>
#include <optimization_report.h>

namespace mp
{

class KOMOSparsePlanner
{
public:
  KOMOSparsePlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : config_(config)
    , komoFactory_(factory)
  {}

  // common parts between all strategies
  std::shared_ptr< ExtensibleKOMO > intializeKOMO( const TreeBuilder & tree, const std::shared_ptr< const rai::KinematicWorld > & ) const;
  std::vector<Vars> getSubProblems( const TreeBuilder & tree, Policy & policy ) const; // deprecated (branch gen)
  std::vector<intA> getSubProblemMasks( const std::vector<Vars> & allVars, uint T ) const;
  void groundPolicyActionsJoint( const TreeBuilder & tree,
                                 Policy & policy,
                                 const std::shared_ptr< ExtensibleKOMO > & komo ) const;
  void watch( const std::shared_ptr< ExtensibleKOMO > & komo ) const;
  void watch( const std::shared_ptr< ExtensibleKOMO > & komo, const TreeBuilder & tree ) const; // watch using only the witness komo -> inconsistent for visualization
  void watch( const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics, // consistent watch with different worlds
              const rai::Array<rai::KinematicSwitch*> switches,
              const Policy & policy,
              const TreeBuilder & tree,
              const XVariable& X,
              const uint stepsPerPhase,
              const uint k_order ) const;

  OptimizationReport getOptimizationReport(const std::shared_ptr< ExtensibleKOMO > & komo, const std::vector<Vars>& allVars ) const; // use witness komo and task grounding inside it

  virtual void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool watch ) const = 0;

protected:
  using W = KomoWrapper;

  const KOMOPlannerConfig& config_;
  const KOMOFactory & komoFactory_;
};

class JointPlanner : KOMOSparsePlanner
{
public:
  JointPlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory)
  {};
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool ) const override;
};

// Ground all tasks, full opt variable, xmasks in ADMM-Graph-Problem reduce the pb
class ADMMSParsePlanner : KOMOSparsePlanner
{
public:
  ADMMSParsePlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory)
  {};
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool ) const override;
};


// Ground only the branches, compressed opt variable, use usual Graph problem
class ADMMCompressedPlanner : KOMOSparsePlanner
{
public:
  ADMMCompressedPlanner(const KOMOPlannerConfig& config, const KOMOFactory& factory, const XVariable& X)
    : KOMOSparsePlanner(config, factory)
    , X_{X}
  {};
  void setDecompositionStrategy(const std::string& strategy, const std::string& nJobs);
  void groundPolicyActionsCompressed( const TreeBuilder & fullTree,
                                      const TreeBuilder & uncompressed,
                                      const TreeBuilder & compressed,
                                      const Mapping & mapping,
                                      Policy & policy,
                                      const std::shared_ptr< ExtensibleKOMO > & komo ) const;
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool ) const override;

private:
  GeneratorFactory generatorFactory_;
  std::string decompositionStrategy_;
  uint nJobs_{8};
  XVariable X_;
};

// Use to evaluate and watch results only, no actual planning
class EvaluationPlanner : KOMOSparsePlanner
{
public:
  EvaluationPlanner(const KOMOPlannerConfig& config,
                    const KOMOFactory& factory,
                    const XVariable& X,
                    const std::string& reportFile)
    : KOMOSparsePlanner(config, factory)
    , X_(X)
    , reportFile_(reportFile)
  {};
  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool ) const override;

  XVariable X_;
  std::string reportFile_;
};

}
