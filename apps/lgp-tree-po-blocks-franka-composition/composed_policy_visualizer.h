#pragma once

#include <komo_sparse_planner.h>

namespace mp
{
struct VisualizationInterval{
  uint start{0};
  uint end{0};
  const rai::Array < rai::KinematicWorld > * frames;
};

class ComposedPolicyVisualizer : public KOMOSparsePlanner
{
public:
  ComposedPolicyVisualizer(const KOMOPlannerConfig& config, const KOMOFactory& factory)
    : KOMOSparsePlanner(config, factory)
  {

  }

  void visualizeComposedPolicy(const rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > & startKinematicsHigh,
                               const rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > & startKinematicsLow,
                               const Policy & policyHigh, const Policy & policyLow,
                               const XVariable& XHigh, const XVariable& XLow,
                               const KOMOFactory& komoFactoryHigh,
                               const KOMOFactory& komoFactoryLow) const;

  std::shared_ptr< ExtensibleKOMO > intializeKOMO( const TreeBuilder & tree,
                                                   const std::shared_ptr< const rai::KinematicWorld > &,
                                                   const KOMOFactory& komoFactory ) const;

  rai::Array< rai::KinematicWorld > createFramesFor(const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics,
                                                    const Policy & policy,
                                                    const XVariable& X,
                                                    const KOMOFactory& komoFactory,
                                                    const uint w) const;

  void optimize( Policy &, const rai::Array< std::shared_ptr< const rai::KinematicWorld > > &, bool watch ) const override {};

  const uint k_order_{2};
};
}
