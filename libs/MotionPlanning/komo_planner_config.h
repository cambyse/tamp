#pragma once

#include <future>
#include <Core/array.h>

struct KOMOPlannerConfig
{
  const rai::String beliefStateTag_  = "BELIEF_START_STATE";

  double kinEqualityWeight_  = 1e4; // deprecated
  double secPerPhase_        = 10.; // viz only
  double maxConstraint_      = 10. * 0.8;
  double minMarkovianCost_   = 0.0;

  bool rejoinStartConfigurationAtPolicyLeaf_{false}; // this is currently used for initialization only!

  uint microSteps_           = 20; // per phase

  std::launch executionPolicy_ = std::launch::async;

  StringA taskIrrelevantForPolicyCost {};
};
