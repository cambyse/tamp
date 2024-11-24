#include "composed_policy_visualizer.h"
#include <trajectory_tree_visualizer.h>
#include <Kin/kin.h>
#include <Kin/switch.h>
#include <Core/array.h>

namespace mp{

std::shared_ptr< ExtensibleKOMO > ComposedPolicyVisualizer::intializeKOMO( const TreeBuilder & tree,
                                                                           const std::shared_ptr< const rai::KinematicWorld > & startKinematic,
                                                                           const KOMOFactory& komoFactory ) const
{
  auto komo = komoFactory.createKomo();
  komo->setModel(*startKinematic);
  komo->sparseOptimization = true;

  CHECK(tree.d() == 0, "support for free prefix komo deactivated!");
  //komo->freePrefix = !(tree.d() == 0); // free prefix is used when decomposing the trajectory in several pieces and optimizing them independantly, this is NOT efficient

  const auto nPhases = tree.n_nodes() - 1;
  komo->setTiming(nPhases, config_.microSteps_, config_.secPerPhase_, 2);

  return komo;
}

void ComposedPolicyVisualizer::visualizeComposedPolicy(
    const rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > & startKinematicsHigh,
    const rai::Array< rai::Array< std::shared_ptr< const rai::KinematicWorld > > > & startKinematicsLow,
    const Policy & policyHigh, const Policy & policyLow,
    const XVariable& XHigh, const XVariable& XLow,
    const KOMOFactory& komoFactoryHigh,
    const KOMOFactory& komoFactoryLow) const
{
  const auto framesHigh = createFramesFor(startKinematicsHigh(0), policyHigh, XHigh, komoFactoryHigh, 4);
  const auto framesHigh_1 = createFramesFor(startKinematicsHigh(1), policyHigh, XHigh, komoFactoryHigh, 4); // blue (side 5)
  const auto framesHigh_2 = createFramesFor(startKinematicsHigh(2), policyHigh, XHigh, komoFactoryHigh, 4); // red  (side 0)
  const auto framesHigh_3 = createFramesFor(startKinematicsHigh(3), policyHigh, XHigh, komoFactoryHigh, 4); // green (side 2)

  //const_cast<rai::KinematicWorld*>(startKinematicsHigh(0)(0).get())->watch(true);
  //const_cast<rai::KinematicWorld*>(startKinematicsHigh(1)(0).get())->watch(true);

  const auto framesLow = createFramesFor(startKinematicsLow(0), policyLow, XLow, komoFactoryLow, 5);   // blue
  const auto framesLow_1 = createFramesFor(startKinematicsLow(1), policyLow, XLow, komoFactoryLow, 0); // red
  const auto framesLow_2 = createFramesFor(startKinematicsLow(2), policyLow, XLow, komoFactoryLow, 2); // green

  rai::Array< rai::KinematicWorld > framesConcatenated;

  uint switchToLow_0{2 * config_.microSteps_ + k_order_};
  uint switchToLow_1{5 * config_.microSteps_ + k_order_};
  uint switchToLow_2{8 * config_.microSteps_ + k_order_};

  std::vector<VisualizationInterval> intervals;
  intervals.push_back(VisualizationInterval{0, switchToLow_0, &framesHigh}); // bring blue
  intervals.push_back(VisualizationInterval{k_order_, framesLow.d0, &framesLow}); // observe blue
  intervals.push_back(VisualizationInterval{switchToLow_0, switchToLow_1, &framesHigh_1}); // place blue and pick red
  intervals.push_back(VisualizationInterval{k_order_, framesLow_1.d0, &framesLow_1}); // observe red
  intervals.push_back(VisualizationInterval{switchToLow_1, switchToLow_2, &framesHigh_2}); // place red and pick green
  intervals.push_back(VisualizationInterval{k_order_, framesLow_2.d0, &framesLow_2}); // observe green
  intervals.push_back(VisualizationInterval{switchToLow_2, framesHigh_3.d0, &framesHigh_3}); // stack green

  uint j{0};

  for(const auto& interval: intervals)
  {
    framesConcatenated.resize(framesConcatenated.d0 + interval.end - interval.start);

    for(uint i{interval.start}; i < interval.end; ++i)
    {
      framesConcatenated(j).copy((*interval.frames)(i));
      ++j;
    }
  }

  rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > framesAll; // for each leave, for each worl, the frame list
  framesAll.resize(1);
  framesAll(0).resize(1);
  framesAll(0)(0) = framesConcatenated;

  TrajectoryTreeVisualizer viz( framesAll, "composed policy", 10 );

  rai::wait();
}

rai::Array< rai::KinematicWorld > ComposedPolicyVisualizer::createFramesFor(const rai::Array< std::shared_ptr< const rai::KinematicWorld > > & startKinematics,
                                                  const Policy & policy,
                                                  const XVariable& X,
                                                  const KOMOFactory& komoFactory,
                                                  const uint w) const
{
  rai::Array< rai::KinematicWorld > frames;

  const auto& startKinematic = startKinematics(w);

  const auto get_leaf_for_world = [](const uint w, const std::list<Policy::GraphNodeTypePtr>& leaves ) -> uint
  {
    const auto leaf_it = std::find_if(leaves.cbegin(), leaves.cend(), [&w](const auto& leaf) { return leaf->data().beliefState[w] > 0.0; } );

    CHECK(leaf_it != leaves.cend(), "policy doesn't seem to solve all belief states!");

    return (*leaf_it)->id();
  };

  const auto tree = buildTree(policy);
  const auto witness = intializeKOMO(tree, startKinematic, komoFactory);

  groundPolicyActionsJoint(tree, policy, witness);

  const auto leaves = policy.leaves();
  const auto leaf = get_leaf_for_world(w, leaves);
  const auto branch = tree.get_branch(leaf);
  const auto vars0 = branch.get_vars0({0.0, branch.n_nodes() - 1}, tree._get_branch(leaf), config_.microSteps_);

  frames.resize(vars0.d0 + k_order_);
  frames(0).copy(*startKinematic, true);

  // copy prefix
  for(auto s{1}; s < k_order_; ++s)
  {
    frames(s).copy(frames(s - 1), true);
  }

  // copy frames and apply q
  for(auto s{0}; s < vars0.d0; ++s)
  {
    auto& K = frames(s + k_order_); // target frame
    K.copy(frames(s + k_order_ - 1), true); // copy preceeding one
    const auto global = vars0(s);

    //apply potential graph switches
    for(auto *sw:witness->switches)
    {
      if(sw->timeOfApplication == global)
      {
        sw->apply(K);
      }
    }

    const auto qDim = K.getJointStateDimension();
    CHECK(qDim != 0, "dimension corruption!");
    CHECK(qDim == X.stepToQDim(global), "dimension corruption!");

    const auto x_start{X.GetGlobalIndex(global)};
    const auto q = X.x({x_start, x_start + qDim - 1});

    K.setJointState(q);
  }

  return frames;
}

}
