#include <mcts_planner_bs.h>

#include <algorithm>    // std::random_shuffle
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <belief_state.h>
#include <utils.h>
#include <stack>

static double m_inf() { return std::numeric_limits< double >::lowest(); }

namespace matp
{
namespace{
double getPolicyNodePriority( const MCTSDecisionTree::GraphNodeType::ptr& node, Values& values )
{
  if( ! node->data().isPotentialSymbolicSolution )
  {
    return std::numeric_limits<double>::lowest();
  }

  return values.getOrDefault( node->id() );
}
}

void MCTSPlannerBs::setFol( const std::string & descrition )
{
  if( ! boost::filesystem::exists( descrition ) ) throw FolFileNotFound();

  parser_.parse( descrition );
  tree_ = MCTSDecisionTreeBs( parser_.engine(), parser_.possibleStartStates(), parser_.egoBeliefState() );
}

void MCTSPlannerBs::solve()
{
  if( tree_.empty() )
  {
    expandMCTS();
  }

  valueIteration();

  buildPolicy();
}

void MCTSPlannerBs::integrate( const Policy & policy )
{
  std::queue< Policy::GraphNodeTypePtr > Q;
  Q.push( policy.root() );

  while( ! Q.empty() )
  {
    auto n = Q.front();
    Q.pop();

    // find parents in tree_
    const auto& decisionTreeN = decisionTreeNodes_.at( n->data().decisionGraphNodeId ).lock();

    if( !decisionTreeN->parents().empty() )
    {
      auto decisionTreeM = decisionTreeN->parents().front().lock();

      rewards_.set( decisionTreeM->id(), decisionTreeN->id(), n->data().markovianReturn );
    }

    // push children on queue
    for( const auto & c : n->children() )
    {
      Q.push( c );
    }
  }
}

// getters
bool MCTSPlannerBs::terminated() const
{
  return false;
}

Policy MCTSPlannerBs::getPolicy() const
{
  return policy_;
}

void MCTSPlannerBs::expandMCTS()
{
  tree_.expandMCTS( rewards_.R0(),
                    nIterMin_,
                    nIterMax_,
                    rollOutMaxSteps_,
                    nRollOutsPerSimulation_,
                    explorationTermC_,
                    verbose_ );
}

void MCTSPlannerBs::buildPolicy()
{
  std::cout << "MCTSPlannerBs::buildPolicy.." << std::endl;

  using NodeTypePtr = std::shared_ptr< MCTSDecisionTree::GraphNodeType >;

  std::stack< std::pair< NodeTypePtr, Policy::GraphNodeTypePtr > > Q;

  // create policy root node from decision graph node
  const auto& root = tree_.root();
  PolicyNodeData rootData;
  rootData.beliefState = tree_.beliefStates_.at( root->data().beliefState_h );

  const auto& policyRoot = GraphNode< PolicyNodeData >::root( rootData );

  Q.push( std::make_pair( tree_.root(), policyRoot ) );

  while( ! Q.empty() )
  {
    auto uPair = Q.top();
    Q.pop();

    const auto& u     = uPair.first;
    const auto& uSke = uPair.second;

    decisionTreeNodes_[ u->id() ] = u; // cache the node ptr for random access when integrating policy

    if( u->children().empty() )
    {
      CHECK(u->data().terminal, "final policy node should be terminal!");
      uSke->data().terminal = u->data().terminal;
      continue;
    }

    const auto v = *std::max_element( u->children().cbegin(), u->children().cend(), [this, &u]( const auto& lhs, const auto& rhs )
    {
      return rewards_.get(u->id(), lhs->id()) + getPolicyNodePriority(lhs, values_) < rewards_.get(u->id(), rhs->id()) + getPolicyNodePriority(rhs, values_);
    });

    decisionTreeNodes_[ v->id() ] = v; // cache the node ptr for random access when integrating policy

    PolicyNodeData data;
    data.beliefState = tree_.beliefStates_.at( v->data().beliefState_h );
    data.markovianReturn = rewards_.get( u->id(), v->id() );
    data.decisionGraphNodeId = v->id();
    data.leadingKomoArgs = decisionArtifactToKomoArgs( tree_.actions_.at( v->data().leadingAction_h ) );
    if(!u->parents().empty() && !u->parent()->parents().empty())
    {
      //CHECK( u->parent()->data().nodeType == MCTSNodeData::NodeType::ACTION, "wrong node type!" );
      data.leadingObservation = getObservation( u->parent()->parent(), u, tree_ );
    }
    data.p = transitionProbability( uSke->data().beliefState, data.beliefState );
    auto vSke = uSke->makeChild( data );
    //std::cout << "build ske from " << uSke->id() << "(" << u->id() << ") to " << vSke->id()<< " (" << v->id() << ") , p = " << data.p << std::endl;

    if( v->children().empty() )
    {
      CHECK(v->data().terminal, "final policy node should be terminal!");
      uSke->data().terminal = v->data().terminal;
    }

    for( const auto& w : v->children() ) // skip obs nodes
    {
      Q.push( std::make_pair( w, vSke ) );
    }
  }

  policy_ = Policy( policyRoot );

  policy_.setValue( values_.get( 0 ) );
  //CHECK(policy_.value() > std::numeric_limits<double>::lowest(), "extracted policy seems to be infeasible (infinite costs)!");

  std::cout << "MCTSPlannerBs::buildPolicy.. end (value=" << policy_.value() << ")" << std::endl;
}

void MCTSPlannerBs::valueIteration()
{
  values_ = ValueIterationOnTreeAlgorithm::process( tree_, rewards_ );
}

}
