#pragma once

#include <array>
#include <cstdlib>
#include <memory>
#include <list>
#include <algorithm>
#include <iostream>

template<std::size_t N>
double norm2(const std::array<double, N> & a, const std::array<double, N> & b)
{
  double sqdist = 0;
  for(auto i = 0; i < N; ++i)
  {
    const auto delta = a[i] - b[i];
    sqdist += delta * delta;
  }

  return sqrt(sqdist);
}

template <std::size_t N>
struct KDTreeNode
{
  uint axis{0};
  double splitting_value;
  std::shared_ptr<KDTreeNode> left;
  std::shared_ptr<KDTreeNode> right;
  std::weak_ptr<KDTreeNode> parent;
  std::array<double, N> state;
  uint id{0};
};

template <std::size_t N>
class KDTree
{
public:
  KDTree(const std::array<double, N> & state)
  {
    root_ = std::make_shared<KDTreeNode<N>>();
    root_->splitting_value = state[0];
    root_->state = state;
  }

  void add_node(const std::array<double, N> & state, uint id)
  {
    auto current = root_;

    // go down the tree to find where to insert the node
    bool insert_left = false;
    bool insert_right = false;

    while(!insert_left && ! insert_right)
    {
      if(state[current->axis] < current->splitting_value)
      {
        // go left
        if(current->left)
          current = current->left;
        else
          insert_left = true;
      }
      else
      {
        // go right
        if(current->right)
          current = current->right;
        else
          insert_right = true;
      }
    }

    // insert
    auto node = std::make_shared<KDTreeNode<N>>();
    node->axis = (current->axis + 1) % N;
    node->splitting_value = state[node->axis];
    node->parent = current;
    node->state = state;
    node->id = id;

    if(insert_left) current->left = node;
    else current->right = node;
  }

  std::shared_ptr<KDTreeNode<N>> nearest_neighbor(const std::array<double, N> & state) const
  {
    double dmin = std::numeric_limits<double>::infinity();
    auto nearest = root_;

    nearest_neighbor(state, root_, nearest, dmin);

    return nearest;
  }

  void nearest_neighbor(const std::array<double, N> & state,
                        const std::shared_ptr<KDTreeNode<N>> from,
                        std::shared_ptr<KDTreeNode<N>> & nearest,
                        double & dmin) const
  {
    // check current
    const auto d = norm2(from->state, state);

    if(d < dmin)
    {
      nearest = from;
      dmin = d;
    }

    // go down
    if(state[from->axis] < from->splitting_value) // search first left
    {
      if(state[from->axis] - dmin < from->splitting_value && from->left)
      {
        nearest_neighbor(state, from->left, nearest, dmin);
      }

      if(state[from->axis] + dmin >= from->splitting_value && from->right)
      {
        nearest_neighbor(state, from->right, nearest, dmin);
      }
    }
    else // search first right
    {
      if(state[from->axis] - dmin < from->splitting_value && from->right)
      {
        nearest_neighbor(state, from->right, nearest, dmin);
      }

      if(state[from->axis] + dmin >= from->splitting_value && from->left)
      {
        nearest_neighbor(state, from->left, nearest, dmin);
      }
    }
  }

  std::shared_ptr<KDTreeNode<N>> root() const { return root_; }

  std::shared_ptr<KDTreeNode<N>> root_;
};