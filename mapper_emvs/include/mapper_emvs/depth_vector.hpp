#pragma once

#include <vector>
#include <glog/logging.h>

namespace EMVS
{

// Stores a set of data associations index <-> depth
// Used to abstract how the depth is sampled

// Uses the "Curiously Recursive Template Pattern" to allow static polymorphism
// for performance (since these functions will be called very often)

template<class DerivedDepthVector>
class DepthVector {
public:
  DerivedDepthVector& derived()
  { return static_cast<DerivedDepthVector&>(*this); }

  DepthVector() {}

  DepthVector(float min_depth,
              float max_depth,
              size_t num_depth_cells)
  {
    CHECK_GT(min_depth, 0.0);
    CHECK_GT(max_depth, 0.0);
    CHECK_GE(num_depth_cells, 1u);

    num_depth_cells_ = num_depth_cells;
    min_depth_ = min_depth;
    max_depth_ = max_depth;
    if(min_depth_ > max_depth_)
      std::swap(min_depth_, max_depth_);

    derived().init();
  }

  size_t size() const { return vec_.size(); }

  float cellIndexToDepth(size_t i)
  {
    return derived().cellIndexToDepth(i);
  }

  size_t depthToCellIndex(float depth)
  {
    return derived().depthToCellIndex(depth);
  }

  float depthToCell(float depth)
  {
    return derived().depthToCell(depth);
  }

  std::vector<float> getDepthVector()
  {
    std::vector<float> out;
    for(size_t i=0; i<num_depth_cells_; ++i)
    {
      out.push_back(cellIndexToDepth(i));
    }
    return out;
  }

protected:
  std::vector<float> vec_;
  size_t num_depth_cells_;
  float min_depth_;
  float max_depth_;
  float depth_to_cell_idx_multiplier_;
};


class LinearDepthVector : public DepthVector<LinearDepthVector>
{
public:

  LinearDepthVector() : DepthVector() {}

  LinearDepthVector(float min_depth,
                    float max_depth,
                    size_t num_depth_cells) : DepthVector(min_depth,
                                                          max_depth,
                                                          num_depth_cells) {}

  void init()
  {
    LOG(INFO) << "Using linear spacing in depth";
    depth_to_cell_idx_multiplier_ = (float)((num_depth_cells_) / (max_depth_-min_depth_));

    vec_.resize(num_depth_cells_);
    for(size_t i=0; i<num_depth_cells_; ++i)
    {
      vec_[i] = min_depth_ + (float)i / depth_to_cell_idx_multiplier_;
    }
  }

  float cellIndexToDepth(size_t i)
  {
    return vec_[i];
  }

  size_t depthToCellIndex(float depth)
  {
    return (size_t) ((depth-min_depth_) * depth_to_cell_idx_multiplier_ + 0.5);
  }

  float depthToCell(float depth)
  {
    return (depth-min_depth_) * depth_to_cell_idx_multiplier_;
  }
};




class InverseDepthVector : public DepthVector<InverseDepthVector>
{
public:

  InverseDepthVector() : DepthVector() {}

  InverseDepthVector(float min_depth,
                     float max_depth,
                     size_t num_depth_cells) : DepthVector(min_depth,
                                                           max_depth,
                                                           num_depth_cells) {}

  void init()
  {
    LOG(INFO) << "Using linear spacing in inverse depth";
    inv_min_depth_ = 1.f / min_depth_;
    inv_max_depth_ = 1.f / max_depth_;
    depth_to_cell_idx_multiplier_ = (float)((num_depth_cells_) / (inv_min_depth_ - inv_max_depth_));

    vec_.resize(num_depth_cells_);
    for(size_t i=0; i<num_depth_cells_; ++i)
    {
      vec_[i] = inv_max_depth_ + (float)i / depth_to_cell_idx_multiplier_;
    }
  }

  float cellIndexToDepth(size_t i)
  {
    return 1.f/vec_[i];
  }

  size_t depthToCellIndex(float depth)
  {
    return (size_t) ((1.f/depth-inv_max_depth_) * depth_to_cell_idx_multiplier_ + 0.5);
  }

  float depthToCell(float depth)
  {
    return (1.f/depth-inv_max_depth_) * depth_to_cell_idx_multiplier_;
  }

private:
  float inv_min_depth_;
  float inv_max_depth_;
};



} // namespace EMVS
