#include "../include/cartesian3dgrid/cartesian3dgrid.h"

#include <iostream>
#include <vector>


Grid3D::Grid3D()
{
  deallocate();
}


Grid3D::Grid3D(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ)
{
  deallocate();
  allocate(dimX,dimY,dimZ);
}


void Grid3D::allocate(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ)
{
  size_[0] = dimX;
  size_[1] = dimY;
  size_[2] = dimZ;
  numCells_ = size_[0]*size_[1]*size_[2];
  data_array_.resize(numCells_);
  resetGrid();
}


Grid3D::~Grid3D()
{
  deallocate();
}


void Grid3D::deallocate()
{
  size_[0] = size_[1] = size_[2] = 0;
  data_array_.clear();
}


void Grid3D::printInfo() const
{
  std::cout << "Grid3D Dimensions: " << "(" << size_[0] << "," << size_[1] << "," << size_[2] << ")" << std::endl;
  std::cout << "Grid3D Data_array size: " << data_array_.size() << std::endl;
}

void Grid3D::resetGrid()
{
  std::fill(data_array_.begin(), data_array_.end(), 0.f);
}


void Grid3D::collapseMaxZSlice(cv::Mat* max_val, cv::Mat* max_pos_idx) const
{
  // Z-slice
  unsigned int u_size = size_[0], 
               v_size = size_[1],
               w_size = size_[2];
  
  *max_val = cv::Mat(v_size,u_size, CV_32FC1); // (y,x) as in images
  *max_pos_idx = cv::Mat(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  std::vector<float> grid_vals_vec(w_size);
  for(unsigned int v=0; v<v_size; v++)
  {
    for(unsigned int u=0; u<u_size; u++)
    {
      // Build vector containing the grid values at a given (u,v), as a function of depth
      for(unsigned int k=0; k<w_size; ++k)
        grid_vals_vec.at(k) = getGridValueAt(u,v,k);
      
      // Look for maximum of vector and its location
      auto max = std::max_element(std::begin(grid_vals_vec), std::end(grid_vals_vec));
      (*max_val).at<float>(v,u) = *max;
      (*max_pos_idx).at<uchar>(v,u) = std::distance(std::begin(grid_vals_vec), max);
    }
  }
}


double Grid3D::computeMeanSquare() const
{
  double result = 0.;
  for (int i=0; i<numCells_; i++)
  {
    double tmp = (double) data_array_.at(i);
    result += tmp*tmp;
  }
  
  return result/numCells_;
}
