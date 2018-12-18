#include "cartesian3dgrid/cartesian3dgrid.h"

#include <iostream>

#include <cnpy.h>


//--------------------- Output for Numpy -----------------------------------------

int Grid3D::writeGridNpy(const char filename[]) const
{
  cnpy::npy_save(std::string(filename),
                 &data_array_[0],
                 {size_[2], size_[1], size_[0]}, "w");
}
