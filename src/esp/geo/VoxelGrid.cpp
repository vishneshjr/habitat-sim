
#include <assert.h>
#include <limits.h>
#include <cmath>

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/Primitives/Cube.h>

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
VoxelGrid::VoxelGrid(const std::unique_ptr<assets::MeshData>& meshData,
                     const std::string& renderAssetHandle,
                     int resolution)
    : m_renderAssetHandle(renderAssetHandle) {
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
  /*VHACD::IVHACD::Parameters params;
  params.m_resolution = resolution;
  params.m_oclAcceleration = false;*/
  Mn::Debug() << "Voxelizing mesh..";

  // run VHACD
  interfaceVHACD->computeVoxelField(&meshData->vbo[0][0], meshData->vbo.size(),
                                    &meshData->ibo[0], meshData->ibo.size() / 3,
                                    resolution);

  // get VHACD volume, set scale and dimensions
  VHACD::Volume* vhacdVolume = interfaceVHACD->getVoxelField();
  double scale = vhacdVolume->getScale();
  m_voxelSize = Mn::Vector3(scale, scale, scale);
  const size_t* dims = vhacdVolume->getDimensions();
  m_voxelGridDimensions = Mn::Vector3i(dims[0], dims[1], dims[2]);

  VHACD::Vec3<double> center = vhacdVolume->getCenter();

  // VHACD computes a axis-aligned bounding box; we need to offset the voxelgrid
  // by the minimum corner of the AABB
  VHACD::Vec3<double> minBB = vhacdVolume->getMinBB();
  m_offset = Mn::Vector3(minBB[0], minBB[1], minBB[2]);
  m_BBMaxOffset = m_offset + Mn::Vector3(m_voxelGridDimensions) * scale;

  // create empty VoxelGrid
  Corrade::Containers::Array<char> cr_grid{Corrade::Containers::ValueInit,
                                           gridSize() * sizeof(bool)};
  grids_.insert(
      std::make_pair("Boundary", std::make_pair("bool", std::move(cr_grid))));

  int num_filled = 0;
  // Transfer data from Volume to VoxelGrid
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        if (vhacdVolume->GetVoxel(i, j, k) >= 2) {
          num_filled++;
          setVoxel(Mn::Vector3i(i, j, k), "Boundary", true);
        } else {
          setVoxel(Mn::Vector3i(i, j, k), "Boundary", false);
        }
      }
    }
  }
}
#endif

VoxelGrid::VoxelGrid(const Mn::Vector3& voxelSize,
                     const Mn::Vector3i& voxelGridDimensions) {
  m_voxelSize = voxelSize;
  m_voxelGridDimensions = voxelGridDimensions;
  m_offset = Mn::Vector3(0.0, 0.0, 0.0);
  Corrade::Containers::Array<char> cr_grid{Corrade::Containers::ValueInit,
                                           gridSize() * sizeof(bool)};
  grids_.insert(
      std::make_pair("Boundary", std::make_pair("bool", std::move(cr_grid))));
}

// Currently naive hashing. TODO: Replace with Magnum container and
// StridedArrayView3D.
int VoxelGrid::hashVoxelIndex(const Mn::Vector3i& coords) {
  assert(isValidIndex(coords));
  int hashed_voxel =
      coords[2] + coords[1] * m_voxelGridDimensions[2] +
      coords[0] * m_voxelGridDimensions[2] * m_voxelGridDimensions[1];
  return hashed_voxel;
}

Mn::Vector3i VoxelGrid::reverseHash(const int hash) {
  assert(hash < gridSize());
  return Mn::Vector3i(
      hash % m_voxelGridDimensions[2],
      hash / m_voxelGridDimensions[2] % m_voxelGridDimensions[1],
      hash / m_voxelGridDimensions[2] / m_voxelGridDimensions[1]);
}

//  --== GETTERS AND SETTERS FOR VOXELS ==--

std::shared_ptr<Mn::Trade::MeshData> VoxelGrid::getMeshData(
    const std::string& gridName) {
  if (meshDataDict_[gridName] == nullptr)
    generateMesh(gridName);
  return meshDataDict_[gridName];
}

Mn::GL::Mesh& VoxelGrid::getMeshGL(const std::string& gridName) {
  if (meshDataDict_[gridName] == nullptr)
    generateMesh(gridName);
  return meshGLDict_[gridName];
}

Mn::Vector3 VoxelGrid::getGlobalCoords(const Mn::Vector3i& coords) {
  Mn::Vector3 global_coords((coords[0]) * m_voxelSize[0],
                            (coords[1]) * m_voxelSize[1],
                            (coords[2]) * m_voxelSize[2]);
  global_coords += m_offset;
  return global_coords;
}

int VoxelGrid::generateBoolGridFromIntGrid(const std::string& intGridName,
                                           const std::string& boolGridName,
                                           int startRange,
                                           int endRange) {
  assert(grids_.find(intGridName) != grids_.end());
  assert(grids_[intGridName].first == "int");
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto intGrid = getGrid<int>(intGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (intGrid[i][j][k] >= startRange && intGrid[i][j][k] <= endRange) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }
  return num_filled;
}

int VoxelGrid::generateBoolGridFromVector3Grid(
    const std::string& vector3GridName,
    const std::string& boolGridName,
    bool func(Mn::Vector3)) {
  assert(grids_.find(vector3GridName) != grids_.end());
  assert(grids_[vector3GridName].first == "vector3");
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto vecGrid = getGrid<Mn::Vector3>(vector3GridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(vecGrid[i][j][k])) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }
  return num_filled;
}

int VoxelGrid::generateBoolGridFromFloatGrid(const std::string& floatGridName,
                                             const std::string& boolGridName,
                                             float startRange,
                                             float endRange) {
  assert(grids_.find(floatGridName) != grids_.end());
  assert(grids_[floatGridName].first == "float");
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto floatGrid = getGrid<float>(floatGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (floatGrid[i][j][k] >= startRange &&
            floatGrid[i][j][k] <= endRange) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }

  return num_filled;
}

void VoxelGrid::fillVoxelSetFromBoolGrid(std::vector<Mn::Vector3i>& voxelSet,
                                         const std::string& boolGridName,
                                         bool (*func)(bool)) {
  assert(grids_.find(boolGridName) != grids_.end());
  assert(grids_[boolGridName].first == "bool");
  auto boolGrid = getGrid<bool>(boolGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(boolGrid[i][j][k])) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
}

void VoxelGrid::fillVoxelSetFromIntGrid(std::vector<Mn::Vector3i>& voxelSet,
                                        const std::string& intGridName,
                                        bool (*func)(int)) {
  assert(grids_.find(intGridName) != grids_.end());
  assert(grids_[intGridName].first == "int");
  auto intGrid = getGrid<int>(intGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(intGrid[i][j][k])) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
}

void VoxelGrid::fillVoxelSetFromFloatGrid(std::vector<Mn::Vector3i>& voxelSet,
                                          const std::string& floatGridName,
                                          bool (*func)(float)) {
  assert(grids_.find(floatGridName) != grids_.end());
  assert(grids_[floatGridName].first == "float");
  auto floatGrid = getGrid<float>(floatGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(floatGrid[i][j][k])) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
}

void VoxelGrid::fillVoxelSetFromVector3Grid(std::vector<Mn::Vector3i>& voxelSet,
                                            const std::string& vector3GridName,
                                            bool (*func)(Mn::Vector3)) {
  assert(grids_.find(vector3GridName) != grids_.end());
  assert(grids_[vector3GridName].first == "vector3");
  auto vecGrid = getGrid<Mn::Vector3>(vector3GridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(vecGrid[i][j][k])) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
}

// 6D SDF - labels each cell as interior (-inf), exterior (+inf), or boundary
void VoxelGrid::generateInteriorExteriorVoxelGrid() {
  // create 6 bool grids
  addGrid<bool>("negXShadow");
  addGrid<bool>("posXShadow");
  addGrid<bool>("negYShadow");
  addGrid<bool>("posYShadow");
  addGrid<bool>("negZShadow");
  addGrid<bool>("posZShadow");
  // fill each grids with ray cast
  bool hit = false;
  int ind = 0;
  std::string gridName = "InteriorExterior";
  // X axis ray casts
  for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
    for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[0] - 1;
      while (ind--) {
        if (hit) {
          setVoxel(Mn::Vector3i(ind, j, k), "negXShadow", true);
          // setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "negXShadow");
        } else if (getVoxel<bool>(Mn::Vector3i(ind, j, k), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(ind, j, k), "negXShadow", true);
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[0]) {
        if (hit) {
          setVoxel(Mn::Vector3i(ind, j, k), "posXShadow", true);
          // setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "negXShadow");
        } else if (getVoxel<bool>(Mn::Vector3i(ind, j, k), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(ind, j, k), "posXShadow", true);
        }
      }
    }
  }
  // Y axis ray casts
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[1] - 1;
      while (ind--) {
        if (hit) {
          setVoxel(Mn::Vector3i(i, ind, k), "negYShadow", true);
        } else if (getVoxel<bool>(Mn::Vector3i(i, ind, k), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(i, ind, k), "negYShadow", true);
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[1]) {
        if (hit) {
          setVoxel(Mn::Vector3i(i, ind, k), "posYShadow", true);
        } else if (getVoxel<bool>(Mn::Vector3i(i, ind, k), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(i, ind, k), "posYShadow", true);
        }
      }
    }
  }
  // Z axis ray casts
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[2] - 1;
      while (ind--) {
        if (hit) {
          setVoxel(Mn::Vector3i(i, j, ind), "negZShadow", true);
        } else if (getVoxel<bool>(Mn::Vector3i(i, j, ind), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(i, j, ind), "negZShadow", true);
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[2]) {
        if (hit) {
          setVoxel(Mn::Vector3i(i, j, ind), "posZShadow", true);
        } else if (getVoxel<bool>(Mn::Vector3i(i, j, ind), "Boundary")) {
          hit = true;
          setVoxel(Mn::Vector3i(i, j, ind), "posZShadow", true);
        }
      }
    }
  }

  // create int grid
  addGrid<int>(gridName);
  bool nX = false, pX = false, nY = false, pY = false, nZ = false, pZ = false;
  int hash = 0;
  // fill in int grid with voting approach
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        if (getVoxel<bool>(index, "Boundary")) {
          setVoxel(index, gridName, 0);
          continue;
        }
        nX = !getVoxel<bool>(index, "negXShadow");
        pX = !getVoxel<bool>(index, "posXShadow");
        nY = !getVoxel<bool>(index, "negYShadow");
        pY = !getVoxel<bool>(index, "posYShadow");
        nZ = !getVoxel<bool>(index, "negZShadow");
        pZ = !getVoxel<bool>(index, "posZShadow");
        // || ((nX || pX) && (nY || pY) && (nZ || pZ))
        if (((nX && pX) || (nY && pY) || (nZ && pZ)) ||
            ((nX || pX) && (nY || pY) && (nZ || pZ))) {
          // Exterior (+inf)
          setVoxel(index, gridName, INT_MAX);
        } else {
          // Interior (-inf)
          setVoxel(index, gridName, INT_MIN);
        }
      }
    }
  }
  // remove the transient utility grids
  removeGrid("negXShadow");
  removeGrid("posXShadow");
  removeGrid("negYShadow");
  removeGrid("posYShadow");
  removeGrid("negZShadow");
  removeGrid("posZShadow");
}

// Manhattan distance SDF - starting from the interior exterior voxel grid,
// computes SDF in terms of manhattan distance with double sweep approach
void VoxelGrid::generateManhattanDistanceSDF(const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (grids_.find("InteriorExterior") == grids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new intGrid and copy data from interior/exterior grid
  addGrid<int>(gridName);
  auto intExtGrid = getGrid<int>("InteriorExterior");
  auto sdfGrid = getGrid<int>(gridName);

  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        // copy value
        sdfGrid[i][j][k] = intExtGrid[i][j][k];
      }
    }
  }

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        int i_behind = INT_MAX, j_behind = INT_MAX, k_behind = INT_MAX;
        if (isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = abs(std::max(sdfGrid[i - 1][j][k], -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = abs(std::max(sdfGrid[i][j - 1][k], -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = abs(std::max(sdfGrid[i][j][k - 1], -2147483646));
        }
        int curVal = sdfGrid[i][j][k];
        int closest = 0;
        if (i_behind <= j_behind && i_behind <= k_behind) {
          // i_behind is closest to nearest obstacle.
          closest = i_behind;
        } else if (j_behind <= i_behind && j_behind <= k_behind) {
          // j_behind is closest to nearest obstacle.
          closest = j_behind;
        } else {
          // k_behind is closest or tied for closest to nearest obstacle.
          closest = k_behind;
        }
        // Get the minimum of the cell that's closest to an obstacle and the
        // current distance to an obstacle, and multiply by the true sign of
        // curVal
        if (closest == INT_MAX)
          closest--;
        curVal = ((curVal > 0) - (curVal < 0)) *
                 std::min(abs(std::max(curVal, -2147483647)), closest + 1);
        sdfGrid[i][j][k] = curVal;
      }
    }
  }
  // second sweep
  for (int i = m_voxelGridDimensions[0] - 1; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1] - 1; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2] - 1; k >= 0; k--) {
        int curVal = getVoxel<int>(Mn::Vector3i(i, j, k), gridName);
        if (curVal == 0)
          continue;
        int i_ahead = INT_MAX, j_ahead = INT_MAX, k_ahead = INT_MAX;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = abs(std::max(sdfGrid[i + 1][j][k], -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = abs(std::max(sdfGrid[i][j + 1][k], -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = abs(std::max(sdfGrid[i][j][k + 1], -2147483646));
        }

        int closest = INT_MAX - 1;
        if (i_ahead <= j_ahead && i_ahead <= k_ahead) {
          // i_ahead is closest to nearest obstacle.
          closest = i_ahead;
        } else if (j_ahead <= i_ahead && j_ahead <= k_ahead) {
          // j_ahead is closest to nearest obstacle.
          closest = j_ahead;
        } else {
          // k_ahead is closest or tied for closest to nearest obstacle.
          closest = k_ahead;
        }
        // Get the minimum of the cell that's closest to an obstacle and the
        // current distance to an obstacle, and multiply by the true sign of
        // curVal
        if (closest == INT_MAX)
          closest--;
        curVal = ((curVal > 0) - (curVal < 0)) *
                 std::min(abs(std::max(curVal, -2147483647)), closest + 1);
        sdfGrid[i][j][k] = curVal;
      }
    }
  }
}

void VoxelGrid::generateEuclideanDistanceSDF(const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (grids_.find("InteriorExterior") == grids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new vector3Grid and fill data from interior/exterior grid
  addGrid<Mn::Vector3>("ClosestBoundaryCell");
  auto intExtGrid = getGrid<int>("InteriorExterior");
  auto closestCellGrid = getGrid<Mn::Vector3>("ClosestBoundaryCell");

  Mn::Debug() << m_voxelGridDimensions;
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        int label = getVoxel<int>(index, "InteriorExterior");
        if (label == 0) {
          closestCellGrid[i][j][k] = Mn::Vector3(i, j, k);
        } else {
          // intializing the closest boundary cell to be very far / invalid, so
          // it is ensured to be overwritten in the SDF calculation sweeps.
          closestCellGrid[i][j][k] = Mn::Vector3(m_voxelGridDimensions) * 2;
        }
      }
    }
  }

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3 i_behind = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_behind = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_behind = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = closestCellGrid[i - 1][j][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = closestCellGrid[i][j - 1][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = closestCellGrid[i][j][k - 1];
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (closestCellGrid[i][j][k] - coords).length();

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_behind - coords).length();
        j_dist = (j_behind - coords).length();
        k_dist = (k_behind - coords).length();

        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          closestCellGrid[i][j][k] = i_behind;
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          closestCellGrid[i][j][k] = j_behind;
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          closestCellGrid[i][j][k] = k_behind;
        }
      }
    }
  }
  // create float grid for distances, will be filled in this sweep.
  addGrid<float>(gridName);
  auto sdfGrid = getGrid<float>(gridName);
  // second sweep
  for (int i = m_voxelGridDimensions[0] - 1; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1] - 1; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2] - 1; k >= 0; k--) {
        Mn::Vector3 i_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_ahead = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = closestCellGrid[i + 1][j][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = closestCellGrid[i][j + 1][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = closestCellGrid[i][j][k + 1];
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (closestCellGrid[i][j][k] - coords).length();
        // get whether the coord is considered interior or exterior
        int intOrExtVal = intExtGrid[i][j][k];
        int intOrExtSign = (intOrExtVal > 0) - (intOrExtVal < 0);

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_ahead - coords).length();
        j_dist = (j_ahead - coords).length();
        k_dist = (k_ahead - coords).length();
        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          closestCellGrid[i][j][k] = i_ahead;
          sdfGrid[i][j][k] = intOrExtSign * i_dist;
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          closestCellGrid[i][j][k] = j_ahead;
          sdfGrid[i][j][k] = intOrExtSign * j_dist;
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          closestCellGrid[i][j][k] = k_ahead;
          sdfGrid[i][j][k] = intOrExtSign * k_dist;
        } else {
          sdfGrid[i][j][k] = intOrExtSign * cur_dist;
        }
      }
    }
  }
}

void VoxelGrid::generateDistanceFlowField(const std::string& gridName) {
  // generateEuclideanDistanceSDF();
  addGrid<Mn::Vector3>(gridName);
  auto flowGrid = getGrid<Mn::Vector3>(gridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        flowGrid[i][j][k] = Mn::Vector3(index) -
                            getVoxel<Mn::Vector3>(index, "ClosestBoundaryCell");
      }
    }
  }
}

void VoxelGrid::addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                         std::vector<Mn::Vector3>& normals,
                                         std::vector<Mn::Color3>& colors,
                                         std::vector<Mn::UnsignedInt>& indices,
                                         const Mn::Vector3i& local_coords) {
  // Using the data of a cubeSolid to create the voxel cube
  Mn::Trade::MeshData cubeData = Mn::Primitives::cubeSolid();

  // add cube to mesh
  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(local_coords);

  auto cubePositions = cubeData.positions3DAsArray();
  auto cubeNormals = cubeData.normalsAsArray();

  for (int i = 0; i < 24; i++) {
    Mn::Vector3 vertOffset = cubePositions[i] * m_voxelSize / 2;
    positions.push_back(vertOffset + mid);
    // Set the normals to be weighted such that cubes look slightly curved
    normals.push_back(cubePositions[i].normalized() * 1 / 4 +
                      cubeNormals[i].normalized() * 3 / 4);
    colors.push_back(Mn::Color3(.4, .8, 1));
  }
  // cube faces
  unsigned int sz = positions.size() - 24;
  auto cubeIndices = cubeData.indices();
  for (int i = 0; i < 36; i++) {
    indices.push_back(sz + cubeIndices[i][0]);
  }
}

void VoxelGrid::addVectorToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                          std::vector<Mn::Vector3>& normals,
                                          std::vector<Mn::Color3>& colors,
                                          std::vector<Mn::UnsignedInt>& indices,
                                          const Mn::Vector3i& local_coords,
                                          const Mn::Vector3& vec) {
  Mn::Vector3 mid = getGlobalCoords(local_coords);
  Mn::Vector3 pos1 = vec.normalized() * m_voxelSize * 1 / 2 + mid;
  Mn::Vector3 orthog1 = Mn::Math::cross(vec, Mn::Vector3(0, 1, 0));
  if (orthog1 == Mn::Vector3(0, 0, 0)) {
    orthog1 = Mn::Vector3(1, 0, 0);
  }
  Mn::Vector3 orthog2 = Mn::Math::cross(vec, orthog1);

  Mn::Vector3 pos2 = mid + orthog1.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos3 = mid + orthog2.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos4 = mid - orthog1.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos5 = mid - orthog2.normalized() * m_voxelSize * 1 / 20;

  positions.push_back(pos1);
  positions.push_back(pos2);
  positions.push_back(pos3);
  positions.push_back(pos4);
  positions.push_back(pos5);

  colors.push_back(Mn::Color3(1, 1, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));

  normals.push_back(vec.normalized());
  normals.push_back((pos1 - mid).normalized());
  normals.push_back((pos2 - mid).normalized());
  normals.push_back((pos3 - mid).normalized());
  normals.push_back((pos4 - mid).normalized());

  unsigned int sz = positions.size() - 5;
  indices.push_back(sz);
  indices.push_back(sz + 1);
  indices.push_back(sz + 2);

  indices.push_back(sz);
  indices.push_back(sz + 2);
  indices.push_back(sz + 3);

  indices.push_back(sz);
  indices.push_back(sz + 3);
  indices.push_back(sz + 4);

  indices.push_back(sz);
  indices.push_back(sz + 4);
  indices.push_back(sz + 1);

  indices.push_back(sz);
  indices.push_back(sz + 4);
  indices.push_back(sz + 1);

  indices.push_back(sz + 1);
  indices.push_back(sz + 2);
  indices.push_back(sz + 3);

  indices.push_back(sz + 1);
  indices.push_back(sz + 3);
  indices.push_back(sz + 4);
}

void VoxelGrid::generateMesh(const std::string& gridName, bool isVectorField) {
  assert(grids_.find(gridName) != grids_.end());
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;
  std::vector<Mn::Vector3> normals;
  std::vector<Mn::Color3> colors;
  int num_filled = 0;

  // iterate through each voxel grid cell
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        if (isVectorField) {
          Mn::Vector3 vec = getVoxel<Mn::Vector3>(local_coords, gridName);
          if (vec != Mn::Vector3(0, 0, 0))
            addVectorToMeshPrimitives(positions, normals, colors, indices,
                                      local_coords, vec);
        } else {
          bool val = getVoxel<bool>(local_coords, gridName);
          if (val) {
            num_filled++;
            addVoxelToMeshPrimitives(positions, normals, colors, indices,
                                     local_coords);
          }
        }
      }
    }
  }

  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
  if (meshDataDict_.find(gridName) != meshDataDict_.end()) {
    Mn::Trade::MeshData positionMeshData{
        Mn::MeshPrimitive::Triangles,
        {},
        indices,
        Mn::Trade::MeshIndexData{indices},
        {},
        positions,
        {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                      Cr::Containers::arrayView(positions)}}};
    meshDataDict_[gridName] =
        std::make_shared<Mn::Trade::MeshData>(Mn::MeshTools::interleave(
            positionMeshData,
            {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Color,
                                          Cr::Containers::arrayView(colors)},
             Mn::Trade::MeshAttributeData{
                 Mn::Trade::MeshAttribute::Normal,
                 Cr::Containers::arrayView(normals)}}));
  } else {
    Mn::Trade::MeshData positionMeshData{
        Mn::MeshPrimitive::Triangles,
        {},
        indices,
        Mn::Trade::MeshIndexData{indices},
        {},
        positions,
        {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                      Cr::Containers::arrayView(positions)}}};
    meshDataDict_.insert(std::make_pair(
        gridName,
        std::make_shared<Mn::Trade::MeshData>(Mn::MeshTools::interleave(
            positionMeshData,
            {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Color,
                                          Cr::Containers::arrayView(colors)},
             Mn::Trade::MeshAttributeData{
                 Mn::Trade::MeshAttribute::Normal,
                 Cr::Containers::arrayView(normals)}}))));
  }

  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
  if (meshGLDict_.find(gridName) != meshGLDict_.end()) {
    meshGLDict_[gridName] = Mn::MeshTools::compile(*meshDataDict_[gridName]);
  } else {
    meshGLDict_.insert(std::make_pair(
        gridName, Mn::MeshTools::compile(*meshDataDict_[gridName])));
  }
}

// --== SAVING AND LOADING VOXEL FIELDS ==--

bool VoxelGrid::saveToSVXFile(const std::string& filepath) {
  // iterate through all grids, calling helper function saveGridToSVXFile for
  // each grid's key
  for (auto const& grid : grids_) {
    std::string type = grid.second.first;
    if (type == "int") {
      saveGridToSVXFile<int>(
          grid.first, Cr::Utility::Directory::join(filepath, grid.first));
    } else if (type == "float") {
      saveGridToSVXFile<float>(
          grid.first, Cr::Utility::Directory::join(filepath, grid.first));
    } else if (type == "bool") {
      saveGridToSVXFile<bool>(
          grid.first, Cr::Utility::Directory::join(filepath, grid.first));
    } else if (type == "vector3") {
      saveGridToSVXFile<Mn::Vector3>(
          grid.first, Cr::Utility::Directory::join(filepath, grid.first));
    }
  }
  return true;
}

bool VoxelGrid::saveGridToSVXFile(const std::string& gridName) {
  assert(grids_.find(gridName) != grids_.end());
  std::string voxelizationNameBase =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::filename(m_renderAssetHandle))
          .first;
  std::string filepath =
      Cr::Utility::Directory::join(Corrade::Utility::Directory::current(),
                                   "data/VoxelGrids/") +
      voxelizationNameBase + "/";
  auto const& grid = grids_[gridName];
  std::string type = grid.first;
  if (type == "int") {
    saveGridToSVXFile<int>(gridName,
                           Cr::Utility::Directory::join(filepath, gridName));
  } else if (type == "float") {
    saveGridToSVXFile<float>(gridName,
                             Cr::Utility::Directory::join(filepath, gridName));
  } else if (type == "bool") {
    saveGridToSVXFile<bool>(gridName,
                            Cr::Utility::Directory::join(filepath, gridName));
  } else if (type == "vector3") {
    saveGridToSVXFile<Mn::Vector3>(
        gridName, Cr::Utility::Directory::join(filepath, gridName));
  }
  return true;
}

bool VoxelGrid::saveToSVXFile() {
  std::string voxelizationNameBase =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::filename(m_renderAssetHandle))
          .first;
  std::string filename =
      Cr::Utility::Directory::join(Corrade::Utility::Directory::current(),
                                   "data/VoxelGrids/") +
      voxelizationNameBase + "/";
  saveToSVXFile(filename);
  return true;
}

}  // namespace geo
}  // namespace esp