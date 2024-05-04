#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>

namespace frontier_exploration
{
/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx,
                                 const costmap_2d::Costmap2D& costmap)
{
  // get 4-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    ROS_WARN("Evaluating nhood for offmap point");
    return out;
  }
  // 左邻域：如果 idx 不在最左边的列（idx % size_x_ > 0），则 idx - 1 为左邻域索引，加入 out。
  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  // 右邻域：如果 idx 不在最右边的列（idx % size_x_ < size_x_ - 1），则 idx + 1 为右邻域索引，加入 out
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  // 上邻域：如果 idx 不在最顶部的行（即 idx 至少为 size_x_），则 idx - size_x_ 为上邻域索引，加入 out。
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  // 下邻域：如果 idx 不在最底部的行（即 idx 小于 size_x_ * (size_y_ - 1)），则 idx + size_x_ 为下邻域索引，加入 out。
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx,
                                 const costmap_2d::Costmap2D& costmap)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }
  // 左上对角：如果 idx 不在最左列且不在最顶行，那么 idx - 1 - size_x_ 为左上邻域索引，加入 out。
  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  // 左下对角：如果 idx 不在最左列且不在最底行，那么 idx - 1 + size_x_ 为左下邻域索引，加入 out。
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  // 右上对角：如果 idx 不在最右列且不在最顶行，那么 idx + 1 - size_x_ 为右上邻域索引，加入 out。
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  // 右下对角：如果 idx 不在最右列且不在最底行，那么 idx + 1 + size_x_ 为右下邻域索引，加入 out。
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val,
                 const costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] == val) {
      result = idx;
      return true;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}
}
#endif
