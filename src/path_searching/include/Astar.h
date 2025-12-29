#ifndef ASTAR_H_
#define ASTAR_H_
#include "ulitys.h"
namespace navi_planner
{
class Astar {
private:

  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;


  ESDF_enviroment::Ptr esdf_environment_;
  bool has_path_ = false;


  double lambda_heu_;
  double margin_;
  int allocate_num_;
  double tie_breaker_;

  double resolution_, inv_resolution_;
  Eigen::Vector2i origin_, map_size_2d_;
  double time_origin_;

  Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
  void retrievePath(NodePtr end_node);

  double getDistCost(double dist_);
  double getEuclHeu(Eigen::Vector2i x1, Eigen::Vector2i x2);

public:
  Astar();
  ~Astar();

  enum { REACH_END = 1, NO_PATH = 2 };

  void setParam();
  void init();
  void reset();
  int search(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt);
  void setEnvironment(const ESDF_enviroment::Ptr& env);

  std::vector<Eigen::Vector2i> getPath();
  std::vector<NodePtr> getVisitedNodes();

  typedef std::shared_ptr<Astar> Ptr;
};
}
#endif