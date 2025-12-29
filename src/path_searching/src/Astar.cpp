#include "Astar.h"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace navi_planner
{
    int Astar::search(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt) {
        
        NodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
        cur_node->position = start_pt;
        cur_node->index = start_pt;
        cur_node->g_score = 0.0;

        //Eigen::Vector2d end_state(6);
        Eigen::Vector2i end_index;
        end_index = end_pt;

        cur_node->f_score = lambda_heu_ * getEuclHeu(start_pt, end_pt);
        cur_node->node_state = IN_OPEN_SET;

        open_set_.push(cur_node);
        use_node_num_ += 1;

        expanded_nodes_.insert(cur_node->index, cur_node);

        // NodePtr neighbor = NULL;
        NodePtr terminate_node = NULL;

        while (!open_set_.empty()) 
        {
            cur_node = open_set_.top();

            bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                abs(cur_node->index(1) - end_index(1)) <= 1;

            if (reach_end) {

                terminate_node = cur_node;
                retrievePath(terminate_node);
                has_path_ = true;
                RCLCPP_INFO(rclcpp::get_logger("Astar"),"after %d iters , reached end",iter_num_);
                return REACH_END;
            }

            open_set_.pop();
            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;


            Eigen::Vector2i cur_pos = cur_node->position;
            Eigen::Vector2i pro_pos;
            // double pro_t;    未使用变量

            Eigen::Vector2i d_pos;

            for (int dx = -2; dx <= 2; dx += 1){
                for (int dy = -2; dy <= 2; dy += 1){

                    if(dx*dx + dy*dy >= 8){continue;}
                    if(dx == 0 && dy == 0){continue;}

                    d_pos << dx, dy;

                    pro_pos = cur_pos + d_pos;

                    if (pro_pos(0) < 0 || pro_pos(0) >= map_size_2d_(0) || pro_pos(1) < 0 ||
                        pro_pos(1) >= map_size_2d_(1) ) {
                    std::cout << "outside map" << std::endl;
                    continue;
                    }

                    Eigen::Vector2i pro_id = pro_pos;

                    NodePtr pro_node = expanded_nodes_.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
                    continue;
                    }

                    bool Collsion_flg= esdf_environment_->checkCollision(pro_pos);
                    if (Collsion_flg) {
                    continue;
                    }

                    // bool UpstairFlg = esdf_environment_->checkUpStairs(cur_pos,pro_pos);
                    // if(UpstairFlg)
                    // {
                    //     continue;
                    // }

                    double tmp_g_score, tmp_f_score;
                    double dist = esdf_environment_->getDist(pro_pos);
                    tmp_g_score = d_pos.norm() + getDistCost(dist) + cur_node->g_score;
                    tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

                    if (pro_node == NULL) {
                        pro_node = path_node_pool_[use_node_num_];
                        pro_node->index = pro_id;
                        pro_node->position = pro_pos;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;
                        open_set_.push(pro_node);
                        expanded_nodes_.insert(pro_id, pro_node);

                        use_node_num_ += 1;
                        if (use_node_num_ == allocate_num_) {
                            std::cout << "run out of memory." << std::endl;
                            return NO_PATH;
                        }
                        else if (pro_node->node_state == IN_OPEN_SET) {
                            if (tmp_g_score < pro_node->g_score) {
                                // pro_node->index = pro_id;
                                pro_node->position = pro_pos;
                                pro_node->f_score = tmp_f_score;
                                pro_node->g_score = tmp_g_score;
                                pro_node->parent = cur_node;
                            }
                            else {
                            //std::cout << "error type in searching: " << pro_node->node_state << std::endl;
                            }
                        }
                    }
                }
            }
        }
        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        return NO_PATH;
    }

    void Astar::setParam() {
        lambda_heu_ = 2.0;
        allocate_num_ = 500000;
        // nh.param("astar/lambda_heu", lambda_heu_, 2.0);
        // nh.param("astar/allocate_num", allocate_num_, 500000);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"lambda_heu:%f",lambda_heu_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"allocate_num_:%d",allocate_num_);
    }

    void Astar::retrievePath(NodePtr end_node) {
        NodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);

        while (cur_node->parent != NULL) {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }

        reverse(path_nodes_.begin(), path_nodes_.end());
    }

    std::vector<Eigen::Vector2i> Astar::getPath() {
        std::vector<Eigen::Vector2i> path;
        for (size_t i = 0; i < path_nodes_.size(); ++i) {
            path.push_back(path_nodes_[i]->position);
        }
        return path;
    }

    double Astar::getEuclHeu(Eigen::Vector2i x1, Eigen::Vector2i x2) {
        return (x2 - x1).norm();
    }

    double Astar::getDistCost(double dist_)
    {
        return 10.0f/(dist_ + 0.1f);
    }

    void Astar::init() {
        /* ---------- map params ---------- */
        esdf_environment_->getMapRegion(map_size_2d_);
        // cout << "origin_: " << origin_.transpose() << endl;
        std::cout << "map size: " << map_size_2d_.transpose() << std::endl;

        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) {
            path_node_pool_[i] = new Node;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }

    void Astar::setEnvironment(const ESDF_enviroment::Ptr& env) {
        this->esdf_environment_ = env;
    }

    void Astar::reset() {
        expanded_nodes_.clear();
        path_nodes_.clear();

        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++) {
            NodePtr node = path_node_pool_[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }

    std::vector<NodePtr> Astar::getVisitedNodes() {
        std::vector<NodePtr> visited;
        visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
        return visited;
    }

    Astar::Astar()
    {
        ;
    }

    Astar::~Astar()
    {
        for (int i = 0; i < allocate_num_; i++) {
            delete path_node_pool_[i];
        }
    }

// Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
//   Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

//   // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
//   // origin_(1)) * inv_resolution_),
//   //     floor((pt(2) - origin_(2)) * inv_resolution_);

//   return idx;
// }
}