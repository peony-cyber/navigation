#ifndef ESDF_H
#define ESDF_H
#include "dynamicvoronoi.h"
#include <Eigen/Core>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>

namespace ESDF_enviroment {

    class esdf
    {
    public:
        bool** bin_map;
        bool** valid_map;
        double** height_map;
        navi_planner::DynamicVoronoi voronoi_map;
        cv::Mat img;
        Eigen::Vector2i Size;
        Eigen::Vector2d Offset;
        bool enable_downstairs = false;
        std::string height_map_path;
    public:
        void esdf_init(bool* bin_map_, int sizeX_, int sizeY_, Eigen::Vector2d offset_, bool enable_downstairs = false);
        void showpt(Eigen::Vector2i pt);
        esdf();
        ~esdf();
        double height_conversion(unsigned char height_);
        bool checkCollision(Eigen::Vector2i pos_);
        bool checkUpStairs(Eigen::Vector2i pos_, Eigen::Vector2i next_pos_);
        double getDist(Eigen::Vector2i pos_);
        void getMapRegion(Eigen::Vector2i & map_size_){map_size_ =  Size;}
        Eigen::Vector2d Index2pos(Eigen::Vector2i index_)
        {
            Eigen::Vector2d index_d((double)index_[1],(double)index_[0]);
            Eigen::Vector2d pos_ = index_d / 20.0f + Offset;
            return pos_;
        }

        Eigen::Vector2i Pos2index(Eigen::Vector2d pos)
        {
            Eigen::Vector2d index_ = (pos - Offset)*20.0f;
            Eigen::Vector2i index((int)index_[1],(int)index_[0]);
            return index;
        }
    };
    typedef std::shared_ptr<esdf> Ptr;
    
    
}
#endif