#include "esdf_map.h"
#include <iostream>

namespace ESDF_enviroment
{
    double esdf::height_conversion(unsigned char height_)
    {
        return ((double)height_ / 80.0f)-1.0f;
    }

    void esdf::showpt(Eigen::Vector2i pt)
    {
        for(int i = -5 ; i <= 5 ; i++ )
        {
            for(int j = -5; j <= 5; j ++)
            {
                img.at<cv::Vec3b>(pt[1]+i,pt[0]+j)[0] = 0;
                img.at<cv::Vec3b>(pt[1]+i,pt[0]+j)[1] = 0;
                img.at<cv::Vec3b>(pt[1]+i,pt[0]+j)[2] = 0;
            }
        }

        cv::imwrite(height_map_path + "test.png",img);
        
    }
    
    
    void esdf::esdf_init(bool* bin_map_, int sizeX_, int sizeY_, Eigen::Vector2d offset_,bool enable_downstairs)
    {
        this->enable_downstairs = enable_downstairs;
        height_map_path = "/home/hustlyrm/sources/shared_maps/";
        Size[0] = sizeX_;
        Size[1] = sizeY_;
        Offset = offset_;

        bin_map = new bool*[sizeX_];
        height_map = new double*[sizeX_];
        valid_map = new bool*[sizeX_];
        for(int i = 0; i < sizeX_; i ++)
        {
            bin_map[i] = new bool[sizeY_];
            height_map[i] = new double[sizeY_];
            valid_map[i] = new bool[sizeY_];
            memcpy(bin_map[i],bin_map_ + sizeY_ * i, sizeY_*sizeof(bool));
        }

        voronoi_map.initializeMap(sizeX_,sizeY_,bin_map);
        voronoi_map.update();
        if(true)
        {
            img = cv::imread(height_map_path + "height_map.png");
            if (img.empty()) 
            {
                std::cout << "[ESDF] height_map.png not found, skipping height init" << std::endl;
                return;
            }
            for(int i = 0 ; i < sizeX_; i ++)
            {
                for(int j = 0; j < sizeY_; j++)
                {
                    std::cout<<"read"<< i << j <<std::endl;
                    height_map[i][j] = height_conversion(img.at<cv::Vec3b>(j,i)[0]);
                    valid_map[i][j] = img.at<cv::Vec3b>(j,i)[2] < 128 ? true : false ;
                    img.at<cv::Vec3b>(j,i)[1] = voronoi_map.isVoronoi(i,j) == true ? 255 : 0;
                }
            }
            cv::imwrite(height_map_path + "test.png",img);
        }
    }

    bool esdf::checkCollision(Eigen::Vector2i pos_)
    {
        //return bin_map[pos_[0]][pos_[1]];// || !voronoi_map.isVoronoi(pos_[0],pos_[1]);
        return !voronoi_map.isVoronoi(pos_[0],pos_[1]);
    }

    bool esdf::checkUpStairs(Eigen::Vector2i pos_, Eigen::Vector2i next_pos_)
    {
        double height_1 = height_map[pos_[0]][pos_[1]];
        double height_2 = height_map[next_pos_[0]][next_pos_[1]];

        double height_delta = height_2 - height_1;
        if(enable_downstairs == false)
            height_delta = abs(height_delta);

        if(height_delta > 0.08f)
            return true;
        return false;
    }

    double esdf::getDist(Eigen::Vector2i pos_)
    {
        return voronoi_map.data[pos_[0]][pos_[1]].dist;
    }
    esdf::esdf()
    {
        ;
    }
    esdf::~esdf()
    {
        for(int i = 0; i < Size[0]; i ++)
        {
            delete[] bin_map[i];
            delete[] height_map[i];
            delete[] valid_map[i];
        }
        delete[] bin_map;
        delete[] height_map;
        delete[] valid_map;
    }
} // namespace name
