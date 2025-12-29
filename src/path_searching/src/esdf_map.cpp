#include "esdf_map.h"
#include <iostream>
#include <queue>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <random>
#include <cmath>
#define DEBUG_ESDF  //验证ESDF是否索引有误，指的是数组和cv::mat的行列对应关系是否正确,使用一次无误注释掉即可

namespace ESDF_enviroment
{
    double esdf::height_conversion(unsigned char height_)
    {
        return ((double)height_ / 80.0f)-1.0f;
    }

    // void esdf::showpt(Eigen::Vector2i pt)
    // {
    //     // draw a small 11x11 black square centered at pt (pt = [row, col])
    //     for(int di = -5 ; di <= 5 ; di++ )
    //     {
    //         for(int dj = -5; dj <= 5; dj++)
    //         {
    //             int r = pt[0] + di;
    //             int c = pt[1] + dj;
    //             if (r < 0 || c < 0 || r >= img.rows || c >= img.cols) continue;
    //             img.at<cv::Vec3b>(r, c)[0] = 0;
    //             img.at<cv::Vec3b>(r, c)[1] = 0;
    //             img.at<cv::Vec3b>(r, c)[2] = 0;
    //         }
    //     }

    //     cv::imwrite(height_map_path + "test.png",img);
        
    // }
    
    
    void esdf::esdf_init(bool* bin_map_, int sizeX_, int sizeY_, Eigen::Vector2d offset_,bool enable_downstairs)
    {
        this->enable_downstairs = enable_downstairs;
        height_map_path = "/home/hustlyrm/sources/shared_maps/";
        // Use row-major representation: bin_map[row][col]
        // External args: sizeX_ = width (cols), sizeY_ = height (rows)
        Size[0] = sizeY_; // rows
        Size[1] = sizeX_; // cols
        Offset = offset_;

        bin_map = new bool*[Size[0]];
        height_map = new double*[Size[0]];
        valid_map = new bool*[Size[0]];
        for(int r = 0; r < Size[0]; r++)
        {
            bin_map[r] = new bool[Size[1]];
            height_map[r] = new double[Size[1]];
            valid_map[r] = new bool[Size[1]];
            // input bin_map_ is assumed row-major: index = r * cols + c
            memcpy(bin_map[r], bin_map_ + Size[1] * r, Size[1]*sizeof(bool)); // 只对占据栅格地图进行初始化
        }

        // allocate ESDF structures (row-major)
        distance_map = new double*[Size[0]];
        nearest_obs_x = new int*[Size[0]];
        nearest_obs_y = new int*[Size[0]];
        for(int r = 0; r < Size[0]; r++)
        {
            distance_map[r] = new double[Size[1]];
            nearest_obs_x[r] = new int[Size[1]];
            nearest_obs_y[r] = new int[Size[1]];
            for(int c = 0; c < Size[1]; c++)
            {
                nearest_obs_x[r][c] = -1;
                nearest_obs_y[r][c] = -1;
                distance_map[r][c] = std::numeric_limits<double>::infinity();
            }
        }

        // initial ESDF computation //初始化ESDF计算
        computeDistanceField();

        if(true)
        {
            img = cv::imread(height_map_path + "height_map.png");
            if (img.empty()) 
            {
                std::cout << "[ESDF] height_map.png not found, skipping height init" << std::endl;
                return;
            }
            for(int r = 0 ; r < Size[0]; r++)
            {
                for(int c = 0; c < Size[1]; c++)
                {
                    // r = row (y), c = col (x)
                    std::cout<<"read"<< r << "," << c <<std::endl;
                    height_map[r][c] = height_conversion(img.at<cv::Vec3b>(r,c)[0]);
                    valid_map[r][c] = img.at<cv::Vec3b>(r,c)[2] < 128 ? true : false ;
                    // mark green channel for occupied cells
                    img.at<cv::Vec3b>(r,c)[1] = bin_map[r][c] ? 255 : 0;
                }
            }
            cv::imwrite(height_map_path + "test.png",img);
        }
    }

    bool esdf::checkCollision(Eigen::Vector2i pos_) //检查是否为占据状态
    {
        return bin_map[pos_[0]][pos_[1]];
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
        if(pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
            return std::numeric_limits<double>::infinity();
        return distance_map[pos_[0]][pos_[1]];
    }

    Eigen::Vector2i esdf::getNearestObstacleIndex(Eigen::Vector2i pos_)
    {
        Eigen::Vector2i idx(-1,-1);
        if(pos_[0] < 0 || pos_[1] < 0 || pos_[0] >= Size[0] || pos_[1] >= Size[1])
            return idx;
        int ox = nearest_obs_x[pos_[0]][pos_[1]];
        int oy = nearest_obs_y[pos_[0]][pos_[1]];
        if(ox < 0) return idx;
        idx[0] = ox; idx[1] = oy;
        return idx;
    }

    void esdf::computeDistanceField()
    {
        // create binary image where obstacles are 0 and free cells 255
        // binImg: rows = Size[0], cols = Size[1]
        cv::Mat binImg(Size[0], Size[1], CV_8UC1);
        for(int r = 0; r < Size[0]; r++)
        {
            for(int c = 0; c < Size[1]; c++)
            {
                binImg.at<uchar>(r,c) = bin_map[r][c] ? 0 : 255;
            }
        }
        cv::Mat dist;
        cv::distanceTransform(binImg, dist, cv::DIST_L2, 3);
        for(int r = 0; r < Size[0]; r++)
        {
            for(int c = 0; c < Size[1]; c++)
            {
                distance_map[r][c] = (double)dist.at<float>(r,c);
            }
        }

        // multi-source BFS to propagate nearest obstacle indices (rows/cols)
        std::queue<std::pair<int,int>> q;
        for(int r = 0; r < Size[0]; r++)
        {
            for(int c = 0; c < Size[1]; c++)
            {
                if(bin_map[r][c])
                {
                    nearest_obs_x[r][c] = r;
                    nearest_obs_y[r][c] = c;
                    q.push(std::make_pair(r,c));
                }
                else
                {
                    nearest_obs_x[r][c] = -1;
                    nearest_obs_y[r][c] = -1;
                }
            }
        }
        const int dr[4] = {-1,1,0,0};
        const int dc[4] = {0,0,-1,1};
        while(!q.empty())
        {
            auto p = q.front(); q.pop();
            int pr = p.first, pc = p.second;
            for(int k = 0; k < 4; k++)
            {
                int nr = pr + dr[k];
                int nc = pc + dc[k];
                if(nr < 0 || nc < 0 || nr >= Size[0] || nc >= Size[1]) continue;
                if(nearest_obs_x[nr][nc] == -1)
                {
                    nearest_obs_x[nr][nc] = nearest_obs_x[pr][pc];
                    nearest_obs_y[nr][nc] = nearest_obs_y[pr][pc];
                    q.push(std::make_pair(nr,nc));
                }
            }
        }

        #ifdef DEBUG_ESDF
        // validation: quick brute-force check (detect transpose/indexing bugs)
        // Note: internal arrays are indexed as bin_map[x][y] (first=index x/col, second=index y/row)
        // OpenCV uses img.at(row, col) == img.at(y, x), so we expect distance_map[x][y] == dist.at<float>(y,x)
            {
                int check_N = std::min(50, Size[0]*Size[1]);
                std::mt19937 rng(123456);
                std::uniform_int_distribution<int> ux(0, Size[0]-1);
                std::uniform_int_distribution<int> uy(0, Size[1]-1);
                int mismatches = 0;
                for (int t = 0; t < check_N; t++)
                {
                    int x = ux(rng);
                    int y = uy(rng);
                    double best = std::numeric_limits<double>::infinity();
                    int bx = -1, by = -1;
                    for (int i = 0; i < Size[0]; i++)
                    {
                        for (int j = 0; j < Size[1]; j++)
                        {
                            if (!bin_map[i][j]) continue;
                            double dx_ = (double)i - (double)x;
                            double dy_ = (double)j - (double)y;
                            double d = std::sqrt(dx_*dx_ + dy_*dy_);
                            if (d < best) { best = d; bx = i; by = j; }
                        }
                    }
                    double esdf_d = distance_map[x][y];
                    int ox = nearest_obs_x[x][y];
                    int oy = nearest_obs_y[x][y];
                    // Allow small differences due to OpenCV distance transform semantics
                    bool dist_ok = (std::abs(esdf_d - best) <= 0.2); // tolerance in pixels
                    bool idx_ok = (std::abs(bx - ox) <= 1 && std::abs(by - oy) <= 1);
                    if (!dist_ok || !idx_ok) mismatches++;
                }
                if (mismatches > 0)
                {
                    std::cout << "[ESDF] WARNING: validation FAILED, mismatches=" << mismatches << ". Possible indexing/transposition bug (bin_map[x][y] vs cv::Mat[row,col])." << std::endl;
                }
                else
                {
                    std::cout << "[ESDF] validation OK" << std::endl;
                }
            }
        #endif
    }

    void esdf::updateDistanceField()
    {
        computeDistanceField();
    }

    esdf::esdf()
    {
        ;
    }
    esdf::~esdf()
    {
        for(int r = 0; r < Size[0]; r++)
        {
            delete[] bin_map[r];
            delete[] height_map[r];
            delete[] valid_map[r];
            delete[] distance_map[r];
            delete[] nearest_obs_x[r];
            delete[] nearest_obs_y[r];
        }
        delete[] bin_map;
        delete[] height_map;
        delete[] valid_map;
        delete[] distance_map;
        delete[] nearest_obs_x;
        delete[] nearest_obs_y;
    }
} // namespace name
