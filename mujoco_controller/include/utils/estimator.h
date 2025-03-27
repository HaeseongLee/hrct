#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>
// #include <algorithm>


class PegInHoleEstimator{

    public:
        PegInHoleEstimator(double move_thres, double approach_thres, double search_thres, double insert_thres);
        bool setMove(const double d);
        bool setApproach(const double f_z);
        bool setSearch(const Eigen::Ref<const Eigen::VectorXd> &f_ext);
        bool setInsert(const double v_z);

        std::string getTargetTask();
    
    private:

        double move_thres_;
        double approach_thres_;
        double search_thres_;
        double insert_thres_;
        
        bool move_completed_;
        bool approach_completed_;
        bool search_completed_;
        bool insert_completed_;
        
        std::string target_task_;

        std::vector<std::string> task_list_ = {"move", "approach", "search", "insert", "homing"};

        // to recognize which task is doing
        std::vector<int> bit_masks_ = {0b0000, 0b1000, 0b1100, 0b1110, 0b1111};

        int cnt_;
};