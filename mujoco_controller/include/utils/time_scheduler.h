#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>
// #include <algorithm>


class TimeScheduler{
    
    public:
        TimeScheduler(){};
        void setScheduler(std::string name, std::vector<std::string> &task_list, Eigen::VectorXd &periods);
        std::string scheduling(const double t);
        bool triggerTransition();
        void updateTimeline(Eigen::VectorXd periods);
        Eigen::VectorXd getTimeline();
        std::string getCurrentTask();
        std::string getPrevTask();
        void reset();
        void finish();
        bool isComplted();

    private:
        std::string name_;
        std::vector<std::string> task_list_;
        Eigen::VectorXd timeline_;
        std::string current_, prev_;

        bool is_transition_;        
        double n_task_;

        bool is_completed_;


};