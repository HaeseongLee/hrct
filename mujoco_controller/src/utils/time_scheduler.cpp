#include "utils/time_scheduler.h"

void TimeScheduler::setScheduler(std::string name, std::vector<std::string> &task_list, Eigen::VectorXd &periods)
{
    n_task_ = task_list.size();

    timeline_.resize(n_task_);
    timeline_.setZero();

    for(int i = 0; i < n_task_; i++){
        if(i == 0)
            timeline_(i) = periods[i];
        else
            timeline_(i) = timeline_[i-1] + periods[i];
    }

    task_list_ = task_list;
    current_ = task_list_.front();
    prev_ = current_;
    
    is_transition_ = false;    
    is_completed_ = false;

    std::cout<<name<<" task scheduler is loaded"<<std::endl;

}

std::string TimeScheduler::scheduling(const double t)
{
    Eigen::VectorXd tmp_t = timeline_.array() - t;

    int idx = (tmp_t.array() < 0).count();

    if(idx >= n_task_)
    {
        current_ = "idle";
        is_completed_ = true;
        //  idx = n_task_ - 1;
    }
    else{
        current_ = task_list_[idx];
    }
    

    if(prev_ != current_){
        is_transition_ = true;
    }
    else{
        is_transition_ = false;
    }

    prev_ = current_;

    return current_;
}

bool TimeScheduler::triggerTransition()
{
    return is_transition_;
}

void TimeScheduler::updateTimeline(Eigen::VectorXd periods)
{
    // reset
    timeline_.setZero();

    for(int i = 0; i < n_task_; i++){
        if(i == 0)
            timeline_(i) = periods[i];
        else
            timeline_(i) = timeline_[i-1] + periods[i];
    }
}

Eigen::VectorXd TimeScheduler::getTimeline()
{
    return timeline_;
}

std::string TimeScheduler::getCurrentTask(){
    return current_;
}

std::string TimeScheduler::getPrevTask(){
    return prev_;
}

void TimeScheduler::reset(){
    is_completed_ = false;
}

void TimeScheduler::finish(){
    is_completed_ = true;
}

bool TimeScheduler::isComplted()
{
    return is_completed_;
}