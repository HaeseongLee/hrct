#include "utils/estimator.h"

PegInHoleEstimator::PegInHoleEstimator(double move_thres, double approach_thres, double search_thres, double insert_thres)
:move_thres_(move_thres), approach_thres_(approach_thres),search_thres_(search_thres), insert_thres_(insert_thres)
{
    move_completed_ = false;
    approach_completed_ = false;
    search_completed_ = false;
    insert_completed_ = false;
    
    target_task_ = "move";

    cnt_ = 0.0;
}

bool PegInHoleEstimator::setMove(const double d)
{

    if(d >= move_thres_){
        move_completed_ = false;
    }
    else{
        move_completed_ = true;
    }

    return move_completed_;
}

bool PegInHoleEstimator::setApproach(const double f_z)
{
    if(f_z >= approach_thres_){
        approach_completed_ = true;
    }
    else{
        approach_completed_ = false;
    }

    return approach_completed_;
}

bool PegInHoleEstimator::setSearch(const Eigen::Ref<const Eigen::VectorXd> &f_ext)
{
    double f;
    f = f_ext.norm();

    if(f >= search_thres_){
        search_completed_ = true;
    }
    else{
        search_completed_ = false;
    }

    return search_completed_;
}

bool PegInHoleEstimator::setInsert(const double v_z)
{

    if(abs(v_z) <= insert_thres_){
        cnt_ ++;
    }
    else{
        cnt_ = 0;
    }

    if(cnt_ >= 300){
        insert_completed_ = true;
    }
    else{
        insert_completed_ = false;
    }

    return insert_completed_;
}

std::string PegInHoleEstimator::getTargetTask()
{   
    int task_index;

    std::vector<bool> boolean_state = {move_completed_, approach_completed_, search_completed_, insert_completed_};
    int task_state = (boolean_state[0] << 3) | (boolean_state[1] << 2) | (boolean_state[2] << 1) | boolean_state[3];

    for (size_t i = 0; i < bit_masks_.size(); ++i) {
        if (task_state == bit_masks_[i]) {
            task_index == i;
            return task_list_[i];
        }
    }

}
