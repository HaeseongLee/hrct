#include "mujoco_controller/fr3_controller.h"

namespace MjController{
    Fr3Controller::Fr3Controller(const std::string urdf_path):urdf_path_(urdf_path)
    {
        std::cout<<"FR3 Controller is ready to be initialized"<<std::endl;   

        initialize(urdf_path);

        double size = pick_task_.size();
        pick_periods_.resize(size);
        pick_periods_ << 2.0, 1.0, 0.5, 1.0; // initialize with arbitraty values       
        pick_timeline_.resize(size);
        pick_timeline_.setZero();
        pick_sch_.setScheduler("pick", pick_task_, pick_periods_);

        size = place_task_.size();
        place_periods_.resize(size);
        place_periods_ << 3.0, 0.5, 2.0; // initialize with arbitraty values       
        place_timeline_.resize(size);
        place_timeline_.setZero();
        place_sch_.setScheduler("place", place_task_, place_periods_);

        next_task_ = false;
        set_attach_ = false;

    }

    void Fr3Controller::initialize(const std::string urdf_path)
    {
        model_.initialize(urdf_path);

        q_desired_.setZero();
        q_init_.setZero();
        
        gw_desired_.setZero();
        gw_init_.setZero();
        
        // q_idle_ << 0.0, -0.785398163397, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.785398163397;
        q_idle_ << 0.0, -1.309, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.785398163397;

        t_ = 0.0;

        set_init_ = true;
    }

    void Fr3Controller::updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                                    const Eigen::Ref<const Eigen::VectorXd> &qd,
                                    const Eigen::Ref<const Eigen::VectorXd> &tau,
                                    const Eigen::Ref<const Eigen::VectorXd> &ft,
                                    const double t)
    {
        t_ = t/hz_;
        q_ = q.head<7>();
        qd_ = qd.head<7>();

        tau_measured_ = tau;
        f_ext_ = ft;
        
        gw_ = q.tail<2>();
        gv_ = qd.tail<2>();

        model_.updateModel(q_, qd_, gw_);
    }

    Eigen::VectorXd Fr3Controller::setIdleConfig(const Eigen::Ref<const Eigen::Vector2d> &time)
    {
        Eigen::Vector7d tau_d;

        // std::cout<<"start time: "<<time(0)<<std::endl;
        // std::cout<<"end   time: "<<time(1)<<std::endl;
        // std::cout<<"q init : "<<q_init_.transpose()<<std::endl;
        // std::cout<<"q idle : "<<q_idle_.transpose()<<std::endl;
        
        for(int i = 0; i < dof_; i++){
            q_desired_(i) = cubic(t_, time(0), time(1), model_.q_init_(i), q_idle_(i), 0.0, 0.0);
        }

        tau_d = 500*(q_desired_ - q_) - 10 * qd_;
        tau_d += model_.NLE_;

        return tau_d;
    }

    Eigen::VectorXd Fr3Controller::gripperOpen(const double target_width, const Eigen::Ref<const Eigen::Vector2d> &time)
    {
        Eigen::Vector2d gw_init;
        Eigen::Vector2d gf;
        double t_0, t_f;

        gw_init = model_.gw_init_;

        t_0 = time(0);
        t_f = time(1);

        for(int i = 0; i < 2; i++){
            gw_desired_(i) = cubic(t_, t_0, t_f, gw_init(i), target_width, 0.0, 0.0);
        }

        gf = 100.0*(gw_desired_ - gw_) - 10.0 * gv_;
        
        return gf;
    }

    Eigen::VectorXd Fr3Controller::gripperClose()
    {
        Eigen::Vector2d gf;
        // gf = gripperOpen(target_width, time);

        for(int i = 0; i < 2; i++) 
            gf(i) = -5.0 - 50*gv_(i);
            // gf_(i) = 1000 * (0.0- gw_(i)) - 10 * gv_(i);


        return gf;
    }

    Eigen::VectorXd Fr3Controller::taskMove(const Eigen::Ref<const Eigen::Vector3d> &x_goal, // goal position
                                            const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                            const Eigen::Ref<const Eigen::Vector2d> &time)   // time information [t_0, t_f]
    // All parameters are w.r.t the robot base
    {
        Eigen::Vector3d x_0, x_f, x, xd;
        Eigen::Vector6d v;
        Eigen::Matrix3d R_0, R_f, R, Rd;
        Eigen::Vector6d f;
        Eigen::Vector7d tau_nll;
        double t_0, t_f;

        x_0 = model_.initial_transform_.translation();
        x = model_.p_;
        x_f = x_goal;
        
        v = model_.xd_;

        R_0 = model_.initial_transform_.linear();
        R = model_.r_;
        R_f = r_goal;
        
        t_0 = time(0);
        t_f = time(1);

        // std::cout<<"t_0: "<<t_0<<std::endl;
        // std::cout<<"t_ : "<<t_<<std::endl;
        // std::cout<<"t_f: "<<t_f<<std::endl;

        for(int i = 0; i < 3; i++){
            xd(i) = common_math::cubic(t_, t_0, t_f, x_0(i), x_f(i), 0.0, 0.0);
        }

        Rd = common_math::rotationCubic(t_, t_0, t_f, R_0, R_f);

        f.head<3>() = 1500*(xd - x) + 80*(-v.head<3>());
        f.tail<3>() = 1200* (-common_math::getPhi(R, Rd)) + 50*(-v.tail<3>());


        tau_nll = 4.0*(model_.q_init_ - model_.q_) - 0.1*(model_.qd_);
        tau_d_ = model_.J_.transpose() * model_.A_ * f + model_.NLE_ + model_.N_ * tau_nll;
        
        return tau_d_;
    }

    Eigen::VectorXd Fr3Controller::pick(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                        const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                        const double duration)   // time information [t_0, t_f]
    {
        // pick == top-down grasp
        // tak sequence
        // reach -> align -> close -> lift

        double grasp_offset = 0.05; // 1cm offset to avoid collision between robot & object
        double t_buffer = 1.0;
        double t_f;

        Eigen::Vector3d reach_x_goal;
        Eigen::Vector8d ctrl;

        reach_x_goal = x_goal;
        reach_x_goal(2) = x_goal(2) + grasp_offset;


        if(set_init_){
            model_.setInitialValues();
            set_init_ = false;
            t_f = t_ + duration + t_buffer;

            // update duration of "reach task"
            pick_periods_[0] = t_f;        
            pick_sch_.updateTimeline(pick_periods_);            
            pick_timeline_ = pick_sch_.getTimeline();

            std::cout<<"Picking ..."<<std::endl;
            // std::cout<<"\nTime: "<<t_<<std::endl;
            // std::cout<<"Curr: "<<pick_sch_.getCurrentTask()<<std::endl;
        }

        std::string task = pick_sch_.scheduling(t_);

        if(pick_sch_.triggerTransition()){
            model_.setInitialValues();
            // std::cout<<"\nTime: "<<t_<<std::endl;
            // std::cout<<"Prev: "<<pick_sch_.getPrevTask()<<std::endl;
            // std::cout<<"Curr: "<<pick_sch_.getCurrentTask()<<std::endl;
        }

        if(task == "reach"){
            Eigen::Vector2d time;
            time(0) = pick_timeline_(0) - (duration + t_buffer); //
            time(1) = pick_timeline_(0); //

            tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperOpen(0.03, time);    
            // std::cout<<tau_d_.transpose()<<std::endl;
        }
        else if(task == "align"){
            Eigen::Vector2d time;
            time(0) = pick_timeline_(0);
            time(1) = pick_timeline_(1);

            tau_d_ = Fr3Controller::taskMove(x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperOpen(0.02, time);    
        }
        else if(task == "close"){
            Eigen::Vector2d time;
            time(0) = pick_timeline_(1);
            time(1) = pick_timeline_(2);

            tau_d_ = Fr3Controller::taskMove(x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperClose();
        }
        else if(task == "lift"){
            Eigen::Vector2d time;
            time(0) = pick_timeline_(2);
            time(1) = pick_timeline_(3);

            tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperClose();
        }
        else{
            Fr3Controller::idleStateWithGrasp();
            pick_sch_.finish();
        }

        ctrl.head<7>() = tau_d_;
        ctrl.tail<1>() = gf_.head<1>();

        return ctrl;
    }
    //TODO : any function to initialize parameters after pick!

    Eigen::VectorXd Fr3Controller::place(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                         const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                         const double duration)                          // time information [t_0, t_f]
    {
        // pick == top-down grasp
        // tak sequence
        // reach -> open -> homing

        double place_offset = 0.1; // 1cm offset to avoid collision between robot & object
        double t_buffer = 1.0;
        double t_f;

        Eigen::Vector3d reach_x_goal;
        Eigen::Vector8d ctrl;

        reach_x_goal = x_goal;
        reach_x_goal(2) = x_goal(2) + place_offset;


        if(set_init_){
            model_.setInitialValues();
            set_init_ = false;
            t_f = t_ + duration + t_buffer;

            // update duration of "reach task"
            place_periods_[0] = t_f;        
            place_sch_.updateTimeline(place_periods_);            
            place_timeline_ = place_sch_.getTimeline();
            std::cout<<"Placing ..."<<std::endl;
            // std::cout<<"\nTime: "<<t_<<std::endl;
            // std::cout<<"Curr: "<<pick_sch_.getCurrentTask()<<std::endl;
        }

        std::string task = place_sch_.scheduling(t_);

        if(place_sch_.triggerTransition()){
            model_.setInitialValues();
            // std::cout<<"\nTime: "<<t_<<std::endl;
            // std::cout<<"Prev: "<<pick_sch_.getPrevTask()<<std::endl;
            // std::cout<<"Curr: "<<pick_sch_.getCurrentTask()<<std::endl;
        }

        if(task == "reach"){
            Eigen::Vector2d time;
            time(0) = place_timeline_(0) - (duration + t_buffer); //
            time(1) = place_timeline_(0); //

            tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperClose();    
        }
        else if(task == "open"){
            Eigen::Vector2d time;
            time(0) = place_timeline_(0);
            time(1) = place_timeline_(1);

            tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperOpen(0.04, time);    
        }
        else if(task == "homing"){
            Eigen::Vector2d time;
            time(0) = place_timeline_(1);
            time(1) = place_timeline_(2);

            tau_d_ = Fr3Controller::setIdleConfig(time);
            gf_ = Fr3Controller::gripperOpen(0.04, time);    
        }
        else{
            Fr3Controller::idleState();
            place_sch_.finish();
        }

        ctrl.head<7>() = tau_d_;
        ctrl.tail<1>() = gf_.head<1>();

        return ctrl;
    }

    Eigen::VectorXd Fr3Controller::pickAndPlace(const Eigen::Ref<const Eigen::MatrixXd> &x_goals,
                                                const Eigen::Ref<const Eigen::MatrixXd> &r_goals,
                                                const Eigen::Ref<const Eigen::VectorXd> &durations)
    {
        // x_goals -> [2 x 3] = [pick; place]
        // r_goals -> [6 x 3] = [pick; place]

        Eigen::VectorXd x_pick, x_place;
        Eigen::MatrixXd r_pick, r_place;
        double t_pick, t_place;
        Eigen::Vector8d ctrl;

        x_pick = x_goals.row(0);
        x_place = x_goals.row(1);

        r_pick = r_goals.topRows(3);
        r_place = r_goals.bottomRows(3);
        
        t_pick = durations(0);
        t_place = durations(1);

        if(!pick_sch_.isComplted() && !place_sch_.isComplted()){
            ctrl = Fr3Controller::pick(x_pick, r_pick, t_pick);

            if(pick_sch_.isComplted()){
                set_attach_ = true;
                set_init_ = true;
            }
        }
        else if(pick_sch_.isComplted() && !place_sch_.isComplted()){
            ctrl = Fr3Controller::place(x_place, r_place, t_place);

            if(place_sch_.isComplted()){
                set_attach_ = false;
                set_init_ = true;
            }
        }
        else{
            ctrl = Fr3Controller::idleState();
            next_task_ = true;
            pick_sch_.reset();
            place_sch_.reset();
        }


        return ctrl;
    }

    Eigen::VectorXd Fr3Controller::approach(const double v, const double t_0, const int dir)
    {

        Eigen::Isometry3d transform_ee_d, transform_d;
        Eigen::Vector3d x, x_d;
        Eigen::Vector6d xdot;
        Eigen::Matrix3d R, R_d;
        Eigen::Vector6d f;
        Eigen::Vector7d tau_d, tau_nll;

        transform_ee_d = primitives::approach(v, t_, t_0, dir); // w.r.t end-effector
        transform_d = model_.initial_transform_*transform_ee_d;

        x_d = transform_d.translation();
        x = model_.p_;
        
        xdot = model_.xd_;

        R_d = transform_d.linear();
        R = model_.r_;
        
        f.head<3>() = 1500*(x_d - x) + 80*(-xdot.head<3>());
        f.tail<3>() = 1200* (-common_math::getPhi(R, R_d)) + 50*(-xdot.tail<3>());

        tau_nll = 4.0*(model_.q_init_ - model_.q_) - 0.1*(model_.qd_);
        tau_d_ = model_.J_.transpose() * model_.A_ * f + model_.NLE_ + model_.N_ * tau_nll;
        
        return tau_d_;
    }

    // masi : Move Approach Search Insert (MASI)
    Eigen::VectorXd Fr3Controller::masi(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                        const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                        const double duration)
    {
        // tak sequence
        // move -> approach -> search -> insert -> homming

        double offset = 0.1; // 1cm offset to avoid collision between robot & object
        double t_buffer = 0.5;

        Eigen::Vector3d reach_x_goal;
        Eigen::Vector8d ctrl;

        reach_x_goal = x_goal;
        reach_x_goal(2) = x_goal(2) + offset;


        if(set_init_){
            model_.setInitialValues();
            set_init_ = false;
            t_init_ = t_;
            f_bias_ = f_ext_;

            std::cout<<"MASI ..."<<std::endl;
        }
        
        if(t_ <= t_init_ + duration + t_buffer){
            Eigen::Vector2d time;
            time(0) = t_init_;
            time(1) = t_init_ + duration; //

            tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperClose();  

            if(t_ >= t_init_ + duration + t_buffer){
                // newly update parameters
                model_.setInitialValues();
                f_bias_ = f_ext_;
            }

            // std::cout<<"f_raw   : "<<f_ext_.transpose()<<std::endl;
            // std::cout<<"f_bias_ : "<<f_bias_.transpose()<<std::endl;
            // std::cout<<"f_ext   : "<<(f_ext_ - f_bias_).transpose()<<std::endl;
            // std::cout<<"---------------------------------"<<std::endl;
        }
        else{
            tau_d_ = Fr3Controller::approach(0.01, t_init_ + duration + t_buffer, 2);
            gf_ = Fr3Controller::gripperClose();  

            std::cout<<"f_raw   : "<<f_ext_.transpose()<<std::endl;
            std::cout<<"f_bias_ : "<<f_bias_.transpose()<<std::endl;
            std::cout<<"f_ext   : "<<(f_ext_ - f_bias_).transpose()<<std::endl;
            std::cout<<"---------------------------------"<<std::endl;

        }

    
        ctrl.head<7>() = tau_d_;
        ctrl.tail<1>() = gf_.head<1>();

        return ctrl;
    }

    Eigen::VectorXd Fr3Controller::pegInHole(const Eigen::Ref<const Eigen::MatrixXd> &x_goals,
                                             const Eigen::Ref<const Eigen::MatrixXd> &r_goals,
                                             const Eigen::Ref<const Eigen::VectorXd> &durations)
    {
        // x_goals -> [2 x 3] = [pick; place]
        // r_goals -> [6 x 3] = [pick; place]

        Eigen::VectorXd x_peg, x_hole;
        Eigen::MatrixXd r_peg, r_hole;
        double t_peg, t_hole;
        Eigen::Vector8d ctrl;

        x_peg = x_goals.row(0);
        x_hole = x_goals.row(1);

        r_peg = r_goals.topRows(3);
        r_hole = r_goals.bottomRows(3);
        
        t_peg = durations(0);
        t_hole = durations(1);

        if(!pick_sch_.isComplted()){
            ctrl = Fr3Controller::pick(x_peg, r_peg, t_peg);

            if(pick_sch_.isComplted()){
                set_attach_ = true;
                set_init_ = true;
            }
        }
        else{
            ctrl = Fr3Controller::masi(x_hole, r_hole, t_hole);
        }
        // else if(pick_sch_.isComplted() && !place_sch_.isComplted()){
        //     ctrl = Fr3Controller::place(x_place, r_place, t_place);

        //     if(place_sch_.isComplted()){
        //         set_attach_ = false;
        //         set_init_ = true;
        //     }
        // }
        // else{
        //     ctrl = Fr3Controller::idleState();
        //     next_task_ = true;
        //     pick_sch_.reset();
        //     place_sch_.reset();
        // }


        return ctrl;


        // Eigen::Vector8d ctrl;

        // if(set_init_){
        //     model_.setInitialValues();
        //     set_init_ = false;
        //     t_init_ = t_;

        //     std::cout<<"\n Approaching ..."<<std::endl;
        // }

        // tau_d_ = Fr3Controller::approach(0.01, t_init_, 2);
        // gf_ = Fr3Controller::gripperClose();

        // ctrl.head<7>() = tau_d_;
        // ctrl.tail<1>() = gf_.head<1>();

        // return ctrl;
    }

    Eigen::VectorXd Fr3Controller::idleState()
    {
        Eigen::Vector8d ctrl;

        tau_d_ = 500 * (model_.q_init_ - q_) - 10 * qd_;
        tau_d_ += model_.NLE_;

        gf_ = 50 * (model_.gw_init_ - gw_) - 10 * gv_;

        ctrl.head<7>() = tau_d_;
        ctrl.tail<1>() = gf_.head<1>();

        return ctrl;
    }

    void Fr3Controller::idleStateWithGrasp(){        
        tau_d_ = 500*(model_.q_init_ - q_) - 10 * qd_;
        tau_d_ += model_.NLE_;

        gf_ = Fr3Controller::gripperClose();
    }


    void Fr3Controller::saveState()
    {   
        save_joint_q<<t_<<" "<< q_.transpose()<<std::endl;
    }
}