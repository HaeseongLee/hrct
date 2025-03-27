#include "mujoco_controller/fr3_controller.h"

namespace MjController{
    Fr3Controller::Fr3Controller(const std::string urdf_path):urdf_path_(urdf_path)
    {
        std::cout<<"FR3 Controller is ready to be initialized"<<std::endl;   

        initialize(urdf_path);
        tau_d_lpf_.setZero();
        f_ext_lpf_.setZero();
        v_lpf_.setZero();
        v_ee_lpf_.setZero();

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


        force_pid_.setGains(1.0, 0.001, 0.0);
        insert_pid_.setGains(1.0, 0.001, 0.0);

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
        
        double f_cut = 30;
        double ts = 1/(2*M_PI*f_cut);
        f_ext_lpf_ = common_math::lowPassFilter(f_ext_, f_ext_lpf_, 0.001, ts);

        
        gw_ = q.tail<2>();
        gv_ = qd.tail<2>();

        model_.updateModel(q_, qd_, gw_, tau_measured_);

        x_ = model_.p_;
        v_ = model_.xd_;

        f_cut = 60;
        ts = 1/(2*M_PI*f_cut);
        v_lpf_ = common_math::lowPassFilter(v_, v_lpf_, 1/hz_, ts);

        R_ = model_.r_;

        tau_ext_ = model_.tau_ext_;

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
        Eigen::Isometry3d transform_ee;
        Eigen::Vector3d x_0, x_f;
        Eigen::Vector3d xd_ddot, xd_dot, xd;
        Eigen::Vector3d xd_set;
        Eigen::Matrix3d R_0, R_f, Rd;
        Eigen::Vector6d f;
        Eigen::Vector7d tau_d, tau_nll;
        double t_0, t_f;

        x_0 = model_.initial_transform_.translation();
        x_f = x_goal;

        R_0 = model_.initial_transform_.linear();
        R_f = r_goal;
        
        // to save robot state
        transform_ee = model_.initial_transform_.inverse()*model_.transform_;
        x_ee_ = transform_ee.translation();
        R_ee_ = transform_ee.linear();

        t_0 = time(0);
        t_f = time(1);
       
        for(int i = 0; i < 3; i++){
            xd_set = common_math::quinticSpline(t_, t_0, t_f, x_0(i), 0.0, 0.0, x_f(i), 0.0, 0.0);
            xd(i) = xd_set(0);
            xd_dot(i) = xd_set(1);
            xd_ddot(i) = xd_set(2);
            // xd(i) = common_math::cubic(t_, t_0, t_f, x_0(i), x_f(i), 0.0, 0.0);            
        }

        Rd = common_math::rotationCubic(t_, t_0, t_f, R_0, R_f);

        // f.head<3>() = 1500*(xd - x_) + 80*(-v_.head<3>());
        f.head<3>() = 1.0 * xd_ddot + (10000*(xd - x_) + 80*(xd_dot - v_lpf_.head<3>()));
        f.tail<3>() = 2000* (-common_math::getPhi(R_, Rd)) + 60*(-v_.tail<3>());


        tau_nll = 4.0*(model_.q_init_ - q_) - 0.1*(qd_);
        tau_d = model_.J_.transpose() * model_.A_ * f + model_.NLE_ + model_.N_ * tau_nll;
        
        // debug_file << xd.transpose()<<" "<<x_.transpose()<<std::endl;

        return tau_d;
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
            gf_ = Fr3Controller::gripperOpen(0.04, time);    
            // std::cout<<tau_d_.transpose()<<std::endl;
        }
        else if(task == "align"){
            Eigen::Vector2d time;
            time(0) = pick_timeline_(0);
            time(1) = pick_timeline_(1);

            tau_d_ = Fr3Controller::taskMove(x_goal, r_goal, time);
            gf_ = Fr3Controller::gripperOpen(0.04, time);    
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
            // Fr3Controller::idleStateWithGrasp();
            Fr3Controller::idleState();
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
                // set_attach_ = true;
                set_init_ = true;
            }
        }
        else if(pick_sch_.isComplted() && !place_sch_.isComplted()){
            ctrl = Fr3Controller::place(x_place, r_place, t_place);

            if(place_sch_.isComplted()){
                // set_attach_ = false;
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

        Eigen::Isometry3d transform_ee_d, transform_d, transform_ee;
        Eigen::Vector3d x_d;        
        Eigen::Matrix3d R_d;
        Eigen::Vector6d f;
        Eigen::Vector7d tau_d, tau_nll;

        transform_ee_d = primitives::approach(v, t_, t_0, dir); // w.r.t end-effector
        transform_d = model_.initial_transform_*transform_ee_d;
        transform_ee = model_.initial_transform_.inverse()*model_.transform_;

        x_d = transform_d.translation();
        R_d = transform_d.linear();
        
        x_ee_ = transform_ee.translation();
        R_ee_ = transform_ee.linear();

        v_ee_lpf_.head<3>() = model_.initial_transform_.linear().inverse()*v_lpf_.head<3>();
        v_ee_lpf_.tail<3>() = model_.initial_transform_.linear().inverse()*v_lpf_.tail<3>();

        f.head<3>() = 800*(x_d - x_) + 100*(-v_lpf_.head<3>());
        f.tail<3>() = 1200* (-common_math::getPhi(R_, R_d)) + 120*(-v_lpf_.tail<3>());

        tau_nll = 4.0*(model_.q_init_ - q_) - 0.1*(qd_);
        tau_d = model_.J_.transpose() * model_.A_ * f + model_.NLE_ + model_.N_ * tau_nll;
        
        return tau_d;
    }

    Eigen::VectorXd Fr3Controller::search(double p, double v, double f, double t, double t_0, double duration)
    {
        Eigen::Isometry3d transform_ee_d, transform_ee;
        Eigen::Matrix3d R_init, R_ee_d;
        Eigen::Vector3d x_ee_d;
        Eigen::Vector3d v_ee, w_ee;
        Eigen::Vector6d f_ee_m; // motion w.r.t base frame
        Eigen::Vector6d f_ee_a; // active force w.r.t base frame
        Eigen::Vector6d f_star, f_d; // forces w.r.t base frame
        Eigen::Vector7d tau_d, tau_nll;

        R_init = model_.initial_transform_.linear(); // initial rotation w.r.t base frame

        // Generation motion-related input -------------------------------------------------------------
        transform_ee_d = primitives::spiral(p, v, t, t_0, duration); // w.r.t end-effector
        transform_ee = model_.initial_transform_.inverse()*model_.transform_;
        
        x_ee_d = transform_ee_d.translation();
        x_ee_ = transform_ee.translation();
        v_ee = R_init.inverse()*model_.xd_.head<3>();

        R_ee_d = Eigen::Matrix3d::Identity(); // keep the initial orientation w.r.t ee_frame
        R_ee_ = transform_ee.linear();
        w_ee = R_init.inverse()*model_.xd_.tail<3>();

        v_ee_lpf_.head<3>() = R_init.inverse()*v_lpf_.head<3>();
        v_ee_lpf_.tail<3>() = R_init.inverse()*v_lpf_.tail<3>();

        f_ee_m.head<3>() = 1200.0*(x_ee_d - x_ee_) + 40.0*(-v_ee_lpf_.head<3>());
        // f_ee_m(2) = 0.0; // no motion along z-axis (assembly direction)
        f_ee_m.tail<3>() = 1000.0* (-common_math::getPhi(R_ee_, R_ee_d)) + 50.0*(-v_ee_lpf_.tail<3>());
        // ---------------------------------------------------------------------------------------------

        // Generation force-related input -------------------------------------------------------------
        f_ee_a.setZero();
        double f_ref = primitives::push(f, t, t_0, 0.5); // only generate force along z-aixs (assembly direction)
        // f_ee_a(2) = f_ref + 0.0*(f_ref - (-f_ext_(2))) + 20.0 *(-v_ee(2)); // damping effect
        // f_ee_a(2) = f_ref + 10.0*(f_ref - f_contact(2)) + 40.0 *(-v_ee_lpf_(2));
        f_ee_a(2) = f_ref + force_pid_.compute(f_ref, f_contact_(2)) + 40.0 *( -v_ee_lpf_(2));
        // f_ee_a(2) = f_ref + 40.0 *(-v_ee_lpf_(2));

        // ---------------------------------------------------------------------------------------------

        f_star.head<3>() = R_init*f_ee_m.head<3>();
        f_star.tail<3>() = R_init*f_ee_m.tail<3>();
        
        f_d.setZero();
        f_d.head<3>() = R_init*f_ee_a.head<3>();

        tau_nll = 4.0*(model_.q_init_ - model_.q_) - 0.1*(model_.qd_);
        tau_d = model_.J_.transpose() * (model_.A_ * f_star + f_d)+ model_.NLE_ + model_.N_ * tau_nll;

        return tau_d;
    }

    Eigen::VectorXd Fr3Controller::insert(double f, double t, double t_0)
    {        
        Eigen::Isometry3d transform_ee_d, transform_ee;
        Eigen::Matrix3d R_init, R_ee_d;
        Eigen::Vector3d x_ee_d;
        Eigen::Vector6d f_ee_m; // motion w.r.t base frame
        Eigen::Vector6d f_ref, f_ee_a; // active force w.r.t base frame
        Eigen::Vector6d f_star, f_d; // forces w.r.t base frame
        Eigen::Vector3d v_ee, w_ee;
        Eigen::Vector7d tau_d, tau_nll;

        R_init = model_.initial_transform_.linear(); // initial rotation w.r.t base frame
        transform_ee = model_.initial_transform_.inverse()*model_.transform_;

        x_ee_d.setZero();
        x_ee_ = transform_ee.translation();

        // Eigen::VectorXd f_contact = f_ext_lpf_ - f_bias_;

        v_ee = R_init.inverse()*model_.xd_.head<3>();
        w_ee = R_init.inverse()*model_.xd_.tail<3>();

        v_ee_lpf_.head<3>() = R_init.inverse()*v_lpf_.head<3>();
        v_ee_lpf_.tail<3>() = R_init.inverse()*v_lpf_.tail<3>();

        R_ee_d = Eigen::Matrix3d::Identity(); // keep the initial orientation w.r.t ee_frame
        R_ee_ = transform_ee.linear();
        w_ee = R_init.inverse()*model_.xd_.tail<3>();

        // f_ee_m.setZero();
        // f_ee_m.tail<3>().setZero();
        f_ee_m.head<3>() = 150.0*(x_ee_d - x_ee_) + 10.0*(-v_ee_lpf_.head<3>());
        f_ee_m(2) = 0.0; // no motion along z-axis (assembly direction)
        f_ee_m.tail<3>() = 150.0* (-common_math::getPhi(R_ee_, R_ee_d)) + 10.0*(-v_ee_lpf_.tail<3>());
        // f_ee_m(3) = 0.0;
        // f_ee_m(4) = 0.0;
        

        // Generation force-related input -------------------------------------------------------------
        f_ee_a.setZero();
        f_ref.setZero();
        f_ref(2) = primitives::push(f, t, t_0, 0.1); // only generate force along z-aixs (assembly direction)

        // f_ee_a.head<2>() = f_ref.head<2>() + 0.5*(f_ref.head<2>()- f_contact_.head<2>()) + 40.0 *(-v_ee_lpf_.head<2>());
        // f_ee_a.head<2>() = f_ref.head<2>() + insert_pid_.compute(f_ref.head<2>(), f_contact_.head<2>()) + 40.0 *(-v_ee_lpf_.head<2>());
        f_ee_a(2) = f_ref(2) + 1.0*(f_ref(2) - f_contact_(2)) + 40.0 *(-v_ee_lpf_(2));

        // f_ee_a.tail<3>() = f_ref.tail<3>() + 0.1*(f_ref.tail<3>()- f_contact_.tail<3>()) + 0.1 *(-v_ee_lpf_.tail<3>());
        // f_ee_a(5) = 0.0;
        // ---------------------------------------------------------------------------------------------

        debug_file<< f_ee_a.transpose()<<" "<< f_contact_.transpose()<<std::endl;


        f_star.head<3>() = R_init*f_ee_m.head<3>();
        f_star.tail<3>() = R_init*f_ee_m.tail<3>();
        // f_star.setZero();


        f_d.setZero();
        f_d.head<3>() = R_init*f_ee_a.head<3>();
        // f_d.tail<3>() = R_init*f_ee_a.tail<3>();

        tau_nll = 4.0*(model_.q_init_ - model_.q_) - 0.1*(model_.qd_);
        tau_d = model_.J_.transpose() * (model_.A_ * f_star + f_d)+ model_.NLE_ + model_.N_ * tau_nll;
        // tau_d = tau_nll;

        return tau_d;
    }


    // masi : Move Approach Search Insert (MASI)
    Eigen::VectorXd Fr3Controller::masi(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                        const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                        const double duration)
    {
        // tak sequence
        // move -> approach -> search -> insert -> homing

        double offset = 0.1; // 1cm offset to avoid collision between robot & object

        Eigen::Vector3d reach_x_goal;
        Eigen::Vector8d ctrl;

        reach_x_goal = x_goal;
        reach_x_goal(2) = x_goal(2) + offset;

        if(set_init_){
            model_.setInitialValues();
            set_init_ = false;
            t_init_ = t_;
            f_bias_ = f_ext_lpf_;

            std::cout<<"MASI ..."<<std::endl;
        }
    
        Eigen::Vector2d time;              
        double dist;
        std::string current_task = estimator_.getTargetTask();

        double approach_v = 0.02;
        double spiral_p = 0.002; // 5 mm
        double spiral_v = 0.01; // 10 mm/s 
        double spiral_f = 0.8; // 1 N
        double insert_f = 0.8;


        f_contact_ = f_ext_lpf_ - f_bias_;

        // std::cout<<current_task<<std::endl;
        switch (est_map_[current_task])
        {
            case 1:    
                time(0) = t_init_;
                time(1) = t_init_ + duration; //
    
                tau_d_ = Fr3Controller::taskMove(reach_x_goal, r_goal, time);
                gf_ = Fr3Controller::gripperClose();  
                
                dist = (model_.p_.head<2>() - reach_x_goal.head<2>()).norm();

                if(estimator_.setMove(dist)){
                    model_.setInitialValues();
                    model_.setTimeStamp(t_);
                    f_bias_ = f_ext_lpf_;
                    std::cout<<"MOVE COMMAND IS COMPLETED ..."<<std::endl;
                }

                if( t_ >= t_init_ + duration + 1.0)
                {
                    estimator_.setMove(0.0001); //enforce to complte the task
                    model_.setInitialValues();
                    model_.setTimeStamp(t_);
                    f_bias_ = f_ext_lpf_;
                    std::cout<<"MOVE COMMAND IS not COMPLETED ..."<<std::endl;
                }
                break;

            case 2:                
                tau_d_ = Fr3Controller::approach(approach_v, model_.t_stamp_, 2);
                gf_ = Fr3Controller::gripperClose();  

                if(estimator_.setApproach(f_contact_(2)))
                {
                    model_.setInitialValues();
                    model_.setTimeStamp(t_);
                    f_bias_ = f_ext_lpf_;
                    std::cout<<"APPROACH IS COMPLETED"<<std::endl;
                }
                break;

            case 3:
                // Fr3Controller::idleState();
                tau_d_ = Fr3Controller::search(spiral_p, spiral_v, spiral_f, t_, model_.t_stamp_, 30.0);
                gf_ = Fr3Controller::gripperClose();

                if(estimator_.setSearch(f_contact_.head<2>())){
                    model_.setInitialValues();
                    model_.setTimeStamp(t_);
                    f_bias_ = f_ext_lpf_;
                    std::cout<<"SEARCH IS COMPLETED"<<std::endl;
                }
                break;

            case 4:
                // Fr3Controller::idleState();

                tau_d_ = Fr3Controller::insert(insert_f, t_, model_.t_stamp_);
                gf_ = Fr3Controller::gripperClose();
                
                if(estimator_.setInsert(v_lpf_(2))){
                    model_.setInitialValues();
                    model_.setTimeStamp(t_);
                    f_bias_ = f_ext_lpf_;
                    std::cout<<"INSERTION IS COMPLETED"<<std::endl;


                    Eigen::Matrix3d goal_rot;
                    Eigen::Vector3d euler;
                    goal_rot << 1, 0, 0, 0, -1, 0, 0, 0, -1;

                    for(int i = 0; i < 3; i++){
                        euler(i) = common_math::eulerAngleError(goal_rot.col(i), model_.r_.col(i));
                    }
                    
                    std::cout<<"Angle Error : "<< euler.transpose()<<std::endl;
                    if (euler(0) > 3.0 || euler(1) > 3.0){
                        std::cout<<"LARGE ANGLE ERROR"<<std::endl;
                    }
                    else{
                        std::cout<<"SMALL ANGLE ERROR"<<std::endl;
                    }

                }
                break;

            case 5:
                time(0) = model_.t_stamp_;
                time(1) = model_.t_stamp_ + 3.0;

                tau_d_ = Fr3Controller::setIdleConfig(time);
                gf_ = Fr3Controller::gripperOpen(0.04, time);                   
                set_attach_ = false;
                break;

            default:
                break;
            }

        double f_cut = 100;
        double ts = 1/(2*M_PI*f_cut);
        tau_d_lpf_ = common_math::lowPassFilter(tau_d_, tau_d_lpf_, 1/hz_, ts);

        // ctrl.head<7>() = tau_d_;
        ctrl.head<7>() = tau_d_lpf_;
        ctrl.tail<1>() = gf_.head<1>();

        save_f_ext<<t_<<" "<<est_map_[current_task]<<" "<< f_contact_.transpose()<<std::endl;

        save_task_p<<est_map_[current_task]<<" "<<x_.transpose()<<" "<<x_ee_.transpose()<<std::endl;
        save_task_v<<est_map_[current_task]<<" "<<v_lpf_.head<3>().transpose()<<" "<<v_ee_lpf_.head<3>().transpose()<<std::endl;
        save_joint_t<<t_<<" "<<est_map_[current_task]<<" "<< (tau_measured_ - model_.G_).transpose()<<" "<<tau_ext_.transpose()<<std::endl;

        Fr3Controller::saveState();
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

        // set_attach_ = true;

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

        return ctrl;
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

    }


    void Fr3Controller::saveState()
    {   
        save_joint_q<<t_<<" "<< q_.transpose()<<std::endl;
        // save_task_p<<t_<<" "<<x_.transpose()<<" "<<v_lpf_.head<3>().transpose()<<std::endl;
    }
}