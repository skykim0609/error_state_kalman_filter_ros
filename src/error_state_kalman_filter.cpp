#include "eskf/error_state_kalman_filter.h"

// Initialize Static member variables
Mat33 ESKF::I33 = Mat33::Identity();
//Mat44 ESKF::I44 = Mat44::Identity();
Mat33 ESKF::O33 = Mat33::Zero();
//Mat44 ESKF::O44 = Mat44::Zero();
//Mat34 ESKF::O34 = Mat34::Zero();
Mat43 ESKF::O43 = Mat43::Zero();
Mat1515 ESKF::I1515 = Mat1515::Identity();

ESKF::ESKF()
        : measurement_noise_(), process_noise_(),
          X_nom_(), dX_(),
          emergency_reset_rules_(),
          isInitialized_(false), isBiasInitialized_(false), init_buffer_(nullptr){
    // initialize error covariance matrix
    P_ = CovarianceMat::Identity()*0.005;

    // initialize Fi matrix
    Fi_ << O33, O33, O33, O33,
            I33, O33, O33, O33,
            O33, I33, O33, O33,
            O33, O33, I33, O33,
            O33, O33, O33, I33;

    ba_init_ << 0.0, 0.0, 0.0;
    bg_init_ << 0.0, 0.0, 0.0;

    X_nom_.setBiasAcc(ba_init_);
    X_nom_.setBiasGyro(bg_init_);

    double cutoff_frequency = 5.0; // Hz
    double sampling_rate = 100.0;
    lpf_gyro_ = new LowPassFilter<Vec3>(cutoff_frequency, sampling_rate);
    lpf_acc_  = new LowPassFilter<Vec3>(cutoff_frequency, sampling_rate);

    std::cout << "Error State Kalman Filter - constructed\n";
}

ESKF::~ESKF(){
    std::cout << "Error State Kalman Filter - destructed\n";
    delete lpf_gyro_;
};

void ESKF::setInitBuffer(int N_init, double w_e, double w_r, int N_acc, const Mat33 &Sigma_iw) {
    if(isBiasInitialized_)
        throw std::runtime_error("setInitBuffer() should be called before 'isBiasInitialized_ == True'\n");

    init_buffer_ = std::make_unique<InitializationBuffer>(N_init, w_e, w_r, N_acc, Sigma_iw);
}

void ESKF::setRotationFromBodyToIMU(const Mat33& R_BI)
{
    if(isInitialized_)
        throw std::runtime_error("setRotationFromBodyToIMU() can only be executed when 'isInitialized_ == false'\n");

    fixed_param_.setRotationFromBodyToIMU(R_BI);

    std::cout << "ESKF::setRotationFromBodyToIMU() with a 3D rotation matrix input:\n";
    std::cout << R_BI << std::endl;
};


void ESKF::setRotationFromBodyToIMU(const Vec4& q_BI)
{
    if(isInitialized_)
        throw std::runtime_error("setRotationFromBodyToIMU() can only be executed when 'isInitialized_ == false'\n");

    fixed_param_.setRotationFromBodyToIMU(q_BI);

    std::cout << "ESKF::setRotationFromBodyToIMU() with a quaternion input:\n";
    std::cout << q_BI.transpose() << std::endl;

};

void ESKF::setTransformFromCameraToIMU(const geometry::Tf &T_CI) {
    if(isInitialized_)
        throw std::runtime_error("setTransformFromCameraToIMU() can only be executed when 'isInitialized_ == false'\n");
    fixed_param_.setTransformFromCameraToIMU(T_CI);

    std::cout <<"ESKF::setTransformFromCameraToIMU() with a Tf input\n";
    std::cout <<"p_ci, q_ci"<<T_CI.getVec7Form().transpose()<<"\n";
}

void ESKF::setBias(double bias_ax, double bias_ay, double bias_az, double bias_gx, double bias_gy, double bias_gz)
{
    // Manually set bias.
    if(isBiasInitialized_)
        throw std::runtime_error("setBias() can only be executed when 'isBiasInitialized_ == false'\n");

    ba_init_ << bias_ax, bias_ay, bias_az;
    bg_init_ << bias_gx, bias_gy, bias_gz;

    X_nom_.setBiasAcc(ba_init_);
    X_nom_.setBiasGyro(bg_init_);

    std::cout << "ESKF::setBias()...\n";
    std::cout << "   Set bias (acc ): " << ba_init_.transpose() << "\n";
    std::cout << "   Set bias (gyro): " << bg_init_.transpose() << "\n";

    isBiasInitialized_ = true;
};

void ESKF::setIMUNoise(double noise_acc, double noise_gyro) {
    if(isInitialized_)
        throw std::runtime_error("setIMUNoise() can only be executed when 'isInitialized_ == false'\n");

    process_noise_.setNoise(noise_acc, noise_gyro, 1e-9, 1e-9);

    std::cout << "ESKF::setIMUNoise()...\n";
    std::cout << "   Set noise_acc : " << noise_acc << "\n";
    std::cout << "   Set noise_gyro: " << noise_gyro << "\n";
};

void ESKF::setObservationNoise(double noise_position, double noise_orientation){
    if(isInitialized_)
        throw std::runtime_error("setObservationNoise() can only be executed when 'isInitialized_ == false'\n");

    measurement_noise_.setNoise(noise_position, noise_orientation);

    std::cout << "ESKF::setObservationNoise()...\n";
    std::cout << "   Set noise_position : " << noise_position << "\n";
    std::cout << "   Set noise_orientation: " << noise_orientation << "\n";
};

void ESKF::setApriltagWorldPoses(const ApriltagInfoArr &apriltag_world_poses) {
    apriltag_world_poses_ = apriltag_world_poses;
    geometry::Tf T_wr; // ref pose
    if(!apriltag_world_poses_.getTxt(ref_tag_id_, T_wr)){
        throw std::runtime_error("Apriltag World Pose should contain ref_tag_id");
    }
    geometry::Tf T_rw = T_wr.Inverse();
    for(const auto& tag_info : apriltag_world_poses_.tag_infos){
        if(tag_info.id == ref_tag_id_) continue; // no need for ref
        else{
            geometry::Tf T_rt = T_rw.mult(tag_info.T_xt);
            apriltag_ref_poses_.push_back(tag_info.id, T_rt);
        }
    }
}

[[nodiscard]] bool ESKF::isInitialized() const{
    return isInitialized_;
};

[[nodiscard]] bool ESKF::isBiasInitialized() const {
    return isBiasInitialized_;
}

void ESKF::predict(const Vec3& am, const Vec3& wm, double t_now){
    if( !isInitialized_ || !isBiasInitialized_) {
        std::cout << "ESKF - predict() : FILTER/Bias IS NOT INITIALIZED YET...\n";
        return;
    }

    // Do implementation
#ifdef VERBOSE_STATE
    std::cout << "Predict...\n";
#endif

    // Low Pass Filtering
    lpf_gyro_->doFilterAndGetEstimation(wm, t_now);
    lpf_acc_->doFilterAndGetEstimation(am, t_now);

    double dt = t_now - t_prev_;
    if(dt > 0.05) {
        std::cout << " WARNNING: TIME MIGHT BE PASSED TOO LONG!... 'dt' is set to 0.05 s.\n";
        dt = 0.05;
    }
    t_prev_ = t_now;

    // Do prediction
    NominalState X_nom_predict;
    ErrorState   dX_predict;

    // 1. nominal state prediction
    predictNominal(X_nom_, am, wm, dt,
                   X_nom_predict);

    // 2. error-state prediction
    FMat F0;
    expmFMat eF0dt;
    errorStateF(X_nom_, am, wm,
                F0);

    this->expm_FMat(F0,dt,5,
                    eF0dt);

    predictError(eF0dt, dX_,
                 dX_predict);

    // 3. Error Covariance propagation
    CovarianceMat P_predict = eF0dt * P_ * eF0dt.transpose() + Fi_*process_noise_.Q*Fi_.transpose();

    if(!emergency_reset_rules_.isPositionInRange(X_nom_predict)){
        std::cout << "==========WANNING!! - estimated position is out of the position limit range!\n";
    }
    //else state OK

    // replace the old with the new.
    X_nom_.replace(X_nom_predict);
    dX_.replace(dX_predict);
    P_ = P_predict;

#ifdef VERBOSE_STATE
    X_nom_.show();
    std::cout <<"--------------------------" << std::endl;
#endif

};

void ESKF::updateOptitrack(const Vec3& p_observe, const Vec4& q_observe, double t_now){

    // Do implementation
#ifdef VERBOSE_STATE
    std::cout << "Update...\n";
#endif
    if(!isInitialized_){
        isInitialized_ = true;
        t_init_ = t_now;
        std::cout << "FILTER INITIALIZED. time: " << t_now - t_init_;
        X_nom_.setPosition(Vec3::Zero());
        X_nom_.setQuaternion(Vec4(1.0, 0.0, 0.0, 0.0));
        X_nom_.show();
        geometry::Tf T_rx_init(p_observe, q_observe);
        T_rx_init_.emplace(0, T_rx_init);
        T_xr_init_.emplace(0, T_rx_init.Inverse());
        std::cout << " Initial transform T_WB(0) : \n" << T_rx_init << "\n";
        return;
    }

    Vec3 p_obs;
    Vec4 q_obs;
    // transform T_wb into T_ni

//    p_obs = fixed_param_.R_IB*p_observe;
//    q_obs = geometry::q1_mult_q2(geometry::q1_mult_q2(fixed_param_.q_IB, q_observe), fixed_param_.q_BI); // modified by KGC ( error in original code)
    const geometry::Tf& T_xr_init = T_xr_init_.at(0);
    Vec4 q_IBB0W = geometry::q1_mult_q2(fixed_param_.q_IB, T_xr_init.rot()); // T_xr_init_ = T_bw(0)
    p_obs = fixed_param_.R_IB * T_xr_init.trans() + geometry::rotate_vec(q_IBB0W, p_observe); //
    q_obs = geometry::q1_mult_q2(q_IBB0W, geometry::q1_mult_q2(q_observe, fixed_param_.q_BI)); // q_IB * q_BW(0) * q_WB(k) * q_BI

    if(!isBiasInitialized_){
        X_nom_.setPosition(p_obs);
        X_nom_.setQuaternion(q_obs);
        t_prev_ = t_now;
        return;
    }

    Vec7 y;
    y << p_obs,q_obs;

    // calculate Linearized observation matrix
    HMat H;
    calcH(X_nom_, H);

    // Kalman gain
    KMat K;
    K = P_*H.transpose()*(H*P_*H.transpose() + measurement_noise_.R).inverse();

    Vec7 y_hat;
    y_hat << (X_nom_.p+dX_.dp),(geometry::q_right_mult(geometry::rotvec2q(dX_.dth))*X_nom_.q);

    ErrorStateVec dX_update_vec;
    ErrorStateVec dX_addition_vec;
    dX_addition_vec =  K*(y-y_hat);
    dX_update_vec = dX_.getVectorform() + dX_addition_vec;

    ErrorState dX_update;
    dX_update.replace(dX_update_vec);

    // Injection of the observed error into the nominal
    X_nom_.injectErrorState(dX_update);

    // Reset dX
    dX_.replace(dX_addition_vec);

    // Replace Error Covariance Matrix
    CovarianceMat P_update = (I1515-K*H)*P_*(I1515-K*H).transpose() + K*measurement_noise_.R*K.transpose();    // dg_ddX << I66, O63, O63,
    //           O36, I33-skewMat(0.5*dX_update.dth), O36,
    //           O66, O63, I66;
    // P_ = dg_ddX*P_*dg_ddX.transpose();
    P_ = P_update;

#ifdef VERBOSE_STATE
    X_nom_.show();
#endif
};

void ESKF::updateAprilTag(const ApriltagInfoArr &apriltag_detections, double t_now) {
#ifdef VERBOSE_STATE
    std::cout <<"Update...\n";
#endif
    if(!isInitialized_) { // bias is initialized at this point usually.
        ApriltagInfo ref_tag;
        bool found_ref = apriltag_detections.find(ref_tag_id_, ref_tag);
        if(!found_ref) return; // should first see ref_tag_id_ since it would be the better observable tag
        isInitialized_ = true;
        t_init_ = t_now;
        std::cout << "Initial Pose from AprilTag obtained.\n";
        X_nom_.setPosition(Vec3::Zero());
        X_nom_.setQuaternion(Vec4(1.0, 0.0, 0.0, 0.0));
        X_nom_.setVelocity(Vec3(0.0, 0.0, 0.0));
        X_nom_.show();
        geometry::Tf T_xr_init = ref_tag.T_xt; // T_ct0(0)
        geometry::Tf T_rx_init = T_xr_init.Inverse(); // T_ct0(0)
        T_xr_init_.emplace(ref_tag_id_, T_xr_init);
        T_rx_init_.emplace(ref_tag_id_, T_rx_init);
        std::cout << " Initial pose of tag " << ref_tag_id_ <<" from camera T_ct(0) : \n" << T_xr_init << "\n";
        for(const auto& tag_ref_pose : apriltag_ref_poses_.tag_infos){
            int id =tag_ref_pose.id;
            geometry::Tf T_xr = T_xr_init.mult(tag_ref_pose.T_xt); // T_ct0 * T_t0t1
            geometry::Tf T_rx = T_xr.Inverse(); // T_t1 c
            T_xr_init_.emplace(id, T_xr);
            T_rx_init_.emplace(id, T_rx);
        }
        return;
    }
    for(const auto& det : apriltag_detections.tag_infos){
        int id = det.id;
        geometry::Tf T_rt;
        if(id == ref_tag_id_){
            T_rt = geometry::Tf::Identity();
        }
        else {
            if(!apriltag_ref_poses_.getTxt(id, T_rt)) continue;
        }
        geometry::Tf T_ct = det.T_xt;
        geometry::Tf T_tc = T_ct.Inverse();
        geometry::Tf T_xr_init = T_xr_init_.at(id);
        // transform to nominal_to_imu transform
        geometry::Tf T_NI = fixed_param_.T_IC.mult(T_xr_init).mult(T_tc).mult(fixed_param_.T_CI); // T_IC(0) * T_CT(0) * T_TC(k) * T_CI(k)
        if(!isBiasInitialized_){ // won't happen since !isInitialized contains !isBiasInitialized
            X_nom_.setPosition(T_NI.trans());
            X_nom_.setQuaternion(T_NI.rot());
            t_prev_ = t_now;
            return;
        }
        Vec6 y = T_NI.getVec6Form();
        // calulate linearized observation matrix
        HMat6 H;
        calcH6(X_nom_, H);
        RMat6 Cov_NI = calcCovNI6(T_ct.rotmat(), det.S);

        // Kalman gain
        KMat6 K = P_ * H.transpose() * (H * P_ * H.transpose() + Cov_NI).inverse();
        // recovered nominal state + error state
        Vec6 y_hat;
        y_hat.head<3>() = X_nom_.p + dX_.dp;
        y_hat.tail<3>() = geometry::q2rotvec(geometry::q1_mult_q2(X_nom_.q, geometry::rotvec2q(dX_.dth)));

        ErrorStateVec dX_addition_vec  =  K*(y-y_hat);
        ErrorStateVec dX_update_vec = dX_.getVectorform() + dX_addition_vec;

        ErrorState dX_update;
        dX_update.replace(dX_update_vec);

        // Injection of the observed error into the nominal
        X_nom_.injectErrorState(dX_update);

        // Reset dX
        dX_.replace(dX_addition_vec);

        // Replace Error Covariance Matrix
        CovarianceMat P_update = (I1515-K*H)*P_*(I1515-K*H).transpose() + K * Cov_NI * K.transpose();
        P_ = P_update;
    }
#ifdef VERBOSE_STATE
    X_nom_.show();
#endif
}

void ESKF::updateGravity(const Vec3& am, double t_now){

    // Do implementation
#ifdef VERBOSE_STATE
    std::cout << "Update...\n";
#endif
    // double ax = am(0);
    // double ay = am(1);
    // double az = am(2);

    //double a_mag = am.norm();

    // Vec2 rp_obs;
    // rp_obs(0) = atan2(ay,az);
    // rp_obs(1) = atan2(ax,sqrt(ay*ay + az*az));

    Vec3 grav_Ik_measure;
    grav_Ik_measure  = am;
    // grav_Ik_measure -= X_nom_.ba;

    if(grav_Ik_measure.norm() > grav_est_min && grav_Ik_measure.norm() < grav_est_max){

        Matrix3d R_B0Ik = geometry::q2r(X_nom_.q);
        Vec3 grav_Ik_est = R_B0Ik.transpose()*fixed_param_.R_IB*fixed_param_.grav;

        // std::cout << "norm grav Ik est  : " << grav_Ik_est.norm() <<"\n";
        // std::cout << "norm grav Ik meas : " << grav_Ik_measure.norm() <<"\n";

        // std::cout << " grav IK meausre: " << grav_Ik_measure.transpose() << std::endl;
        // std::cout << " grav IK est    : " << grav_Ik_est.transpose() << std::endl;

    }

};

ESKF::FixedParameters ESKF::getFixedParameters(){
    return fixed_param_;
};

void ESKF::getFilteredStates(NominalState& X_nom_filtered) {
    this->X_nom_.copyTo(X_nom_filtered);
};


void ESKF::getGyroLowPassFiltered(Vec3& filtered_gyro){
    filtered_gyro = this->lpf_gyro_->getFilteredValue();
};
void ESKF::getAccLowPassFiltered(Vec3& filtered_acc){
    filtered_acc = this->lpf_acc_->getFilteredValue();
};

void ESKF::getCovariance(Mat1515 &S) {
    S = this->P_;
}

void ESKF::showFilterStates(){
    std::cout << "---- Current estimation ----\n";
    std::cout << "p: " << X_nom_.p.transpose() << " m \n";
    std::cout << "v: " << X_nom_.v.transpose() << " m/s\n";
    std::cout << "q: " << X_nom_.q.transpose() << "\n";
    std::cout << "ba: " << X_nom_.ba.transpose() << " m/s2\n";
    std::cout << "bg: " << X_nom_.bg.transpose() << " rad/s\n";
    std::cout << "std_dp : " << std::sqrt(P_(0,0))   << "," << std::sqrt(P_(1,1))   << "," << std::sqrt(P_(2,2))   << " m\n";
    std::cout << "std_dv : " << std::sqrt(P_(3,3))   << "," << std::sqrt(P_(4,4))   << "," << std::sqrt(P_(5,5))   << " m/s \n";
    std::cout << "std_dq : " << std::sqrt(P_(6,6))   << "," << std::sqrt(P_(7,7))   << "," << std::sqrt(P_(8,8))   << "\n";
    std::cout << "std_dba: " << std::sqrt(P_(9,9))   << "," << std::sqrt(P_(10,10)) << "," << std::sqrt(P_(11,11)) << "\n";
    std::cout << "std_dbg: " << std::sqrt(P_(12,12)) << "," << std::sqrt(P_(13,13)) << "," << std::sqrt(P_(14,14)) << "\n\n";
};

// Added by KGC
ESKF::NominalState ESKF::getWorldFrameState(const geometry::Tf &T_wr, const std::string& mode) const {
    geometry::Tf T_NI(X_nom_.q, X_nom_.p);
    geometry::Tf T_XI, T_rx_init;
    if(mode == "optitrack"){
        T_XI = geometry::Tf(fixed_param_.q_BI, Vec3::Zero());
    }
    else{
        T_XI = geometry::Tf(fixed_param_.T_CI);
    }
    T_rx_init = T_rx_init_.at(ref_tag_id_);
    geometry::Tf T_WI0 = T_wr.mult(T_rx_init).mult(T_XI);
    geometry::Tf T_WI = T_WI0.mult(T_NI);
    ESKF::NominalState X_wi;
    X_wi.setPosition(T_WI.trans());
    X_wi.setQuaternion(T_WI.rot());
    Vec3 v_WI = geometry::rotate_vec(T_WI0.rot(), X_nom_.v);
    X_wi.setVelocity(v_WI);
    X_wi.setBiasAcc(X_nom_.ba);
    X_wi.setBiasGyro(X_nom_.bg);

    return X_wi;
}


// private

void ESKF::predictNominal(const NominalState& X_nom, const Vec3& am, const Vec3& wm, double dt,
                          NominalState& X_nom_update){
    Matrix3d R_WIk = geometry::q2r(X_nom.q);

    // Update nominal state
    Vec3 Adt = (R_WIk * (am - X_nom.ba) - fixed_param_.grav) * dt;
    X_nom_update.p  = X_nom.p + (X_nom.v + 0.5*Adt)*dt;
    X_nom_update.v  = X_nom.v + Adt;
    X_nom_update.q  = geometry::q_right_mult(geometry::rotvec2q((wm-X_nom.bg)*dt))*X_nom.q;
    X_nom_update.ba = X_nom.ba;
    X_nom_update.bg = X_nom.bg;
};

void ESKF::predictError(const expmFMat& eF0dt, const ErrorState& dX,
                        ErrorState& dX_update)
{
    // Update nominal state
    ErrorStateVec dX_vec_update;
    dX_vec_update = eF0dt*(dX.getVectorform());

    dX_update.replace(dX_vec_update);
};

void ESKF::errorStateF(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                       FMat& res)
{
    Matrix3d R_B0Ik = geometry::q2r(X_nom.q);
    res.setZero();
    res << O33, I33, O33, O33, O33,
            O33, O33, -R_B0Ik*geometry::skewMat(am-X_nom.ba),-R_B0Ik, O33,
            O33, O33, -geometry::skewMat(wm-X_nom.bg), O33, -I33,
            O33, O33, O33, O33, O33,
            O33, O33, O33, O33, O33;
};

void ESKF::expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                     expmFMat& expmF){

    expmF.setZero();

    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double dt4 = dt3*dt;
    double dt5 = dt4*dt;
    double mul2 = 0.5;
    double mul3 = 1.0/6.0;
    double mul4 = 1.0/24.0;
    double mul5 = 1.0/120.0;

    double dtm2 = mul2*dt2;
    double dtm3 = mul3*dt3;
    double dtm4 = mul4*dt4;
    double dtm5 = mul5*dt5;

    // fourth order approx?
    // expmF = I + F0*dt + 0.5*F0F0*dtdt  + 1/6*F0F0F0*dtdtdt
    //           + 1/24*F0F0F0F0*dtdtdtdt + 1/120*F0F0F0F0F0*dtdtdtdtdt;
    Matrix3d RBI       = -F.block<3,3>(3,9);

    Matrix3d skW     =  -F.block<3,3>(6,6);
    Matrix3d skW2    =  skW *skW;
    Matrix3d skW3    =  skW2*skW;
    Matrix3d skW4    =  skW3*skW;
    Matrix3d skW5    =  skW4*skW;

    Matrix3d RBI_skA       = -F.block<3,3>(3,6);
    Matrix3d RBI_skA_skW   = RBI_skA*skW;
    Matrix3d RBI_skA_skW2  = RBI_skA_skW*skW;
    Matrix3d RBI_skA_skW3  = RBI_skA_skW2*skW;
    Matrix3d RBI_skA_skW4  = RBI_skA_skW3*skW;


    Matrix3d I33 = Matrix3d::Identity();
    Matrix3d O33 = Matrix3d::Zero();

    if(max_approx_order < 2) max_approx_order = 2;
    if(max_approx_order > 5) max_approx_order = 5;

    if(max_approx_order >= 0){
        // 0th order 
        BLOCK33(expmF,0,0) = I33;
        BLOCK33(expmF,1,1) = I33;
        BLOCK33(expmF,2,2) = I33;
        BLOCK33(expmF,3,3) = I33;
        BLOCK33(expmF,4,4) = I33;
    }

    if(max_approx_order >= 1){
        // 1st order
        BLOCK33(expmF,0,1) += I33*dt;
        BLOCK33(expmF,1,2) += -RBI_skA*dt;
        BLOCK33(expmF,1,3) += -RBI*dt;
        BLOCK33(expmF,2,2) += -skW*dt;
        BLOCK33(expmF,2,4) += -I33*dt;
    }

    if(max_approx_order >= 2){
        // 2nd order
        BLOCK33(expmF,0,2) += -RBI_skA*dtm2;
        BLOCK33(expmF,0,3) += -RBI*dtm2;
        BLOCK33(expmF,1,2) += RBI_skA_skW*dtm2;
        BLOCK33(expmF,1,4) += RBI_skA*dtm2;
        BLOCK33(expmF,2,2) += skW2*dtm2;
        BLOCK33(expmF,2,4) += skW*dtm2;
    }

    if(max_approx_order >= 3){
        // 3rd order
        BLOCK33(expmF,0,2) += RBI_skA_skW*dtm3;
        BLOCK33(expmF,0,4) += RBI_skA*dtm3;
        BLOCK33(expmF,1,2) += -RBI_skA_skW2*dtm3;
        BLOCK33(expmF,1,4) += -RBI_skA_skW*dtm3;
        BLOCK33(expmF,2,2) += -skW3*dtm3;
        BLOCK33(expmF,2,4) += -skW2*dtm3;
    }

    if(max_approx_order >= 4){
        // 4th order
        BLOCK33(expmF,0,2) += -RBI_skA_skW2*dtm4;
        BLOCK33(expmF,0,4) += -RBI_skA_skW*dtm4;
        BLOCK33(expmF,1,2) += RBI_skA_skW3*dtm4;
        BLOCK33(expmF,1,4) += RBI_skA_skW2*dtm4;
        BLOCK33(expmF,2,2) += skW4*dtm4;
        BLOCK33(expmF,2,4) += skW3*dtm4;
    }

    if(max_approx_order >=5){
        // 5th order
        BLOCK33(expmF,0,2) += RBI_skA_skW3*dtm5;
        BLOCK33(expmF,0,4) += RBI_skA_skW2*dtm5;
        BLOCK33(expmF,1,2) += -RBI_skA_skW4*dtm5;
        BLOCK33(expmF,1,4) += -RBI_skA_skW3*dtm5;
        BLOCK33(expmF,2,2) += -skW5*dtm5;
        BLOCK33(expmF,2,4) += -skW4*dtm5;
    }

};

void ESKF::calcH(const NominalState& X_nom,
                 HMat& H) const
{
    H.setZero();

    Mat43 mat;
    mat << 0,0,0, 0.5*Mat33::Identity();
    H << I33, O33, O33, O33, O33,
            O43, O43, geometry::q_left_mult(X_nom.q)*mat, O43, O43;
};

void ESKF::calcH6(const ESKF::NominalState &X_nom, HMat6 &H) const{
    H.setZero();
    H << I33, O33, O33, O33, O33,
    O33, O33, I33, O33, O33;
}

[[nodiscard]] RMat6 ESKF::calcCovNI6(const Mat33 &R_ct, const RMat6 &Cov_ct) const {
    RMat6 M_left;
    M_left.setZero();

    geometry::Tf T_xr_init = T_xr_init_.at(ref_tag_id_);
    Mat33 R_i0t = fixed_param_.T_IC.rotmat() * T_xr_init.rotmat();  //  q_ic * q_ct(0)00
    Mat33 R_i0tct = R_i0t * R_ct;
    M_left.block<3, 3>(0, 0) = R_i0tct;
    M_left.block<3, 3>(3, 3) = R_i0t;
    RMat6 Cov_NI = M_left * Cov_ct * M_left.transpose();
    return Cov_NI;
}

bool ESKF::tryInitializeBias(const Vec3 &am, const Vec3 &wm, double t) {
    /**
     * try Initialization of Bias. Only if not initialized
     */
    if(isBiasInitialized_){
        std::cerr<<"The bias is already initialized. \n";
        return false;
    }
    if(init_buffer_ == nullptr){
        std::cerr<<"The init_buffer_ parameters are not set...\n";
        return false;
    }

    bool added = init_buffer_->addImuData(wm, am, t);
    if(init_buffer_->isInitializationReady()){
        std::pair<Vec3, Vec4> ba_dq_pair = init_buffer_->estimateAccBias(fixed_param_.q_IW_init_nominal, fixed_param_.grav);
        ba_init_ = ba_dq_pair.first;
        bg_init_ = init_buffer_->estimateGyroBias();
        std::cout<<"Bias Initialized!\n";
        std::cout<<"ba_init : "<<ba_init_.transpose()<<"\n";
        std::cout<<"bg_init : "<<bg_init_.transpose()<<"\n";
        std::cout<<"q_IW_init_nominal difference : "<<ba_dq_pair.second.transpose() <<"\n";
        fixed_param_.q_IW_init_nominal = geometry::q1_mult_q2(fixed_param_.q_IW_init_nominal, ba_dq_pair.second);
        X_nom_.setBiasGyro(bg_init_);
        X_nom_.setBiasAcc(ba_init_);
        isBiasInitialized_ = true;
    }

    return isBiasInitialized_;
}

void ESKF::test_FMat(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                     FMat& res)
{
    this->errorStateF(X_nom, am, wm, res);
    std::cout << "FMat:\n";
    std::cout << res << std::endl;
};

void ESKF::test_expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                          expmFMat& res)
{
    this->expm_FMat(F,dt, max_approx_order, res);
    std::cout << "expm_FMat:\n";
    std::cout << res << std::endl;
};