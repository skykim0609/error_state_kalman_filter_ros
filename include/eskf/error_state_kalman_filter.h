#ifndef ERROR_STATE_KALMAN_FILTER_H_
#define ERROR_STATE_KALMAN_FILTER_H_

#include <iostream>
#include <memory>
#include <limits>
#include <map>
#include <Eigen/Dense>

#include "eskf/geometry_library.h"
#include "eskf/low_pass_filter.h"
#include "eskf/random_utils.h"
#include "eskf/apriltag_info.h"

// #define VERBOSE_STATE

using namespace Eigen;
typedef Matrix<double,15,1> ErrorStateVec; //p, v, r, b_a, b_g
typedef Matrix<double,16,1> NominalStateVec;

typedef Matrix<double,15,15> CovarianceMat;
typedef Matrix<double,15,15> FMat;
typedef Matrix<double,15,15> expmFMat;
typedef Matrix<double,7,15>  HMat;
typedef Matrix<double,15,7>  KMat;
typedef Matrix<double,7,7>   RMat;
typedef Matrix<double,12,12> QMat;


// for 6DoF representation of pose
typedef Matrix<double, 6, 15> HMat6;
typedef Matrix<double, 15, 6> KMat6;
typedef Matrix<double, 6, 6> RMat6;

/**
 * Kalman filter formulation:
 */

typedef Matrix<double,15,15> Mat1515;

inline static constexpr double GRAVITY_MAGNITUDE = 9.7996; // Measured in Seoul, Korea
inline static constexpr double grav_est_min = GRAVITY_MAGNITUDE - 0.02;
inline static constexpr double grav_est_max = GRAVITY_MAGNITUDE + 0.02;


#define POW2(x) ((x)*(x))
#define BLOCK33(A,i,j) ((A).block<3,3>(3*i,3*j))

class ESKF{
public:

    struct FixedParameters;

    struct ErrorStateCovariance;
    struct NominalState;
    struct ErrorState;

    struct ProcessNoise;
    struct MeasurementNoise;

    struct EmergencyResetRules;

    struct InitializationBuffer;
private:
    // Related to Kalman filter implementation .
    void predictNominal(const NominalState& X_nom, const Vec3& am, const Vec3& wm, double dt,
                        NominalState& X_nom_update);
    void predictError(const expmFMat& eF0dt, const ErrorState& dX,
                      ErrorState& dX_update);

    void errorStateF(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                     FMat& res);
    void expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                   expmFMat& expmF);
    void calcH(const NominalState& X_nom, HMat& H) const;
    void calcH6(const NominalState& X_nom, HMat6& H) const;
    [[nodiscard]] RMat6 calcCovNI6(const Mat33& R_ct, const RMat6& Cov_ct) const;

public:
    ESKF();
    ~ESKF();

public:
    // Added by KGC
    void setInitBuffer(int N_init, double w_e, double w_r, int N_acc, const Mat33& Sigma_iw);

    void setRotationFromBodyToIMU(const Mat33& R_BI);
    void setRotationFromBodyToIMU(const Vec4& q_BI);

    void setTransformFromCameraToIMU(const geometry::Tf& T_CI);

    void setBias(double bias_ax, double bias_ay, double bias_az, double bias_gx, double bias_gy, double bias_gz);

    void setIMUNoise(double noise_acc, double noise_gyro);

    void setObservationNoise(double noise_position, double noise_orientation);

    void setApriltagWorldPoses(const ApriltagInfoArr& apriltag_world_poses);

    // Added by KGC
    void setRefTagId(int tag_id){
        ref_tag_id_ = tag_id;
    }

public:
    [[nodiscard]] bool isInitialized() const;
    [[nodiscard]] bool isBiasInitialized() const;

    // Added by KGC
    bool tryInitializeBias(const Vec3& am, const Vec3& wm, double t);

    void predict(const Vec3& am, const Vec3& wm, double t_now); // by imu
    // void updateMagnetometer(const Vec3& p_observe, const Vec4& q_observe); // by magnetometer
    void updateOptitrack(const Vec3& p_observe, const Vec4& q_observe, double t_now); // by optitrack
    void updateAprilTag(const ApriltagInfoArr &apriltag_detections, double t_now); // by Apriltag

    void updateGravity(const Vec3& am, double t_now);

    FixedParameters getFixedParameters();
    void getFilteredStates(NominalState& x_nom_filtered);
    void getGyroLowPassFiltered(Vec3& filtered_gyro);
    void getAccLowPassFiltered(Vec3& filtered_acc);
    void getCovariance(Mat1515& S);

    void showFilterStates();

    NominalState getWorldFrameState(const geometry::Tf &T_wr, const std::string& mode) const;

// Test functions
public:
    void test_FMat(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                   FMat& res);
    void test_expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                        expmFMat& res);

public:
    static Mat33 I33;
    static Mat44 I44;
    static Mat33 O33;
    static Mat44 O44;
    static Mat34 O34;
    static Mat43 O43;
    static Mat1515 I1515;

    struct FixedParameters{
        Mat33 R_BI; // SO(3), rotation only. drone body frame (== optitrack coordinate frame)
        // to the IMU frame
        Mat33 R_IB; // R_BI.transpose();
        Vec4  q_BI; // quaternion, rotation only. drone body frame to IMU frame
        Vec4  q_IB; // q_BI.conjugate();

        //geometry::Tf T_BI, T_IB;

        Vec3 grav; // gravity w.r.t. the global frame

        // camera to IMU transformation
        geometry::Tf T_CI, T_IC;

        // nominal q_IW_init.
        Vec4 q_IW_init_nominal;

        FixedParameters(){
            // R_BI << 1,0,0, 0,-1,0, 0,0,-1;
            R_BI << 0,0,1, 0,-1,0, 1,0,0;
            R_IB = R_BI.transpose();

            q_BI = geometry::r2q(R_BI);
            q_IB = geometry::q_conj(q_BI);

            // grav << 0.0, 0.0, -GRAVITY_MAGNITUDE; // VN100t
            // grav << GRAVITY_MAGNITUDE,0.0,0.0; // MPU9250
            //grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE); // ? Why? -> g_ib in initial imu frame?
            grav = Vec3(0.0, 0.0, GRAVITY_MAGNITUDE);

            Vec4 q_CI;
            Vec3 t_CI;
            q_CI << 1.0, 0.0, 0.0, 0.0;
            t_CI = Vec3::Zero();
            T_CI = geometry::Tf(q_CI, t_CI);
            T_IC = T_CI.Inverse();

            q_IW_init_nominal << 0.5, 0.5, -0.5, 0.5;

            // Added by KGC. for debugging purpose
            auto R_IW_init_nominal = geometry::q2r(q_IW_init_nominal);
            std::cout<<"R_IW_init_nominal : \n"<<R_IW_init_nominal<<"\n";
        };

        void setTransformFromCameraToIMU(const Vec4& q_CI_in, const Vec3& t_CI_in){
            T_CI = geometry::Tf(q_CI_in, t_CI_in);
            T_IC = T_CI.Inverse();
        }

        void setTransformFromCameraToIMU(const geometry::Tf& T_CI_in){
            T_CI = T_CI_in;
            T_IC = T_CI.Inverse();
        }

        void setNominalInitRotation(const Vec4& q_IW_init_nominal_in){
            q_IW_init_nominal = q_IW_init_nominal_in;
        }

        void setRotationFromBodyToIMU(const Mat33& R_BI_input){
            R_BI = R_BI_input;
            R_IB = R_BI.transpose();

            q_BI = geometry::r2q(R_BI);
            q_IB = geometry::q_conj(q_BI);
            grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE);
        };

        void setRotationFromBodyToIMU(const Vec4& q_BI_input){
            q_BI = q_BI_input;
            q_IB = geometry::q_conj(q_BI);

            R_BI = geometry::q2r(q_BI);
            R_IB = R_BI.transpose();
            grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE);
        };
    };

    struct NominalState{
        Vec3 p;
        Vec3 v;
        Vec4 q;
        Vec3 ba;
        Vec3 bg;
        NominalState() {
            p  = Vec3::Zero();
            v  = Vec3::Zero();
            q  = Vec4::Zero();  q(0) = 1.0;
            ba = Vec3::Zero();
            bg = Vec3::Zero();
        };
        NominalState(const NominalState& nom){
            p  = nom.p;
            v  = nom.v;
            q  = nom.q;
            ba = nom.ba;
            bg = nom.bg;
        };
        void setValues(const Vec3& pi, const Vec3& vi, const Vec4& qi, const Vec3& bai, const Vec3& bgi){
            p  = pi;
            v  = vi;
            q  = qi;
            ba = bai;
            bg = bgi;
        };
        void setPosition(const Vec3& pi)  { p  = pi;  };
        void setVelocity(const Vec3& vi)  { v  = vi;  };
        void setQuaternion(const Vec4& qi){ q  = qi;  };
        void setBiasAcc(const Vec3& bai)  { ba = bai; };
        void setBiasGyro(const Vec3& bgi) { bg = bgi; };
        void replace(const NominalState& nom){ // replace the current state with nom
            p  = nom.p;
            v  = nom.v;
            q  = nom.q;
            ba = nom.ba;
            bg = nom.bg;
        };
        void copyTo(NominalState& X_nom) const {
            X_nom.p  = p;
            X_nom.v  = v;
            X_nom.q  = q;
            X_nom.ba = ba;
            X_nom.bg = bg;
        };
        void injectErrorState(const ErrorState& dX){
            p  += dX.dp;
            v  += dX.dv;
            q = geometry::q_right_mult(geometry::rotvec2q(dX.dth))*q;
            ba += dX.dba;
            bg += dX.dbg;
        };
        void getTf(geometry::Tf& T){
            T = geometry::Tf(p, q);
        }

        void show() const{
            std::cout << "X_nom.p:"  << p.transpose()  << "\n";
            std::cout << "X_nom.v:"  << v.transpose()  << "\n";
            std::cout << "X_nom.q:"  << q.transpose()  << "\n";
            std::cout << "X_nom.ba:" << ba.transpose() << "\n";
            std::cout << "X_nom.bg:" << bg.transpose() << "\n\n";
        };
    };

    struct ErrorStateCovariance{
        Vec3 cov_dp;
        Vec3 cov_dv;
        Vec3 cov_dth;
        Vec3 cov_dba;
        Vec3 cov_dbg;

        ErrorStateCovariance() {
            cov_dp  = Vec3::Zero();
            cov_dv  = Vec3::Zero();
            cov_dth = Vec3::Zero();
            cov_dba = Vec3::Zero();
            cov_dbg = Vec3::Zero();
        };
        void setValues(const CovarianceMat& cov_mat){
            cov_dp  << cov_mat(0,0),   cov_mat(1,1),   cov_mat(2,2);
            cov_dv  << cov_mat(3,3),   cov_mat(4,4),   cov_mat(5,5);
            cov_dth << cov_mat(6,6),   cov_mat(7,7),   cov_mat(8,8);
            cov_dba << cov_mat(9,9),   cov_mat(10,10), cov_mat(11,11);
            cov_dbg << cov_mat(12,12), cov_mat(13,13), cov_mat(14,14);
        };
        void show(){
            std::cout << "cov.dp:"  << cov_dp.transpose() << "\n";
            std::cout << "cov.dv:"  << cov_dv.transpose() << "\n";
            std::cout << "cov.dth:" << cov_dth.transpose() << "\n";
            std::cout << "cov.dba:" << cov_dba.transpose() << "\n";
            std::cout << "cov.dbg:" << cov_dbg.transpose() << "\n\n";
        };
    };

    struct ErrorState{
        Vec3 dp;
        Vec3 dv;
        Vec3 dth;
        Vec3 dba;
        Vec3 dbg;

        ErrorStateCovariance covariance;

        ErrorState() {
            dp  = Vec3::Zero();
            dv  = Vec3::Zero();
            dth = Vec3::Zero();
            dba = Vec3::Zero();
            dbg = Vec3::Zero();
            covariance.setValues(CovarianceMat::Identity()*0.005);
        };
        ErrorState(const ErrorState& dX, const CovarianceMat& cov_mat){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
            covariance.setValues(cov_mat);
        };
        void replace(const ErrorState& dX){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
        };
        void replace(const ErrorState& dX, const CovarianceMat& cov_mat){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
            covariance.setValues(cov_mat);
        };
        void replace(const ErrorStateVec& dX_vec){
            dp = dX_vec.block<3,1>(0,0);
            dv = dX_vec.block<3,1>(3,0);
            dth = dX_vec.block<3,1>(6,0);
            dba = dX_vec.block<3,1>(9,0);
            dbg = dX_vec.block<3,1>(12,0);
        };
        void replace(const ErrorStateVec& dX_vec, const CovarianceMat& cov_mat){
            dp = dX_vec.block<3,1>(0,0);
            dv = dX_vec.block<3,1>(3,0);
            dth = dX_vec.block<3,1>(6,0);
            dba = dX_vec.block<3,1>(9,0);
            dbg = dX_vec.block<3,1>(12,0);
            covariance.setValues(cov_mat);
        };
        ErrorStateCovariance getCovariance(){
            return covariance;
        };

        ErrorStateVec getVectorform() const {
            ErrorStateVec vec;
            vec << dp, dv, dth, dba, dbg;
            return vec;
        };
    };

    struct ProcessNoise{
        double sig_na; // acceleration measurement noise (0.0008)
        double sig_ng; // gyro measurement noise (0.000006)
        double sig_nba; // acc. bias noise (1e-15)
        double sig_nbg; // gyro bias noise (1e-15)
        int dim;
        QMat Q;
        ProcessNoise() :
        sig_na(0.0008), sig_ng(0.000006), sig_nba(1e-12), sig_nbg(1e-12), dim(12) {
            Q = QMat::Identity();
            for(int i = 0; i < 3; ++i){
                Q(i,i) = POW2(sig_na);
                Q(3+i,3+i) = POW2(sig_ng);
                Q(6+i,6+i) = POW2(sig_nba);
                Q(9+i,9+i) = POW2(sig_nbg);
            }
        };

        void setNoise(double noise_acc, double noise_gyro, double noise_ba, double noise_bg)
        {
            if(noise_acc <= 0.0000001)
                throw std::runtime_error("sig_na should be larger then 0.0000001.");

            if(noise_gyro <= 0.0000001)
                throw std::runtime_error("sig_ng should be larger then 0.0000001.");

            if(noise_ba <= 0.0)
                throw std::runtime_error("sig_nba should be larger then 0.0.");

            if(noise_bg <= 0.0)
                throw std::runtime_error("sig_nbg should be larger then 0.0.");

            sig_na  = noise_acc;
            sig_ng  = noise_gyro;
            sig_nba = noise_ba;
            sig_nbg = noise_bg;

            Q = QMat::Identity();
            for(int i = 0; i < 3; ++i){
                Q(i,i) = POW2(sig_na);
                Q(3+i,3+i) = POW2(sig_ng);
                Q(6+i,6+i) = POW2(sig_nba);
                Q(9+i,9+i) = POW2(sig_nbg);
            }
        };
    };

    struct MeasurementNoise{
        double sig_p; // optitrack position noise // 0.005 (5 mm)
        double sig_q; // quaternion noise // 0.015 ( 0.015 rad)
        int dim;
        RMat R;
        MeasurementNoise() : sig_p(0.005), sig_q(0.01), dim(7) {
            R = RMat::Identity();
            for(int i = 0; i < 3; ++i) R(i,i) = POW2(sig_p);
            for(int i = 0; i < 4; ++i) R(3+i,3+i) = POW2(sig_q);
        };

        void setNoise(double noise_position, double noise_orientation){
            if(noise_position <= 0.0000001)
                throw std::runtime_error("noise_position should be larger then 0.0000001.");

            if(noise_orientation <= 0.0000001)
                throw std::runtime_error("noise_orientation should be larger then 0.0000001.");

            sig_p = noise_position;
            sig_q = noise_orientation;

            R = RMat::Identity();
            for(int i = 0; i < 3; ++i) R(i,i) = POW2(sig_p);
            for(int i = 0; i < 4; ++i) R(3+i,3+i) = POW2(sig_q);
        };
    };

    struct EmergencyResetRules{ // for Emergency state changer.
        double thres_quaternion; // in radian
        double thres_position;   // in meters
        double thres_cov_p;      // in meter^2
        double thres_cov_v;      // in m/2^2
        double thres_cov_q;      // in ???

        // Parameterize position limit
        Vec3 xyz_min_;
        Vec3 xyz_max_;

        EmergencyResetRules(){
            // default values
            thres_position    = 0.03;
            thres_quaternion  = cos(0.05); // 0.05 radians == 2.645916 degrees
            thres_cov_p       = 1.0;
            thres_cov_v       = 1.0;
            thres_cov_q       = 1.0;

            xyz_min_ << -10.0, -10.0, -0.5;
            xyz_max_ << 10.0, 10.0, 5.5;

        };

        EmergencyResetRules(double tp, double tq, double tcp, double tcv, double tcq, const Vec3& xyz_min, const Vec3& xyz_max):
                thres_position(tp), thres_quaternion(tq), thres_cov_p(tcp), thres_cov_q(tcq), thres_cov_v(tcv),
                xyz_min_(xyz_min), xyz_max_(xyz_max){}
        bool isPositionInRange(const NominalState& X_nom){
            bool isOK=true;
            if(X_nom.p(0) >= xyz_min_(0) && X_nom.p(0) <= xyz_max_(0)&&
               X_nom.p(1) >= xyz_min_(1) && X_nom.p(1) <= xyz_max_(1) &&
               X_nom.p(2) >= xyz_min_(2) && X_nom.p(2) <= xyz_max_(2)){
                isOK = true;
            }
            else isOK = false;

            return isOK;
        };
        bool isStateOK(const Vec3& p_measure, const Vec4& q_measure, const NominalState& X_nom){
            // check position
            Vec3 diff_p = p_measure - X_nom.p;
            if(diff_p.norm() >= thres_position) {
                std::cout << "==========diff p: " << diff_p.norm() <<" / thres:" << thres_position << std::endl;
                return false;
            }

            // check quaternion
            Vec4 diff_q = geometry::q1_mult_q2(q_measure, geometry::q_conj(X_nom.q));
            if(diff_q(0) <= thres_quaternion) {
                std::cout << "==========diff q: " << diff_q(0) <<" / thres:" << thres_quaternion  << std::endl;
                return false;
            }

            // check covariance
            // NOT IMPLEMENTED...

            return true;
        };
    };

    // Added by KGC for initialization
    struct InitializationBuffer{
        // Parameters
        int N_init_min_samples; // # of imu data required to estimate the imu bias
        double w_err, w_rot; // parameters for initial acc bias estimation
        int N_acc_samples; // # of samples to compare

        int N_imu_data;
        Vec3 wm_dt_sum;
        Vec3 am_dt_sum;
        double dt_sum;

        Vec3 wm_last;
        Vec3 am_last;
        Mat33 S_iw;

        double t_last_imu;

        InitializationBuffer(int N_init, double w_e, double w_r, int N_acc, const Mat33& Sigma_iw):
            N_imu_data(0), t_last_imu(0.0), dt_sum(0.0), N_init_min_samples(N_init), w_err(w_e),
            S_iw(Sigma_iw), w_rot(w_r), N_acc_samples(N_acc){
            wm_dt_sum.setZero();
            am_dt_sum.setZero();
        }

        bool addImuData(const Vec3& wm, const Vec3& am, double t){
            if(N_imu_data > 0) {
                if (t < t_last_imu) {
                    std::cerr << "FATAL : IMU timestamp reversal happend. " << t << " < " << t_last_imu
                              << "(last imu time)\n";
                    return false;
                }
                double dt = t - t_last_imu;
                dt_sum += dt;
                wm_dt_sum += dt * (wm + wm_last) / 2.0;
                am_dt_sum += dt * (am + am_last) / 2.0;
            }
            ++N_imu_data;
            
            // update last
            t_last_imu = t;
            wm_last = wm;
            am_last = am;
            return true;
        }

        [[nodiscard]] bool isInitializationReady() const{
            return N_imu_data > N_init_min_samples;
        }

        [[nodiscard]] Vec3 estimateGyroBias() const{
            return wm_dt_sum / dt_sum;
        }

        [[nodiscard]] std::pair<Vec3, Vec4> estimateAccBias(const Vec4& q_iw_nominal, const Vec3& g_w) const{
            double min_cost = std::numeric_limits<double>::max();
            const Vec3 dw_mean = Vec3::Zero();
            Vec3 am_mean = am_dt_sum / dt_sum;
            Vec3 ba_best;
            ba_best.setZero();
            Vec4 dq_best;
            for( int i = 0; i < N_acc_samples; ++i){
                Vec3 dw = RandomUtils::generateGaussianRV<3>(dw_mean, S_iw);
                Vec4 dq = geometry::rotvec2q(dw);
                Vec4 q_iw = geometry::q1_mult_q2(q_iw_nominal, dq);
                Vec3 g_i = geometry::rotate_vec(q_iw, g_w);
                Vec3 ba = am_mean - g_i;
                double cost = w_err * ba.squaredNorm() + w_rot * dq.squaredNorm();
                if(cost < min_cost){
                    min_cost = cost;
                    ba_best = ba;
                    dq_best = dq;
                }
            }
            return std::make_pair(ba_best, dq_best);
        }
    };

private:
    FixedParameters fixed_param_;
    std::unique_ptr<InitializationBuffer> init_buffer_;

    Vec3 ba_init_;
    Vec3 bg_init_;

    NominalState X_nom_;
    ErrorState   dX_;
    std::map<int, geometry::Tf> T_xr_init_; // T_bw(0) for optitrack, T_ct(0) for apriltag
    std::map<int, geometry::Tf> T_rx_init_; // T_wb(0) for optitrack, T_tc(0) for apriltag
    int ref_tag_id_; // recommended : The better observed(closely observed) tag. 0 for optitrack
    ApriltagInfoArr apriltag_world_poses_;
    ApriltagInfoArr apriltag_ref_poses_; // pose w.r.t. ref tag
    CovarianceMat P_;

    ProcessNoise process_noise_;
    MeasurementNoise measurement_noise_;

    EmergencyResetRules emergency_reset_rules_;

    Matrix<double,15,12> Fi_;

    double t_prev_;
    double t_init_;

    bool isInitialized_;
    bool isBiasInitialized_; // bias initialization

    // LPF for angular rate
    LowPassFilter<Vec3>* lpf_gyro_;
    LowPassFilter<Vec3>* lpf_acc_;

    // Added by KGC
};

#endif