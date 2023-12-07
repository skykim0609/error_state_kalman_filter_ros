#ifndef _GEOMETRY_LIBRARY_H_
#define _GEOMETRY_LIBRARY_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,2,1> Vec2;
typedef Matrix<double,3,1> Vec3;
typedef Matrix<double,4,1> Vec4;
typedef Matrix<double,6,1> Vec6;
typedef Matrix<double,7,1> Vec7;
typedef Matrix<double,3,3> Mat33;
typedef Matrix<double,4,4> Mat44;
typedef Matrix<double,3,4> Mat34;
typedef Matrix<double,4,3> Mat43;
typedef Matrix<double,6,6> Mat66;
typedef Matrix<double,15,15> Mat1515;

namespace geometry {
    Mat33 skewMat(const Vec3& v);
    Mat44 q_right_mult(const Vec4& q);
    Mat44 q_left_mult(const Vec4& q);
    Vec4 q_conj(const Vec4& q);
    Vec4 q1_mult_q2(const Vec4& q1, const Vec4& q2);
    Mat33 q2r(const Vec4& q);
    Vec4 rotvec2q(const Vec3& w);
    Vec3 q2rotvec(const Vec4& q);
    Mat33 a2r(double r, double p, double y);
    Vec4 r2q(const Mat33& R);

    // Followings are added by KGC
    double diff_q1_q2(const Vec4& q1, const Vec4& q2);
    Vec3 rotate_vec(const Vec4& q1, const Vec3& v);

    class Tf{
    public:
        Vec4 q;
        Vec3 p;
        static const Tf Identity(){
            return Tf();
        }
        Tf(){
            q << 1.0, 0.0, 0.0, 0.0;
            p.setZero();
        }
        Tf(const std::array<double, 4>& q_in, const std::array<double, 3>& p_in){
            q << q_in[0], q_in[1], q_in[2], q_in[3];
            q = q / q.norm();
            p << p_in[0], p_in[1], p_in[2];
        }
        Tf(const Vec4& q_in, const Vec3& p_in):q(q_in), p(p_in){
            q = q / q.norm();
        }
        Tf(const Vec3& p_in, const Vec4& q_in):q(q_in), p(p_in){
            q = q / q.norm();
        }
        Tf Inverse() const{
            const Vec4 q_inv = q_conj(q);
            const Vec3 p_inv = -rotate_vec(q_inv, p);
            return Tf(q_inv, p_inv);
        }
        Tf mult(const Tf& T) const{
            // this * T
            const Vec4 q_out = q1_mult_q2(q, T.q);
            const Vec3 p_out = p + rotate_vec(q, T.p);
            return Tf(q_out, p_out);
        }
        const Vec3& trans() const{
            return p;
        }
        const Vec4& rot() const{
            return q;
        }
        void setTrans(const Vec3& p_){
            p = p_;
        }
        void setRot(const Vec4& q_){
            q = q_/ q_.norm();
        }

        const Mat33 rotmat() const{
            return q2r(q);
        }
        const Vec7 getVec7Form() const{
            Vec7 pq;
            pq << p(0), p(1), p(2), q(0), q(1), q(2), q(3);
            return pq;
        }
        const Vec6 getVec6Form() const{
            Vec6 pw;
            Vec3 w = q2rotvec(q);
            pw << p, w;
            return pw;
        }
        friend std::ostream& operator<<(std::ostream& os, const Tf& T);
    };
};

//std::ostream& operator<<(std::ostream& os, const geometry::Tf& T);

#endif