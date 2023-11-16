#include "eskf/geometry_library.h"
using namespace Eigen;
namespace geometry {

    Mat33 skewMat(const Vec3& v){
        Mat33 res_mat;
        res_mat << 0,-v(2),v(1),
                   v(2),0,-v(0),
                  -v(1),v(0),0;
        return res_mat;
    };

    Mat44 q_right_mult(const Vec4& q){
        Mat44 omega_r;
        omega_r << q(0),-q(1),-q(2),-q(3),
                   q(1), q(0), q(3),-q(2),
                   q(2),-q(3), q(0), q(1),
                   q(3), q(2),-q(1), q(0);
        return omega_r;
    };
    Mat44 q_left_mult(const Vec4& q){
        Mat44 omega_l;
        omega_l << q(0),-q(1),-q(2),-q(3),
                   q(1), q(0),-q(3), q(2),
                   q(2), q(3), q(0),-q(1),
                   q(3),-q(2), q(1), q(0);
        return omega_l;
    };
    Vec4 q_conj(const Vec4& q){
        Vec4 q_c;
        q_c << q(0), -q(1),-q(2),-q(3);
        return q_c;
    };
    Vec4 q1_mult_q2(const Vec4& q1, const Vec4& q2){
        Vec4 q;
        q << q1(0)*q2(0)-q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3),
             q1(0)*q2(1)+q1(1)*q2(0)+q1(2)*q2(3)-q1(3)*q2(2),
             q1(0)*q2(2)-q1(1)*q2(3)+q1(2)*q2(0)+q1(3)*q2(1),
             q1(0)*q2(3)+q1(1)*q2(2)-q1(2)*q2(1)+q1(3)*q2(0);
        return q;
    };

    Mat33 q2r(const Vec4& q){
        Mat33 R;
        double qw = q(0);
        double qx = q(1);
        double qy = q(2);
        double qz = q(3);

        double qw2 = qw*qw;
        double qx2 = qx*qx;
        double qy2 = qy*qy;
        double qz2 = qz*qz;

        double qxqy = qx*qy;
        double qwqz = qw*qz;
        double qxqz = qx*qz;
        double qwqy = qw*qy;
        double qwqx = qw*qx;
        double qyqz = qy*qz;

        R <<  qw2+qx2-qy2-qz2, 2.0*(qxqy-qwqz), 2.0*(qxqz+qwqy),
              2.0*(qxqy+qwqz), qw2-qx2+qy2-qz2, 2.0*(qyqz-qwqx),
              2.0*(qxqz-qwqy), 2.0*(qyqz+qwqx), qw2-qx2-qy2+qz2;    

        return R;
    };

    Vec4 rotvec2q(const Vec3& w){
        Vec4 q_res;
        double th = w(0)*w(0) + w(1)*w(1) + w(2)*w(2);
        th = std::sqrt(th);
        if(th < 1e-7){
            q_res << 1.0, 0.0, 0.0, 0.0;
        }
        else{
            double invthsinth05 = sin(th*0.5)/th;
            q_res << cos(th*0.5),w(0)*invthsinth05, w(1)*invthsinth05, w(2)*invthsinth05;
            q_res /= q_res.norm();
        }
        return q_res;
    };

    Vec3 q2rotvec(const Vec4& q){
        double cos_th2 = q(0);
        double sin_th2 = q.tail<3>().norm();
        double th2 = atan2(sin_th2, cos_th2);
        Vec3 v = q.tail<3>() / sin_th2;
        return th2 * v;
    }

    Mat33 a2r(double r, double p, double y){
        Mat33 Rx;
        Mat33 Ry;
        Mat33 Rz;

        Rx << 1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r);
        Ry << cos(p),0,sin(p),0,1,0,-sin(p),0,cos(p);
        Rz << cos(y),-sin(y),0,sin(y),cos(y),0,0,0,1;

        return (Rz*Ry*Rx);
    };

    Vec4 r2q(const Mat33& R){
        Vec4 q;
        double qw,qx,qy,qz;

        double m00 = R(0,0);
        double m11 = R(1,1);
        double m22 = R(2,2);

        double m21 = R(2,1);
        double m12 = R(1,2);
        double m02 = R(0,2);
        double m20 = R(2,0);
        double m10 = R(1,0);
        double m01 = R(0,1);

        double tr = R(0,0) + R(1,1) + R(2,2);

        if (tr > 0) { 
            double S = sqrt(tr+1.0) * 2; // S=4*qw 
            qw = 0.25 * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S; 
            qz = (m10 - m01) / S; 
        } else if ((m00 > m11)&(m00 > m22)) { 
            double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
            qw = (m21 - m12) / S;
            qx = 0.25 * S;
            qy = (m01 + m10) / S; 
            qz = (m02 + m20) / S; 
        } else if (m11 > m22) { 
            double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S; 
            qy = 0.25 * S;
            qz = (m12 + m21) / S; 
        } else { 
            double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25 * S;
        }

        q(0) = qw;
        q(1) = qx;
        q(2) = qy;
        q(3) = qz;
        return q;
    }

    double diff_q1_q2(const Vec4& q1, const Vec4& q2) {
        /**
         * orientation difference between q1 and q2
         * q1 * conj(q2), evaluate the 2*arcsin ( x2+ y2 +z2 )
         */
        Vec4 dq = q1_mult_q2(q1, q_conj(q2));
        return 2*asin(sqrt(dq(1)*dq(1) + dq(2)*dq(2) + dq(3)*dq(3)));
    }
    
    
    Vec3 rotate_vec(const Vec4& q1, const Vec3& v){
        return q2r(q1) * v;
    }
};
