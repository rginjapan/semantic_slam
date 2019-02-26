//
// Created by zss on 19-2-23.
//

#ifndef ORB_SLAM2_G2O_LINE_EDGE_H
#define ORB_SLAM2_G2O_LINE_EDGE_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

namespace g2o {

    class  EdgeSE3ProjectLineOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectLineOnlyPose(){}/////什么时候有分号

        virtual bool read(std::istream& is)
        {
            return true;
        };

        virtual bool write(std::ostream& os) const
        {
            return os.good();
        };

        Vector3d unproject2d(const Vector2d& v)  {
            Vector3d res;
            res(0) = v(0);
            res(1) = v(1);
            res(2) = 1;
            return res;
        }

        Vector2d project2d(const Vector3d& v)  {
            Vector2d res;
            res(0) = v(0)/v(2);
            res(1) = v(1)/v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d & trans_xyz) {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0]*fx + cx;
            res[1] = proj[1]*fy + cy;
            return res;
        }

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            SE3Quat T(v1->estimate());
            Eigen::Matrix3d R = T.rotation().toRotationMatrix();
            Eigen::Vector3d t = T.translation();
            Eigen::Matrix3d t_Matrix;
            t_Matrix.setZero();
            t_Matrix(0,1)=-t[2];
            t_Matrix(0,2)=t[1];
            t_Matrix(1,0)=t[2];
            t_Matrix(1,2)=-t[0];
            t_Matrix(2,0)=-t[1];
            t_Matrix(2,1)=t[0];
            Vector3d nw;
            nw[0]=line_world[0];
            nw[1]=line_world[1];
            nw[2]=line_world[2];

            Vector3d vw;
            vw[0]=line_world[3];
            vw[1]=line_world[4];
            vw[2]=line_world[5];

            Vector3d nc=R*nw+t_Matrix*vw;
            Vector3d vc=R*vw;

            //// 上一帧的世界坐标系的直线转化为像素坐标

            p_c=unproject2d(cam_project(start_point_camera));
            q_c=unproject2d(cam_project(end_point_camera));

            /// 相机坐标系下的nc乘以K转化到图像坐标
            Line[0]=fx*nc[0];
            Line[1]=fy*nc[1];
            Line[2]=-fy*cx*nc[0]+fx*cy*nc[1]+fx*fy*nc[2];
            float dist=sqrtf(Line[0]*Line[0]+Line[1]*Line[1]+Line[2]*Line[2]);
            Vector2d err;
            err[0] = p_c.dot(Line)/dist;
            err[1] = q_c.dot(Line)/dist;
            _error = err;

//            cout<<"dist:"<<dist<<endl;
//            cout<<"error："<<_error<<endl;

        }

        void linearizeOplus()
        {
            VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
            SE3Quat T(vi->estimate());
            Eigen::Matrix3d R = T.rotation().toRotationMatrix();
            Eigen::Vector3d t = T.translation();
            Eigen::Matrix3d t_Matrix;
            t_Matrix.setZero();
            t_Matrix(0,1)=-t[2];
            t_Matrix(0,2)=t[1];
            t_Matrix(1,0)=t[2];
            t_Matrix(1,2)=-t[0];
            t_Matrix(2,0)=-t[1];
            t_Matrix(2,1)=t[0];

            float l_dist1=pow(Line[0]*Line[0]+Line[1]*Line[1],3/2);
            float l_dist2=pow(Line[0]*Line[0]+Line[1]*Line[1],1/2);

            Matrix<double,2,3> error_el;
            error_el(0,0) = (p_c[0]*Line[1]*Line[1]-Line[0]*Line[1]*p_c[1]-Line[0]*Line[2])/l_dist1;
            error_el(0,1) = (p_c[1]*Line[0]*Line[0]-Line[0]*Line[1]*p_c[0]-Line[1]*Line[2])/l_dist1;
            error_el(0,2) = 1/l_dist2;
            error_el(1,0) = (q_c[0]*Line[1]*Line[1]-Line[0]*Line[1]*q_c[1]-Line[0]*Line[2])/l_dist1;
            error_el(1,1) = (q_c[1]*Line[0]*Line[0]-Line[0]*Line[1]*q_c[0]-Line[1]*Line[2])/l_dist1;
            error_el(1,2) = 1/l_dist2;

            Matrix<double,3,6> error_lL;
            error_lL.setZero();
            error_lL(0,0)=fx;
            error_lL(1,1)=fy;
            error_lL(2,0)=-fy*cx;
            error_lL(2,1)=fx*cy;
            error_lL(2,2)=fx*fy;

            Matrix<double,6,6> error_cw;//// 是不是和相机的转换矩阵rt是一样的
            error_cw.block<3,3>(0,0)=R;
            error_cw.block<3,3>(0,3)=t_Matrix*R;/// 叉乘写成反对称矩阵
            error_cw.block<3,3>(3,0).setZero();//// 不可以直接赋值为0
            error_cw.block<3,3>(3,3)=R;

            _jacobianOplusXi = error_el*error_lL*error_cw;
        }


        double fx, fy, cx, cy;

//        Vector3d start_point;//// 世界坐标系下的线段起点,非齐次坐标
//        Vector3d end_point;//// 世界坐标系下的线段终点,非齐次坐标

        Vector3d start_point_camera;//// 世界坐标系下的线段起点,非齐次坐标
        Vector3d end_point_camera;//// 世界坐标系下的线段终点,非齐次坐标

        Vector6d line_world;//// Lw=[nw,vw]
//        Vector6d line_camera;//// Lc=[nc,vc]
        Vector3d Line;
//        Vector3d p_w;
//        Vector3d q_w;
        Vector3d p_c;
        Vector3d q_c;
    };

}
#endif //ORB_SLAM2_G2O_LINE_EDGE_H
