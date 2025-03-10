#ifndef WHEEL_ODOM_EDGE_H
#define WHEEL_ODOM_EDGE_H

#include "VWO/type.h"
#include "VWO/optimize/internal/se3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/se3quat.h>

namespace optimize {
namespace internal {
namespace se3 {
//error dimension, measurement, keyframe1, keyframe2
class WheelOdomBinaryEdge : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, shot_vertex, shot_vertex>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        WheelOdomBinaryEdge();
        
        bool read(std::istream& is) override;

        bool write(std::ostream& os) const override;

        void computeError() override; // error 계산

        void linearizeOplus() override; // error 미분
};

inline WheelOdomBinaryEdge::WheelOdomBinaryEdge()
    : g2o::BaseBinaryEdge<6, g2o::SE3Quat, shot_vertex, shot_vertex>()
{

}

inline bool WheelOdomBinaryEdge::read(std::istream& is)
{
    return false;
}

inline bool WheelOdomBinaryEdge::write(std::ostream& os) const
{
    return false;
}

inline void WheelOdomBinaryEdge::computeError()
{
    const shot_vertex* v1 = static_cast<const shot_vertex*>(_vertices[0]);
    const shot_vertex* v2 = static_cast<const shot_vertex*>(_vertices[1]);

    g2o::SE3Quat C(_measurement);
    g2o::SE3Quat error_ = v2->estimate().inverse() * C * v1->estimate();
    _error = error_.log();
}

inline void WheelOdomBinaryEdge::linearizeOplus()
{
    shot_vertex* vi = static_cast<shot_vertex*>(_vertices[0]);
    g2o::SE3Quat Ti(vi->estimate());

    shot_vertex* vj = static_cast<shot_vertex*>(_vertices[1]);
    g2o::SE3Quat Tj(vj->estimate());

    const g2o::SE3Quat& Tij = _measurement;
    g2o::SE3Quat invTij = Tij.inverse();

    g2o::SE3Quat invTj_Tij = Tj.inverse() * Tij;
    g2o::SE3Quat infTi_invTij = Ti.inverse() * invTij;

    _jacobianOplusXi = invTj_Tij.adj();
    _jacobianOplusXj = -infTi_invTij.adj();
}



} // namespace se3
} // namespace internal
} // namespace optimize

#endif