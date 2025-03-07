#ifndef WHEEL_ODOM_EDGE_WRAPPER_H
#define WHEEL_ODOM_EDGE_WRAPPER_H

#include "VWO/camera/perspective.h"
#include "VWO/optimize/internal/se3/wheel_odom_edge.h"

#include <g2o/core/robust_kernel_impl.h>

#include <memory>


namespace data {
class landmark;
} // namespace data

namespace optimize {
namespace internal {
namespace se3 {


template<typename T>
class wheel_odom_edge_wrapper
{
    public:
        wheel_odom_edge_wrapper() = delete;
        wheel_odom_edge_wrapper(const std::shared_ptr<T>& shot1, shot_vertex* shot_vtx1,
                                const std::shared_ptr<T>& shot2, shot_vertex* shot_vtx2,
                                g2o::SE3Quat relative_1_to_2);
        
        g2o::OptimizableGraph::Edge* edge_;
};

template<typename T>
inline wheel_odom_edge_wrapper<T>::wheel_odom_edge_wrapper(const std::shared_ptr<T>& shot1, shot_vertex* shot_vtx1,
                                                           const std::shared_ptr<T>& shot2, shot_vertex* shot_vtx2,
                                                           g2o::SE3Quat relative_1_to_2)
{
    auto edge = new WheelOdomBinaryEdge();
    edge->setMeasurement(relative_1_to_2);
    edge->setInformation(Mat66_t::Identity());
    edge->setVertex(0, shot_vtx1);
    edge->setVertex(1, shot_vtx2);
    edge_ = edge;
}

}
}
}

#endif