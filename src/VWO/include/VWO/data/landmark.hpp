#ifndef STELLA_VSLAM_DATA_LANDMARK_H
#define STELLA_VSLAM_DATA_LANDMARK_H

#include "VWO/type.hpp"

#include <map>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <nlohmann/json_fwd.hpp>
#include <sqlite3.h>

namespace data
{

    class Landmark : public std::enable_shared_from_this<Landmark>
    {
        public:
        private:
    };
}
#endif