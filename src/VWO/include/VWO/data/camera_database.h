#ifndef STELLA_VSLAM_DATA_CAMERA_DATABASE_H
#define STELLA_VSLAM_DATA_CAMERA_DATABASE_H

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

#include "VWO/camera/camera.h"

typedef struct sqlite3 sqlite3;

// namespace camera {
// class base;
// } // namespace camera

namespace data {

class camera_database {
public:
    explicit camera_database();

    ~camera_database();

    void add_camera(Camera* camera);

    Camera* get_camera(const std::string& camera_name) const;

    void from_json(const nlohmann::json& json_cameras);

    nlohmann::json to_json() const;

    bool from_db(sqlite3* db);

    bool to_db(sqlite3* db) const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! database (key: camera name, value: pointer of camera::base)
    std::unordered_map<std::string, Camera*> cameras_;
};

} // namespace data

#endif // STELLA_VSLAM_DATA_CAMERA_DATABASE_H
