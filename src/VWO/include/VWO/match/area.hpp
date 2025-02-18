#ifndef AREA_H
#define AREA_H

#include "VWO/match/base.hpp"


namespace data {
class Frame;
} // namespace data

namespace match {

class area final : public base {
public:
    area(const float lowe_ratio, const bool check_orientation)
        : base(lowe_ratio, check_orientation) {}

    ~area() final = default;

    unsigned int match_in_consistent_area(data::Frame& frm_1, data::Frame& frm_2, std::vector<cv::Point2f>& prev_matched_pts,
                                          std::vector<int>& matched_indices_2_in_frm_1, int margin = 10);
};

} // namespace match

#endif // AREA_H
