#ifndef FEATURE_EXTRACTOR_HPP
#define FEATURE_EXTRACTOR_HPP

#include <iostream>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

class FeatureExtractor
{
    public:
        virtual ~FeatureExtractor() {};
        virtual void extractFeatures(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                            std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {};

    private:
};

#endif