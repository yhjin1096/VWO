#ifndef STELLA_VSLAM_DATA_BOW_VOCABULARY_H
#define STELLA_VSLAM_DATA_BOW_VOCABULARY_H

#include "VWO/data/bow_vocabulary_fwd.h"

#ifdef USE_DBOW2
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#else
#include <fbow/vocabulary.h>
#endif // USE_DBOW2

namespace data {
namespace bow_vocabulary_util {

float score(bow_vocabulary* bow_vocab, const bow_vector& bow_vec1, const bow_vector& bow_vec2);
void compute_bow(bow_vocabulary* bow_vocab, const cv::Mat& descriptors, bow_vector& bow_vec, bow_feature_vector& bow_feat_vec);
bow_vocabulary* load(std::string path);

}; // namespace bow_vocabulary_util
}; // namespace data

#endif // STELLA_VSLAM_DATA_BOW_VOCABULARY_H
