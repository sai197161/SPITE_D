#pragma once

// port of map-manager's UVdetector
//   1. Build the U-map: for every image column, histogram the depth
//      values into bins (rows of the U-map). Obstacles show up as
//      horizontal line segments of high bin counts.
//   2. Group adjacent segments into U-map bounding boxes (union-find
//      over row segments, as in the original).
//   3. For each U-box, scan its depth-image columns for vertically
//      contiguous pixels inside the box's depth interval to recover the
//      image-space vertical extent.
//   4. Deproject to a camera-frame axis-aligned 3D box.

#include <opencv2/core.hpp>
#include <cstddef>
#include <vector>

namespace spite_d {

struct CameraFrameBox {
    float x{0}, y{0}, z{0};
    float xWidth{0}, yWidth{0}, zWidth{0};
    cv::Rect depthRect;
    cv::Rect birdRect;
};

class UvDetector {
 public:
  struct Params {
    double fx{608.087}, fy{608.179}, px{317.483}, py{234.116};
    /// Raw depth value -> millimeters is value/depthScale*1000
    double depthScale{1000.0};
    int minDistMm{10};
    int maxDistMm{8000};
    int rowDownsample{4};     ///< depth rows per U-map row (histogram bins).
    double colScale{0.5};     ///< horizontal rescale before histogramming.
    double thresholdPoint{3}; ///< U-map point-of-interest threshold factor.
    double thresholdLine{2};  ///< line-sum vs line-max acceptance factor.
    int minLengthLine{6};     ///< minimum accepted segment length (px).
    int minUBoxArea{25};      ///< minimum accepted U-box area.
    int numCheck{15};         ///< contiguous pixels required in column scan.
  };

  explicit UvDetector(const Params& params) : m_params(params) {}

  /// Detect obstacles in one 16UC1 depth image.
  const std::vector<CameraFrameBox>& Detect(const cv::Mat& depth16u);

  /// The U-map from the last Detect call (for tests/debugging).
  const cv::Mat& UMap() const { return m_uMap; }

 private:
  void ExtractUMap(const cv::Mat& depth16u);
  void ExtractUBoxes();
  void ExtractBoxes(const cv::Mat& depth16u);

  Params m_params;
  cv::Mat m_uMap;
  std::vector<cv::Rect> m_uBoxes;
  std::vector<CameraFrameBox> m_boxes;
};

}