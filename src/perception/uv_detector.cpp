#include "perception/uv_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace spite_d {

namespace {

// grouping U-map lines (port from map_manager)
struct Segment {
    int id;
    int topParent;
    cv::Rect bb;
};

void MergeInto(Segment& father, const Segment& son) {
  const int top = std::min(father.bb.tl().y, son.bb.tl().y);
  const int left = std::min(father.bb.tl().x, son.bb.tl().x);
  const int bottom = std::max(father.bb.br().y, son.bb.br().y);
  const int right = std::max(father.bb.br().x, son.bb.br().x);
  father.bb = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
}

}

const std::vector<CameraFrameBox>& UvDetector::Detect(const cv::Mat& depth16u) {
    CV_Assert(depth16u.type() == CV_16UC1);
    ExtractUMap(depth16u);
    ExtractUBoxes();
    ExtractBoxes(depth16u);
    return m_boxes;
}

void UvDetector::ExtractUMap(const cv::Mat& depth16u) {
  cv::Mat depthRescale;
  cv::resize(depth16u, depthRescale, cv::Size(), m_params.colScale, 1);

  const int histSize = depth16u.rows / m_params.rowDownsample;
  const int binWidth =
      std::ceil((m_params.maxDistMm - m_params.minDistMm) / float(histSize));

  m_uMap = cv::Mat::zeros(histSize, depthRescale.cols, CV_8UC1);

  for (int col = 0; col < depthRescale.cols; ++col) {
    for (int row = 0; row < depthRescale.rows; ++row) {
      const int mm = int(float(depthRescale.at<uint16_t>(row, col)) /
                         m_params.depthScale * 1000.0);
      if (mm > m_params.minDistMm && mm < m_params.maxDistMm) {
        const int bin = (mm - m_params.minDistMm) / binWidth;
        if (bin < histSize && m_uMap.at<uchar>(bin, col) < 255)
          m_uMap.at<uchar>(bin, col)++;
      }
    }
  }

  cv::GaussianBlur(m_uMap, m_uMap, cv::Size(5, 9), 10, 10);
}

void UvDetector::ExtractUBoxes() {
  const int uMin = int(m_params.thresholdPoint * m_params.rowDownsample);
  std::vector<std::vector<int>> mask(m_uMap.rows,
                                     std::vector<int>(m_uMap.cols, 0));
  std::vector<Segment> segments;
  int sumLine = 0, maxLine = 0, lengthLine = 0, segId = 0;

  for (int row = 0; row < m_uMap.rows; ++row) {
    for (int col = 0; col < m_uMap.cols; ++col) {
      if (m_uMap.at<uchar>(row, col) >= uMin) {
        ++lengthLine;
        sumLine += m_uMap.at<uchar>(row, col);
        maxLine = std::max<int>(maxLine, m_uMap.at<uchar>(row, col));
      }
      if (m_uMap.at<uchar>(row, col) < uMin || col == m_uMap.cols - 1) {
        if (col == m_uMap.cols - 1) ++col;
        if (lengthLine > m_params.minLengthLine &&
            sumLine > m_params.thresholdLine * maxLine) {
          ++segId;
          segments.push_back(
              {segId, segId,
               cv::Rect(cv::Point(col - lengthLine, row),
                        cv::Point(col - 1, row))});
          for (int c = col - lengthLine; c < col - 1; ++c) mask[row][c] = segId;

          // Union with segments touching from the row above.
          if (row != 0) {
            for (int c = col - lengthLine; c < col - 1; ++c) {
              if (mask[row - 1][c] == 0) continue;
              Segment& above = segments[mask[row - 1][c] - 1];
              Segment& current = segments.back();
              if (above.topParent < current.topParent) {
                current.topParent = above.topParent;
              } else {
                const int old = above.topParent;
                for (auto& s : segments)
                  if (s.topParent == old) s.topParent = current.topParent;
              }
            }
          }
        }
        sumLine = maxLine = lengthLine = 0;
      }
    }
  }

  m_uBoxes.clear();
  for (size_t b = 0; b < segments.size(); ++b) {
    if (segments[b].id != segments[b].topParent) continue;
    for (size_t s = b + 1; s < segments.size(); ++s)
      if (segments[s].topParent == segments[b].id)
        MergeInto(segments[b], segments[s]);
    if (segments[b].bb.area() >= m_params.minUBoxArea)
      m_uBoxes.push_back(segments[b].bb);
  }
}

void UvDetector::ExtractBoxes(const cv::Mat& depth16u) {
  cv::Mat depthResize;
  cv::resize(depth16u, depthResize, cv::Size(), m_params.colScale, 1);

  const float histSize = float(depth16u.rows) / m_params.rowDownsample;
  const float binWidth =
      std::ceil((m_params.maxDistMm - m_params.minDistMm) / histSize);

  const auto depthMm = [&](int row, int col) {
    return float(depthResize.at<uint16_t>(row, col)) / m_params.depthScale *
           1000.0f;
  };

  m_boxes.clear();
  for (const cv::Rect& ub : m_uBoxes) {
    const int x = ub.tl().x;
    const int width = ub.width;
    int yUp = depthResize.rows;
    int yDown = 0;

    const float depthNear = ub.tl().y * binWidth + m_params.minDistMm;
    const float depthOfDepth = float(ub.br().y - ub.tl().y) * binWidth;
    // Extend past the far bin: the far side is usually self-occluded.
    const float depthFar = depthOfDepth * 1.3f + depthNear;

    // Column scan: find rows whose depth stays inside [near, far] for
    // numCheck consecutive pixels (bounds-guarded, unlike the original).
    const int lastRow = depthResize.rows - 1 - (m_params.numCheck + 1);
    for (int i = x; i < x + width && i < depthResize.cols; ++i) {
      for (int j = 0; j <= lastRow; ++j) {
        const float d = depthMm(j, i);
        if (d < depthNear || d > depthFar) continue;
        bool contiguous = true;
        for (int check = 0; check < m_params.numCheck; ++check) {
          const float dc = depthMm(j + check + 1, i);
          if (dc < depthNear || dc > depthFar) {
            contiguous = false;
            break;
          }
        }
        if (contiguous) {
          yUp = std::min(yUp, j);
          yDown = std::max(yDown, j);
        }
      }
    }
    if (yDown <= yUp) continue;  // No vertical support found.

    CameraFrameBox box;
    box.depthRect =
        cv::Rect(int(x / m_params.colScale), yUp,
                 int(width / m_params.colScale), yDown - yUp);

    // Bird's-view rect (10mm units) for the tracker's association.
    {
      const float bbDepth = ub.br().y * binWidth / 10.0f;
      const float bbWidth = bbDepth * ub.width / m_params.fx;
      const float bbHeight = ub.height * binWidth / 10.0f;
      const float bbX =
          bbDepth * (ub.tl().x / m_params.colScale - m_params.px) / m_params.fx;
      const float bbY = bbDepth - 0.5f * bbHeight;
      box.birdRect = cv::Rect(bbX, bbY, bbWidth, bbHeight);
    }

    const float centerX = (x + width / 2.0f) / m_params.colScale;
    const float centerY = (yDown + yUp) / 2.0f;
    const float depthCenter = (depthNear + depthFar) / 2.0f;  // mm

    box.x = (centerX - m_params.px) * depthCenter / m_params.fx / 1000.0f;
    box.y = (centerY - m_params.py) * depthCenter / m_params.fy / 1000.0f;
    box.z = depthCenter / 1000.0f;
    box.xWidth =
        (width / m_params.colScale) * depthCenter / m_params.fx / 1000.0f;
    box.yWidth = (yDown - yUp) * depthCenter / m_params.fy / 1000.0f;
    box.zWidth = (depthFar - depthNear) / 1000.0f;
    m_boxes.push_back(box);
  }
}

}