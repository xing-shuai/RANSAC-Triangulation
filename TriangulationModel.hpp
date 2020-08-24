#pragma once

#include <opencv2/opencv.hpp>
#include "AbstractModel.hpp"

// triangulation using SVD
void TriangulateWithConf(const std::vector<cv::Point3f>& pointsOnEachCamera,
                         const std::vector<cv::Mat>& cameraMatrices,
                         cv::Mat& reconstructedPoint) {
  const auto numberCameras = (int)cameraMatrices.size();
  cv::Mat A = cv::Mat::zeros(numberCameras * 2, 4, CV_32F);
  for (auto i = 0; i < numberCameras; i++) {
    A.row(i * 2) = (pointsOnEachCamera[i].x * cameraMatrices[i].row(2) -
                    cameraMatrices[i].row(0)) *
                   pointsOnEachCamera[i].z;

    A.row(i * 2 + 1) = (pointsOnEachCamera[i].y * cameraMatrices[i].row(2) -
                        cameraMatrices[i].row(1)) *
                       pointsOnEachCamera[i].z;
  }
  // Solve x for Ax = 0 --> SVD on A
  cv::SVD::solveZ(A, reconstructedPoint);
  reconstructedPoint /= reconstructedPoint.at<float>(3);
}

// per view sample
class SingleView : public GRANSAC::AbstractParameter {
 public:
  SingleView(GRANSAC::VPFloat x, GRANSAC::VPFloat y, GRANSAC::VPFloat conf,
             cv::Mat& ProjMatrix)
      : m_ProjMatrix(ProjMatrix) {
    m_Point2D = cv::Point3f(x, y, conf);
  };

  cv::Point3f m_Point2D;
  cv::Mat& m_ProjMatrix;
};

// our model
class TriangulationRansacModel : public GRANSAC::AbstractModel<2> {
 protected:
  // Parametric form
  cv::Mat m_ReconstructedPoint;

  // calculate reprojection error
  virtual GRANSAC::VPFloat ComputeDistanceMeasure(
      std::shared_ptr<GRANSAC::AbstractParameter> Param) override {
    auto singleView = std::dynamic_pointer_cast<SingleView>(Param);
    if (singleView == nullptr)
      throw std::runtime_error(
          "TriangulationRansacModel::ComputeDistanceMeasure() - Passed "
          "parameter are not of type Point2D.");

    cv::Mat projPoint2D = singleView->m_ProjMatrix * m_ReconstructedPoint;
    projPoint2D = projPoint2D / projPoint2D.at<float>(2);
    cv::Point3f& point2D = singleView->m_Point2D;
    GRANSAC::VPFloat Dist = sqrt(pow(point2D.x - projPoint2D.at<float>(0), 2) +
                                 pow(point2D.y - projPoint2D.at<float>(1), 2));

    return Dist;
  };

 public:
  TriangulationRansacModel(
      const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>&
          InputParams) {
    Initialize(InputParams);
  };

  // estimate 3d point from two views
  virtual void Initialize(
      const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>&
          InputParams) override {
    if (InputParams.size() != 2)
      throw std::runtime_error(
          "TriangulationRansacModel - Number of input parameters does not "
          "match minimum number required for this model.");

    // Check for AbstractParamter types
    auto singleView1 = std::dynamic_pointer_cast<SingleView>(InputParams[0]);
    auto singleView2 = std::dynamic_pointer_cast<SingleView>(InputParams[1]);
    if (singleView1 == nullptr || singleView2 == nullptr)
      throw std::runtime_error(
          "TriangulationRansacModel - InputParams type mismatch. It is not a "
          "Point2D.");

    std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

    // reconstruct 3d point
    std::vector<cv::Point3f> pointsOnEachCamera{singleView1->m_Point2D,
                                                singleView2->m_Point2D};
    std::vector<cv::Mat> cameraMatrices{singleView1->m_ProjMatrix,
                                        singleView2->m_ProjMatrix};
    TriangulateWithConf(pointsOnEachCamera, cameraMatrices,
                        m_ReconstructedPoint);
  };

  virtual std::pair<GRANSAC::VPFloat,
                    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>>
  Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>&
               EvaluateParams,
           GRANSAC::VPFloat Threshold) {
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
    int nTotalParams = EvaluateParams.size();
    int nInliers = 0;

    for (auto& Param : EvaluateParams) {
      if (ComputeDistanceMeasure(Param) < Threshold) {
        Inliers.push_back(Param);
        nInliers++;
      }
    }

    GRANSAC::VPFloat InlierFraction =
        GRANSAC::VPFloat(nInliers) /
        GRANSAC::VPFloat(nTotalParams);  // This is the inlier fraction

    return std::make_pair(InlierFraction, Inliers);
  };

  // get final reconstructed 3d point using best inliers
  cv::Mat GetBestReconstructed3DPoint(
      const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>&
          BestInliers) {
    std::vector<cv::Point3f> pointsOnEachCamera;
    std::vector<cv::Mat> cameraMatrices;

    for (auto& Inlier : BestInliers) {
      auto sv = std::dynamic_pointer_cast<SingleView>(Inlier);
      pointsOnEachCamera.emplace_back(sv->m_Point2D);
      cameraMatrices.emplace_back(sv->m_ProjMatrix);
    }
    cv::Mat reconstructed3DPoint;
    TriangulateWithConf(pointsOnEachCamera, cameraMatrices,
                        reconstructed3DPoint);
    return reconstructed3DPoint;
  }
};
