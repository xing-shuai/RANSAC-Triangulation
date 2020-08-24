#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "GRANSAC.hpp"
#include "TriangulationModel.hpp"

int main(int argc, char* arcv[]) {
  // load projection matrix
  unsigned int nCameraViews = 5;
  std::vector<cv::Mat> Ps;

  Ps.resize(nCameraViews);
  cv::FileStorage psread("/home/shuai/Desktop/Shelf_Ps.xml",
                         cv::FileStorage::READ);
  for (unsigned int i = 0; i < nCameraViews; i++) {
    cv::Mat m;
    psread["p_" + std::to_string(i)] >> m;
    Ps.at(i) = m;
  }
  psread.release();

  // prepare candidate view data
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandViews;

  CandViews.emplace_back(
      std::make_shared<SingleView>(577, 768, 0.732, Ps.at(0)));
  CandViews.emplace_back(
      std::make_shared<SingleView>(933, 666, 1.06, Ps.at(1)));
  CandViews.emplace_back(
      std::make_shared<SingleView>(828, 467, 1.072, Ps.at(2)));
  CandViews.emplace_back(
      std::make_shared<SingleView>(389, 475, 1.009, Ps.at(4)));

  // RANSAC Triangulation
  GRANSAC::RANSAC<TriangulationRansacModel, 2> Estimator;
  Estimator.Initialize(15, 10);  // Threshold, iterations
  int64_t start = cv::getTickCount();
  Estimator.Estimate(CandViews);
  int64_t end = cv::getTickCount();
  std::cout << "RANSAC Took: "
            << GRANSAC::VPFloat(end - start) /
                   GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0
            << " ms." << std::endl;

  // other informations to output
  auto& BestInliers = Estimator.GetBestInliers();
  std::cout << std::endl;
  std::cout << "Best Inliers: " << std::endl;
  for (auto& Inlier : BestInliers) {
    auto sv = std::dynamic_pointer_cast<SingleView>(Inlier);
    std::cout << sv->m_Point2D << std::endl;
  }

  auto bestModel = Estimator.GetBestModel();
  auto reconstruct3DPoint =
      std::dynamic_pointer_cast<TriangulationRansacModel>(bestModel)
          ->GetBestReconstructed3DPoint(BestInliers);

  std::cout << std::endl;
  std::cout << "Reconstructed 3D Point: " << std::endl
            << reconstruct3DPoint << std::endl;

  std::cout << std::endl;
  return 0;
}
