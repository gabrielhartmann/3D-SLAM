#include "simScene.hpp"

SimScene::SimScene()
{
    landmarks.push_back(Eigen::Vector3d(100.0, 100.0, 0.0));
    landmarks.push_back(Eigen::Vector3d(100.0, 75.0, 0.0));
    landmarks.push_back(Eigen::Vector3d(100.0, 50.0, 0.0));
    landmarks.push_back(Eigen::Vector3d(100.0, 25.0, 0.0));
    landmarks.push_back(Eigen::Vector3d(100.0, 0.0, 0.0));
}