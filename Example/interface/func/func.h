#pragma once

#include <iostream>
#include <Frame.h>
#include <System.h>
#include <Optimizer.h>
#include <Tracking.h>
#include <Map.h>
#include <include/utils/dataprocess_utils.h>

#include <src/config/Config.h>

#include <Eigen/Core>

MatrixXd TrajetoryToMat(Trajectory& estTraj);
Trajectory MatToTrajectory(MatrixXd& mat);

MatrixXd GenerateSelectedGtMat(MatrixXd &estMat, MatrixXd &gtMat);
void alignTrajectory(MatrixXd& estMat, MatrixXd& gtMat, g2o::SE3Quat& transform, Trajectory &alignedGt, Trajectory &estTrajSelected);
double CalculateRMSE(Trajectory& estTraj, Trajectory& gtTrajInEst);
string FindFileUsingTimestamp(std::vector<string>& mvDetFileNames, double timestamp);
void LoadObjectsObservationsToFrame(MatrixXd& detMat, Frame* pFrame, int iType);
std::vector<Frame*> LoadDataAsFrames(const string& path_odom, const string& dir_detectResult, int iType = 0, int iJump = 1);
void SaveTrajectoryTUM(const string &filename, std::vector<Frame*>& vpKFs);

void GenerateSelectedGtMatAndEstMat(MatrixXd &estMat, MatrixXd &gtMat, MatrixXd &estMatSelected_, MatrixXd& gtMatSelected_);