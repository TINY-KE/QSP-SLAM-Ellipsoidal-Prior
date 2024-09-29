/**
 * This file is part of https://github.com/JingwenWang95/DSP-SLAM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "Map.h"
#include <mutex>

#include "src/symmetry/PointCloudFilter.h"

namespace ORB_SLAM2 {

void Map::AddMapObject(MapObject *pMO) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.insert(pMO);
}

void Map::EraseMapObject(MapObject *pMO) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.erase(pMO);
}

vector<MapObject *> Map::GetAllMapObjects() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapObject *>(mspMapObjects.begin(), mspMapObjects.end());
}

MapObject *Map::GetMapObject(int object_id) {
    unique_lock<mutex> lock(mMutexMap);
    for (auto mspMapObject : mspMapObjects) {
        if (mspMapObject->mnId != object_id)
            continue;
        return mspMapObject;
    }
    return NULL;
}

/**
 * Plane
 */

void Map::addPlane(plane *pPlane, int visual_group) {
    unique_lock<mutex> lock(mMutexMap);
    pPlane->miVisualGroup = visual_group;
    mspPlanes.insert(pPlane);
}

vector<plane *> Map::GetAllPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<plane *>(mspPlanes.begin(), mspPlanes.end());
}

void Map::clearPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    mspPlanes.clear();
}

/**
 * PointCloud
 */

bool Map::AddPointCloudList(const string &name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &vCloudPCL, g2o::SE3Quat &Twc, int type) {
    if (type == REPLACE_POINT_CLOUD) {
        DeletePointCloudList(name, COMPLETE_MATCHING);
    }
    srand(time(0));
    for (auto &cloud : vCloudPCL) {
        ORB_SLAM2::PointCloud cloudQuadri = pclToQuadricPointCloud(cloud);
        ORB_SLAM2::PointCloud *pCloudGlobal = new ORB_SLAM2::PointCloud(cloudQuadri);
        transformPointCloudSelf(pCloudGlobal, &Twc);

        int r = rand() % 155;
        int g = rand() % 155;
        int b = rand() % 155;
        SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
        bool result = AddPointCloudList(name, pCloudGlobal, ADD_POINT_CLOUD);
        if (!result) {
            delete pCloudGlobal;
            pCloudGlobal = NULL;
        }
    }
    return true;
}

// Default type = 0: replace when exist , 1: add when exist
bool Map::AddPointCloudList(const string &name, PointCloud *pCloud, int type) {
    unique_lock<mutex> lock(mMutexMap);
    if (pCloud == NULL) {
        std::cout << "NULL point cloud." << std::endl;
        return false;
    }
    // std::cout << "!!!!Map's pointcloud now: \n";
    // for (auto &pair: mmPointCloudLists) {
    //     auto strPoints = pair.first;
    //     std::cout << strPoints << "," << std::endl;
    // }

    // cout << "[Map::AddPointCloudList] " << name << std::endl;

    // Check repetition
    if (mmPointCloudLists.find(name) != mmPointCloudLists.end()) {
        // Exist
        // std::cout << "Exist" << std::endl;
        auto pCloudInMap = mmPointCloudLists[name];
        if (pCloudInMap == NULL) {
            // std::cout << "Error: the cloud " << name << " has been deleted." << std::endl;
            return false;
        }

        if (type == 0) {
            // replace it. 
            // std::cout << "Replace mmPointCloudLists:" << name << std::endl;
            pCloudInMap->clear(); // release it
            mmPointCloudLists[name] = pCloud;
        } else if (type == 1) {
            // add together
            // std::cout << "pCloudInMap->push_back(p), " << name << std::endl;
            for (auto &p : *pCloud)
                pCloudInMap->push_back(p);
        } else {
            // std::cout << "Wrong type : " << type << std::endl;
        }
        return false;
    } else {
        // std::cout << "Insert pointcloud :" << name << std::endl;
        mmPointCloudLists.insert(make_pair(name, pCloud));
        return true;
    }
}

// 删除点云
bool Map::DeletePointCloudList(const string &name, int type) {
    unique_lock<mutex> lock(mMutexMap);

    // std::cout << "!!!!!!!!!Ready to delete pointcloud: " << name << std::endl;

    if (type == 0) // complete matching: the name must be the same
    {
        auto iter = mmPointCloudLists.find(name);
        if (iter != mmPointCloudLists.end()) {
            // std::cout << "Exist " << std::endl;
            PointCloud *pCloud = iter->second;
            if (pCloud != NULL) {
                delete pCloud;
                pCloud = NULL;
            }
            mmPointCloudLists.erase(iter);
            return true;
        } else {
            // std::cerr << "PointCloud name " << name << " doesn't exsit. Can't delete it." << std::endl;
            return false;
        }
    } else if (type == 1) // partial matching
    {
        bool deleteSome = false;
        for (auto iter = mmPointCloudLists.begin(); iter != mmPointCloudLists.end();) {
            auto strPoints = iter->first;
            if (strPoints.find(name) != strPoints.npos) {
                // std::cout << "Ready to delete pointcloud: " << name << std::endl;
                PointCloud *pCloud = iter->second;
                if (pCloud != NULL) {
                    delete pCloud;  // bug: 这里出现了报错: free(): double free detected in tcache 2
                    pCloud = NULL;
                }
                iter = mmPointCloudLists.erase(iter);
                deleteSome = true;
                continue;
            }
            iter++;
        }
        return deleteSome;
    }

    return false;
}

bool Map::ClearPointCloudLists() {
    unique_lock<mutex> lock(mMutexMap);

    mmPointCloudLists.clear();
    return true;
}

std::map<string, PointCloud *> Map::GetPointCloudList() {
    unique_lock<mutex> lock(mMutexMap);
    return mmPointCloudLists;
}

PointCloud Map::GetPointCloudInList(const string &name) {
    unique_lock<mutex> lock(mMutexMap);

    if (mmPointCloudLists.find(name) != mmPointCloudLists.end())
        return *mmPointCloudLists[name];
    else
        return PointCloud(); // 空
}

void Map::addEllipsoidVisual(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsVisual.push_back(pObj);
}

vector<ellipsoid *> Map::GetAllEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsVisual;
}

void Map::ClearEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    // cout << "!!!! Map::ClearEllipsoidsVisual !!!!" << endl;
    mspEllipsoidsVisual.clear();
}

void Map::addEllipsoidObjects(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsObjects.push_back(pObj);
}

vector<ellipsoid *> Map::GetAllEllipsoidsObjects() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsObjects;
}

void Map::ClearEllipsoidsObjects() {
    unique_lock<mutex> lock(mMutexMap);
    // cout << "!!!! Map::ClearEllipsoidsObjects !!!!" << endl;
    mspEllipsoidsObjects.clear();
}

void Map::addEllipsoidObservation(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsObservation.push_back(pObj);
}

vector<ellipsoid *> Map::GetObservationEllipsoids() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsObservation;
}

void Map::ClearEllipsoidsObservation() {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsObservation.clear();
}

void Map::addBoundingbox(Boundingbox *pBox) {
    unique_lock<mutex> lock(mMutexMap);
    mvBoundingboxes.push_back(pBox);
}

std::vector<Boundingbox *> Map::GetBoundingboxes() {
    unique_lock<mutex> lock(mMutexMap);
    return mvBoundingboxes;
}

void Map::ClearBoundingboxes() {
    unique_lock<mutex> lock(mMutexMap);
    mvBoundingboxes.clear();
}

void Map::addToTrajectoryWithName(SE3QuatWithStamp *state, const string &name) {
    unique_lock<mutex> lock(mMutexMap);

    // 没有则生成一个
    if (mmNameToTrajectory.find(name) == mmNameToTrajectory.end()) {
        mmNameToTrajectory[name] = Trajectory();
        mmNameToTrajectory[name].push_back(state);
    } else {
        mmNameToTrajectory[name].push_back(state);
    }
}

Trajectory Map::getTrajectoryWithName(const string &name) {
    unique_lock<mutex> lock(mMutexMap);
    if (mmNameToTrajectory.find(name) == mmNameToTrajectory.end()) {
        return Trajectory();
    } else {
        return mmNameToTrajectory[name];
    }
}

bool Map::clearTrajectoryWithName(const string &name) {
    unique_lock<mutex> lock(mMutexMap);
    if (mmNameToTrajectory.find(name) != mmNameToTrajectory.end()) {
        mmNameToTrajectory[name].clear();
        return true;
    }

    return false;
}

bool Map::addOneTrajectory(Trajectory &traj, const string &name) {
    unique_lock<mutex> lock(mMutexMap);
    mmNameToTrajectory[name] = traj;

    return true;
}

void Map::addArrow(const Vector3d &center, const Vector3d &norm, const Vector3d &color) {
    unique_lock<mutex> lock(mMutexMap);
    Arrow ar;
    ar.center = center;
    ar.norm = norm;
    ar.color = color;
    mvArrows.push_back(ar);

    return;
}

std::vector<Arrow> Map::GetArrows() {
    unique_lock<mutex> lock(mMutexMap);
    return mvArrows;
}

void Map::clearArrows() {
    unique_lock<mutex> lock(mMutexMap);
    mvArrows.clear();
    return;
}

void Map::addEllipsoid(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoids.push_back(pObj);
    std::cout << "map add ellipsoid, mspEllipsoids.size() = " << mspEllipsoids.size() << std::endl;
}

vector<ellipsoid *> Map::GetAllEllipsoids() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoids;
}


void Map::ShowMapInfo()
{
    unique_lock<mutex> lock(mMutexMap);
    std::cout << "\n[Map Info]" << std::endl;
    int obj_num = mspMapObjects.size();
    std::cout << obj_num << " objects" << std::endl;
}

} // namespace ORB_SLAM2
