// Basic input and output of the TUM-RGBD dataset.

#include "io.h"
#include "utils/dataprocess_utils.h"
#include "utils/matrix_utils.h"

#include <iostream>
#include <string>
#include <fstream>

using namespace std;
namespace TUMRGBD
{
    inline bool is_fileExist(const string& name)
    {
        ifstream f(name.c_str());
        return f.good();
    }

    void Dataset::loadDataset(string &path){
        cout << "Load dataset from: " << path << endl;
        
        msDatasetDir = path;
        msRGBDir = msDatasetDir + "/rgb/";
        msDepthDir = msDatasetDir + "/depth/";
        msGroundtruthPath = msDatasetDir + "/groundtruth.txt";
        msAssociatePath = msDatasetDir + "/associate.txt";
        msAssociateGroundtruthPath = msDatasetDir + "/associateGroundtruth.txt";

        // get all the file names under the directory
        // std::cout << "Load rgb file names..." << std::endl;
        GetFileNamesUnderDir(msRGBDir, mvRGBFileNames);
        sortFileNames(mvRGBFileNames, mvRGBFileNames);
        miTotalNum = mvRGBFileNames.size();

        // generate the map between the timestamps to the depth images
        loadGroundTruthToMap(msGroundtruthPath);
        // std::cout << "2..." << std::endl;

        // generate the index from ID to the timestamps of rgb images
        generateIndexIdToRGBTimeStamp();      
        // std::cout << "2/1..." << std::endl;
  
        // generate the associations from rgb Timestamp to depth Timestamp
        LoadAssociationRGBToDepth(msAssociatePath);
        
        // std::cout << "3..." << std::endl;
        if(is_fileExist(msAssociateGroundtruthPath)){
            // generate the associations from rgb timestamp to the groundtruth
            LoadAssociationRGBToGroundtruth(msAssociateGroundtruthPath);
        }
        else 
        {
            std::cout << "Please check associateGroundtruth.txt!" << std::endl;
            abort();

            // 不存在则默认一样的方法感觉不可行，会导致大量帧不满足条件被跳过。
            // std::cout << " [Rgb->Gt] No associateGroundtruth.txt, keep the same as RGB timestamp." << std::endl;
            // mvIdToGroundtruthTimeStamp = mvIdToRGBTimeStamp;    // assume rgb,gt timestamps are the same
        }

        // get debug file num:
        std::cout << "Load groundtruth poses: " << mmTimeStampToPose.size() << std::endl;
        std::cout << "Images: " << mvIdToRGBTimeStamp.size() << std::endl;
        // std::cout << "mvIdToGroundtruthTimeStamp: " << mvIdToGroundtruthTimeStamp.size() << std::endl;
        

        miCurrentID = -1;
        mbDetectionLoaded = false;
        mbOdomSet = false; 
        mbDepthPreprocess = false;

        mBboxType = BBOX_STORAGE_TWO_POINT;
    }

    bool Dataset::readFrame(cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose){
        if(miCurrentID < miTotalNum-1) {
            miCurrentID++;
            bool result = findFrameUsingID(miCurrentID, rgb, depth, pose);
            return result;
        }
        else
        {
            std::cout << "[Dataset] no data left." << std::endl;
            return false;
        }
    }

    int Dataset::findFrameUsingTimestampGetId(double timestamp, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose)
    {
        // 找到timestamp对应的最近的id
        int id = 0;
        for(;id<miTotalNum;id++)
        {
            if(std::abs(timestamp - stod(mvIdToRGBTimeStamp[id])) < 0.001)
                break;
        }

        if(findFrameUsingID(id, rgb,depth,pose))
            return id;  
        else 
            return -1;
    }

    bool Dataset::findFrameUsingTimestamp(double timestamp, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose)
    {
        // 找到timestamp对应的最近的id
        int id = 0;
        for(;id<miTotalNum;id++)
        {
            if(std::abs(timestamp - stod(mvIdToRGBTimeStamp[id])) < 0.001)
                break;
        }

        return findFrameUsingID(id, rgb,depth,pose);
    }

    bool Dataset::findFrameUsingID(int id, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose){
        if( id <0 || id>=miTotalNum) return false;
        
        int currentID = id;
        string depthTimeStampAssociated = mvIdToDepthTimeStamp[currentID];
        if(depthTimeStampAssociated == ""){
            // std::cout << "[Dataset] fail to load the depth timestamp." << std::endl;
            return false;  
        }

        string depthPath = msDatasetDir + "/" + mvIdToDepthImagePath[currentID];

        string gtTimeStamp = mvIdToGroundtruthTimeStamp[currentID];

        bool bFindPose = false;
        
        if( !mbOdomSet )    // if the odometry is set, return the pose of the odometry instead of the groundtruth
            bFindPose = getPoseFromTimeStamp(gtTimeStamp, pose);
        else
            bFindPose = getPoseFromRGBTimeStamp(mvIdToRGBTimeStamp[currentID], pose); 

        if(bFindPose){
            rgb = cv::imread(mvRGBFileNames[currentID], IMREAD_UNCHANGED);
            depth = ReadDepthImage(depthPath);

            // check if rgb, depth is valid
            if(rgb.rows==0)
                std::cout << "Please check the image exists: " << mvRGBFileNames[currentID] << std::endl;
            
            if(depth.rows==0)
                std::cout << "Please check the image exists: " << depthPath << std::endl;

            return true;
        }
        else
        {
            // std::cout << "[Dataset] fail to find the pose ." << std::endl;
            return false;   
        }
    }

    inline double calculateRealZ(double f, double d, double xi, double yi){
        return f*sqrt(d*d/(xi*xi+f*f+yi*yi));
    }

    cv::Mat Dataset::preprocessDepth(cv::Mat& depth, double fx, double cx, double cy){
        cv::Mat output(depth.rows, depth.cols, CV_16UC1);

        for( int y = 0; y< output.rows; y++)
            for( int x = 0; x< output.cols; x++)
            {
                ushort d = depth.at<ushort>(y,x);
                double realz = calculateRealZ(fx, double(d), (x - cx), (y - cy));
                output.at<ushort>(y, x)  = ushort(realz);

            }

        return output;

    }

    cv::Mat Dataset::ReadDepthImage(const string& imName)
    {
        cv::Mat depth_png = cv::imread(imName, IMREAD_UNCHANGED);
        cv::Mat depth;
        double fx = mK(0,0);        // stay positive.
        double cx = mK(0,2);
        double cy = mK(1,2);
        
        if(mbDepthPreprocess)
        {
            depth = preprocessDepth(depth_png, fx, cx, cy);
        }
        else 
            depth = depth_png;
        return depth;
    }

    // 可认为有序，用二分查找
    map<string, string>::const_iterator AssociateWithNumber(const map<string, string>& map, const string &timestamp){
        // ************************ 使用 min vector 加速， 幻想：这个最小化是做了优化的，速度可以快10倍 ***************
        // // map 转为 vector
        // auto iter = map.begin();
        // VectorXd vec; vec.resize(map.size());
        // int id = 0;
        // for( ; iter != map.end(); iter++ )
        //     vec(id++) = atof(iter->first.c_str());

        // VectorXd vec_diff = (vec.array() - atof(timestamp.c_str())).cwiseAbs();
        
        // MatrixXd::Index minRow, minCol;
        // double min = vec_diff.minCoeff(&minRow,&minCol);

        // int id_min = minRow;
        // if(min < 0.001){
        //     auto iter_out = map.begin();
        //     std::advance(iter_out, id_min);
        //     return iter_out;
        // }
        // else 
        //     return map.begin();
        // ************************ 使用 min vector 加速， 幻想：这个最小化是做了优化的，速度可以快10倍 ***************

        // ************* 新方法，使用二分查找加速 **************

        auto iter = map.begin();
        for( ; iter != map.end(); iter++ )
        {
            if( iter->first == "" || timestamp == "" ) continue;
            if( std::abs(atof(iter->first.c_str()) - atof(timestamp.c_str())) < 0.001 )
            {
                // get it
                return iter;
            }
        }
        return iter;
    }

    map<string, VectorXd>::const_iterator AssociateWithNumber(const map<string, VectorXd>& map, const string &timestamp){
        auto iter = map.begin();
        for( ; iter != map.end(); iter++ )
        {
            if( iter->first == "" || timestamp == "" ) continue;
            if( std::abs(atof(iter->first.c_str()) - atof(timestamp.c_str())) < 0.001 )
            {
                // get it
                return iter;
            }
        }
        return iter;
    }

    // get pose from the map using the timestamp as index.
    // attention: this version uses string type of timestamp to search, which means their timestamp MUST be totally the same.
    //      it will be updated to double type in the future.
    bool Dataset::getPoseFromTimeStamp(string &timestamp, VectorXd &pose){
        for( auto iter : mmTimeStampToPose )
        {
            if( std::abs(std::atof(iter.first.c_str()) - std::atof(timestamp.c_str())) < 0.001 )
            {
                pose = iter.second;
                return true;
            }
        }

        // std::cout << "timestamp: " << timestamp << " x " << std::endl;
        return false;        
    }

    bool Dataset::getPoseFromRGBTimeStamp(string &timestamp, VectorXd &pose){
        auto iter = AssociateWithNumber(mmOdomRGBStampToPose, timestamp);
        // auto iter = mmOdomRGBStampToPose.find(timestamp);
        if( iter != mmOdomRGBStampToPose.end() )
        {
            pose = iter->second;
            return true;
        }
        else
        {
            return false;
        }
        
    }

    void Dataset::loadGroundTruthToMap(string &path){
        std::vector<std::vector<std::string>> strMat = readStringFromFile(path.c_str(), 0);

        int totalPose = strMat.size();
        for(int i=0;i<totalPose; i++)
        {
            std::vector<std::string> strVec = strMat[i];
            string timestamp = strVec[0];

            VectorXd pose; pose.resize(7);
            for(int p=1;p<8;p++)
                pose(p-1) = stod(strVec[p]);
            
            mmTimeStampToPose.insert(make_pair(timestamp, pose));
        }

    }

    void Dataset::LoadAssociationRGBToDepth(string &path)
    {
        std::vector<std::vector<std::string>> associateMat = readStringFromFile(msAssociatePath.c_str());

        // std::cout << " - 1 " << std::endl;
        map<string, string> mapRGBToDepth;
        map<string, string> mapRGBToDepthImagePath;
        int associationNum = associateMat.size();
        for(int i=0;i<associationNum; i++)
        {
            // std::cout << " - start :  " << i << std::endl;
            std::vector<std::string> lineVec = associateMat[i];
            string rgbTS = lineVec[0];
            string depthTS = lineVec[2];
            // std::cout << " - end :  " << i << std::endl;
            mapRGBToDepth.insert(make_pair(rgbTS, depthTS));
            mapRGBToDepthImagePath.insert(make_pair(rgbTS, lineVec[3]));
        }
        // std::cout << " - 2 " << std::endl;

        // for every rgb timestamp, find an associated depth timestamp
        mvIdToDepthTimeStamp.resize(miTotalNum);
        mvIdToDepthImagePath.resize(miTotalNum);

        std::cout << "TotalNum: " << miTotalNum << " / idToRGB : " << mvIdToRGBTimeStamp.size() << std::endl;

        clock_t time_frameStart = clock();
        // 9-28 该步骤太慢。 TotalNum = 19729
        for( int p=0;p<miTotalNum;p++)
        {
            auto iter = AssociateWithNumber(mapRGBToDepth, mvIdToRGBTimeStamp[p]);
            // auto iter = mapRGBToDepth.find(mvIdToRGBTimeStamp[p]);
            if(iter!=mapRGBToDepth.end())
            {
                mvIdToDepthTimeStamp[p] = iter->second;
                mvIdToDepthImagePath[p] = mapRGBToDepthImagePath[iter->first];
            }
            else
            {
                mvIdToDepthTimeStamp[p] = "";   // empty stands for null
                mvIdToDepthImagePath[p] = "";
            }
        }
        clock_t time_frameEnd = clock();
        double time_cost = (double)(time_frameEnd - time_frameStart) / CLOCKS_PER_SEC;
        std::cout << "TIME COST FOR ASSOCIATING : " << time_cost << " s. " << std::endl;
    }

    void Dataset::LoadAssociationRGBToGroundtruth(const string &path)
    {
        std::vector<std::vector<std::string>> associateMat = readStringFromFile(path.c_str());

        map<string, string> mapRGBToGt;
        int associationNum = associateMat.size();
        for(int i=0;i<associationNum; i++)
        {
            std::vector<std::string> lineVec = associateMat[i];
            string rgbTS = lineVec[0];
            string gtTS = lineVec[2];

            // Considering the precision of the timestamps of the result from the associate.py in TUM-RGB-D dataset,
            // we need to eliminate two zeros in the tails to make the groundtruth and the association have the same precision
            gtTS = gtTS.substr(0, gtTS.length()-2);

            mapRGBToGt.insert(make_pair(rgbTS, gtTS));
        }

        // for every rgb timestamp, find an associated groundtruth timestamp
        mvIdToGroundtruthTimeStamp.resize(miTotalNum);
        for( int p=0;p<miTotalNum;p++)
        {
            auto iter = AssociateWithNumber(mapRGBToGt, mvIdToRGBTimeStamp[p]);
            // auto iter = mapRGBToGt.find(mvIdToRGBTimeStamp[p]);
            if(iter!=mapRGBToGt.end())
            {
                mvIdToGroundtruthTimeStamp[p] = iter->second;
            }
            else
            {
                mvIdToGroundtruthTimeStamp[p] = "";   // empty stands for null
            }
            
        }
    }

    void Dataset::generateIndexIdToRGBTimeStamp(){
        // extract the bare name from a full path
        mvIdToRGBTimeStamp.clear();
        for(auto s:mvRGBFileNames)
        {
            string bareName = splitFileNameFromFullDir(s, true);
            mvIdToRGBTimeStamp.push_back(bareName);
        }
    }

    bool Dataset::empty(){
        return miCurrentID >= miTotalNum-1;
    }

    int Dataset::getCurrentID(){
        return miCurrentID;
    }

    int Dataset::getTotalNum()
    {
        return miTotalNum;
    }

    bool Dataset::loadDetectionDir(string &path)
    {
        msDetectionDir = path;
        mbDetectionLoaded = true;
    }

    // id: -1, currentID; 
    Eigen::MatrixXd Dataset::getDetectionMat(int id){
        Eigen::MatrixXd detMat;
        if(!mbDetectionLoaded){
            std::cerr << "Detection dir has not loaded yet." << std::endl;
            return detMat;
        }
        // get the RGB timestamp as the name of the object detection file
        int currentId = id!=-1?id:miCurrentID;

        string bairName = mvIdToRGBTimeStamp[currentId];
        string fullPath = msDetectionDir + bairName + ".txt";

        detMat = readDataFromFile(fullPath.c_str());

        Eigen::MatrixXd bbox_output;
        if(mBboxType == BBOX_STORAGE_YOLO)
        {
            std::cout << "DEBUG detMat \n " << detMat << std::endl;
            // calculate x1,y1,x2,y2 from center, width, height
            // x1
            Eigen::VectorXd col_center_x = detMat.col(1);
            Eigen::VectorXd col_center_y = detMat.col(2);
            Eigen::VectorXd col_w = detMat.col(3);
            Eigen::VectorXd col_h = detMat.col(4);

            Eigen::VectorXd col_x1 = (col_center_x - col_w / 2.0);
            Eigen::VectorXd col_y1 = (col_center_y - col_h / 2.0);
            Eigen::VectorXd col_x2 = (col_center_x + col_w / 2.0);
            Eigen::VectorXd col_y2 = (col_center_y + col_h / 2.0);

            detMat.col(1) = col_x1;
            detMat.col(2) = col_y1;
            detMat.col(3) = col_x2;
            detMat.col(4) = col_y2;

            std::cout << "DEBUG detMat \n " << detMat << std::endl;
        }

        bbox_output = detMat;
        return bbox_output;
        
    }

    vector<int> Dataset::generateValidVector(){
        vector<int> validVec;
        for(int i=0; i<miTotalNum; i++)
        {
            if(judgeValid(i)) 
                validVec.push_back(i);
        }

        return validVec;
    }

    bool Dataset::judgeValid(int id)
    {
        if( id <0 || id>=miTotalNum) return false;
        
        int currentID = id;
        
        string depthTimeStampAssociated = mvIdToDepthTimeStamp[currentID];
        if(depthTimeStampAssociated == ""){
            std::cout << "No depthTimeStampAssociated. " << std::endl;
            return false;   
        }

        string gtTimeStamp = mvIdToGroundtruthTimeStamp[currentID];

        VectorXd pose;
        if(getPoseFromTimeStamp(gtTimeStamp, pose))
            return true;
        else
        {
            return false;  
        }
            
    }

    bool Dataset::SetOdometryAndLoadAssociate(const string& dir_odom, const string& dir_associate, bool calibrate)
    {
        bool bOdom = false, bAsso = false;
        
        bOdom = SetOdometry(dir_odom, calibrate);
        if(is_fileExist(dir_associate)){
            LoadAssociationRGBToGroundtruth(dir_associate);
            bAsso = true;
        }
        else
        {
            // 提醒 odom 的每一条与 rgb 要一样
            std::cout << " ====== ATTENTION: Odometry timestamps are the same as RGB timestamps. !!! =====" << std::endl;
            bAsso = true;
        }
        

        return bOdom && bAsso;
    }

    bool Dataset::SetOdometry(const string& dir_odom, bool calibrate){

        std::vector<std::vector<std::string>> strMat = readStringFromFile(dir_odom.c_str(), 0);

        if(strMat.size() == 0) {
            std::cerr << " Odometry dir error! Keep gt. " << std::endl;
            return false;
        }

        mmOdomRGBStampToPose.clear();
        int totalPose = strMat.size();

        for(int i=0;i<totalPose; i++)
        {
            std::vector<std::string> strVec = strMat[i];
            string timestamp = strVec[0];

            VectorXd pose; pose.resize(7);
            for(int p=1;p<8;p++)
                pose(p-1) = stod(strVec[p]);
            
            mmOdomRGBStampToPose.insert(make_pair(timestamp, pose));
        }
        std::cout << "Setting odometry succeeds." << std::endl;


        if( calibrate )
        {
            std::cout << "Get calibrate transform... " << std::endl;

            // find the corresponding groundtruth of the first timestamp of the odometry
            bool findCalibTrans = false;
            int transId = 0;
            for(auto timestampOdomPair: mmOdomRGBStampToPose)
            {
                string timestamp_gt = mvIdToGroundtruthTimeStamp[transId];  // assume that all the rgb images have corresponding odometry values
                
                assert( mvIdToRGBTimeStamp[transId] == timestampOdomPair.first && "Odom should start from the first rgb frame." );

                VectorXd gtPose;
                if( getPoseFromTimeStamp(timestamp_gt, gtPose) )
                {
                    g2o::SE3Quat pose_wc; pose_wc.fromVector(gtPose);
                    g2o::SE3Quat pose_oc; pose_oc.fromVector(timestampOdomPair.second);
                    mTransGtCalibrate = new g2o::SE3Quat;
                    *mTransGtCalibrate = pose_wc * pose_oc.inverse();

                    findCalibTrans = true;

                    break;
                }
                transId ++ ;
            }

            if( !findCalibTrans)
            {
                std::cerr << "Can't find calibrate transformation... Close calibraton!"<< std::endl;
                calibrate = false;
            }
            else
            {
                std::cout << "Find calibration trans ID: " << transId << std::endl;

                
                for(auto timestampOdomPair: mmOdomRGBStampToPose)
                {
                    // calibrate all
                    VectorXd pose = timestampOdomPair.second;
                    VectorXd pose_processed; 
                    if(calibrate)
                        pose_processed = calibratePose(pose);
                    else
                        pose_processed = pose;    

                    mmOdomRGBStampToPose[timestampOdomPair.first] = pose_processed;
                    
                }
                
            }
            
        }

        mbOdomSet = true;
        return true;
    }

    VectorXd Dataset::calibratePose(VectorXd& pose)
    {
        g2o::SE3Quat pose_c; pose_c.fromVector(pose.tail(7));
        g2o::SE3Quat pose_w = (*mTransGtCalibrate) * pose_c;

        return pose_w.toVector();
    }

    void Dataset::SetCurrentID(int id)
    {
        if ( id >= miTotalNum ){
            std::cout << "Fail. id is larger than totalNum : " << miTotalNum << std::endl;
            return;
        }

        miCurrentID = id;

        return;
    }

    double Dataset::GetCurrentTimestamp()
    {
        return GetTimestamp(miCurrentID);
    }

    double Dataset::GetTimestamp(int id)
    {
        return stod(mvIdToRGBTimeStamp[id]);
    }

    void Dataset::OpenDepthPreprocess(const Eigen::Matrix3d& K)
    {
        std::cout << "[Dataset] Open depth preprocess." << std::endl;
        mbDepthPreprocess = true;
        mK = K;
    }

    void Dataset::SetBboxStorage(int type)
    {
        std::cout << "[Dataset] Choose bbox type : " << type << std::endl;
        mBboxType = type;
    }

    MatrixXd Dataset::GetGroundtruthTrajectory()
    {
        MatrixXd gtMat; gtMat.resize(0, 8); // timestamp xyz qx-w
        
        // 默认并非按时间戳顺序，而是字符串顺序.
        // map<string, VectorXd> mmTimeStampToPose

        for( int i=0; i< mvIdToGroundtruthTimeStamp.size(); i++)
        {
            string gtTimeStamp = mvIdToGroundtruthTimeStamp[i];

            VectorXd pose;
            if(!getPoseFromTimeStamp(gtTimeStamp, pose))
                continue;
            double timestamp = stod(gtTimeStamp);
            VectorXd pose_withtime; pose_withtime.resize(8);
            pose_withtime << timestamp, pose;
            addVecToMatirx(gtMat, pose_withtime);
        }

        return gtMat;
    }

    bool Dataset::readImageFrame(cv::Mat &rgb, cv::Mat &depth)
    {
        if(empty()) return false;
        miCurrentID++;

        string depthPath = msDatasetDir + "/" + mvIdToDepthImagePath[miCurrentID];
        rgb = cv::imread(mvRGBFileNames[miCurrentID], IMREAD_UNCHANGED);
        depth = ReadDepthImage(depthPath);
        
        return true;
    }

    std::string Dataset::GetDatasetDir()
    {
        return msDatasetDir;
    }

}