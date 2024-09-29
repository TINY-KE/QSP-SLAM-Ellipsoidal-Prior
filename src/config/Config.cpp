
#include "Config.h"

namespace ORB_SLAM2{
    void Config::SetParameterFile( const std::string& filename )
    {
        if ( mConfig == nullptr )
            mConfig = shared_ptr<Config>(new Config);
        mConfig->mFile = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
        if ( !mConfig->mFile.isOpened())
        {
            std::cerr<<"parameter file "<< filename <<" does not exist."<<std::endl;
            mConfig->mFile.release();
            return;
        }
    }

    Config::~Config()
    {
        if ( mFile.isOpened() )
            mFile.release();
    }

    void Config::Init(){
        if ( mConfig == nullptr ){
            mConfig = shared_ptr<Config>(new Config);

            //  ************ default parameters here *************
            mConfig->SetValue("Tracking_MINIMUM_INITIALIZATION_FRAME", 15);       // The minimum frame number required for ellipsoid initialization using 2d observations.
            mConfig->SetValue("EllipsoidExtractor_DEPTH_RANGE", 6);         // the valid depth range (m)

        }
    }

    shared_ptr<Config> Config::mConfig = nullptr;

/**
 * 用来对比标准参数文件，检查参数是否齐全
*/
    void Config::CheckParams( const string& filename )
    {
        if ( mConfig == nullptr) {
            std::cerr << "Class Config has not been inited." << std::endl;
            return;
        }
        else
        {
            cv::FileStorage mCheckFile( filename.c_str(), cv::FileStorage::READ );
            if ( !mCheckFile.isOpened())
            {
                std::cerr << "parameter file "<< filename << " does not exist." << std::endl;
                mCheckFile.release();
                return;
            }

            // 获取根节点
            cv::FileNode rootCheck = mCheckFile.root();
            cv::FileNode root = mConfig->mFile.root();

            // 遍历根节点的子节点
            for (cv::FileNodeIterator it = rootCheck.begin(); it != rootCheck.end(); ++it) {
                // 获取子节点的名称
                std::string paramName = (*it).name();
                
                if (!root[paramName].empty()) {
                    // std::cout << "FileStorage contains parameter '" << paramName << "'" << std::endl;
                } else {
                    std::cout << "Param missing: " << paramName << "" << std::endl;
                }

            }
        }
    }
}