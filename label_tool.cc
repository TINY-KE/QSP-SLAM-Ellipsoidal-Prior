#include <iostream>
#include <thread>
// #include "src/tools/Labeller.h"
#include "Labeller.h"

using namespace std;

int main()
{
    std::cout << "Hello, world." << std::endl;
    std::string settingfile = "/home/lj/Documents/codes/QSP-SLAM/configs/redwood_chair_01053.yaml";
    std::string pcd_filepath = "/home/lj/Documents/codes/QSP-SLAM/map/redwood/01053/map.pcd";
    std::string ply_filepath = "/home/lj/Documents/dataset/RedwoodOS/data/mesh/01053.ply";
    // // Labeller labeller(settingfile);
    
    Labeller* mpLabeller;

    mpLabeller = new Labeller(settingfile);
    mpLabeller->ReadPCDFile(pcd_filepath);

    // mpLabeller->ReadPLYFile(ply_filepath);

    mpLabeller->Run();

    return 0;
}