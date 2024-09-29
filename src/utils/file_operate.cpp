#include <boost/filesystem.hpp>
#include <sys/resource.h>

#include <iostream>


bool CreateDirIfNotExist(const std::string file_path)
{
  if (!boost::filesystem::exists(file_path)) {
    if (!boost::filesystem::create_directories(file_path)) {
    //   ROS_ERROR("Failed to create directory: %s", file_path.c_str());
      std::cerr << "Failed to create directory: " << file_path << std::endl;
      return false;
    }
  }
  return true;
}

void printMemoryUsage(){
  struct rusage rusage;
  if (getrusage(RUSAGE_SELF, &rusage) == 0) {
      std::cout << "\nMemory usage: " << (double)rusage.ru_maxrss / 1024. << " MB" << std::endl;
  } else {
      std::cerr << "Failed to get memory usage." << std::endl;
  }
}