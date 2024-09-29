#include "utils/dataprocess_utils.h"

#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sstream>
#include <unistd.h>
#include <dirent.h>

#include <string>

#include <iomanip> // for setprecision

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace std;
using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;

bool compare_func_stringasdouble(string &s1, string &s2)
{
    string bareS1 = splitFileNameFromFullDir(s1, true);
    string bareS2 = splitFileNameFromFullDir(s2, true);
    bool result;
    try{
        result = stod(bareS1) < stod(bareS2);
    }
    catch(std::invalid_argument)
    {
        std::cout << "Error occurs when trying to get the number because at least ONE of these: " << std::endl;
        std::cout << bareS1 << ", from : " << s1 << std::endl;
        std::cout << bareS2 << ", from : " << s2 << std::endl;
        abort();
    }

    return result;
}

// update : 2020-6-23 , 不考虑文件夹.
void GetFileNamesUnderDir(string path,vector<string>& filenames)
{
    filenames.clear();
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist : " << path.c_str() <<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            // 跳过文件夹
            bool is_dir = (strstr(ptr->d_name,"."))==NULL; // 不包含后缀
            if(is_dir) continue;
            filenames.push_back(path + "/" + ptr->d_name);
    }
    }
    closedir(pDir);
}

string splitFileNameFromFullDir(string &s, bool bare)
{

    int pos = s.find_last_of('/');
    string name(s.substr(pos+1));

    if( bare )
    {
        int posBare = name.find_last_of('.');
        string bare_name(name.substr(0, posBare));
        return bare_name;
    }

    return name;

}

void sortFileNames(vector<string>& filenames, vector<string>& filenamesSorted)
{
    filenamesSorted = filenames;
    sort(filenamesSorted.begin(), filenamesSorted.end(), compare_func_stringasdouble);
}

Eigen::MatrixXd readDataFromFile(const char* fileName, bool dropFirstline){
    ifstream fin(fileName);
    string line;

    if(dropFirstline)
        getline(fin, line);  // drop this line

    MatrixXd mat;
    int line_num = 0;
    while( getline(fin, line) )
    {
        if(line.size()==0) continue;
        if(line[0]=='#') continue;          // filt comments ( start with # )
        
        vector<string> s;
        boost::split( s, line, boost::is_any_of( " \t," ), boost::token_compress_on );

        // 输出s里的每个东西看看
        // std::cout << " === Output Line: " << std::endl;
        // for(auto s_bit:s)
        // {
        //     std::cout << s_bit.size() << " : " << s_bit << std::endl;
        //     if(s_bit.size()==1) std::cout << int(*s_bit.c_str()) << std::endl;
        // }

        // 检查最后一个是否有空格
        int line_size = s.size();
        if(int(*(s[line_size-1].c_str()))==13) {
            line_size--;
            std::cout << " - Find a Return and clear it." << std::endl;
            // std::cout << 
        }
        
        VectorXd lineVector(line_size);
        bool bValid = true;
        for (int i=0;i<line_size;i++){
            try {
                lineVector(i) = stod(s[i]);
            }
            catch(std::out_of_range)
            {
                std::cout << "out of range : " << s[i] << "; " << line << std::endl;
                // abort();
                // 忽略该行
                bValid = false;
                break;
            }
        }

        if(bValid){
            if(line_num == 0)
                mat.conservativeResize(1, line_size);
            else
                // vector to matrix.
                mat.conservativeResize(mat.rows()+1, mat.cols());

            mat.row(mat.rows()-1) = lineVector;

            line_num++;
        }
    }
    fin.close();

    return mat;
}

bool saveMatToFile(Eigen::MatrixXd &matIn, const char* fileName){
    ofstream fout;
    fout.open(fileName, ios::trunc|ios::out|ios::in);

    int rows = matIn.rows();
    for(int i=0;i<rows;i++)
    {
        VectorXd v = matIn.row(i);
        int nums = v.rows();
        for( int m=0;m<nums;m++){
            fout << std::setprecision(18) << v(m);

            if( m== nums-1 )
                break;
            fout << " ";
        }

        fout << std::endl;
    }
    fout.close();

    return true;
}

std::vector<std::vector<string>> readStringFromFile(const char* fileName, int dropLines){
    ifstream fin(fileName);
    string line;

    if(dropLines>0){
        for(int i=0; i<dropLines;i++)
            getline(fin, line); // drop this line
    }
    
    std::vector<std::vector<string>> strMat;
    while( getline(fin, line) )
    {
        if(line.size()==0) continue;
        if(line[0]=='#') continue;          // filt comments ( start with # )
        
        vector<string> s;
        boost::split( s, line, boost::is_any_of( " \t," ), boost::token_compress_on );

        strMat.push_back(s);
    }
    fin.close();

    return strMat;
}

// 不要被名字迷惑，该函数专门用于判断该观测是否满足条件：位于边界？太小？
// return True if the measure lies on the image border or does not meet the size requirement
bool calibrateMeasurement(Vector4d &measure , int rows, int cols, int config_boarder, int config_size){

    int x_length = measure[2] - measure[0];
    int y_length = measure[3] - measure[1];
    if( x_length < config_size || y_length < config_size ){
        std::cout << " [small detection " << config_size << "] invalid. " << std::endl;
        return true;
    }

    Vector4d measure_calibrated(-1,-1,-1,-1);
    Vector4d measure_uncalibrated(-1,-1,-1,-1);

    int correct_num = 0;
    if(  measure[0]>config_boarder && measure[0]<cols-1-config_boarder )
    {
        measure_calibrated[0] = measure[0];
        correct_num++;
    }
    if(  measure[2]>config_boarder && measure[2]<cols-1-config_boarder )
    {
        measure_calibrated[2] = measure[2];
        correct_num++;
    }
    if(  measure[1]>config_boarder && measure[1]<rows-1-config_boarder )
    {
        measure_calibrated[1] = measure[1];
        correct_num++;
    }
    if(  measure[3]>config_boarder && measure[3]<rows-1-config_boarder )
    {
        measure_calibrated[3] = measure[3];
        correct_num++;
    }

    // measure 保持原样!
    // measure = measure_calibrated;

    // 新增过滤: 要求至少2个以上，则认为 非边界
    // if( correct_num < 2)
    if( correct_num < 4)
        return true;
    else
        return false;

}