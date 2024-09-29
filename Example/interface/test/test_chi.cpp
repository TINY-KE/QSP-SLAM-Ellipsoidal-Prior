// 本函数用于测试卡方分布
// 2020-6-16晚 check成功!

#include <cmath>
#include <vector>
#include <iostream>

#include <boost/math/distributions/chi_squared.hpp>

double chi2cdf(int degree, double chi)
{
    boost::math::chi_squared mydist(degree);
    double p = boost::math::cdf(mydist,chi);
    return p;
}

double chi2pdf(int degree, double chi)
{
    double gamma = std::tgamma(degree / 2.0);

    double fenzi = pow(chi, degree/2.0-1) * exp(-chi/2.0);
    double fenmu = gamma * pow(2, degree/2.0);

    return (fenzi/fenmu);
}

int main()
{

    std::cout << "Test chi2 distribution value." << std::endl;

    std::vector<std::vector<double>> valueGroup = 
    {
        {1, 2},
        {2, 3},
        {3, 4},
        {6, 4},
        {2, 0.211},
        {14, 23.685},
    };

    std::cout << " Degree \t Chi2 \t Prob " << std::endl;
    for(int i=0;i<valueGroup.size();i++)
    {
        int deg = round(valueGroup[i][0]);
        double chi2 = valueGroup[i][1];
        std::cout << deg << " \t " << chi2 << " \t " << chi2pdf(deg, chi2) << std::endl;
    }

    std::cout << "DONE. " << std::endl;


    std::cout << "Begin CDF ... " << std::endl;

    std::cout << " Degree \t Chi2 \t Prob " << std::endl;
    for(int i=0;i<valueGroup.size();i++)
    {
        int deg = round(valueGroup[i][0]);
        double chi2 = valueGroup[i][1];
        std::cout << deg << " \t " << chi2 << " \t " << chi2cdf(deg, chi2) << std::endl;
    }
    
    return 0;

}