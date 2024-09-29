//
// Created by hu on 10/19/21.
//
# include<Eigen/Core>


# include<fstream>
# include<iostream>

#include "Initializer.h"
#include "core/ConstrainPlane.h"

int main()
{
	std::ifstream fin("./svd_generate.txt");
	if (!fin.is_open())
		std::cout << "cannot open file";
	double lf_cache[5];
	Eigen::Matrix3d k = Eigen::Matrix3d::Zero(3,3);
	fin.ignore(128, ':'); // 跳过坏输入
	for (double & i : lf_cache)
	{
		fin >> i;
		if (fin.bad())
		{
			std::cout << "bad bad!!";
			break;
		}
		else
		{
			std::cout << i << '\n';
		}
	}
	k(0,0) = lf_cache[0];
	k(1,1) = lf_cache[1];
	k(0,2) = lf_cache[3];
	k(1,2) = lf_cache[4];
	k(2,2) = 1;
	std::cout << k << std::endl;

	fin.ignore(128, ':'); // 跳过坏输入
	int n;
	fin >> n;
	Eigen::MatrixXd se3s(n,7);
	Eigen::MatrixXd boxes(n,4);
	// ellips wants : x y z qx qy qz qw
	for (int i = 0; i < n; ++i)
	{
		if (fin.bad())
		{
			std::cout << "bad bad!!";
			break;
		}
		fin.ignore(128, ':'); // 跳过坏输入
		for (int j = 0; j < 7; ++j)
		{
			fin >> se3s(i,j);
		}
		fin.ignore(128, ':'); // 跳过坏输入

		for (int j = 0; j < 4; ++j)
		{
			fin >> boxes(i,j);
		}
	}
	std::cout << se3s << std::endl;
	std::cout << boxes << std::endl;

	ORB_SLAM2::Initializer svd_init(99999,99999);
	svd_init.flag_openFilter = false;

	auto q = svd_init.initializeQuadric(se3s,boxes,k);

	std::cout << q.generateQuadric() << std::endl;


	int count_p = 0;
	for (auto & it : q.mvCPlanesWorld)
	{
		std::cout << count_p << "======================" << std::endl;
		std::cout << it->toVector() << std::endl;
		count_p++;
	}
	fin.close();
	return 0;
}
