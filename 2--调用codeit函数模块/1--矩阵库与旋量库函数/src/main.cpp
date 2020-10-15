#include <iostream>
#include <codeit.hpp>

using namespace std;
using namespace codeit::core;
using namespace codeit::controller;
using namespace codeit::model;
using namespace codeit::system;
using namespace codeit::function;

int main()
{
	/*调用codeit矩阵运算函数
	函数：double s_norm(Size n, double *x,)
	      计算维数为n的一维向量x的模
	*/
	double x[3] = { 1,2,3 };
	Size n = 3;
	double x_norm = s_norm(n, x);



	return 0;
}
