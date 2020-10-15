#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <sstream>
#include <codeit.hpp>

using namespace std;

int main()
{
	codeit::core::Calculator c;

	{
		codeit::core::Matrix X(0.1);
		codeit::core::Matrix Y = X;
	}

	try
	{
		c.addVariable("fiveee", "Number", 5.0);
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
	c.addVariable("FIVE", "Number", 5.0);
	c.addVariable("TWO", "Number", 2.0);
	c.addVariable("PI", "Number", 3.141592653);
	c.addFunction("multiply", {"Matrix", "Matrix" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return std::any_cast<codeit::core::Matrix&>(matrices.at(0))*std::any_cast<codeit::core::Matrix&>(matrices.at(1));
	});
	c.addFunction("multiply", { "Number", "Matrix" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return std::any_cast<double&>(matrices.at(0))*std::any_cast<codeit::core::Matrix&>(matrices.at(1));
	});
	c.addFunction("multiply", { "Matrix", "Number" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return std::any_cast<codeit::core::Matrix&>(matrices.at(0))*std::any_cast<double&>(matrices.at(1));
	});
	c.addFunction("multiply", { "Number", "Number" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return codeit::core::Matrix(std::any_cast<double&>(matrices.at(0))*std::any_cast<double&>(matrices.at(1)));
	});


	c.addFunction("addTwo", { "Matrix" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return std::any_cast<codeit::core::Matrix&>(matrices.at(0)) + 2.0;
	});
	c.addFunction("addTwo", { "Number" }, "Matrix", [](std::vector<std::any> &matrices)
	{
		return codeit::core::Matrix(std::any_cast<double&>(matrices.at(0)) + 2.0);
	});

	std::vector<double> t = { 1, 2, 3 };

	double x = 0.1;

	codeit::core::Matrix m = { 1, 2.2, 3,{}, 2, 3.2, 4 };
	codeit::core::Matrix n = { x,{}, x + 0.1 };

	//m.transpose();

	codeit::core::Matrix m2;

	//std::vector<Matrix> v = { m, m2 };

	codeit::core::Matrix k = { { m,{}, m },{ n,{}, n },{ 1,{},2,{},{},3,{},4 } };
	k.dsp();

	k(1, 1) = 31.415926532345e16;

	std::cout << k.toString();


	//not compatible for clang
	k = {};

	try
	{
		m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("1.5+Matrix({3.2*FIVE})").second);//{-2448.4937065177,1}
		m.dsp();
		
		m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("{3.2*FIVE}").second);//{-2448.4937065177,1}
		m.dsp();
		
		m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("{multiply(-100+multiply(1,2)+1.23*addTwo(2.1-3.2*FIVE*(3-1)*(TWO*PI - 1))-1.38, 8.1),1}").second);//{-2448.4937065177,1}
		m.dsp();
		//m = c.calculateExpression("multiply(multiply(1,2),2)");
		m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("2e-2+{1,{2,3};{4;5},{5;6},{7;8}}*3").second);
		m.dsp();
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << endl;
	}
	
	m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("1.0>{1.02,1.0,2e-2}").second);
	m.dsp();

	m = std::any_cast<codeit::core::Matrix>(c.calculateExpression("{1.02,1.0,2e-2}<=1.0").second);
	m.dsp();

	std::cout << "-------------------------------------------------" << std::endl;

	codeit::core::Calculator cmp;
	cmp.addVariable("tool_pq", "Matrix", codeit::core::Matrix(1.0));
	auto ret = cmp.calculateExpression("{tool_pq, 0.0}");
	auto mat = std::any_cast<codeit::core::Matrix&>(ret.second);
	std::cout << mat.toString() << std::endl;

	cmp.addVariable("tool_pq_s", "String", std::string("aaaa"));
	ret = cmp.calculateExpression("tool_pq_s");
	auto str__s = std::any_cast<std::string&>(ret.second);
	std::cout << str__s << std::endl;

	cmp.addVariable("i", "Number", double(1.0));
	auto v1 = cmp.calculateExpression("\"++i\"");
	std::cout << v1.first << std::endl;
	std::cout << std::any_cast<std::string>(v1.second) << std::endl;
	auto v2 = cmp.calculateExpression("++i");
	std::cout << std::any_cast<double>(v2.second) << std::endl;

	//auto v = cmp.calculateExpression("{{2.0,1};pi+sin(pi*2/3)+(1+(+2) - 100*5)/6 + sin(1.5*2.5+sin(0.1)),1}");
	//std::cout << v.first << std::endl;
	//std::any_cast<aris::core::Matrix&>(v.second).dsp();

	;
	auto v3 = cmp.calculateExpression("i = 1.523654");
	std::cout << std::any_cast<double>(v3.second) << std::endl;


	char aaa;
	cin >> aaa;
	return 0;
}

