#include <iostream>
#include <codeit.hpp>

using namespace std;
using namespace codeit::core;
using namespace codeit::controller;
using namespace codeit::model;
using namespace codeit::system;
using namespace codeit::function;
struct GetParam
{
	std::vector<double> part_pq;
	std::vector<int> motion_state;
	std::string currentplan;
	bool flag {true};
};

int main()
{
	GetParam out_data;
	out_data.part_pq.resize(3, 0);
	out_data.motion_state.resize(5, 0.3);
	//将JSON形式的字符串转换为vector//
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("part_pq", out_data.part_pq));
	out_param.push_back(std::make_pair<std::string, std::any>("motion_state", out_data.motion_state));
	out_param.push_back(std::make_pair<std::string, std::any>("currentplan", out_data.currentplan));
	out_param.push_back(std::make_pair<std::string, std::any>("flag", out_data.flag));
	std::string ret_str=codeit::system::parse_ret_value(out_param);



	//将JSON形式的字符串转换为vector//
	std::string str = "{\"di_vec\":[6,6,5,5,5,5],\"ai_vec\":[3,9,5,5,5,5],\"cmd_vec\":[\"wangwu\",\"wwu\"]}";
	vector<double> di_vec;
	vector<double> ai_vec;
	vector<std::string> cmd_vec;
	std::vector<std::pair<std::string, std::any>> ret;
	codeit::system::parse_recv_str(str, ret);


	return 0;
}
