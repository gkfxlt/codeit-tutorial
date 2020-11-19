#include<iostream>
#include<codeit.hpp>
#include"basicsystem.hpp"
using namespace codeit::system; 
int main(int argc, char *argv[])
{
	//cs代表成员函数的引用，codeit是头文件，system是命名空间，ControlSystem是结构体
    auto&cs = codeit::system::ControlSystem::instance();
    cs.resetController(new codeit::controller::EthercatController);
	cs.resetModelPool(createModelPool().release());
	cs.resetErrorInfoPool(createErrorInfoPool().release());
    cs.resetFuncRoot(createFuncRoot().release());
    std::cout<<"start"<<std::endl;
	//启动线程
	cs.init();
	cs.start();
	cs.runCmdLine();
}
