#include<iostream>
//程序引用codeit库的头文件
#include<codeit.hpp>
#include"basicsystem.hpp"

using namespace std;
//调用codeit库中的function模块
using namespace codeit::system; 

int main(int argc, char *argv[])
{
	//cs代表成员函数的引用，codeit是头文件，system是命名空间，ControlSystem是结构体
    auto&cs = codeit::system::ControlSystem::instance();
    cs.resetController(createController().release());
	cs.resetModelPool(createModelPool().release());
    cs.resetFuncRoot(createFuncRoot().release());
	cs.resetErrorInfoPool(codeit::system::createErrorInfoPool().release());
	

    std::cout<<"start"<<std::endl;
	//启动线程
	cs.init();
	cs.start();
#ifdef WIN32
	for (auto& m : cs.controller().motionPool())
	{
		dynamic_cast<codeit::controller::EthercatMotor&>(m).setVirtual(true);
	}
#endif // WIN32
	cs.runCmdLine();
}
