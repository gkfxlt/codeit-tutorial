#include <codeit.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "basicsystem.hpp"

int main()
{
	auto&cs = codeit::system::ControlSystem::instance();
	cs.resetController(codeit::system::createController().release());
	cs.resetModelPool(codeit::system::createModelPool().release());
	cs.resetSensorRoot(new codeit::sensor::SensorRoot);
	cs.resetFuncRoot(codeit::system::createFuncRoot().release());
	codeit::core::SerialPort::ComOptions options = { 1, CBR_9600, 'N',8,1,EV_RXCHAR };
	cs.interfacePool().add<codeit::system::ComInterface>("COM", options);

	cs.saveXmlFile(std::string("codeit.xml"));//将整个控制系统的配置保存为xml
	cs.modelPool().saveXmlFile(std::string("model.xml"));//只将控制系统的模型配置保存为xml
	cs.funcRoot().saveXmlFile(std::string("func.xml"));//只将控制系统的指令信息保存为xml
	
	std::cout << cs.interfacePool().xmlString() << endl;//输出interface的配置信息

	//对xml进行更改后，codeit从xml读取新的配置信息//
	cs.loadXmlFile(std::string("codeit.xml"));//更新整个控制系统的配置
	cs.modelPool().loadXmlFile(std::string("model.xml"));//更新控制系统的模型配置
	cs.funcRoot().loadXmlFile(std::string("func.xml"));//更新控制系统的指令信息





	cs.init();

	//启动实时线程
    cs.start();
	cs.open();
	cs.runCmdLine();
	return 0;
}
