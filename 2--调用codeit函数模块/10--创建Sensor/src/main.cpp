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
	cs.resetFuncRoot(codeit::system::createFuncRoot().release());
	
	cs.sensorPool().add<codeit::sensor::SensorTCP>("sensor", "7001", codeit::core::Socket::TCP, sizeof(codeit::sensor::DemoData));
	cs.init();

	//启动实时线程
    cs.start();
	
	auto& sensor = cs.sensorPool()[0];
	codeit::sensor::DemoData data;
	while (true) {
		sensor.display<codeit::sensor::DemoData>(data);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		std::cout << data.a << std::endl;
	}
		
	cs.runCmdLine();
	
	
	return 0;
}
