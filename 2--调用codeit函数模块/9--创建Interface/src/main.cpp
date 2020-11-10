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

	cs.init();

	//启动实时线程
    cs.start();
	cs.open();
	cs.runCmdLine();
	return 0;
}
