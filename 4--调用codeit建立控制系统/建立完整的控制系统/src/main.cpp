#include <iostream>
#include <codeit.hpp>
#include <atomic>
#include "basicsystem.hpp"

using namespace std;
using namespace codeit::core;
using namespace codeit::controller;
using namespace codeit::model;
using namespace codeit::system;

int main()
{
	auto& cs = codeit::system::ControlSystem::instance();
    cs.resetController(createVrepController().release());
	cs.resetNrtControllerPool(createNrtControllerPool().release());
	cs.resetSensorRoot(new codeit::sensor::SensorRoot);
	cs.resetModelPool(createModelPool().release());
	cs.resetIOModelPool(createIOModelPool().release());
	cs.resetFuncRoot(createFuncRoot().release());
	cs.resetErrorInfoPool(createErrorInfoPool().release());

	cs.interfacePool().add<codeit::system::WebInterface>("ControlSock", "5866", codeit::core::Socket::TCP);
	//cs.interfacePool().add<codeit::cmdtarget::ProInterface>("ControlSock", "5866", core::Socket::WEB);

	cs.interfacePool().add<codeit::system::StateRtInterface>("StateSock", "5867", codeit::core::Socket::TCP);
	cs.interfacePool().add<codeit::system::WebInterface>("ErrorSock", "5868", codeit::core::Socket::TCP);

#ifdef WIN32
    codeit::core::SerialPort::ComOptions options = { 1, CBR_9600, 'N',8,1,EV_RXCHAR };
    cs.interfacePool().add<codeit::system::ComInterface>("COM", options);
#endif
	cs.saveXmlFile(std::string("kaanh.xml"));
	cs.model().saveXmlFile(std::string("model.xml"));
	cs.model().pointPool().saveXmlFile(std::string("data.xml"));

	cs.loadXmlFile(std::string("kaanh.xml"));
	cs.model().loadXmlFile(std::string("model.xml"));
	cs.model().pointPool().loadXmlFile(std::string("data.xml"));


	cs.init();
	cs.model().variablePool().add<codeit::model::MatrixVariable>("fine", codeit::core::Matrix(1, 2, 0.0));

	auto& cal = cs.model().calculator();
	createUserDataType(cal);
	cs.start();

	cs.setErrorinfoVer(0);
#ifdef WIN32
	auto sd = cs.controller().motionPool().size();
	for (auto& m : cs.controller().motionPool())
	{
		dynamic_cast<codeit::controller::VrepMotor&>(m).setVirtual(true);
	}
	for (auto& m : cs.controller().externMotionPool())
	{
		dynamic_cast<codeit::controller::VrepExternMotor&>(m).setVirtual(true);
	}
	/*for (auto& m : cs.controller().ioPool())
	{
		dynamic_cast<codeit::controller::EthercatIO&>(m).setVirtual(true);
	}*/
#endif // WIN32

	cs.open();//启动socket


	cs.runCmdLine();
	/*for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			cs.executeCmd(command_in);
		}
		catch (std::exception& e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}*/


	return 0;
}
