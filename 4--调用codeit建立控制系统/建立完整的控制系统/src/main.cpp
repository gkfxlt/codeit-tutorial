#include <iostream>
#include <codeit.hpp>
#include "basicsystem.hpp"

using namespace std;
using namespace codeit::core;
using namespace codeit::controller;
using namespace codeit::model;
using namespace codeit::system;
using namespace codeit::function;

int main()
{
	auto& cs = codeit::system::ControlSystem::instance();
#ifdef WIN32
	cs.resetController(createVrepController().release());
#endif
#ifdef UNIX
	cs.resetController(createEcatController().release());
#endif

	cs.resetModelPool(createModelPool().release());
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
	cs.saveXmlFile(std::string("codeit.xml"));
	cs.model().saveXmlFile(std::string("model.xml"));
	cs.model().pointPool().saveXmlFile(std::string("data.xml"));

	cs.loadXmlFile(std::string("codeit.xml"));
	cs.model().loadXmlFile(std::string("model.xml"));
	cs.model().pointPool().loadXmlFile(std::string("data.xml"));

	cs.init();
	
	auto& cal = cs.model().calculator();
	cs.start();

#ifdef WIN32
	for (auto& m : cs.controller().motionPool())
	{
		dynamic_cast<codeit::controller::VrepMotor&>(m).setVirtual(true);
	}
#endif // WIN32

	cs.open();//启动socket


	cs.runCmdLine();

	return 0;
}
