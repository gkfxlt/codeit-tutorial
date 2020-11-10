#include <codeit.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "basicsystem.hpp"

int main()
{
	auto&cs = codeit::system::ControlSystem::instance();
	cs.resetController(codeit::system::createController().release());
	cs.resetNrtControllerPool(codeit::system::createNrtControllerPool().release());
	cs.resetSensorRoot(new codeit::sensor::SensorRoot);
	cs.init();

	//1、主站对象mst的成员函数setControlStrategy()创建一个实时线程。其形参即是被调用的实时函数，在实时核中每1ms被调用1次
	//2、被调用函数可以实现主站与从站之间的数据交互，打印服务，log服务。(本例以读、写IO信号，并打印为例)
    auto mst= dynamic_cast<codeit::controller::SocketMaster*>(&cs.nrtControllerPool().atNrt(0));
    mst->setControlStrategy([&](Size i)
    {
        static int count{ 0 }; //count用于计数本函数setControlStrategy()执行的次数//
		static std::uint8_t value{ 0x01 };
        if (++count % 10 == 0)	//非实时核周期性执行，每执行1000次，执行本if条件内的语句
        {
            value = value << 1;
            if(value == 0) value = 0x01;

			std::cout << "count:" << std::dec << count << std::endl;
        }
    });
	
	//启动实时线程
    cs.start();
	//非实时线程睡眠100秒钟
    std::this_thread::sleep_for(std::chrono::seconds(100));
	//关闭实时线程
    cs.stop();
	return 0;
}
