#include <codeit.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "basicsystem.hpp"

int main()
{
    std::cout << "start testing IO board" << std::endl;

	auto&cs = codeit::system::ControlSystem::instance();
	cs.resetController(codeit::system::createController().release());
	cs.resetSensorRoot(new codeit::sensor::SensorRoot);
	cs.init();

	//1、主站对象mst的成员函数setControlStrategy()创建一个实时线程。其形参即是被调用的实时函数，在实时核中每1ms被调用1次
	//2、被调用函数可以实现主站与从站之间的数据交互，打印服务，log服务。(本例以读、写IO信号，并打印为例)
   auto mst= dynamic_cast<codeit::controller::EthercatMaster*>(&cs.controller());
   mst->setControlStrategy([&]()
    {
        static int count{ 0 }; //count用于计数本函数setControlStrategy()执行的次数//
		static std::uint8_t value{ 0x01 };
        if (++count % 100 == 0)	//实时核执行一次周期是1ms，每执行1000次，执行本if条件内的语句
        {
			//控制EtherCAT IO板卡的DO口实现“走马灯”逻辑
            value = value << 1;
            if(value == 0) value = 0x01;
			
			//成员函数mout()是实时核打印函数接口，成员函数lout()是实时核log函数接口
            mst->mout() << "count:" << std::dec << count << std::endl;
			mst->lout() << "count:" << std::dec << count << std::endl;
			
			//1、成员函数ecSlavePool()创建从站vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()，at(1)表示第2个从站，即EtherCAT IO板卡在物理连接层面属于第二个从站
            //2、writePdo是写函数，第一个形参是index，第二个形参是subindex，第三个形参写DO的数值，第四个形参表示写操作的bit数
			mst->slavePool().at(1).writePdo(0x7001,0x01,&value,8);
        }
    });
	
	
	//启动实时线程
    cs.start();
	
	//非实时线程睡眠100秒钟
    std::this_thread::sleep_for(std::chrono::seconds(10));
	
	//关闭实时线程
    cs.stop();

	return 0;
}
