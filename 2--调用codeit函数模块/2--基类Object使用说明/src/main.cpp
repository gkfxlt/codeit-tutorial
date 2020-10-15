/// 本例子展示构造数据结构的过程:
///

#include <codeit.hpp>
using namespace codeit::core;

int main()
{
	// 手动构造family1 object //
	Object family1("family1");
	auto &father1 = family1.add<Object>("father");
	family1.add<Object>("uncle");
	father1.add<Object>("tom");
	father1.add<Object>("bob");
	// 将family1的xml字符串打印出来 //
	std::cout << family1.xmlString() << std::endl;


	// 使用xml字符串构造family2 //
	Object family2;
	family2.loadXmlStr(
		"<Object name=\"family2\">"
		"	<Object name=\"father\">"
		"		<Object name=\"tom\"/>"
		"		<Object name=\"bob\"/>"
		"	</Object>"
		"	<Object name=\"uncle\"/>"
		"</Object>");
	// 将family2的xml字符串打印出来 //
	std::cout << family2.xmlString() << std::endl;


	// 使用自己定义的Family，Man，Child类型构造family3 //
	Family family3("family3");
	auto &father3 = family3.add<Man>("father", 35, "teacher");
	family3.add<Man>("uncle", 33, "policeman");
	father3.add<Child>("tom", 8);
	father3.add<Child>("bob", 6);
	std::cout << family3.xmlString() << std::endl;

	// 以上代码也可以使用xml字符串构造 //
	Family family4;
	family4.loadXmlStr(
		"<Family name=\"family4\">"
		"	<Man name=\"father\" age=\"35\" job=\"teacher\">"
		"		<Child name=\"tom\" age=\"8\"/>"
		"		<Child name=\"bob\" age=\"6\"/>"
		"	</Man>"
		"	<Man name=\"uncle\" age=\"33\" job=\"policeman\"/>"
		"</Family>");
	std::cout << family4.xmlString() << std::endl;


	// 注册新类型 //
	Family family5;
	family5.loadXmlStr(
		"<Family name=\"family5\">"
		"	<Man name=\"father\" age=\"35\" job=\"teacher\">"
		"		<Child name=\"tom\" age=\"8\"/>"
		"		<Child name=\"bob\" age=\"6\"/>"
		"		<Child name=\"bill\" age=\"3\"/>"
		"	</Man>"
		"	<Man name=\"uncle\" age=\"33\" job=\"policeman\"/>"
		"</Family>");
	std::cout << family5.xmlString() << std::endl;

	return 0;
}

