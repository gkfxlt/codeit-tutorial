#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>

using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	class Child :public Object
	{
	public:
		auto virtual saveXml(codeit::core::XmlElement& xml_ele) const->void override
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("age", age_);
		}
		auto virtual loadXml(const codeit::core::XmlElement& xml_ele)->void override
		{
			Object::loadXml(xml_ele);
			age_ = attributeInt32(xml_ele, "age");
		}
		virtual ~Child() = default;
		explicit Child(const std::string& name = "child", int age = 0) :Object(name), age_(age) {};
		CODEIT_REGISTER_TYPE(Child);
		CODEIT_DEFINE_BIG_FOUR(Child);

	private:
		int age_;

	};


	class Man :public Object
	{
	public:
		auto virtual saveXml(codeit::core::XmlElement& xml_ele) const->void override
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("age", age_);
			xml_ele.SetAttribute("job", job_.c_str());
		}
		auto virtual loadXml(const codeit::core::XmlElement& xml_ele)->void override
		{
			Object::loadXml(xml_ele);
			age_ = attributeInt32(xml_ele, "age");
			job_ = attributeString(xml_ele, "job");
		}
		virtual ~Man() = default;
		explicit Man(const std::string& name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job) {};
		CODEIT_REGISTER_TYPE(Man);
		CODEIT_DEFINE_BIG_FOUR(Man);

	private:
		int age_;
		std::string job_;	
	};
	class Family :public Object
	{
	public:
		virtual ~Family() = default;
		explicit Family(const std::string& name = "family") :Object(name) {};
		CODEIT_REGISTER_TYPE(Family);
		CODEIT_DEFINE_BIG_FOUR(Family);
	};

	class Boy :public Child
	{
	public:
		virtual ~Boy() = default;
		explicit Boy(const std::string& name = "boy", int age = 0) :Child(name, age) {};
		CODEIT_REGISTER_TYPE(Boy);
		CODEIT_DEFINE_BIG_FOUR(Boy);
	};


	auto createDefaultData(codeit::model::Model& model)->void;
	auto updateStateRt(codeit::core::Msg& msg)->void;

}

#endif
