#include <gmapping/sensor/sensor_base/sensor.h>

namespace GMapping{

//传感器基类的构造函数
Sensor::Sensor(const std::string& name) {
	m_name=name;
}

Sensor::~Sensor() {
}

}// end namespace
