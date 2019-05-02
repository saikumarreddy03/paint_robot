#include <Planar2R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar2R obj(2, 3);
	IRlibrary::Vec2 q = {0, M_PI/2.};
	obj.setConfig(q);
	q = obj.getXY();
	obj.setXY(q);
	auto q1 = obj.getConfig();
	std::cout << q[0] << " " << q[1] << std::endl;
	std::cout << q1[0] << " " << q1[1] << std::endl;
	return 0;
}

