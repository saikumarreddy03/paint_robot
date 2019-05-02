#include <Spatial3R.hpp>
#include <iostream>
#include <cmath>

int main(int argc, char **argv){
	IRlibrary::Spatial3R Spatial3Robj(2,3,4);
	IRlibrary::Vec3 q ={1,2,3};
	Spatial3Robj.setConfig(q);
	auto position = Spatial3Robj.getX();
	Spatial3Robj.setX(position);
	auto q1  = Spatial3Robj.getConfig();
	std::cout << q[0] << " " << q[1] << " " << q[1] << std::endl;
	std::cout << q1[0] << " " << q1[1] << " " << q1[1] << std::endl;
}
