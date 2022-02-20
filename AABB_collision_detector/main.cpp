#include "AABB_Collision_Detector.h"
#include <iostream>
#include <ctime>
int main(int argc, char** argv) {
	std::vector<scu::Bound> bounds;
	double dimiss = 0.001;
	for (int i = 0; i < 1000000; i++) {
		bounds.push_back(scu::Bound(scu::Vertex(0.0 + dimiss * i, 0.0 + dimiss * i, 0.0), scu::Vertex(1.0 + dimiss * i, 1.0 + dimiss * i, 1.0)));
	}
	scu::Bound testBound = scu::Bound(scu::Vertex(0.5, 0.5, 0.0), scu::Vertex(0.9, 0.9, 0.9));
	long t0s = clock();
	scu::AABB_Collision_Detector detector(bounds); std::vector<unsigned int> output;
	long t0e = clock();
	std::cout << "Construct cost: " << t0e - t0s << " ms." << std::endl;
	long t1s = clock();
	for (int i = 0; i < 1000; i++)
		detector.Detect(testBound, output);
	long t1e = clock();
	std::cout << "detect cost:    " << t1e - t1s << " ms." << std::endl;
	std::cout << "detect :        " << output.size() << " objects." << std::endl;
	long t2s = clock();
	for (int j = 0; j < 1000; j++) {
		output.clear();
		for (int i = 0; i < bounds.size(); i++) {
			if (!testBound.DisJoint(bounds[i]))
				output.push_back(i);
		}
	}
	long t2e = clock();
	std::cout << "gungun cost:    " << t2e - t2s << " ms." << std::endl;
	std::cout << "detect :        " << output.size() << " objects." << std::endl;
	return 0;
}