#pragma once
#include "Common.h"
#include <vector>
namespace scu {
	class Triangulation{
	public:
		static void GetIndexFromVertexLoop(const std::vector<Vertex>& loop, std::vector<Face>& faces, const scu::Vector3& normal = scu::Vector3(0.0, 0.0, 0.0));
	};
}