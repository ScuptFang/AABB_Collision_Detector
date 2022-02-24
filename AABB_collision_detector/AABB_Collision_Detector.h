#pragma once
#include "Common.h"
#include <vector>

namespace scu{
	class Mesh {
	public:
		std::vector<Vertex> _vertexs;
		std::vector<Face> _faces;
		std::vector<Edge> _edges;

		Bound _bound;
	};

	class AABB_Collision_Detector {
	public:
		AABB_Collision_Detector(std::vector<Bound> AABBs, unsigned int dimension = 3) {
			Construct(AABBs, dimension);
		}
		~AABB_Collision_Detector() { Clear(); }
		bool Construct(std::vector<Bound> AABBs, unsigned int dimension = 3);
		bool Clear();

		bool Detect(Bound bound, std::vector<unsigned int>& collisionBoundsIndex);
	private:
		enum class EventType { START, END };
		struct Event {

			unsigned int origin_index;
			double splint;
			EventType _type;

			Event():origin_index(-1), splint(0.0), _type(EventType::START) {}
			Event(unsigned int id, double v, EventType t):origin_index(id), splint(v), _type(t){}

			bool operator < (Event& e) {
				if (splint == e.splint) {
					return _type < e._type;
				}
				else
					return splint < e.splint;
			}
			bool operator < (const Event& e) const {
				if (splint == e.splint) {
					return _type < e._type;
				}
				else
					return splint < e.splint;
			}
			bool operator == (Event& e) {
				return _type == e._type && splint == e.splint;
			}
			bool operator == (const Event& e) const{
				return _type == e._type && splint == e.splint;
			}
			bool operator > (Event& e) {
				if (splint == e.splint) {
					return _type > e._type;
				}
				else
					return splint > e.splint;
			}
			bool operator > (const Event& e) const {
				if (splint == e.splint) {
					return _type > e._type;
				}
				else
					return splint > e.splint;
			}
			bool operator >= (Event& e) {
				return *this > e || *this == e;
			}
			bool operator >= (const Event& e) const  {
				return *this > e || *this == e;
			}
			bool operator <= (Event& e) {
				return *this < e || *this == e;
			}
			bool operator <= (const Event& e) const {
				return *this < e || *this == e;
			}
		};

		unsigned int tryInsertEvent(Event& e, unsigned int _axis, std::vector<unsigned int>& searchingList);

		unsigned int _dimension;
		size_t _num_element;
		std::vector<std::vector<Event>> _events;
		std::vector<std::vector<unsigned int>> _events_dimension_index;
		Vertex _maxDim, _minDim;

		std::vector<unsigned int> default_searching_index, searching_index, jointAxis;
	};
}