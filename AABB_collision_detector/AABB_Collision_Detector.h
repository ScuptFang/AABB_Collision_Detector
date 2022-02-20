#pragma once
#include <algorithm>
#include <vector>

namespace scu{
	class Vertex {
	public:
		Vertex() {
			for (int i = 0; i < 3; i++) 
				_data[i] = 0.0;
		}

		Vertex(double x, double y, double z) {
			_data[0] = x;
			_data[1] = y;
			_data[2] = z;
		}

		Vertex& operator =(Vertex& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}
		Vertex& operator =(const Vertex& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}
		Vertex operator -(Vertex& e) {
			Vertex r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		Vertex operator -(const Vertex& e) {
			Vertex r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		
		double& operator[] (const unsigned int& index){
			return _data[index];
		}
		double& operator[] (const int& index) {
			return _data[index];
		}
		const double& operator[] (const unsigned int& index) const {
			return _data[index];
		}
		const double& operator[] (const int& index) const {
			return _data[index];
		}
	private:
		double _data[3];
	};

	class Face {
	public:
		Face() {
			for (int i = 0; i < 3; i++)
				_data[i] = 0;
		}
		Face& operator =(Face& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}

		unsigned int& operator[] (const unsigned int& index) {
			return _data[index];
		}
	private:
		unsigned int _data[3];
	};

	class Edge {
		Edge() {
			for (int i = 0; i < 2; i++)
				_data[i] = 0;
		}
		Edge& operator =(Edge& e) {
			for (int i = 0; i < 2; i++)
				_data[i] = e[i];
			return *this;
		}

		unsigned int& operator[] (const unsigned int& index) {
			return _data[index];
		}
	private:
		unsigned int _data[2];
	};

	class Bound {
	public:
		Bound() : _min_bound_vertex(Vertex()), _max_bound_vertex(Vertex()){}
		Bound(const Vertex& minBound, const Vertex& maxBound) : _min_bound_vertex(minBound), _max_bound_vertex(maxBound) {
			_dim = _max_bound_vertex - _min_bound_vertex;
		}

		void operator = (Bound& e) {
			_min_bound_vertex = e._min_bound_vertex;
			_max_bound_vertex = e._max_bound_vertex;
			_dim = e._dim;
		}

		void operator = (Bound e) {
			_min_bound_vertex = e._min_bound_vertex;
			_max_bound_vertex = e._max_bound_vertex;
			_dim = e._dim;
		}

		void AddPoint(const Vertex& v) {
			for (int i = 0; i < 3; i++) {
				_min_bound_vertex[i] = std::min(_min_bound_vertex[i], v[i]);
				_max_bound_vertex[i] = std::max(_max_bound_vertex[i], v[i]);
				_dim[i] = _max_bound_vertex[i] - _min_bound_vertex[i];
			}
		}
		bool DisJoint(const Bound& b) {
			return _max_bound_vertex[0] < b._min_bound_vertex[0] || _max_bound_vertex[1] < b._min_bound_vertex[1] || _max_bound_vertex[2] < b._min_bound_vertex[2] ||
				b._max_bound_vertex[0] < _min_bound_vertex[0] || b._max_bound_vertex[1] < _min_bound_vertex[1] || b._max_bound_vertex[2] < _min_bound_vertex[2];
		}

		Vertex _min_bound_vertex, _max_bound_vertex;
		Vertex _dim;
	};

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

			bool operator < (const Event& e) {
				if (splint == e.splint) {
					return _type < e._type;
				}
				else
					return splint < e.splint;
			}
			bool operator == (const Event& e) {
				return _type == e._type && splint == e.splint;
			}
			bool operator > (const Event& e) {
				if (splint == e.splint) {
					return _type > e._type;
				}
				else
					return splint > e.splint;
			}
			bool operator >= (const Event& e) {
				return *this > e || *this == e;
			}
			bool operator <= (const Event& e) {
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