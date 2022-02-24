#pragma once
#include <algorithm>
#include <iostream>

namespace scu {
	template<class T, unsigned int L = 1> class _Vector {
	public:
		_Vector<T, L>() {
			for (int i = 0; i < L; i++)
				_data[i] = (T)0;
		}

		_Vector<T, L>(T x, T y) {
			_data[0] = x;
			_data[1] = y;
		}

		_Vector<T, L>(T x, T y, T z) {
			_data[0] = x;
			_data[1] = y;
			_data[2] = z;
		}

		_Vector<T, L>(T x, T y, T z, T w) {
			_data[0] = x;
			_data[1] = y;
			_data[2] = z;
			_data[3] = w;
		}


		_Vector<T, L>& operator =(_Vector<T, L>& e) {
			for (int i = 0; i < L; i++)
				_data[i] = e[i];
			return *this;
		}
		_Vector<T, L>& operator =(const _Vector<T, L>& e){
			for (int i = 0; i < L; i++)
				_data[i] = e[i];
			return *this;
		}
		_Vector<T, L> operator -(_Vector<T, L>& e) {
			_Vector<T, L> r;
			for (int i = 0; i < L; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		_Vector<T, L> operator -(const _Vector<T, L>& e) const {
			_Vector<T, L> r;
			for (int i = 0; i < L; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		_Vector<T, L> operator +(_Vector<T, L>& e) {
			_Vector<T, L> r;
			for (int i = 0; i < L; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		_Vector<T, L> operator +(const _Vector<T, L>& e) const{
			_Vector<T, L> r;
			for (int i = 0; i < L; i++)
				r[i] = _data[i] - e[i];
			return r;
		}

		T& operator[] (const unsigned int& index) {
			return _data[index];
		}
		T& operator[] (const int& index) {
			return _data[index];
		}
		const T& operator[] (const unsigned int& index) const {
			return _data[index];
		}
		const T& operator[] (const int& index) const {
			return _data[index];
		}

		friend std::ostream& operator<< (std::ostream& out, _Vector<T, L>& e) {
			out << "[ ";
			for (int i = 0; i < L; i++) {
				out << e[i] << " ";
			}
			out << "]" << std::endl;
			return out;
		}
		friend std::istream& operator>> (std::istream& in, _Vector<T, L>& e) {
			for (int i = 0; i < L; i++) 
				in >> e[i];
			return in;
		}

		T magnitude2() {
			T sum = (T)0;
			for (int i = 0; i < L; i++)
				sum += (_data[i] * _data[i]);
			return sum;
		}

		T magnitude2() const {
			T sum = (T)0;
			for (int i = 0; i < L; i++)
				sum += (_data[i] * _data[i]);
			return sum;
		}

		double magnitude() {
			double value = (double)magnitude2();
			return sqrt(value);
		}

		double magnitude() const {
			double value = (double)magnitude2();
			return sqrt(value);
		}

		T dot(_Vector<T, L>& e) {
			T sum = (T)0;
			for (int i = 0; i < L; i++) 
				sum += (_data[i] * e[i]);
			return sum;
		}

		T dot(const _Vector<T, L>& e) const{
			T sum = (T)0;
			for (int i = 0; i < L; i++)
				sum += (_data[i] * e[i]);
			return sum;
		}

		_Vector<T, 3> cross(_Vector<T, 3>& e) {
			_Vector<T, 3> c;
			c[0] = _data[1] * e[2] - _data[2] * e[1];
			c[1] = _data[2] * e[0] - _data[0] * e[2];
			c[2] = _data[0] * e[1] - _data[1] * e[0];
			return c;
		}

		_Vector<T, 3> cross(const _Vector<T, 3>& e) const {
			_Vector<T, 3> c;
			c[0] = _data[1] * e[2] - _data[2] * e[1];
			c[1] = _data[2] * e[0] - _data[0] * e[2];
			c[2] = _data[0] * e[1] - _data[1] * e[0];
			return c;
		}

		_Vector<double, L> normalize() {
			_Vector<double, L> n;
			double t = (double)magnitude();
			if (t <= std::numeric_limits<double>::min()) return n;
			t = 1.0 / t;
			for (int i = 0; i < L; i++) 
				n[i] = (double)_data[i] * t;
			return n;
		}

		_Vector<double, L> normalize() const{
			_Vector<double, L> n;
			double t = (double)magnitude();
			if (t <= std::numeric_limits<double>::min()) return n;
			t = 1.0 / t;
			for (int i = 0; i < L; i++)
				n[i] = (double)_data[i] * t;
			return n;
		}

		bool isNormalized() {
			return magnitude() - 1.0 <= std::numeric_limits<double>::min();
		}

		bool isNormalized() const {
			return magnitude() - 1.0 <= std::numeric_limits<double>::min();
		}

	protected:
		T _data[L];
	};

	typedef _Vector<double, 2> Vector2;
	typedef _Vector<double, 3> Vector3;

	class Vertex : public Vector3 {
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
		Vector3& operator =(Vertex& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}

		Vector3& operator =(const Vertex& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}

		Vector3 operator -(Vertex& e) {
			Vector3 r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		Vector3 operator -(const Vertex& e) const {
			Vector3 r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		Vector3 operator +(Vertex& e) {
			Vector3 r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
		Vector3 operator +(const Vertex& e) const {
			Vector3 r;
			for (int i = 0; i < 3; i++)
				r[i] = _data[i] - e[i];
			return r;
		}
	};

	class Face {
	public:
		Face() {
			for (int i = 0; i < 3; i++)
				_data[i] = 0;
		}

		Face(unsigned int v0, unsigned int v1, unsigned int v2) {
			_data[0] = v0;
			_data[1] = v1;
			_data[2] = v2;
		}

		Face& operator =(Face& e) {
			for (int i = 0; i < 3; i++)
				_data[i] = e[i];
			return *this;
		}

		unsigned int& operator[] (const unsigned int& index) {
			return _data[index];
		}

		friend std::ostream& operator<< (std::ostream& out, Face& e) {
			out << "[ ";
			for (int i = 0; i < 3; i++) {
				out << e[i] << " ";
			}
			out << "]" << std::endl;
			return out;
		}
		friend std::istream& operator>> (std::istream& in, Face& e) {
			for (int i = 0; i < 3; i++)
				in >> e[i];
			return in;
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
		Bound() : _min_bound_vertex(Vertex()), _max_bound_vertex(Vertex()) {}
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
		Vector3 _dim;
	};
}