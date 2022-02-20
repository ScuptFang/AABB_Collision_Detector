#include "AABB_Collision_Detector.h"

namespace scu {
	bool AABB_Collision_Detector::Construct(std::vector<Bound> AABBs, unsigned int dimension)
	{
		_num_element = AABBs.size();
		size_t num2 = _num_element * 2;
		if (num2 >= std::numeric_limits<unsigned int>::max()) return false;
		_dimension = dimension;
		_events.resize(_dimension); _events_dimension_index.resize(_dimension);
		for (unsigned int i = 0; i < _dimension; i++) {
			_events[i].reserve(num2);
			_events_dimension_index[i].resize(num2, 0);
		}

		for (unsigned int i = 0; i < _num_element; i++) {
			for (unsigned int j = 0; j < _dimension; j++) {
				_maxDim[j] = std::max(AABBs[i]._dim[j], _maxDim[j]);
				double& v0 = AABBs[i]._min_bound_vertex[j];
				double& v1 = AABBs[i]._max_bound_vertex[j];
				_events[j].push_back(Event(i, v0, EventType::START));
				_events[j].push_back(Event(i, v1, EventType::END));
			}
		}

		unsigned int count = 0;
		for (unsigned int i = 0; i < _dimension; i++) {
			std::sort(_events[i].begin(), _events[i].end()); count = 0;
			for (const auto& id : _events[i]) {
				if (id._type == EventType::START)
					_events_dimension_index[i][id.origin_index] = count++;
				else if(id._type == EventType::END)
					_events_dimension_index[i][id.origin_index + _num_element] = count++;
			}
		}
		default_searching_index.reserve(count);
		searching_index.reserve(count);
		jointAxis.reserve(_num_element);
		for (unsigned int i = 0; i < count; i++) {
			default_searching_index.push_back(i);
		}
		return true;
	}
	bool AABB_Collision_Detector::Clear()
	{
		_dimension = 0;
		_num_element = 0;
		_events.clear();
		_events_dimension_index.clear();
		default_searching_index.clear();
		searching_index.clear();
		jointAxis.clear();
		return false;
	}
	bool AABB_Collision_Detector::Detect(Bound bound, std::vector<unsigned int>& collisionBoundsIndex)
	{
		searching_index = default_searching_index;
		for (unsigned int i = 0, j = 1; i < _dimension; i = j++) {
			jointAxis.clear();
			auto& es = _events[i];
			double& v0 = bound._min_bound_vertex[i];
			double& v1 = bound._max_bound_vertex[i];
			double& dim = bound._dim[i]; double& maxDim = _maxDim[i];
			double extend = maxDim - dim;
			Event ses(-1, v0, EventType::START);
			Event see(-1, v1, EventType::END);
			Event sesex(-1, v0 - extend, EventType::START);
			Event seeex(-1, v1 + extend, EventType::END);
			
			//searching is still slow.
			auto index = tryInsertEvent(sesex, i, searching_index);
			if (index == std::numeric_limits<unsigned int>::max())
				return false;
			for (; index < searching_index.size(); index++) {
				auto& currentIndex = searching_index[index];
				auto& currentEvent = es[currentIndex];
				auto& anotherIndex = currentEvent._type == EventType::END ? _events_dimension_index[i][currentEvent.origin_index] : _events_dimension_index[i][currentEvent.origin_index + _num_element];
				auto& anotherEvent = es[anotherIndex];
				if (currentEvent > seeex) break;
				else if (anotherEvent >= sesex && currentEvent <= seeex && anotherEvent._type == EventType::START) continue;
				else if (!(currentEvent >= see || currentEvent <= ses)) 
					jointAxis.push_back(currentEvent.origin_index);
				else if (currentEvent <= ses && anotherEvent >= see && anotherEvent <= seeex) 
					jointAxis.push_back(currentEvent.origin_index);
			}

			if (jointAxis.empty()) return false;

			if (j == _dimension) {
				jointAxis.swap(collisionBoundsIndex);
				break;
			}
			searching_index.clear();
			for (unsigned int k = 0; k < jointAxis.size(); k++) {
				searching_index.push_back(_events_dimension_index[j][jointAxis[k]]);
				searching_index.push_back(_events_dimension_index[j][jointAxis[k] + _num_element]);
			}
			std::sort(searching_index.begin(), searching_index.end());
		}
		return !collisionBoundsIndex.empty();
	}
	unsigned int AABB_Collision_Detector::tryInsertEvent(Event& e, unsigned int _axis, std::vector<unsigned int>& searchingList)
	{
		if (searchingList.empty() || _axis >= _dimension) return std::numeric_limits<unsigned int>::max();
		unsigned int length = (unsigned int)searchingList.size() - 1;
		auto& es = _events[_axis];

		unsigned int step = length >> 1;
		unsigned int current = step;
		bool complete = false;
		while (true) {
			bool isLeft = e < es[searchingList[current]];
			if (current == 0 || es[searchingList[current - 1]] <= e)
				break;
			step >>= 1;
			if (step < 1) step = 1;
			current = isLeft ? current - step : current + step;
		}
		return current;
	}
}