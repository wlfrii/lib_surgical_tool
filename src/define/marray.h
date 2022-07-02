#ifndef LIB_SURGICAL_TOOL_M_ARRAY_H_LF
#define LIB_SURGICAL_TOOL_M_ARRAY_H_LF
#include <stdint.h>
#include <initializer_list>

/**
 * @brief A template class store some objects with same type T,
 * and the objects can be get one by one by get() interface.
 * 
 * @tparam T Typename.
 * @tparam N The number of the objects.
 */
template<typename T, uint8_t N>
struct mArray
{
	mArray(std::initializer_list<T> ilist)
		: _index(0)
	{
		_size = 0;
		for (auto p = ilist.begin(); p != ilist.end(); p++){
			_elements[_size++] = *p;
			if (_size >= N) break;
		}
	}

	const T& get()
	{
		if (_index >= _size){
			_index = 0;
		}
		return _elements[_index++];
	}
private:
	uint8_t _index;
	uint8_t _size;
	T 		_elements[N];
};

#endif // LIB_SURGICAL_TOOL_M_ARRAY_H_LF
