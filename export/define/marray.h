#ifndef LIB_INSTRUMENT_M_ARRAY_H_LF
#define LIB_INSTRUMENT_M_ARRAY_H_LF
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


/**
 * @brief A template class store some objects with same type T.
 * The object can be added by add() interface and accessed with 
 * operator[] interface. The size can be get by count() interface.
 * 
 * @tparam T Typename.
 * @tparam N The number of the objects.
 */
template <typename T, uint8_t N>
class ArrayRepo
{
public:
    ArrayRepo()
        : _count(0)
    {}

    bool add(const T& item)
    {
        if (_count >= N){
            return false;
        }
        _elements[_count++] = item;
        return true;
    }

    const T& operator[](uint8_t index) const
    {
        return _elements[index];
    }

    T& operator[](uint8_t index)
    {
        return _elements[index];
    }

    int count() const
    {
        return _count;
    }

protected:
    uint8_t _count;
    T	 	_elements[N];
};


#endif // LIB_INSTRUMENT_M_ARRAY_H_LF