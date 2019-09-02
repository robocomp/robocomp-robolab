#ifndef TSQHASH_H
#define TSQHASH_H
#include <QHash>
#include <QMutex>

template <typename Key, typename T>
class TSQHash: public QHash<Key, T>
{
	
private:
	QMutex mutex;
public:
// 	TSQHash();
// 	~TSQHash();
// 	void clear();
	
	TSQHash():QHash<Key, T>()
	{
		
	}

	~TSQHash()
	{
		
	}

	void clear()
	{
		mutex.lock();
			QHash<Key,T>::clear();
		mutex.unlock();
	}
	
	int remove(const Key key)
	{
		mutex.lock();
			int value = QHash<Key, T>::remove(key);
		mutex.unlock();
		return value;
	}
	
	T &operator[](const Key &key)
	{
		//mutex.lock();
			T& tValue = QHash<Key, T>::operator[](key);
		//mutex.unlock();
		return tValue;
	}
	
	const T operator[](const Key &key) const
	{
		//FIXME: implementar
	}
	
	QList<T> values()
	{
		QList<T> res;
		mutex.lock();
			res = QHash<Key, T>::values();
		mutex.unlock();
		return res;
	}
};

#endif