//
// Created by robolab on 24/07/18.
//

#ifndef PROJECT_DOUBLEBUFFER_H
#define PROJECT_DOUBLEBUFFER_H

template <class T> class DoubleBuffer
{
    QMutex bufferMutex;
    T bufferA, *writer, *reader, bufferB;
public:
    int size;
    DoubleBuffer(){};
    void resize(int size_)
    {
        if (size_!=size) {
            bufferA.resize(size_);
            writer = &bufferA;
            bufferB.resize(size_);
            reader = &bufferB;
            size = size_;
        }
    }
    void swap()
    {
        bufferMutex.lock();
        writer->swap(*reader);
        bufferMutex.unlock();
    }

    inline typename T::value_type& operator[](int i){ return (*writer)[i]; };

    void copy(T &points)
    {
        points.resize(size);
        bufferMutex.lock();
        points = *reader;
        bufferMutex.unlock();
    }
    T* getWriter()
    {
        return writer;
    }
};

#endif //PROJECT_DOUBLEBUFFER_H

