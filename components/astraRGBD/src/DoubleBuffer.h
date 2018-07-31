//
// Created by robolab on 24/07/18.
//

#ifndef PROJECT_DOUBLEBUFFER_H
#define PROJECT_DOUBLEBUFFER_H
#include <mutex>

template <class I, class O> class DoubleBuffer
{
    std::mutex bufferMutex;
    O bufferA, bufferB;
    O &readBuffer = bufferA;
    O &writeBuffer = bufferB;
    //, *readBuffer, bufferB;

public:
    std::size_t size=0;
    DoubleBuffer()
    {
        init(640*480);
    };

    void init(std::size_t v_size)
    {
        bufferA.resize(v_size);
        bufferB.resize(v_size);
    }

    void resize(std::size_t size_)
    {
        if (size_!= size)
        {
            bufferA.resize(size_);
            writeBuffer = bufferA;
            bufferB.resize(size_);
            readBuffer = bufferB;
            size = size_;
        }
    }
    void swap()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        writeBuffer.swap(readBuffer);
    }

    inline typename O::value_type& operator[](int i) { return (writeBuffer)[i]; };

    O get()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return readBuffer;
    }

    O& getNextPtr()
    {
        return writeBuffer;
    }

    void put_memcpy(const I &d, std::size_t data_size) {
        if (d.is_valid())
        {

//            this->resize(d.width() * d.height()*data_size);
            memcpy(&writeBuffer[0], d.data(), d.width()*d.height()*data_size);
//            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            swap();
        }
    }

    void put_stdcopy(const I &d, std::size_t data_size) {
        if (d.is_valid())
        {
            this->resize(d.width() * d.height()*data_size);
            std::copy(d.data(), &d.data()[0]+(500), begin(*writeBuffer));
        }
    }
};

#endif //PROJECT_DOUBLEBUFFER_H


