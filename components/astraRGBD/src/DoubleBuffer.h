//
// Created by robolab on 24/07/18.
//

#ifndef PROJECT_DOUBLEBUFFER_H
#define PROJECT_DOUBLEBUFFER_H
#include <mutex>

template <class I, class O> class Converter
{
    public:
        virtual bool ItoO(const I & iTypeData, O &oTypeData)=0;
        virtual bool OtoI(const O & oTypeData, I &iTypeData)=0;
};

template <class I, class O, class C> class DoubleBuffer
{
    std::mutex bufferMutex;
    O bufferA, bufferB;
    O &readBuffer = bufferA;
    O &writeBuffer = bufferB;
    C *converter;
    //, *readBuffer, bufferB;

public:
    std::size_t size=0;
    DoubleBuffer()
    {
        resize(640*480);
    };

    void init(std::size_t v_size, C &converter)
    {
        resize(v_size);
        this->converter = &converter;
    }

    void init(C& converter)
    {
        converter = &converter;
    }

    void resize(std::size_t size_)
    {
        if (size_!= size)
        {
            bufferA.resize(size_);
            bufferB.resize(size_);
            size = size_;
        }
    }
    void swap()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        writeBuffer.swap(readBuffer);
    }

    inline typename O::value_type& operator[](int i) { return (writeBuffer)[i]; };

    void get(O &oData)
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        oData = readBuffer;
    }

    O& getNextPtr()
    {
        return writeBuffer;
    }

    void put(const I &d, std::size_t data_size) {
        if( converter->ItoO(d,writeBuffer))
        {
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


