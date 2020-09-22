#ifndef GADEN_COMMON_OBJECT_POOL_HPP_INCLUDED
#define GADEN_COMMON_OBJECT_POOL_HPP_INCLUDED

#include <tbb/concurrent_queue.h>

namespace gaden {

template <typename T>
class ObjectPool
{
public:
    ObjectPool()
        : available_(std::make_unique<tbb::concurrent_queue<T*>>())
    {}

    ~ObjectPool()
    {
        T* object;
        while (objects_.try_pop(object))
            delete object;
    }

    //template <class... Args>
    //T* pop(Args&&... args)
    T* pop()
    {
        T* object;
        if (available_->try_pop(object))
            return object;

        // no object available --> create a new one
        T* new_object = new T(); // (std::forward<Args>(args)...)
        objects_.push(new_object);
        return new_object;
    }

    void push(T* object)
    {
        available_->push(object);
    }

    void pushAll()
    {
        // Makes all objects available, i.e. available_ = objects_.
        available_ = std::make_unique<tbb::concurrent_queue<T*>>(objects_);
    }

private:
    tbb::concurrent_queue<T*> objects_;
    std::unique_ptr<tbb::concurrent_queue<T*>> available_; // See in pushAll() why we use a unique_ptr.
};

} // namespace gaden

#endif // GADEN_COMMON_OBJECT_POOL_HPP_INCLUDED
