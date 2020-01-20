#include <assert.h>

#ifndef MAYBE_HPP
#define MAYBE_HPP

/* Simple implementation of a Maybe/Option type (or monad, if you're into that sort of thing).
 * Allows the safe passing of a value (or lack of a value!) without the need for null values.
 *
 * For example, Maybe<double> stores a double value OR the lack of such value.
 *
 * To use this class, create a Maybe<Type> via Maybe<Type> example_maybe(Type value)
 * To assign a new value, use example_maybe = Type value
 * To check if a Maybe contains a value, use example_maybe.Valid()
 * To get the stored value, use example_maybe.Get() or example_maybe.GetImmutable()
 * Note that both getter functions assert that a valid value is containted!
 *
 */
namespace Maybe
{
    template <typename T>
    class Maybe
    {
    protected:

        bool maybe_;
        T value_;

    public:

        Maybe() : maybe_(false) {}

        Maybe(const T& value) : maybe_(true), value_(value) {}

        Maybe(T&& value) : maybe_(true), value_(value) {}

        bool Valid() const
        {
            return maybe_;
        }

        T& Get()
        {
            assert(maybe_);
            return value_;
        }

        const T& GetImmutable() const
        {
            assert(maybe_);
            return value_;
        }

        void Set(const T& value)
        {
            maybe_ = true;
            value_ = value;
        }

        void Set(T&& value)
        {
            maybe_ = true;
            value_ = value;
        }

        Maybe& operator=(const T& value)
        {
            maybe_ = true;
            value_ = value;
            return *this;
        }

        Maybe& operator=(T&& value)
        {
            maybe_ = true;
            value_ = value;
            return *this;
        }
    };
}

#endif // MAYBE_HPP
