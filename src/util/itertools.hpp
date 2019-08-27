#ifndef QUADMESH_SRC_UTIL_ITERTOOLS_HPP_
#define QUADMESH_SRC_UTIL_ITERTOOLS_HPP_

#include <functional>
#include <iostream>
#include <iterator>
#include <optional>
#include <utility>

template <class InputIt1, class InputIt2>
struct RangePrint {
    RangePrint(InputIt1 first, InputIt2 last)
        : first_(first)
        , last_(last)
    {
    }

    InputIt1 first() const
    {
        return first_;
    }

    InputIt2 last() const
    {
        return last_;
    }

private:
    InputIt1 first_;
    InputIt2 last_;
};

template <class InputIt1, class InputIt2>
std::ostream& operator<<(std::ostream& os,
                         const RangePrint<InputIt1, InputIt2>& obj)
{
    os << "[";
    for (auto i = obj.first(); i != obj.last(); ++i) {
        os << (*i) << ((std::next(i) != obj.last()) ? ", " : "");
    }
    return os << "]";
}

template <class InputIt, class FunctionType>
class ApplyIt {
public:
    using difference_type =
        typename std::iterator_traits<InputIt>::difference_type;
    using arg_type = typename std::iterator_traits<InputIt>::reference;
    using result_type = typename std::result_of<FunctionType(arg_type)>::type;
    using value_type = const result_type;
    using pointer = value_type*;
    using reference = result_type&;
    using iterator_category =
        typename std::iterator_traits<InputIt>::iterator_category;

    ApplyIt(InputIt&& iter, FunctionType&& func)
        : iter_(std::move(iter))
        , func_(std::move(func))
    {
    }

    reference operator*() const
    {
        if (!value_.has_value()) {
            value_ = func_(*iter_);
        }
        return value_.value();
    }

    pointer operator->() const
    {
        return &(this->operator*());
    }

    ApplyIt& operator++()
    {
        ++iter_;
        value_ = nullopt;
        return *this;
    }

    ApplyIt operator++(int)
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    friend bool operator==(const ApplyIt& lhs, const ApplyIt& rhs);
    friend bool operator!=(const ApplyIt& lhs, const ApplyIt& rhs);
    friend bool operator==(const ApplyIt& lhs, const InputIt& rhs);
    friend bool operator==(const InputIt& lhs, const ApplyIt& rhs);
    friend bool operator!=(const ApplyIt& lhs, const InputIt& rhs);
    friend bool operator!=(const InputIt& lhs, const ApplyIt& rhs);

private:
    InputIt iter_;
    FunctionType func_;
    std::optional<result_type> value_;
};

template <class InputIt, class ResultType>
bool operator==(const ApplyIt<InputIt, ResultType>& lhs,
                const ApplyIt<InputIt, ResultType>& rhs)
{
    return lhs.iter_ == rhs.iter_;
}

template <class InputIt, class ResultType>
bool operator!=(const ApplyIt<InputIt, ResultType>& lhs,
                const ApplyIt<InputIt, ResultType>& rhs)
{
    return lhs.iter_ != rhs.iter_;
}

template <class InputIt, class ResultType>
bool operator==(const ApplyIt<InputIt, ResultType>& lhs, const InputIt& rhs)
{
    return lhs.iter_ == rhs;
}

template <class InputIt, class ResultType>
bool operator==(const InputIt& lhs, const ApplyIt<InputIt, ResultType>& rhs)
{
    return lhs == rhs.iter_;
}

template <class InputIt, class ResultType>
bool operator!=(const ApplyIt<InputIt, ResultType>& lhs, const InputIt& rhs)
{
    return lhs.iter_ != rhs;
}

template <class InputIt, class ResultType>
bool operator!=(const InputIt& lhs, const ApplyIt<InputIt, ResultType>& rhs)
{
    return lhs != rhs.iter_;
}

template <class BidirIt>
struct Cycle {
    using difference_type =
        typename std::iterator_traits<BidirIt>::difference_type;
    using value_type = typename std::iterator_traits<BidirIt>::value_type;
    using pointer = typename std::iterator_traits<BidirIt>::pointer;
    using reference = typename std::iterator_traits<BidirIt>::reference;
    using iterator_category =
        typename std::iterator_traits<BidirIt>::iterator_category;

    Cycle(BidirIt first, BidirIt last)
        : first_(first)
        , last_(last)
        , cur_(first)
    {
    }

    reference operator*() const
    {
        return cur_.operator*();
    }

    pointer operator->() const
    {
        return cur_.operator->();
    }

    const BidirIt& iter() const
    {
        return cur_;
    }

    Cycle& set_iter(BidirIt new_iter)
    {
        cur_ = new_iter;
        return *this;
    }

    const BidirIt& first() const
    {
        return first_;
    }

    const BidirIt& last() const
    {
        return last_;
    }

    Cycle& operator++()
    {
        auto n = std::next(cur_);
        cur_ = (n != last_) ? n : first_;
        return *this;
    }

    Cycle operator++(int)
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    Cycle& operator--()
    {
        cur_ = std::prev((cur_ != first_) ? cur_ : last_);
        return *this;
    }

    Cycle operator--(int)
    {
        auto result = *this;
        --(*this);
        return result;
    }

private:
    BidirIt first_;
    BidirIt last_;
    BidirIt cur_;
};

#endif // QUADMESH_SRC_UTIL_ITERTOOLS_HPP_
