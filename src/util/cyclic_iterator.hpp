#ifndef QUADMESH_SRC_UTIL_CYCLIC_ITERATOR_HPP_
#define QUADMESH_SRC_UTIL_CYCLIC_ITERATOR_HPP_

#include <iterator>
#include <type_traits>

template <
    class BidirIt,
    class = std::enable_if_t<
        std::is_same_v<
            typename std::iterator_traits<BidirIt>::iterator_category,
            std::
                bidirectional_iterator_tag> || std::is_same_v<typename std::iterator_traits<BidirIt>::iterator_category, std::random_access_iterator_tag>>>
struct CyclicIterator {
    using difference_type =
        typename std::iterator_traits<BidirIt>::difference_type;
    using value_type = typename std::iterator_traits<BidirIt>::value_type;
    using pointer = typename std::iterator_traits<BidirIt>::pointer;
    using reference = typename std::iterator_traits<BidirIt>::reference;
    using iterator_category = std::bidirectional_iterator_tag;

    CyclicIterator(BidirIt first, BidirIt last)
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

    CyclicIterator& set_iter(BidirIt new_iter)
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

    CyclicIterator& operator++()
    {
        auto n = std::next(cur_);
        cur_ = (n != last_) ? n : first_;
        return *this;
    }

    CyclicIterator operator++(int)
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    CyclicIterator& operator--()
    {
        cur_ = std::prev((cur_ != first_) ? cur_ : last_);
        return *this;
    }

    CyclicIterator operator--(int)
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

#endif // QUADMESH_SRC_UTIL_CYCLIC_ITERATOR_HPP_
