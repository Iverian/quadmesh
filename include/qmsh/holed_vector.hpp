#ifndef QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_
#define QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_

#include <cstddef>
#include <iterator>
#include <type_traits>
#include <variant>
#include <vector>

namespace qmsh {

template <class T>
class HoledVector {
public:
    using value_type = T;
    using reference = std::add_lvalue_reference_t<T>;
    using const_reference = std::add_const_t<reference>;
    using pointer = std::add_pointer_t<T>;
    using const_pointer = std::add_const_t<pointer>;
    using size_type = size_t;

    using BaseVector = std::vector<std::variant<value_type, size_type>>;

    static constexpr size_type npos = size_type(-1);

    template <typename BaseIterator, typename BaseType>
    class Iterator {
    public:
        using difference_type = typename BaseIterator::difference_type;
        using value_type = std::pair<difference_type, BaseType>;
        using pointer = std::add_pointer_t<value_type>;
        using reference = std::add_lvalue_reference_t<value_type>;
        using iterator_tag = std::forward_iterator_tag;

        explicit Iterator(BaseIterator iter, BaseIterator end)
            : value_()
            , iter_(iter)
            , begin_(iter)
            , end_(iter)
        {
        }

        Iterator(BaseIterator iter, BaseIterator begin, BaseIterator end)
            : value_()
            , iter_(iter)
            , begin_(begin)
            , end_(end)
        {
        }

        void get() const noexcept
        {
            while (!std::holds_alternative<value_type>(*iter_)
                   && iter_ != end_) {
                ++iter_;
            }

            value_.first = std::distance(begin_, iter_);
            value_.second = std::get<typename value_type::second_type>(*iter_);
        }

        reference operator*() const noexcept
        {
            get();
            return value_;
        }

        pointer operator->() const noexcept
        {
            get();
            return &value_;
        }

        Iterator& operator++() noexcept
        {
            do {
                ++iter_;
            } while (!std::holds_alternative<value_type>(*iter_)
                     && iter_ != end_);

            return *this;
        }

        Iterator operator++(int) noexcept
        {
            auto copy = *this;
            (*this)++;
            return copy;
        }

        bool operator==(const Iterator& rhs) const noexcept
        {
            return iter_ == rhs.iter_;
        }

        bool operator!=(const Iterator& rhs) const noexcept
        {
            return iter_ != rhs.iter_;
        }

    private:
        mutable value_type value_;
        mutable BaseIterator iter_;
        BaseIterator begin_;
        BaseIterator end_;
    };

    using iterator = Iterator<typename BaseVector::iterator, value_type>;
    using const_iterator = Iterator<typename BaseVector::const_iterator,
                                    std::add_const_t<value_type>>;

    reference operator[](size_type index) noexcept
    {
        return std::get<value_type>(items_[index]);
    }

    const_reference operator[](size_type index) const noexcept
    {
        return std::get<value_type>(items_[index]);
    }

    reference at(size_type index)
    {
        return std::get<value_type>(items_.at(index));
    }

    const_reference at(size_type index) const
    {
        return std::get<value_type>(items_.at(index));
    }

    pointer get(size_type index) noexcept
    {
        return std::get_if<value_type>(&items_[index]);
    }

    const_pointer get(size_type index) const noexcept
    {
        return std::get_if<value_type>(&items_[index]);
    }

    size_type size() const noexcept
    {
        return items_.size();
    }

    iterator begin()
    {
        return iterator(items_.begin(), items_.end());
    }

    iterator end()
    {
        return iterator(items_.end(), items_.begin(), items_.end());
    }

    const_iterator begin() const
    {
        return const_iterator(items_.begin(), items_.end());
    }

    const_iterator end() const
    {
        return const_iterator(items_.end(), items_.begin(), items_.end());
    }

    size_type insert(value_type value)
    {
        size_type result = npos;
        if (tail_ == npos) {
            result = items_.size();
            items_.emplace_back(std::move(value));
        } else {
            result = tail_;
            tail_ = std::get<size_type>(items_[tail_]);
            items_[result].template emplace<value_type>(std::move(value));
        }
        return result;
    }

    void erase(size_type index)
    {
        items_[index].template emplace<size_type>(tail_);
        tail_ = index;
    }

private:
    size_type tail_;
    BaseVector items_;
};

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_
