#ifndef QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_
#define QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_

#include <iterator>
#include <type_traits>
#include <variant>
#include <vector>

namespace qmsh {

template <class T>
class HoledVector {
    using VariantType = std::variant<T, size_t>;
    using BaseVector = std::vector<VariantType>;

public:
    using value_type = T;
    using reference = std::add_lvalue_reference_t<value_type>;
    using const_reference
        = std::add_lvalue_reference_t<std::add_const_t<value_type>>;
    using pointer = std::add_pointer_t<value_type>;
    using const_pointer = std::add_pointer_t<std::add_const_t<value_type>>;
    using size_type = typename BaseVector::size_type;

    static constexpr size_type npos = size_type(-1);

    template <class BaseContainerPtr, class BaseIterator>
    class ViewIterator {
        friend HoledVector;
        using traits_type = typename std::iterator_traits<BaseIterator>;
        using base_value_type
            = std::variant_alternative_t<0, typename BaseIterator::value_type>;

    public:
        using difference_type = typename traits_type::difference_type;
        using value_type = std::pair<base_value_type, difference_type>;
        using pointer = std::add_pointer_t<value_type>;
        using reference = std::add_lvalue_reference_t<value_type>;
        using iterator_tag = std::forward_iterator_tag;

        ViewIterator(const ViewIterator&) = default;
        ViewIterator(ViewIterator&&) noexcept = default;
        ViewIterator& operator=(const ViewIterator&) = default;
        ViewIterator& operator=(ViewIterator&&) noexcept = default;

        reference operator*() const noexcept
        {
            return value_;
        }

        pointer operator->() const noexcept
        {
            return &value_;
        }

        ViewIterator& operator++() noexcept
        {
            ++iter_;
            get();
            return *this;
        }

        ViewIterator operator++(int) noexcept
        {
            auto copy = *this;
            (*this)++;
            return copy;
        }

        bool operator==(const ViewIterator& rhs) const noexcept
        {
            return iter_ == rhs.iter_;
        }

        bool operator!=(const ViewIterator& rhs) const noexcept
        {
            return iter_ != rhs.iter_;
        }

    private:
        ViewIterator(BaseContainerPtr base, BaseIterator iter)
            : base_(base)
            , iter_(iter)
            , value_(T(), 0)
        {
            get();
        }

        void get()
        {
            for (; iter_ != base_->end() && iter_->index() != 0; ++iter_)
                ;
            if (iter_ != base_->end())
                value_ = value_type(std::get<0>(*iter_),
                                    std::distance(base_->begin(), iter_));
        }

        BaseContainerPtr base_;
        BaseIterator iter_;
        mutable value_type value_;
    };

    template <class Iterator>
    class HoledVectorView {
        friend HoledVector;

    public:
        using iterator = Iterator;

        Iterator begin()
        {
            return begin_;
        }
        Iterator end()
        {
            return end_;
        }

    private:
        HoledVectorView(Iterator begin, Iterator end)
            : begin_(begin)
            , end_(end)
        {
        }

        Iterator begin_;
        Iterator end_;
    };

    using view_type = HoledVectorView<
        ViewIterator<BaseVector*, typename BaseVector::iterator>>;
    using const_view_type = HoledVectorView<
        ViewIterator<const BaseVector*, typename BaseVector::const_iterator>>;

    HoledVector()
        : tail_(npos)
        , items_()
    {
    }

    HoledVector(std::initializer_list<value_type> ilist)
        : tail_(npos)
        , items_(std::begin(ilist), std::end(ilist))
    {
    }

    HoledVector(const HoledVector&) = default;
    HoledVector(HoledVector&&) noexcept = default;
    HoledVector& operator=(const HoledVector&) = default;
    HoledVector& operator=(HoledVector&&) noexcept = default;

    reference operator[](size_type index) noexcept
    {
        return *get(index);
    }

    const_reference operator[](size_type index) const noexcept
    {
        return *get(index);
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

    void reserve(typename BaseVector::size_type size)
    {
        items_.reserve(size);
    }

    void shrink_to_fit()
    {
        items_.shrink_to_fit();
    }

    void clear() noexcept
    {
        items_.clear();
        tail_ = npos;
    }

    size_type filled_size() const noexcept
    {
        size_type sum = 0;
        for (auto i = tail_; i != npos; i = std::get<1>(items_[i])) {
            ++sum;
        }
        return items_.size() - sum;
    }

    size_type back_index() const noexcept
    {
        auto i = items_.size() - 1;
        for (; i != npos && items_[i].index() != 0; --i)
            ;
        return i;
    }

    size_type front_index() const noexcept
    {
        auto i = 0;
        for (; i != items_.size() && items_[i].index() != 0; ++i)
            ;
        return i;
    }

    size_type size() const noexcept
    {
        return items_.size();
    }

    template <class... Args>
    size_type emplace(Args&&... args)
    {
        size_type result = npos;
        if (tail_ == npos) {
            result = items_.size();
            items_.emplace_back(std::in_place_index_t<0>(),
                                std::forward<Args>(args)...);
        } else {
            result = tail_;
            tail_ = std::get<1>(items_[tail_]);
            items_[result].template emplace<0>(std::forward<Args>(args)...);
        }
        return result;
    }

    size_type insert(value_type value)
    {
        return emplace(std::move(value));
    }

    void erase(size_type index)
    {
        items_[index].template emplace<size_type>(tail_);
        tail_ = index;
    }

    view_type view() noexcept
    {
        return view_type(
            typename view_type::iterator(&items_, std::begin(items_)),
            typename view_type::iterator(&items_, std::end(items_)));
    }

    const_view_type view() const noexcept
    {
        return const_view_type(
            typename const_view_type::iterator(&items_, std::begin(items_)),
            typename const_view_type::iterator(&items_, std::end(items_)));
    }

private:
    size_type tail_ = npos;
    BaseVector items_;
};

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_HOLED_VECTOR_HPP_
