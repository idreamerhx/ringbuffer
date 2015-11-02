// ringbuffer : a fixed (at compile time) sized circular
//              buffer type.
//
// Author: Dalton Woodard
// Contact: daltonmwoodar@gmail.com
// License: Please see LICENSE.md
//


// note:
//      This implementation is NOT threadsafe. Please see
//      https://github.com/daltonwoodard/ringbuffer.git for
//      a thread-safe version implemented with C++ atomic
//      primitives.
//
// note:
//      While this type is lasted by a std::array object,
//      and can therefore exist entirely on the stack, it
//      is meant to be heap allocated to avoid stack size
//      limitations. The use of std::array is to provide
//      locality of the buffered data with respect to the
//      managing ringbuffer object.
//
//  note:
//      If and only if the stored object type T has strong exception safe
//      constructors is the following guaranteed:
//
//      If this object is successfull constructed, then throughout the
//      object's lifetime:
//      
//          1)  all view operations are guaranteed as nothrow/noexcept;
//          2) `read` opeartions provide the basic exception safety guarantee;
//          3) `safe_read` operations provide the strong exception safety
//             guarantee.
//
//     Notice that the read operations modify the data structure (by removing
//     elements from the front); hence they are, in actuallity, writes.
//
//     All of the above are documented as such in source. 

#ifndef RINGBUFFER_HPP
#define RINGBUFFER_HPP

#include <array>       // std::array
#include <type_traits> // std::is_default_constructible
#include <vector>      // std::vector

namespace ringbuffer
{    
namespace detail
{
    template <typename T>
    struct has_reserve
    {
        struct succeeds {};
        struct fails    {};

        template <typename U, void (U::*)(std::size_t)>
        struct sfinae {};

        template <typename U>
        static succeeds test (sfinae<U, &U::reserve>*);

        template <typename U>
        static fails test (...);

        static constexpr bool value =
            std::is_same<decltype(test<T>(0)), succeeds>::value;
    };

    template <typename T>
    struct memblock
    {
        alignas (alignof(T)) unsigned char data[sizeof(T)];
    };
} // namespace detail
 
    template <typename T, std::size_t N>
    class ringbuffer
    {
        static_assert (N > 0, "empty ringbuffer disallowed");
        static_assert (std::is_default_constructible<T>::value,
                       "cannot buffer non-default_constructible objects");
    private:
        using backing_type = std::array<detail::memblock<T>, N>;
        using backing_ptr  = typename backing_type::pointer;

        backing_type buff;

        backing_ptr const first;
        backing_ptr const last;

        backing_ptr writepos;
        backing_ptr readpos;
        std::size_t available;

        template <typename U>
        class rb_iterator :
            public std::iterator<std::random_access_iterator_tag,
                                 typename std::remove_cv<U>::type,
                                 std::ptrdiff_t,
                                 U*,
                                 U&>
        {
        private:
            U* iter;
            U const* first;
            U const* last;

            using piter_type = std::iterator<std::random_access_iterator_tag,
                                             typename std::remove_cv<U>::type,
                                             std::ptrdiff_t,
                                             U*,
                                             U&>;

        public:
            using difference_type   = typename piter_type::difference_type;
            using value_type        = typename piter_type::value_type;
            using pointer           = typename piter_type::pointer;
            using reference         = typename piter_type::reference;
            using iterator_category = typename piter_type::iterator_category;

            rb_iterator (void) = delete;
            rb_iterator (U * p, U const* f, U const* l)
                : iter (p), first (f), last (l)
            {}

            void swap (rb_iterator & other)
            {
                std::swap (iter, other.iter);
                std::swap (first, other.first);
                std::swap (last, other.last);
            }

            rb_iterator<U>& operator++ (void)
            {
                if (iter == last)
                    iter = first;
                else
                    iter += 1;
                return *this;
            }

            rb_iterator<U>& operator-- (void)
            {
                if (iter == first)
                    iter = last;
                else
                    iter -= 1;
                return *this;
            }

            rb_iterator<U> operator++ (int)
            {
                auto tmp (*this);
                if (iter == last)
                    iter = first;
                else
                    iter += 1;
                return tmp;
            }

            rb_iterator<U> operator-- (int)
            {
                auto tmp (*this);
                if (iter == first)
                    iter = last;
                else
                    iter -= 1;
                return tmp;
            }

            rb_iterator<U>& operator+= (difference_type n)
            {
                if (n > N || -n > N) n = n % N;

                if (n >= 0) {
                    if (iter + n > last)
                        iter = first + (n - 1 - (last - iter));
                    else
                        iter = iter + n;
                } else {
                    auto m = -n;
                    if (iter - m < first)
                        iter = last - (m - 1 - (iter - first));
                    else
                        iter = iter - m;
                }

                return *this;
            }

            rb_iterator<U>& operator-= (difference_type n)
            {
                return this->operator+= (-n);
            }

            rb_iterator<U> operator+ (difference_type n) const
            {
                auto tmp = *this;
                return (tmp += n); 
            }

            rb_iterator<U> operator- (difference_type n) const
            {
                auto tmp = *this;
                return (tmp -= n);
            }

            difference_type operator- (rb_iterator<U const> const rhs) const
            {
                return iter - rhs.iter;
            }

            template <template <typename...> class O>
            bool operator== (O<U> const& rhs) const
            {
                return iter == rhs.iter;
            }

            template <template <typename...> class O>
            bool operator!= (O<U> const& rhs) const
            {
                return iter != rhs.iter;
            }

            template <template <typename...> class O>
            bool operator< (O<U> const& rhs) const
            {
                return iter < rhs.iter;
            }

            template <template <typename...> class O>
            bool operator> (O<U> const& rhs) const
            {
                return iter > rhs.iter;
            }

            template <template <typename...> class O>
            bool operator<= (O<U> const& rhs) const
            {
                return iter <= rhs.iter;
            }

            template <template <typename...> class O>
            bool operator>= (O<U> const& rhs) const
            {
                return iter >= rhs.iter;
            }

            U& operator* (void) const
            {
                return *iter;
            }

            U& operator[] (difference_type n)
            {
                if (n > N || -n > N) n = n % N;
                return *(*this + n);
            }

            U& operator-> (void) const
            {
                return *iter;
            }

            operator rb_iterator<U const> (void) const
            {
                return rb_iterator<U const> (iter, first, last);
            }
        };


        // increment pointer through the buffer with modular
        // arithmetic.
        // 
        backing_ptr wrapfront
            (backing_ptr const p, std::size_t n = 1) const noexcept
        {
            if (n > N) n = n % N;

            if (p + n > last) {
                return first + (n - 1 - (last - p));
            } else {
                return p + n;
            }
        }


        // generate a temporary copy of the buffer elements
        //
        std::vector<T> temporary_copy (std::size_t n) const
        {
            // ensure we don't actually read more elements
            // than is possible.
            n = std::min (n, available);

            std::vector<T> buffcopy;
            buffcopy.reserve (n);

            if (readpos + n <= last) {
                auto const buff_begin =
                    std::next (std::begin(buff), readpos - first);
                auto const buff_end = std::next (buff_begin, n);
                buffcopy.insert
                    (std::begin (buffcopy),
                     buff_begin,
                     buff_end);
            } else {
                auto const m = last - readpos;

                auto const first_buff_begin =
                    std::next (std::begin(buff), readpos - first);
                auto const first_buff_end =
                    std::next (first_buff_begin, m + 1);

                auto const second_buff_begin = std::begin (buff);
                auto const second_buff_end =
                    std::next (second_buff_begin, n - m + 1);

                buffcopy.insert
                    (std::begin (buffcopy),
                     first_buff_begin,
                     first_buff_end);
                buffcopy.insert
                    (std::begin (buffcopy),
                     second_buff_begin,
                     second_buff_end);
            }

            return buffcopy;
        }

    public:
        using value_type      = T;
        using size_type       = std::size_t;
        using difference_type = std::ptrdiff_t;
        using reference       = T&;
        using const_reference = T const&;
        using pointer         = T*;
        using const_pointer   = T const*;

        // no modification through iterators; only
        // observatino.
        using iterator        = rb_iterator<T const>;
        using const_iterator  = rb_iterator<T const>;


        // constructor/destructor
        //
        // -- RAII/RRID compliant
        //    (see en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization)
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this constructor either:
        //      succeeds
        //                  --- or ---
        //      propogates an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        ringbuffer (void) noexcept(noexcept(std::begin(buff)))
            : buff      (), 
              first     (buff.data()),
              last      (buff.data() + N - 1),
              writepos  (buff.data()),
              readpos   (buff.data()),
              available (0)
        {}

        ~ringbuffer (void) noexcept (noexcept(~T())) = default;


        // obtain number of buffered objects; i.e., available
        // elements in buffer.
        //
        // -- nothrow
        //
        std::size_t size (void) const noexcept
        {
            return available;
        }


        // obtain the remaining capacity to add objects
        // to buffer.
        //
        // -- nothrow
        //
        std::size_t capacity (void) const noexcept
        {
            return N - available;
        }


        // obtain iterator to the start of the buffer;
        // no modifications are allowed through iterators,
        // so we return a const_iterator.
        //
        // -- nothrow
        //
        const_iterator begin (void) const noexcept
        {
            return cbegin ();
        }


        // obtain iterator to the end of the buffer'
        // no modifications are allowed through iterators,
        // so we return a const_iterator.
        //
        // -- nothrow
        //
        const_iterator end (void) const noexcept
        {
            return cend ();
        }


        // obtain const iterator to the start of the buffer 
        //
        // -- nothrow
        //
        const_iterator cbegin (void) const noexcept
        {
            return const_iterator (readpos, first, last);
        }


        // obtain const iterator to the end of the buffer 
        //
        // -- nothrow
        //
        const_iterator cend (void) const noexcept
        {
            return const_iterator
                (wrapfront (readpos, available),
                 first,
                 last);
        }


        // add object to buffer, if room is available; otherwise
        // do nothing.
        //
        // -- IF AND ONLY IF T's constructors provide the strong
        //    exception safe guarantee does the following hold:
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      propogates an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        template <typename ... Args>
        void emplace (Args && ... args) noexcept(noexcept(T(args...)))
        {
            if (available < N) {
                new (&writepos->data[0]) T(args...);
                writepos   = wrapfront (writepos);
                available += 1;
            }
        }


        // remove first element from buffer, if it exists;
        // otherwise do nothing.
        //
        // IF AND ONLY IF T's destructor is nothrow
        // (which it should be) does the following hold:
        //
        // -- nothrow
        //
        // otherwise, if T's destructor may throw an exception
        // catchable as std::exception:
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      propogates an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        // otherwise no guarantees are made whatsoever.
        //
        void pop (void) noexcept (noexcept(~T()))
        {
            if (available) {
                reinterpret_cast<T*>(&readpos->data[0])->~T();
                readpos    = wrapfront (readpos);
                available -= 1;
            }
        }


        // access first element of the ringbuffer without
        // removal, if it exists; otherwise throw
        // std::out_of_range exception.
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        T front (void)
        {
            if (available) {
                return *reinterpret_cast<T*>(&readpos->data[0]);
            } else {
                 throw std::out_of_range
                     ("cannot access first of empty ringbuffer");           
            }
        }


        // read first element of ringbuffer with
        // removal, if it exists; otherwise throw
        // std::out_of_range exception.
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        T read (void)
        {
            if (available) {
                auto const ptr = reinterpret_cast<T*>(&readpos->data[0]);
                auto const val = *ptr;
                readpos    = wrapfront (readpos);
                available -= 1;
                ptr->~T();
                return val;
            } else {
                throw std::out_of_range ("invalid read on empty ringbuffer");
            }
        }


        // read first n elements of ringbuffer with
        // removal, if they exist, directly into memory
        // pointed to by dst; otherwise throw
        // std::out_of_range exception.
        //
        // Provided dst points to a valid memory location
        // wide enough to accomodate the requested data:
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        void read (std::size_t n, T * dst)
        {
            if (n <= available) {
                auto cap = available - n;
                while (available > cap) {
                    auto const ptr = reinterpret_cast<T*>(&readpos->data);
                    *dst++     = *ptr;
                    readpos    = wrapfront (readpos);
                    available -= 1;
                    ptr->~T();
                }
            } else {
                throw std::out_of_range
                    ("cannot read more than availalbe elements");
            }
        }
 

        // read n elements from the ringbuffer, if there are sufficiently many
        // available elements to access; otherwise throw std::out_of_range
        // exception.
        //
        // note:
        //      uses OutputContainer::reserve, if this method exists; otherwise
        //      default to the next version of read without this usage.
        //
        // -- basic exception guarantee:
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      the state of the ringbuffer object is modified according
        //      to the current progress of read up to the failure point.
        //
        template <template <typename...> class OutputContainer = std::vector,
            typename = typename std::enable_if
                <detail::has_reserve<OutputContainer<T>>::value>::type>
        OutputContainer<T> read (std::size_t n)
        {
            if (n <= available) {
                OutputContainer<T> result;
                result.reserve (n);
                auto const cap = available - n;
                while (available > cap) {
                    auto const ptr = reinterpret_cast<T*>(&readpos->data);
                    auto const val = *ptr;
                    result.emplace_back (std::move(val));
                    readpos    = wrapfront (readpos);
                    available -= 1;
                    ptr->~T();
                }
                return result;
            } else {
                throw std::out_of_range
                    ("cannot read more than available elements");
            }
        }


        // read n elements from the ringbuffer, if there are sufficiently many
        // available elements to access; otherwise throw std::out_of_range
        // exception.
        //
        // -- basic exception guarantee:
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      the state of the ringbuffer object is modified according
        //      to the current progress of read up to the failure point.
        //
        template <template <typename...> class OutputContainer = std::vector,
            typename = typename std::enable_if
                <not detail::has_reserve<OutputContainer<T>>::value>::type,
            bool /*unused, avoids template redeclaration*/ = bool{}>
        OutputContainer<T> read (std::size_t n)
        {
            if (n <= available) {
                OutputContainer<T> result;
                auto const cap = available - n;
                while (available > cap) {
                    auto const ptr = reinterpret_cast<T*>(&readpos->data);
                    auto const val = *ptr;
                    result.emplace_back (std::move(val));
                    readpos    = wrapfront (readpos);
                    available -= 1;
                    ptr->~T();
                }
                return result;
            } else {
                throw std::out_of_range
                    ("cannot read more than available elements");
            }
        }


        // safely read n elements from the ringbuffer, if there are sufficiently
        // many available elements to access; otherwise throw std::out_of_range
        // exception.
        //
        // note:
        //      uses OutputContainer::reserve, if this method exists; otherwise
        //      default to the next version of read without this usage.
        //
        // note:
        //      To achieve exception safety we make a temporary copy of the
        //      underlying buffer. If the performance penalties involved
        //      are not acceptable, consider using a combination of
        //      `read` and (if necessary) an appropriate recovery mechanism
        //      to refill the buffer in the case of failure.
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        template <template <typename...> class OutputContainer = std::vector,
            typename = typename std::enable_if
                <detail::has_reserve<OutputContainer<T>>::value>::type>
        OutputContainer<T> safe_read (std::size_t n)
        {
            if (n <= available) {
                OutputContainer<T> result;
                result.reserve (n);
                {
                    auto tmpcopy = temporary_copy (n);
                    for (auto& e : tmpcopy)
                        result.emplace_back (std::move(e));
                }
                readpos    = wrapfront(readpos, n);
                available -= n;
                return result;
            } else {
                throw std::out_of_range
                    ("cannot read more than available elements");
            }
        }


        // safely read n elements from the ringbuffer, if there are sufficiently
        // many available elements to access; otherwise throw std::out_of_range
        // exception.
        //
        // note:
        //      To achieve exception safety we make a temporary copy of the
        //      underlying buffer. If the performance penalties involved
        //      are not acceptable, consider using a combination of
        //      `read` and (if necessary) an appropriate recovery mechanism
        //      to refill the buffer in the case of failure.
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        template <template <typename...> class OutputContainer = std::vector,
            typename = typename std::enable_if
                <not detail::has_reserve<OutputContainer<T>>::value>::type,
            bool /*unused, avoids template redeclaration*/ = bool{}>
        OutputContainer<T> safe_read (std::size_t n)
        {
            if (n <= available) {
                OutputContainer<T> result;
                {
                    auto tmpcopy = temporary_copy (n);
                    for (auto& e : tmpcopy)
                        result.emplace_back (std::move(e));
                }
                readpos    = wrapfront (readpos, n);
                available -= n;
                return result;
            } else {
                throw std::out_of_range
                    ("cannot read more than available elements");
            }
        }


        // read all elements from the buffer
        //
        // -- basic exception guarantee:
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      the state of the ringbuffer object is modified according
        //      to the current progress of read up to the failure point.
        //
        template <template <typename...> class OutputContainer = std::vector>
        OutputContainer<T> read_all (void)
        {
            return read<OutputContainer> (available);
        }


        // safely read all elements from the buffer
        // 
        // note:
        //      To achieve exception safety we make a temporary copy of the
        //      underlying buffer. If the performance penalties involved
        //      are not acceptable, consider using a combination of
        //      `read` and (if necessary) an appropriate recovery mechanism
        //      to refill the buffer in the case of failure.
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      throws an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        template <template <typename...> class OutputContainer = std::vector>
        OutputContainer<T> safe_read_all (void)
        {
            return safe_read<OutputContainer> (available);
        }


        // remove n elements from the buffer,
        // as if by repeatedly calling pop().
        //
        // IF AND ONLY IF T's destructor is nothrow
        // (which it should be) does the following hold:
        //
        // -- nothrow
        //
        // otherwise, if T's destructor may throw an exception
        // catchable as std::exception:
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      propogates an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        // otherwise no guarantees are made whatsoever.
        //
        void erase (std::size_t n) noexcept (noexcept(~T()))
        {
            auto m = std::min (n, available);
            while (m--) {
                reinterpret_cast<T*>(&readpos->data)->~T();
                readpos    = wrapfront (readpos);
                available -= 1;
            }
        }


        // clear all contents from the buffer
        //
        // IF AND ONLY IF T's destructor is nothrow
        // (which it should be) does the following hold:
        //
        // -- nothrow
        //
        // otherwise, if T's destructor may throw an exception
        // catchable as std::exception:
        //
        // -- strong exception guarantee
        //      (see https://en.wikipedia.org/wiki/Exception_safety)
        //    this function either:
        //      succeeds
        //                  --- or ---
        //      propogates an exception catchable as std::exception;
        //      no state is modified, as if the function was not called.
        //
        // otherwise no guarantees are made whatsoever.
        //
        void clear (void) noexcept (noexcept (~T()))
        {
            erase (available);
        }
    };
} // namespace ringbuffer
#endif // ifndef RINGBUFFER_HPP
