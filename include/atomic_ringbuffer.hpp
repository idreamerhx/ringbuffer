// atomic_ringbuffer
//      : a threadsafe, fixed (at compile time) sized circular
//        buffer type supporting multiple readers and one
//        writer.
//
// Author: Dalton Woodard
// Contact: daltonmwoodar@gmail.com
// License: Please see LICENSE.md
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
//      The stored object type T must be default constructable.
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
//     All of the above are documented as such in source. 

#ifndef ATOMIC_RINGBUFFER_HPP
#define ATOMIC_RINGBUFFER_HPP

#include <array>       // std::array
#include <atomic>      // std::atomic<..>
#include <mutex>       // std::shared_mutext, std::lock_guard,
                       // std::shared_lock
#include <type_traits> // std::is_default_constructible
#include <vector>      // std::vector

namespace ringbuffer
{
namespace detail
{
    template <typename T>
    struct has_reserve
    {
        struct succeeds { char _[8]; };
        struct fails    { char _[0]; };

        template <typename U, void (U::*)(std::size_t)>
        struct sfinae {};

        template <typename U>
        static succeeds test (sfinae<U, &U::reserve>*);

        template <typename U>
        static fails test (...);

        static constexpr bool value =
            sizeof(test<T>(0)) == sizeof(succeeds);
    };


    // a simple multiple reader, single
    // writer shared mutex, which spins
    // when waiting for a write oprotunity.
    //
    // -- RAII/RRID compliant
    //
    class rw_mutex
    {
    private:
        mutable std::mutex mut;
        mutable std::atomic_size_t readers;

        struct reader_entity
        {
        private:
            rw_mutex const * parent;
        public:
            reader_entity (rw_mutex const * p) noexcept
                : parent (p)
            {
                parent->read_lock ();
            }

            ~reader_entity (void) noexcept
            {
                parent->read_unlock ();
            }
        };

        struct writer_entity
        {
        private:
            rw_mutex const * parent;
        public:
            writer_entity (rw_mutex const * p) noexcept
                : parent (p)
            {
                parent->write_lock ();
            }

            ~writer_entity (void) noexcept
            {
                parent->write_unlock ();
            }
        };

        void read_lock (void) const noexcept
        {
            mut.lock ();
            readers += 1;
            mut.unlock ();
        }

        void read_unlock (void) const noexcept
        {
            if (readers.load())
                readers -= 1;
        }

        void write_lock (void) const noexcept
        {
            while (!mut.try_lock())
                continue;
        }

        void write_unlock (void) const noexcept
        {
            mut.unlock ();
        }

    public:
        rw_mutex  (void) noexcept = default;
        ~rw_mutex (void) noexcept = default;

        rw_mutex (rw_mutex const&) = delete;
        rw_mutex& operator= (rw_mutex const&) = delete;

        reader_entity reader (void) const noexcept
        {
            return reader_entity (this);
        }

        writer_entity writer (void) const noexcept
        {
            return writer_entity (this);
        }
    };
} // namespace detail
 
    template <typename T, std::size_t N>
    class ringbuffer
    {
        static_assert (N > 0, "empty ringbuffer disallowed");
        static_assert (std::is_default_constructible<T>::value,
                       "cannot buffer non-default_constructible objects");
    private:
        using backing_type = std::array<T, N>;
        using backing_ptr  = typename backing_type::pointer;

        backing_type buff;

        backing_ptr const first;
        backing_ptr const last;

        std::atomic<backing_ptr> writepos;
        std::atomic<backing_ptr> readpos;
        std::atomic_size_t       available;

        detail::rw_mutex rwlock;

        template <typename U,
            typename NoQualsU = typename std::remove_cv<U>::type>
        class atomic_rb_iterator :
            public std::iterator<std::random_access_iterator_tag,
                                 NoQualsU,
                                 std::ptrdiff_t,
                                 U*,
                                 U&>
        {
        private:
            U* iter;
            U const* first;
            U const* last;
            detail::rw_mutex * rwlock;

            using piter_type = std::iterator<std::random_access_iterator_tag,
                                             NoQualsU,
                                             std::ptrdiff_t,
                                             U*,
                                             U&>;

        public:
            using difference_type   = typename piter_type::difference_type;
            using value_type        = typename piter_type::value_type;
            using pointer           = typename piter_type::pointer;
            using reference         = typename piter_type::reference;
            using iterator_category = typename piter_type::iterator_category;

            atomic_rb_iterator (void) = delete;
            atomic_rb_iterator
                (U* p, U const* f, U const* l, detail::rw_mutex * lck)
                    : iter (p), first (f), last (l), rwlock (lck)
            {}

            void swap (atomic_rb_iterator & other)
            {
                std::swap (iter, other.iter);
                std::swap (first, other.first);
                std::swap (last, other.last);
                std::swap (rwlock, other.rwlock);
            }

            atomic_rb_iterator<U>& operator++ (void)
            {
                if (iter == last)
                    iter = first;
                else
                    iter += 1;
                return *this;
            }

            atomic_rb_iterator<U>& operator-- (void)
            {
                if (iter == first)
                    iter = last;
                else
                    iter -= 1;
                return *this;
            }

            atomic_rb_iterator<U> operator++ (int)
            {
                auto tmp (*this);
                if (iter == last)
                    iter = first;
                else
                    iter += 1;
                return tmp;
            }

            atomic_rb_iterator<U> operator-- (int)
            {
                auto tmp (*this);
                if (iter == first)
                    iter = last;
                else
                    iter -= 1;
                return tmp;
            }

            atomic_rb_iterator<U>& operator+= (difference_type n)
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

            atomic_rb_iterator<U>& operator-= (difference_type n)
            {
                return this->operator+= (-n);
            }

            atomic_rb_iterator<U> operator+ (difference_type n) const
            {
                auto tmp = *this;
                return (tmp += n); 
            }

            atomic_rb_iterator<U> operator- (difference_type n) const
            {
                auto tmp = *this;
                return (tmp -= n);
            }

            difference_type operator-
                (atomic_rb_iterator<U const> const rhs) const
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
                auto read = rwlock->reader ();
                return *iter;
            }

            U& operator[] (difference_type n)
            {
                auto read = rwlock->reader ();
                if (n > N || -n > N) n = n % N;
                return *(*this + n);
            }

            U& operator-> (void) const
            {
                auto read = rwlock->reader ();
                return *iter;
            }

            operator atomic_rb_iterator<U const> (void) const
            {
                return atomic_rb_iterator<U const> (iter, first, last);
            }
        };

        inline backing_ptr wrapfront
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
        // note:
        //      this operation blocks all writes until completed. 
        //
        std::vector<T> temporary_copy (std::size_t n) const
        {
            // ensure no writes happen durring copy
            auto read = rwlock.reader ();

            // ensure we don't actually read more elements
            // than is possible.
            n = std::min (n, available.load());

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
        using value_type = T;
        using size_type       = std::size_t;
        using difference_type = std::ptrdiff_t;
        using reference       = T&;
        using const_reference = T const&;
        using pointer         = T*;
        using const_pointer   = T const*;

        // no modification through iterators; only
        // observatino.
        using iterator        = atomic_rb_iterator<T const>;
        using const_iterator  = atomic_rb_iterator<T const>;


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
        inline std::size_t size (void) const noexcept
        {
            return available.load ();
        }


        // obtain the remaining capacity to add objects
        // to buffer.
        //
        // -- nothrow
        //
        inline std::size_t capacity (void) const noexcept
        {
            return N - available.load ();
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
            return const_iterator (readpos.load(), first, last);
        }


        // obtain const iterator to the end of the buffer 
        //
        // -- nothrow
        //
        const_iterator cend (void) const noexcept
        {
            return const_iterator
                (wrapfront (readpos.load(), available.load()),
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
        inline void emplace (Args && ... args) noexcept(noexcept(T(args...)))
        {
            auto write = rwlock.writer ();
            if (available.load() < N) {
                new (writepos) T(args...);
                writepos.store (wrapfront(writepos.load()));
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
        inline void pop (void) noexcept (noexcept(~T()))
        {
            auto write = rwlock.writer ();
            if (available.load()) {
                readpos->~T();
                readpos.store (wrapfront(readpos.load()));
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
        inline T front (void)
        {
            auto read = rwlock.reader ();
            if (available.load ()) {
                return *readpos;
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
        inline T read (void)
        {
            auto write = rwlock.writer ();
            if (available) {
                available -= 1;
                auto deref = readpos;
                readpos.store (wrapfront(readpos.load()));
                return *deref;
            } else {
                throw std::out_of_range ("invalid read on empty ringbuffer");
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
        inline OutputContainer<T> read (std::size_t n)
        {
            auto write = rwlock.writer ();
            if (n <= available) {
                OutputContainer<T> result;
                result.reserve (n);
                auto const cap = available - n;
                while (available-- > cap) {
                    auto deref = readpos;
                    readpos.store (wrapfront(readpos.load()));
                    result.emplace_back (*deref);
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
        inline OutputContainer<T> read (std::size_t n)
        {
            auto write = rwlock.writer ();
            if (n <= available) {
                OutputContainer<T> result;
                auto const cap = available - n;
                while (available-- > cap) {
                    auto deref = readpos;
                    readpos.store (wrapfront(readpos.load()));
                    result.emplace_back (*deref);
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
        inline OutputContainer<T> safe_read (std::size_t n)
        {
            auto write = rwlock.writer ();
            if (n <= available) {
                OutputContainer<T> result;
                result.reserve (n);
                {
                    auto tmpcopy = temporary_copy (n);
                    for (auto& e : tmpcopy)
                        result.emplace_back (std::move(e));
                }
                available -= n;
                readpos.store (wrapfront(readpos.load(), n));
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
        inline OutputContainer<T> safe_read (std::size_t n)
        {
            auto write = rwlock.writer ();
            if (n <= available) {
                OutputContainer<T> result;
                {
                    auto tmpcopy = temporary_copy (n);
                    for (auto& e : tmpcopy)
                        result.emplace_back (std::move(e));
                }
                available -= n;
                readpos.store (wrapfront(readpos.load(), n));
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
        inline OutputContainer<T> read_all (void)
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
        inline OutputContainer<T> safe_read_all (void)
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
        inline void erase (std::size_t n) noexcept (noexcept(~T()))
        {
            auto write = rwlock.writer ();
            auto m = std::min (n, available.load());
            while (m--) {
                readpos->~T();
                readpos.store (wrapfront(readpos.load()));
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
        inline void clear (void) noexcept (noexcept (~T()))
        {
            erase (available);
        }
    };
} // namespace ringbuffer
#endif // ifndef ATOMIC_RINGBUFFER_HPP
