
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <numeric>
#include <thread>

#include "../include/ringbuffer.hpp"
#include "../include/atomic_ringbuffer.hpp"

static constexpr std::size_t N = 1000000;

using rb_type  = ringbuffer::ringbuffer<uint64_t, N>;
using arb_type = atomic_ringbuffer::atomic_ringbuffer<uint64_t, N>;

uint64_t fill_and_sum (rb_type * rb)
{
    for (std::size_t i = 0; i < N; ++i)
        rb->emplace (i);

    uint64_t sum = 0;
    for (std::size_t i = 0; i < N; ++i)
        sum += rb->read ();

    return sum;
}

uint64_t fill_and_sum (arb_type * arb)
{

    for (std::size_t i = 0; i < N; ++i)
        arb->emplace (i);

    auto f =
    [] (arb_type * b, std::size_t n)
    {
        uint64_t sum = 0;
        for (std::size_t i = 0; i < n; ++i)
            sum += b->read ();
        return sum;
    };

    std::size_t max_thrd = std::thread::hardware_concurrency ();

    std::vector<std::future<uint64_t>> futresults;
    for (std::size_t i = 0; i < max_thrd; ++i)
        futresults.push_back
            (std::async
                (std::launch::async, f, arb, N / max_thrd));

    std::vector<uint64_t> res;
    for (auto& fut : futresults)
        res.push_back (fut.get());

    return std::accumulate (std::begin(res), std::end(res), 0ULL);
}

int main (void)
{
    //std::unique_ptr<rb_type>  rb  (new rb_type);
    std::unique_ptr<arb_type> arb (new arb_type);

    //std::cout << fill_and_sum (rb.get()) << std::endl;
    std::cout << fill_and_sum (arb.get()) << std::endl;

    return 0;
}
