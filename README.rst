==========
ringbuffer
==========

A single header C++ implementation of threadsafe and non-threadsafe fixed-size,
templated, STL-style cicular buffers (``atomic_ringbuffer.hpp`` and
``ringbuffer.hpp``, respectively). Both versions support full RAII/RRID
compliance and the strong exception safety guarantee wherever possible.

----
Info
----

Both files are known to compile with clang++ (ver. Apple LLVM 7.0.0),
and g++ (ver. 5.2.0) on OS X 10.11, under both ``-std=c++11`` and
``-std=c++14``.

-------
License
-------

ringbuffer is licensed under the OSI Approved MIT License Copyrighted (c) 2015
by Dalton Woodard. Please see the file LICENSE.md distributed with this
package for details.
