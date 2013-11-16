/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AA_MEM_HPP
#define AA_MEM_HPP

#include <memory>
#include <limits>
#include <list>
#include <vector>
#include <map>

/**
 * \file amino/mem.hpp
 */


/** Allocate memory for an object of a memory region
 *
 * Do not pass the pointer to delete.
 */
inline void *
operator new ( size_t n, aa_mem_region_t *reg )
{
    return aa_mem_region_alloc( reg, n );
}

namespace amino {

/** Allocate out of an aa_mem_region_t
 */
template<class T>
class RegionAllocator {

public :
    aa_mem_region *region;

    /* Typedefs */
    typedef T value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    /* Rebind */
    template<class U>
    struct rebind {
        typedef RegionAllocator<U> other;
    };

    /* This won't work! */
    // inline RegionAllocator() {
    //     abort();
    // }

    inline RegionAllocator(aa_mem_region_t *r) :
    region(r) {}

    inline RegionAllocator(const RegionAllocator &other) :
    region(other.region) {}

    template<class U>
    inline RegionAllocator( const RegionAllocator<U> & other) :
        region(other.region) {}

    inline ~RegionAllocator() {}

    inline pointer address(reference r) { return &r; }
    inline const_pointer address(const_reference r) { return &r; }

    inline pointer allocate(size_type cnt,
                            typename std::allocator<void>::const_pointer = 0) {
        return reinterpret_cast<pointer>( aa_mem_region_alloc(region, cnt*sizeof(value_type)) );
    }
    inline void deallocate(pointer p, size_type) { /* nop */ }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max() / sizeof(T);

    }

    template <class U>
    RegionAllocator& operator=(const RegionAllocator<U>&) { return *this; }


    inline void construct(pointer p, const T& t) { new(p) T(t); }
    inline void destroy(pointer p) { p->~T(); }

    inline bool operator==(RegionAllocator const& a) { return region == a.region; }
    inline bool operator!=(RegionAllocator const& a) { return !operator==(a); }

};

/* Typedefs for STL lists using region allocator */
template<class T>
struct RegionList
{
    typedef T value_type;
    typedef RegionAllocator<value_type> allocator;
    typedef std::list<T, allocator > type;
    typedef typename type::iterator iterator;
};

/* Typedefs for STL vector using region allocator */
template<class T>
struct RegionVector
{
    typedef T value_type;
    typedef RegionAllocator<value_type> allocator;
    typedef std::vector<T, allocator > type;
    typedef typename type::iterator iterator;
};


/* Typedefs for STL map using region allocator */
template<class K, class T, class C=std::less<K> >
struct RegionMap
{
    typedef std::pair<const K, T> value_type;
    typedef RegionAllocator<value_type> allocator;
    typedef std::map<K, T, C, allocator > type;
    typedef typename type::iterator iterator;
};

}



#endif //AA_MEM_HPP
