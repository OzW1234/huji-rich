/*! \file assert.hpp
\brief A wrapper around Boost::Assert that integrates well with gtest
\author Itay Zandbank
*/


#ifndef ASSERT_HPP
#define ASSERT_HPP 1

#ifdef NDEBUG
#define BOOST_DISABLED_ASSERTS
#else
#define BOOST_ENABLE_ASSERT_HANDLER
#endif

#include <boost/assert.hpp>

extern void(*BOOST_ASSERT_HANDLER)(const char *expr, const char *function, const char *file, long line);

#endif // ASSERT_HPP