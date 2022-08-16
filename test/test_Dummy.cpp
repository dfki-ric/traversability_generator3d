#include <boost/test/unit_test.hpp>
#include <traversability_generator3d/Dummy.hpp>

using namespace traversability_generator3d;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    traversability_generator3d::DummyClass dummy;
    dummy.welcome();
}
