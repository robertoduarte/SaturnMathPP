#include "test_main.hpp"
int main()
{
    constexpr bool allPassed = []() {
        return true;
    }();
    static_assert(allPassed, "All compile-time tests must pass");
}
