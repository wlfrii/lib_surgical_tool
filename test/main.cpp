#include "./src/test_all.h"

GTEST_API_ int main(int argc, char* argv[])
{
    printf("Test successfully!\n");
    InstrumentHandle handle;
    handle.libtest();
    //testing::InitGoogleTest(&argc, argv);
    return 0;//RUN_ALL_TESTS();
}