#include "gtest/gtest.h"
#include "subfolder/header.hpp"

TEST(capra_robotics_eng_test_tests, ex1_all_same_digits)
{
    EXPECT_FALSE(capra::all_digits_unique(111));
};

TEST(capra_robotics_eng_test_tests, ex1_all_diff_digits)
{
    EXPECT_TRUE(capra::all_digits_unique(123));
};

TEST(capra_robotics_eng_test_testsvvvvv, ex1_finish0)
{
    EXPECT_TRUE(capra::all_digits_unique(17384590));
};

TEST(capra_robotics_eng_test_tests, ex1_exampletest_1)
{
    EXPECT_FALSE(capra::all_digits_unique(48778584));
};

TEST(capra_robotics_eng_test_tests, ex1_exampletest_2)
{
    EXPECT_TRUE(capra::all_digits_unique(17308459));
};

TEST(capra_robotics_eng_test_tests, ex2_exampletest_1)
{
    std::string input{"sort the letters in this string"};
    std::string sort_order{" isetlgornh"};
    std::string expected{"     iiisssseeettttttlgorrrnnhh"};
    capra::sort_letters(input, sort_order);
    EXPECT_EQ(input, expected);
}

struct set_up
{
    std::shared_ptr<capra::Room> r1 = std::make_shared<capra::Room>("room_1");
    std::shared_ptr<capra::Room> r2 = std::make_shared<capra::Room>("room_2");
    std::shared_ptr<capra::Room> r3 = std::make_shared<capra::Room>("room_3");
    std::shared_ptr<capra::Room> r4 = std::make_shared<capra::Room>("room_4");
    std::shared_ptr<capra::Room> r5 = std::make_shared<capra::Room>("room_5");
    std::shared_ptr<capra::Room> r6 = std::make_shared<capra::Room>("room_6");
    std::shared_ptr<capra::Room> r7 = std::make_shared<capra::Room>("room_7");
    std::shared_ptr<capra::Room> r8 = std::make_shared<capra::Room>("room_8");
    std::shared_ptr<capra::Room> r9 = std::make_shared<capra::Room>("room_9");

    set_up()
    {
        r1->connect(r2, capra::DOOR_ORIENTATION::NORTH);
        r1->connect(r3, capra::DOOR_ORIENTATION::SOUTH);
        r1->connect(r4, capra::DOOR_ORIENTATION::EAST);
        r1->connect(r5, capra::DOOR_ORIENTATION::WEST);

        r2->connect(r5, capra::DOOR_ORIENTATION::EAST);
        r2->connect(r6, capra::DOOR_ORIENTATION::WEST);

        r5->connect(r7, capra::DOOR_ORIENTATION::SOUTH);

        r6->connect(r8, capra::DOOR_ORIENTATION::EAST);

        r7->connect(r9, capra::DOOR_ORIENTATION::WEST);
    };
};
set_up su;
TEST(capra_robotics_eng_test_tests, ex3_exampletest_1)
{
    EXPECT_TRUE(su.r1->path_exists_to("room_9"));
}

TEST(capra_robotics_eng_test_tests, ex3_exampletest_2)
{
    EXPECT_FALSE(su.r1->path_exists_to("room_10"));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}