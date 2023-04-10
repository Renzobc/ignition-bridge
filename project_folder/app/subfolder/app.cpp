#include <subfolder/header.hpp>

int main()
{
    capra::all_digits_unique(17308459);
    capra::all_digits_unique(48778584);
    std::string input{"sort the letters in this string"};
    capra::sort_letters(input, " isetlgornh");

    std::shared_ptr<capra::Room> r1 = std::make_shared<capra::Room>("room_1");
    std::shared_ptr<capra::Room> r2 = std::make_shared<capra::Room>("room_2");
    std::shared_ptr<capra::Room> r3 = std::make_shared<capra::Room>("room_3");
    std::shared_ptr<capra::Room> r4 = std::make_shared<capra::Room>("room_4");
    std::shared_ptr<capra::Room> r5 = std::make_shared<capra::Room>("room_5");
    std::shared_ptr<capra::Room> r6 = std::make_shared<capra::Room>("room_6");
    std::shared_ptr<capra::Room> r7 = std::make_shared<capra::Room>("room_7");
    std::shared_ptr<capra::Room> r8 = std::make_shared<capra::Room>("room_8");
    std::shared_ptr<capra::Room> r9 = std::make_shared<capra::Room>("room_9");

    r1->connect(r2, capra::DOOR_ORIENTATION::NORTH);
    r1->connect(r3, capra::DOOR_ORIENTATION::SOUTH);
    r1->connect(r4, capra::DOOR_ORIENTATION::EAST);
    r1->connect(r5, capra::DOOR_ORIENTATION::WEST);

    r2->connect(r5, capra::DOOR_ORIENTATION::EAST);
    r2->connect(r6, capra::DOOR_ORIENTATION::WEST);

    r5->connect(r7, capra::DOOR_ORIENTATION::SOUTH);

    r6->connect(r8, capra::DOOR_ORIENTATION::EAST);

    r7->connect(r9, capra::DOOR_ORIENTATION::WEST);

    r1->path_exists_to("room_9");
};