#include <subfolder/header.hpp>

// Complexity O(n) with n being the number of digits in the input value
bool capra::all_digits_unique(uint32_t value)
{
    uint32_t integral{0};
    uint32_t digit{0};
    // Only ten digits to layout and count occurences
    std::vector digit_layout(10, 0);

    while (value > 0)
    {
        // Separate last digit
        integral = int(value / 10);
        digit = int(std::round((double(value) / 10 - integral) * 10)) % 10;
        // Divide the value to iterate in the next digit
        value = integral;
        // Count occurences with the digit taken as index
        digit_layout[digit] += 1;
    }

    // Check if the occurences of each digit is < 2
    return std::all_of(digit_layout.begin(), digit_layout.end(), [](const int e) -> bool
                       { return e < 2; });
};

// Complexity
// n letters in input
// m unique letters, given by sort order
// partition takes n operation from first to last but it takes less elements when it advances.
// m number of times one need to separate the group given by sort order.
void capra::sort_letters(std::string &input, std::string sort_order)
{
    // std::remove(input.begin(), input.end(), ' ');

    std::string::iterator i_itr = input.begin();
    std::string::iterator order_itr = sort_order.begin();

    while (order_itr != sort_order.end())
    {
        i_itr = std::partition(i_itr, input.end(), [&order_itr](const char e) -> bool
                               { return e == *order_itr; });
        std::advance(order_itr, 1);
    };
};

void capra::Room::connect(std::shared_ptr<Room> &r, DOOR_ORIENTATION door)
{
    switch (door)
    {
    case DOOR_ORIENTATION::NORTH:
        north = r;
        break;
    case DOOR_ORIENTATION::SOUTH:
        south = r;
        break;
    case DOOR_ORIENTATION::WEST:
        west = r;
        break;
    case DOOR_ORIENTATION::EAST:
        east = r;
        break;
    default:
        break;
    }
};

bool capra::Room::path_exists_to(const std::string ending_room_name)
{
    std::set<Room> checked;
    std::set<Room> rooms;

    if (this->north.lock().get())
        rooms.insert(*this->north.lock().get());
    if (this->south.lock().get())
        rooms.insert(*this->south.lock().get());
    if (this->east.lock().get())
        rooms.insert(*this->east.lock().get());
    if (this->west.lock().get())
        rooms.insert(*this->west.lock().get());

    while (rooms.size() > 0)
    {
        Room current = *rooms.begin();

        if (current.name == ending_room_name)
            return true;

        if (current.north.lock().get() && checked.find(*current.north.lock().get()) == checked.end())
            rooms.insert(*current.north.lock().get());

        if (current.south.lock().get() && checked.find(*current.south.lock().get()) == checked.end())
            rooms.insert(*current.south.lock().get());

        if (current.east.lock().get() && checked.find(*current.east.lock().get()) == checked.end())
            rooms.insert(*current.east.lock().get());

        if (current.west.lock().get() && checked.find(*current.west.lock().get()) == checked.end())
            rooms.insert(*current.west.lock().get());

        checked.insert(current);
        rooms.erase(current);
    }
    return false;
};
