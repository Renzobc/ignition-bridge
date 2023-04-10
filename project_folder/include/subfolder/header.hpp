/**
 * Copyright 2023 Renzo Bruzzone
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// 1. (Mandatory) Write a function that takes an unsigned 32 bits integer as input and
// returns true if all the digits in the base 10 representation of that number are unique.
// bool all_digits_unique(uint32_t value)
// Example:
// all_digits_unique (48778584) returns false
// all_digits_unique (17308459) returns true

#include <algorithm>
#include <vector>
#include <cmath>
#include <string>
#include <memory>
#include <set>
#include <iostream>

namespace capra
{

    bool all_digits_unique(uint32_t value);

    void sort_letters(std::string &input, std::string sort_order);

    enum class DOOR_ORIENTATION : char
    {
        NORTH = 'n',
        SOUTH = 's',
        WEST = 'w',
        EAST = 'e'
    };

    struct Room
    {
        // weak ptr referencing the copy count block of the shared ptr
        // the reference to a room should NOT keep the room alive if needed to be deleted.
        std::weak_ptr<Room> north;
        std::weak_ptr<Room> south;
        std::weak_ptr<Room> east;
        std::weak_ptr<Room> west;

        std::string name;

        // Room() = delete;
        Room(std::string r_name) : name(r_name){};

        // Connects to a already created shared ptr of type room
        void connect(std::shared_ptr<Room> &r, DOOR_ORIENTATION door);

        // find if path exists
        bool path_exists_to(const std::string ending_room_name);

        bool operator<(const Room other) const
        {
            return std::basic_string<char>(this->name) < std::basic_string<char>(other.name);
        }

        bool operator==(const Room other) const
        {
            return other.name == this->name;
        }
    };

};