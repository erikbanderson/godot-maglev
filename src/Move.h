
#pragma once

#include <godot_cpp/classes/object.hpp>


using namespace godot;

class Move : public Object {
    GDCLASS(Move, Object);

    double x;
    double y;
    double z;

protected:
    static void _bind_methods();

public:
    double getX();
    double getY();
    double getZ();

    Move();
};