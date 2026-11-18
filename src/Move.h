
#pragma once

#include <godot_cpp/classes/node3d.hpp>


using namespace godot;

class Move : public Node3D {
    GDCLASS(Move, Node3D);

    double x;
    double y;
    double z;

protected:
    static void _bind_methods();

public:
    int initScanner();
    void calibrate();

    void _ready() override;
    void _process(double delta) override;
    Move();
};