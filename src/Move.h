
#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <array>

using namespace godot;

class Move : public Node3D {
    GDCLASS(Move, Node3D);

private:
    std::array<Node3D*, 2> fingers;
    std::array<Vector3, 2> corrections;
    std::array<Vector3, 2> coords;
    std::array<Basis, 2> bases;

protected:
    static void _bind_methods();


public:
    int initScanner();
    void calibrate();
    void updateCoords();
    void _ready() override;
    void _input(const Ref<InputEvent> &event);

    void _process(double delta) override;
    Move();
};