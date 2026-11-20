
#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <vector>

using namespace godot;

class Move : public Node3D {
    GDCLASS(Move, Node3D);

private:
    std::vector<Node3D*> fingers;
    std::vector<Vector3> corrections;
    std::vector<Vector3> coords;
    std::vector<Basis> bases;

protected:
    static void _bind_methods();

public:
    int initScanner();
    void calibrate();

    void _ready() override;
    void _process(double delta) override;
    Move();
};