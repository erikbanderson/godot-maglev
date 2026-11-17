
#include "Move.h"
#include <godot_cpp/core/class_db.hpp>


void Move::_bind_methods() {
    ClassDB::bind_method(D_METHOD("getX"), & Move:: getX);
    ClassDB::bind_method(D_METHOD("getY"), & Move:: getY);
    ClassDB::bind_method(D_METHOD("getZ"), & Move:: getZ);

}

double Move::getX() {
    return x;
}
double Move::getY() {
    return y;
}
double Move::getZ() {
    return z;
}

Move::Move() {
    x = 1;
    y = 1;
    z = 1;
}