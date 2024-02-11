#pragma once
#include "vector2d.h"
#include <cmath>
#define PI 3.14159

Vector2d::Vector2d(double x, double y) : _orientation(0),
                                         _x(0),
                                         _y(0),
                                         _length(0)
{
    updateVectorfromCartesian(x, y);
}

Vector2d::Vector2d(double length, Rotation2d orientation) : _orientation(0),
                                                            _x(0),
                                                            _y(0),
                                                            _length(0)
{
    updateVectorfromPolar(length, orientation);
}

void Vector2d::updateVectorfromCartesian(double x, double y)
{
    _x = x;
    _y = y;
    _length = sqrt(x * x + y * y);
    Rotation2d temp(atan2(y, x));
    _orientation += temp;
}
void Vector2d::updateVectorfromPolar(double l, Rotation2d orientation)
{
    _length = l;
    _orientation.setAngleRad(orientation.getRad());
    _x = _length * _orientation.getCos();
    _y = _length * _orientation.getSin();
}

double Vector2d::getX() const { return _x; }
double Vector2d::getY() const { return _y; }
double Vector2d::getLength() const { return _length; }
Rotation2d Vector2d::getOrientation() const { return _orientation; }

void Vector2d::operator*=(double scalar)
{
    this->updateVectorfromCartesian(_x * scalar, _y * scalar);
}

void Vector2d::operator+=(Vector2d b)
{
    this->updateVectorfromCartesian(_x + b.getX(), _y + b.getY());
}
void Vector2d::operator-=(Vector2d b)
{
    this->updateVectorfromCartesian(_x - b.getX(), _y - b.getY());
}
Vector2d *Vector2d::operator*(double scalar) const
{
    return new Vector2d(scalar * _x, scalar * _y);
}
Vector2d *Vector2d::operator+(Vector2d b) const
{
    return new Vector2d(_x + b.getX(), _y + b.getY());
}
Vector2d *Vector2d::operator-(Vector2d b) const
{
    return new Vector2d(_x - b.getX(), _y - b.getY());
}

double Vector2d::dot(Vector2d a, Vector2d b)
{
    return a.getX() * b.getX() + a.getY() * b.getY();
}
void Vector2d::rotateBy(Vector2d &a, Rotation2d theta)
{
    Rotation2d temp(a.getOrientation().getRad() + theta.getRad());
    a.updateVectorfromPolar(a.getLength(), temp);
}