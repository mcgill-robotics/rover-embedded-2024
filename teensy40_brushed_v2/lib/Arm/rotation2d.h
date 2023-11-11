#pragma once
#include <cmath>
#define PI 3.14159
/**
 * A simple class that represents a 2d Rotation regardless of unit.
 * @author David Ly
*/
class Rotation2d 
{
    private: 
    double _theta_rad, _sin, _cos;
    public:
    /**
     * Creates a new Rotation2d with given angle in radians.
     * @param theta_rad The angle of the rotation in radians.
    */
    explicit Rotation2d (double theta_rad)
    {
        _theta_rad = 0;
        _sin = 0;
        _cos = 0;
        setAngleRad(theta_rad);
    }

    /**
     * Creates a new Rotation2d with given angle in degrees.
     * @param theta_deg The angle of the rotation in degrees.
    */
    static Rotation2d *getRotationFromDeg(double theta)
    {
        return new Rotation2d(theta/180*PI);
    }

    /**
     * Creates a new Rotation2d with given angle in rotations.
     * @param theta The angle of the rotation in rotations.
    */
    static Rotation2d *getRotationFromRotation(double theta)
    {
        return new Rotation2d(theta*PI);
    }

    /**
     * Sets the angle of the rotation in radians.
     * @return The angle of the rotation in radians.
    */
    void setAngleRad(double theta_rad)
    {
        _theta_rad = theta_rad;
        _sin = sin(_theta_rad);
        _cos = cos(_theta_rad);
    }

    /**
     * Gets the angle of the rotation in radians.
     * @return The angle of the rotation in radians.
    */
    double getRad() const
    {
        return _theta_rad;
    }

    /**
     * Gets the angle of the rotation in degrees.
     * @return The angle of the rotation in degrees.
    */
    double getDeg() const
    {
        return _theta_rad*180/PI;
    }

    /**
     * Gets the angle of the rotation in rotations.
     * @return The angle of the rotation in rotations.
    */
    double getRotation() const
    {
        return _theta_rad/(2*PI);
    }

    /**
     * Gets the sin of the rotation.
     * @return The sin of the rotation.
    */
    double getSin() const{return _sin;}

    /**
     * Gets the cos of the rotation.
     * @return The cos of the rotation.
    */
    double getCos() const{return _cos;}
    
    /**
     * Adds the given rotation to this rotation.
     * @param rot The rotation to add to this rotation.
    */
    void operator += (Rotation2d rot)
    {
        this->setAngleRad(this->_theta_rad+rot._theta_rad);
    }
    /**
     * Subtracts the given rotation from this rotation.
     * @param rot The rotation to subtract from this rotation.
    */
    void operator -= (Rotation2d rot)
    {
        this->setAngleRad(this->_theta_rad-rot._theta_rad);
    }
    /**
     * Adds the given rotation to this rotation.
     * Returns a new object that's dynamically allocated
     * @param rot The rotation to add to this rotation.
     * @return The sum of the two rotations.
    */
    Rotation2d *operator + (Rotation2d rot) const
    {
        return new Rotation2d(this->_theta_rad+rot._theta_rad);
    }
    /**
     * Subtracts the given rotation from this rotation.
     * Returns a new object that's dynamically allocated
     * @param rot The rotation to subtract from this rotation.
     * @return The difference of the two rotations.
    */
    Rotation2d *operator - (Rotation2d rot) const
    {
        return new Rotation2d(this->_theta_rad-rot._theta_rad);
    }
};