#pragma once
#include "rotation2d.h"

/**
 * A simple class that represents a vector in 2d space.
 * @author David LY
*/
class Vector2d
{
    private:
    double _x,_y;
    double _length;
    Rotation2d _orientation;    

    public:
    /**
     * Creates a new vector with given x and y components.
     * @param x The x component of the vector.
     * @param y The y component of the vector.
    */
   
    Vector2d (double x, double y);
    /**
     * Creates a new vector with given length and angle.
     * @param length The length of the vector.
     * @param orientation The angle of the vector.
    */

    Vector2d (double length, Rotation2d orientation);
    /**
     * Change vector with given x and y components.
     * @param x The x component of the vector.
     * @param y The y component of the vector.
    */

    void updateVectorfromCartesian(double x, double y);
    /**
     * Change vector with given length and angle.
     * @param length The length of the vector.
     * @param orientation The angle of the vector.
    */
    void updateVectorfromPolar(double l, Rotation2d orientation);

    /**
     * @return The x component of the vector.
    */
    double getX() const;

    /**
     * @return The y component of the vector.
    */
    double getY() const;

    /**
     * @return The length of the vector.
    */
    double getLength() const;

    /**
     * @return The orientaion of the vector.
    */
    Rotation2d getOrientation() const;
    
    /**
     * Scalar multiplication of the vector.
     * @param scalar The scalar to multiply the vector by.
    */
    void operator*=(double scalar);

    /**
     * Vector addition of the vector.
     * @param b The vector to add to the vector.
    */
    void operator+=(Vector2d b);

    /**
     * Vector subtraction of the vector.
     * @param b The vector to subtract from the vector.
    */

    void operator-=(Vector2d b);
    /**
     * Scalar multiplication of the vector.
     * New dynamic Vector2d object is created.
     * @return new Vector2d multiplied by the scalar.
    */
    Vector2d *operator * (double scalar) const;

    /**
     * Vector addition of the vector.
     * New dynamic Vector2d object is created.
     * @return new Vector2d added to the vector.
    */
    Vector2d *operator + (Vector2d b) const;

    /**
     * Vector subtraction of the vector.
     * New dynamic Vector2d object is created.
     * @return new Vector2d subtracted from the vector.
    */
    Vector2d *operator - (Vector2d b) const;

    /**
     * Dot Product computation of two vector.
     * @param a The first vector.
     * @param b The second vector.
     * @return The dot product of two vector.
    */
    static double dot(Vector2d a, Vector2d b);

    /**
     * Rotation of a vector
     * @param a The vector to rotate. This vector is changed by the method.
     * @param theta The angle to rotate the vector by.
    */
    static void rotateBy(Vector2d &a, Rotation2d theta);

};