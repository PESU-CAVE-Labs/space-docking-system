#pragma once

#include <glm/glm.hpp>

// Forward declarations
class Mat3;
class Quaternion;

class Vec3 {
public:
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Math operations
    Vec3 operator+(const Vec3& rhs) const { return Vec3(x + rhs.x, y + rhs.y, z + rhs.z); }
    Vec3 operator-(const Vec3& rhs) const { return Vec3(x - rhs.x, y - rhs.y, z - rhs.z); }
    Vec3 operator*(double scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }
    Vec3 operator/(double scalar) const { return Vec3(x / scalar, y / scalar, z / scalar); }

    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& operator*=(double scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }

    // Unary minus
    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    // Vector operations
    double Dot(const Vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    Vec3 Cross(const Vec3& rhs) const {
        return Vec3(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    double MagnitudeSq() const { return Dot(*this); }
    double Magnitude() const { return std::sqrt(MagnitudeSq()); }
    
    Vec3 Normalized() const {
        double mag = Magnitude();
        return (mag > 1e-12) ? (*this / mag) : Vec3(0, 0, 0);
    }
};

inline Vec3 operator*(double scalar, const Vec3& v) {
    return v * scalar;
}
