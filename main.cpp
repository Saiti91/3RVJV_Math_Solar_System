#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <ctime>

#define _USE_MATH_DEFINES
#include <math.h>

// Forward declarations
class Vector3D;
class Vector4D;
class Matrix3x3;
class Matrix4x4;
class Quaternion;

// Vector3D class for 3D vector operations
class Vector3D {
public:
    float x, y, z;

    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Vector addition
    Vector3D operator+(const Vector3D& v) const {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }

    // Vector subtraction
    Vector3D operator-(const Vector3D& v) const {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }

    // Scalar multiplication
    Vector3D operator*(float scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    // Dot product
    float dot(const Vector3D& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector3D cross(const Vector3D& v) const {
        return Vector3D(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Magnitude (norm) of the vector
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Normalize the vector
    Vector3D normalize() const {
        float mag = magnitude();
        if (mag > 0) {
            return Vector3D(x / mag, y / mag, z / mag);
        }
        return *this;
    }
};

// Vector4D class for 4D vector operations
class Vector4D {
public:
    float x, y, z, w;

    Vector4D() : x(0), y(0), z(0), w(0) {}
    Vector4D(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    // Vector addition
    Vector4D operator+(const Vector4D& v) const {
        return Vector4D(x + v.x, y + v.y, z + v.z, w + v.w);
    }

    // Vector subtraction
    Vector4D operator-(const Vector4D& v) const {
        return Vector4D(x - v.x, y - v.y, z - v.z, w - v.w);
    }

    // Scalar multiplication
    Vector4D operator*(float scalar) const {
        return Vector4D(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    // Dot product
    float dot(const Vector4D& v) const {
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }

    // Magnitude (norm) of the vector
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z + w * w);
    }

    // Normalize the vector
    Vector4D normalize() const {
        float mag = magnitude();
        if (mag > 0) {
            return Vector4D(x / mag, y / mag, z / mag, w / mag);
        }
        return *this;
    }
};

// Matrix3x3 class for 3x3 matrix operations
class Matrix3x3 {
public:
    float m[3][3];

    Matrix3x3() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                m[i][j] = (i == j) ? 1.0f : 0.0f; // Identity matrix
            }
        }
    }

    Matrix3x3(float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    // Matrix multiplication
    Matrix3x3 operator*(const Matrix3x3& mat) const {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    result.m[i][j] += m[i][k] * mat.m[k][j];
                }
            }
        }
        return result;
    }

    // Matrix-vector multiplication
    Vector3D operator*(const Vector3D& v) const {
        return Vector3D(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    // Determinant
    float determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    // Transpose
    Matrix3x3 transpose() const {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }

    // Inverse
    Matrix3x3 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            // Matrix is singular
            return Matrix3x3();
        }

        float invDet = 1.0f / det;
        Matrix3x3 result;

        result.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
        result.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
        result.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

        result.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
        result.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
        result.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

        result.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
        result.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
        result.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

        return result;
    }
};

// Matrix4x4 class for 4x4 matrix operations
class Matrix4x4 {
public:
    float m[4][4];

    Matrix4x4() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m[i][j] = (i == j) ? 1.0f : 0.0f; // Identity matrix
            }
        }
    }

    Matrix4x4(float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }

    // Matrix multiplication
    Matrix4x4 operator*(const Matrix4x4& mat) const {
        Matrix4x4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * mat.m[k][j];
                }
            }
        }
        return result;
    }

    // Matrix-vector multiplication
    Vector4D operator*(const Vector4D& v) const {
        return Vector4D(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
        );
    }

    // Transpose
    Matrix4x4 transpose() const {
        Matrix4x4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }

    // Create translation matrix
    static Matrix4x4 createTranslation(float x, float y, float z) {
        Matrix4x4 result;
        result.m[0][3] = x;
        result.m[1][3] = y;
        result.m[2][3] = z;
        return result;
    }

    // Create scaling matrix
    static Matrix4x4 createScaling(float x, float y, float z) {
        Matrix4x4 result;
        result.m[0][0] = x;
        result.m[1][1] = y;
        result.m[2][2] = z;
        return result;
    }

    // Create rotation matrix around X axis
    static Matrix4x4 createRotationX(float angle) {
        float c = std::cos(angle);
        float s = std::sin(angle);
        Matrix4x4 result;
        result.m[1][1] = c;
        result.m[1][2] = -s;
        result.m[2][1] = s;
        result.m[2][2] = c;
        return result;
    }

    // Create rotation matrix around Y axis
    static Matrix4x4 createRotationY(float angle) {
        float c = std::cos(angle);
        float s = std::sin(angle);
        Matrix4x4 result;
        result.m[0][0] = c;
        result.m[0][2] = s;
        result.m[2][0] = -s;
        result.m[2][2] = c;
        return result;
    }

    // Create rotation matrix around Z axis
    static Matrix4x4 createRotationZ(float angle) {
        float c = std::cos(angle);
        float s = std::sin(angle);
        Matrix4x4 result;
        result.m[0][0] = c;
        result.m[0][1] = -s;
        result.m[1][0] = s;
        result.m[1][1] = c;
        return result;
    }

    // Create shear matrix
    static Matrix4x4 createShear(float xy, float xz, float yx, float yz, float zx, float zy) {
        Matrix4x4 result;
        result.m[0][1] = xy;
        result.m[0][2] = xz;
        result.m[1][0] = yx;
        result.m[1][2] = yz;
        result.m[2][0] = zx;
        result.m[2][1] = zy;
        return result;
    }

    // Create perspective projection matrix
    static Matrix4x4 createPerspective(float fovY, float aspect, float zNear, float zFar) {
        Matrix4x4 result;
        float tanHalfFovy = std::tan(fovY * 0.5f);

        result.m[0][0] = 1.0f / (aspect * tanHalfFovy);
        result.m[1][1] = 1.0f / tanHalfFovy;
        result.m[2][2] = -(zFar + zNear) / (zFar - zNear);
        result.m[2][3] = -2.0f * zFar * zNear / (zFar - zNear);
        result.m[3][2] = -1.0f;
        result.m[3][3] = 0.0f;

        return result;
    }

    // Create look-at view matrix
    static Matrix4x4 createLookAt(const Vector3D& eye, const Vector3D& center, const Vector3D& up) {
        Vector3D f = (center - eye).normalize();
        Vector3D s = f.cross(up).normalize();
        Vector3D u = s.cross(f);

        Matrix4x4 result;
        result.m[0][0] = s.x;
        result.m[0][1] = s.y;
        result.m[0][2] = s.z;
        result.m[1][0] = u.x;
        result.m[1][1] = u.y;
        result.m[1][2] = u.z;
        result.m[2][0] = -f.x;
        result.m[2][1] = -f.y;
        result.m[2][2] = -f.z;
        result.m[0][3] = -s.dot(eye);
        result.m[1][3] = -u.dot(eye);
        result.m[2][3] = f.dot(eye);

        return result;
    }
};

// Quaternion class with all required operations
class Quaternion {
public:
    float w, x, y, z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Addition
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    // Multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Scalar multiplication
    Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Conjugate
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Norm (magnitude)
    float norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalize to unit quaternion
    Quaternion normalize() const {
        float n = norm();
        if (n > 0) {
            return Quaternion(w / n, x / n, y / n, z / n);
        }
        return Quaternion(1, 0, 0, 0); // Return identity if norm is 0
    }

    // Inverse quaternion
    Quaternion inverse() const {
        float n = norm();
        n = n * n; // Square
        if (n > 0) {
            float invN = 1.0f / n;
            return Quaternion(w * invN, -x * invN, -y * invN, -z * invN);
        }
        return Quaternion(1, 0, 0, 0); // Return identity if norm is 0
    }

    // Create a quaternion from an axis and an angle
    static Quaternion fromAxisAngle(float angle, float axisX, float axisY, float axisZ) {
        Vector3D axis(axisX, axisY, axisZ);
        axis = axis.normalize();

        float halfAngle = angle * 0.5f;
        float sinHalfAngle = std::sin(halfAngle);

        return Quaternion(
            std::cos(halfAngle),
            axis.x * sinHalfAngle,
            axis.y * sinHalfAngle,
            axis.z * sinHalfAngle
        );
    }

    // Convert to 3x3 rotation matrix
    Matrix3x3 toRotationMatrix3x3() const {
        Quaternion q = normalize();
        float xx = q.x * q.x;
        float xy = q.x * q.y;
        float xz = q.x * q.z;
        float xw = q.x * q.w;
        float yy = q.y * q.y;
        float yz = q.y * q.z;
        float yw = q.y * q.w;
        float zz = q.z * q.z;
        float zw = q.z * q.w;

        return Matrix3x3(
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw),
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw),
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)
        );
    }

    // Convert to 4x4 rotation matrix
    Matrix4x4 toRotationMatrix4x4() const {
        Quaternion q = normalize();
        float xx = q.x * q.x;
        float xy = q.x * q.y;
        float xz = q.x * q.z;
        float xw = q.x * q.w;
        float yy = q.y * q.y;
        float yz = q.y * q.z;
        float yw = q.y * q.w;
        float zz = q.z * q.z;
        float zw = q.z * q.w;

        return Matrix4x4(
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0,
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0,
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1
        );
    }

    // Convert a 3x3 rotation matrix to quaternion
    static Quaternion fromRotationMatrix3x3(const Matrix3x3& m) {
        float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        float w, x, y, z;

        if (trace > 0) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            w = 0.25f / s;
            x = (m.m[2][1] - m.m[1][2]) * s;
            y = (m.m[0][2] - m.m[2][0]) * s;
            z = (m.m[1][0] - m.m[0][1]) * s;
        }
        else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
            float s = 2.0f * std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]);
            w = (m.m[2][1] - m.m[1][2]) / s;
            x = 0.25f * s;
            y = (m.m[0][1] + m.m[1][0]) / s;
            z = (m.m[0][2] + m.m[2][0]) / s;
        }
        else if (m.m[1][1] > m.m[2][2]) {
            float s = 2.0f * std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]);
            w = (m.m[0][2] - m.m[2][0]) / s;
            x = (m.m[0][1] + m.m[1][0]) / s;
            y = 0.25f * s;
            z = (m.m[1][2] + m.m[2][1]) / s;
        }
        else {
            float s = 2.0f * std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]);
            w = (m.m[1][0] - m.m[0][1]) / s;
            x = (m.m[0][2] + m.m[2][0]) / s;
            y = (m.m[1][2] + m.m[2][1]) / s;
            z = 0.25f * s;
        }

        return Quaternion(w, x, y, z);
    }

    // Convert a 4x4 rotation matrix to quaternion (using the upper 3x3 part)
    static Quaternion fromRotationMatrix4x4(const Matrix4x4& m) {
        Matrix3x3 m3(
            m.m[0][0], m.m[0][1], m.m[0][2],
            m.m[1][0], m.m[1][1], m.m[1][2],
            m.m[2][0], m.m[2][1], m.m[2][2]
        );
        return fromRotationMatrix3x3(m3);
    }

    // Rotate a 3D vector
    Vector3D rotateVector(const Vector3D& v) const {
        Quaternion vq(0, v.x, v.y, v.z);
        Quaternion result = (*this) * vq * conjugate();
        return Vector3D(result.x, result.y, result.z);
    }

    // Convert to rotation around an axis
    void toAxisAngle(Vector3D& axis, float& angle) const {
        Quaternion q = normalize();
        angle = 2.0f * std::acos(q.w);
        float s = std::sqrt(1.0f - q.w * q.w);

        if (s < 0.001f) {
            // If s is close to zero, direction doesn't matter
            axis.x = q.x;
            axis.y = q.y;
            axis.z = q.z;
        }
        else {
            // Normalize the axis
            axis.x = q.x / s;
            axis.y = q.y / s;
            axis.z = q.z / s;
        }
    }

    // Create a rotation quaternion that rotates from one vector to another
    static Quaternion fromTwoVectors(const Vector3D& from, const Vector3D& to) {
        Vector3D fromNorm = from.normalize();
        Vector3D toNorm = to.normalize();

        float dot = fromNorm.dot(toNorm);

        // If vectors are nearly parallel
        if (dot > 0.99999f) {
            return Quaternion(1, 0, 0, 0); // Identity quaternion
        }
        // If vectors are nearly opposite
        else if (dot < -0.99999f) {
            // Find an orthogonal vector to 'from'
            Vector3D axis = Vector3D(1, 0, 0).cross(fromNorm);
            if (axis.magnitude() < 0.00001f) {
                axis = Vector3D(0, 1, 0).cross(fromNorm);
            }
            return Quaternion::fromAxisAngle(M_PI, axis.x, axis.y, axis.z);
        }

        Vector3D axis = fromNorm.cross(toNorm).normalize();
        float angle = std::acos(dot);

        return Quaternion::fromAxisAngle(angle, axis.x, axis.y, axis.z);
    }

    // Create a quaternion for a rotation around a pivot point
    static Quaternion createPivotRotation(const Vector3D& pivot, float angle, const Vector3D& axis) {
        // For rotating around a pivot:
        // 1. Translate to origin
        // 2. Rotate
        // 3. Translate back

        // We'll return only the rotation quaternion, the caller will need to handle the translations
        return fromAxisAngle(angle, axis.x, axis.y, axis.z);
    }
};

// Main application class that encapsulates all functionality
class SolarSystemApp {
private:
    // Shader-like structure for OpenGL 3.0+ compatibility
    struct Shader {
        unsigned int ID;

        Shader() : ID(0) {}

        // Load and compile shader from file
        void load(const char* vertexPath, const char* fragmentPath) {
            // In a real application, this would load and compile shaders
            // For this example, we'll use fixed pipeline OpenGL
            ID = 1; // Placeholder ID
        }

        void use() {
            // In a real application, this would use the shader program
            // For this example, we'll use fixed pipeline OpenGL
        }

        void setInt(const std::string& name, int value) const {}
        void setFloat(const std::string& name, float value) const {}
        void setVec3(const std::string& name, const Vector3D& value) const {}
        void setMat4(const std::string& name, const Matrix4x4& mat) const {}
    };

    // Material class
    struct Material {
        std::string name;
        Vector3D ambient;
        Vector3D diffuse;
        Vector3D specular;
        float shininess;

        Material(const std::string& name,
            const Vector3D& ambient,
            const Vector3D& diffuse,
            const Vector3D& specular,
            float shininess)
            : name(name), ambient(ambient), diffuse(diffuse), specular(specular), shininess(shininess) {
        }

        void apply() const {
            // Apply material properties to OpenGL
            GLfloat mat_ambient[] = { ambient.x, ambient.y, ambient.z, 1.0f };
            GLfloat mat_diffuse[] = { diffuse.x, diffuse.y, diffuse.z, 1.0f };
            GLfloat mat_specular[] = { specular.x, specular.y, specular.z, 1.0f };

            glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
            glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
            glMaterialf(GL_FRONT, GL_SHININESS, shininess);

            // Enable color material for better compatibility with explicit color setting
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
        }
    };

    // Light class
    struct Light {
        enum LightType { DIRECTIONAL, POINT, SPOT };

        LightType type;
        std::string name;
        Vector3D position;
        Vector3D direction;
        Vector3D ambient;
        Vector3D diffuse;
        Vector3D specular;

        // Attenuation factors
        float constant;
        float linear;
        float quadratic;

        // Spotlight parameters
        float cutOff;
        float outerCutOff;
        GLenum lightID; // OpenGL light ID (GL_LIGHT0, etc.)

        // Constructor for point light (most common for our simulation)
        Light(GLenum lightID, const std::string& name, const Vector3D& position,
            const Vector3D& ambient, const Vector3D& diffuse, const Vector3D& specular)
            : type(POINT), lightID(lightID), name(name), position(position),
            ambient(ambient), diffuse(diffuse), specular(specular),
            constant(1.0f), linear(0.02f), quadratic(0.005f) {
        } // Reduced attenuation

        void apply() const {
            // Set light properties in OpenGL
            GLfloat light_position[] = { position.x, position.y, position.z, 1.0f };
            GLfloat light_ambient[] = { ambient.x, ambient.y, ambient.z, 1.0f };
            GLfloat light_diffuse[] = { diffuse.x, diffuse.y, diffuse.z, 1.0f };
            GLfloat light_specular[] = { specular.x, specular.y, specular.z, 1.0f };

            glLightfv(lightID, GL_POSITION, light_position);
            glLightfv(lightID, GL_AMBIENT, light_ambient);
            glLightfv(lightID, GL_DIFFUSE, light_diffuse);
            glLightfv(lightID, GL_SPECULAR, light_specular);

            // Set attenuation factors
            glLightf(lightID, GL_CONSTANT_ATTENUATION, constant);
            glLightf(lightID, GL_LINEAR_ATTENUATION, linear);
            glLightf(lightID, GL_QUADRATIC_ATTENUATION, quadratic);

            glEnable(lightID);
        }
    };

    // Planet class
    struct Planet {
        std::string name;
        float orbitRadius;
        float radius;
        float orbitSpeed;
        float rotationSpeed;
        float angle;
        float rotationAngle;
        bool active;
        Vector3D position;
        const Material* material;

        Planet(const std::string& name, const Material* material,
            float orbRadius, float radius, float orbSpeed, float rotSpeed)
            : name(name), material(material),
            orbitRadius(orbRadius), radius(radius),
            orbitSpeed(orbSpeed), rotationSpeed(rotSpeed),
            angle(0.0f), rotationAngle(0.0f), active(true),
            position(Vector3D(0.0f, 0.0f, 0.0f)) {
        }

        void update(float deltaTime) {
            if (!active) return;

            // Update orbit
            angle += orbitSpeed * deltaTime;
            if (angle > 2 * M_PI) angle -= 2 * M_PI;

            // Update rotation
            rotationAngle += rotationSpeed * deltaTime;
            if (rotationAngle > 2 * M_PI) rotationAngle -= 2 * M_PI;

            // Calculate position using quaternion rotation
            Quaternion q = Quaternion::fromAxisAngle(angle, 0.0f, 1.0f, 0.0f);
            Vector3D orbitPos(orbitRadius, 0.0f, 0.0f);
            Vector3D rotatedPos = q.rotateVector(orbitPos);

            position = rotatedPos;
        }

        void drawOrbit() const {
            if (!active) return;

            // Draw orbit as a circle
            glDisable(GL_LIGHTING); // Temporarily disable lighting for the orbit
            glColor3f(0.3f, 0.3f, 0.3f);

            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 100; ++i) {
                float theta = (2.0f * M_PI * i) / 100;
                float x = orbitRadius * std::cos(theta);
                float z = orbitRadius * std::sin(theta);
                glVertex3f(x, 0.0f, z);
            }
            glEnd();

            glEnable(GL_LIGHTING); // Re-enable lighting
        }

        void draw() const {
            if (!active) return;

            // Apply planet's material
            material->apply();

            // Save current matrix
            glPushMatrix();

            // Translate to planet position
            glTranslatef(position.x, position.y, position.z);

            // Apply self-rotation
            // Convert quaternion to axis-angle for OpenGL
            Vector3D axis;
            float angle;
            Quaternion rotQuat = Quaternion::fromAxisAngle(rotationAngle, 0.0f, 1.0f, 0.0f);
            rotQuat.toAxisAngle(axis, angle);
            glRotatef(angle * 180.0f / M_PI, axis.x, axis.y, axis.z);

            // Draw sphere for planet - add color boost to make planets more visible
            glColor3f(material->diffuse.x, material->diffuse.y, material->diffuse.z);
            glutSolidSphere(radius, 32, 32);

            // Restore matrix
            glPopMatrix();
        }

        std::vector<Vector3D> createFragmentPositions(int count) {
            std::vector<Vector3D> fragments;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * M_PI);
            std::uniform_real_distribution<float> elevationDist(0.0f, M_PI);
            std::uniform_real_distribution<float> speedDist(5.0f, 15.0f);

            for (int i = 0; i < count; ++i) {
                float angle1 = angleDist(gen);
                float angle2 = elevationDist(gen);
                float speed = speedDist(gen);

                float vx = speed * std::sin(angle2) * std::cos(angle1);
                float vy = speed * std::cos(angle2);
                float vz = speed * std::sin(angle2) * std::sin(angle1);

                fragments.push_back(position);
            }

            return fragments;
        }
    };

    // Fragment class for supernova debris
    struct Fragment {
        Vector3D position;
        Vector3D velocity;
        float radius;
        const Material* material;

        Fragment(const Vector3D& pos, const Vector3D& vel, float r, const Material* mat)
            : position(pos), velocity(vel), radius(r), material(mat) {
        }

        void update(float deltaTime) {
            position.x += velocity.x * deltaTime;
            position.y += velocity.y * deltaTime;
            position.z += velocity.z * deltaTime;
        }

        void draw() const {
            // Apply fragment's material
            material->apply();

            // Save current matrix
            glPushMatrix();

            // Translate to fragment position
            glTranslatef(position.x, position.y, position.z);

            // Set color explicitly
            glColor3f(material->diffuse.x, material->diffuse.y, material->diffuse.z);

            // Draw sphere for fragment
            glutSolidSphere(radius, 8, 8);

            // Restore matrix
            glPopMatrix();
        }
    };

    // Camera class
    struct Camera {
        Vector3D position;
        Vector3D front;
        Vector3D up;
        Vector3D right;
        Vector3D worldUp;

        float yaw;
        float pitch;
        float zoom;

        float movementSpeed;
        float mouseSensitivity;

        Camera(Vector3D position = Vector3D(0.0f, 20.0f, 40.0f),
            Vector3D up = Vector3D(0.0f, 1.0f, 0.0f),
            float yaw = -90.0f, float pitch = 0.0f)
            : position(position), worldUp(up), yaw(yaw), pitch(pitch), zoom(45.0f),
            movementSpeed(2.5f), mouseSensitivity(0.1f) {
            updateCameraVectors();
        }

        Matrix4x4 getViewMatrix() const {
            return Matrix4x4::createLookAt(position, position + front, up);
        }

        float getZoom() const {
            return zoom;
        }

        void processKeyboard(unsigned char key, float deltaTime) {
            float velocity = movementSpeed * deltaTime;

            if (key == 'w')
                position = position + front * velocity;
            if (key == 's')
                position = position - front * velocity;
            if (key == 'a')
                position = position - right * velocity;
            if (key == 'd')
                position = position + right * velocity;
            if (key == '+')
                position.y += velocity;
            if (key == '-')
                position.y -= velocity;
        }

        void processSpecialKeys(int key, float deltaTime) {
            if (key == GLUT_KEY_UP)
                pitch += mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_DOWN)
                pitch -= mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_LEFT)
                yaw -= mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_RIGHT)
                yaw += mouseSensitivity * 10.0f;

            // Constrain pitch
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;

            updateCameraVectors();
        }

        void processMouseMovement(float xoffset, float yoffset) {
            xoffset *= mouseSensitivity;
            yoffset *= mouseSensitivity;

            yaw += xoffset;
            pitch += yoffset;

            // Constrain pitch
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;

            updateCameraVectors();
        }

        void processMouseScroll(float yoffset) {
            zoom -= yoffset;
            if (zoom < 1.0f) zoom = 1.0f;
            if (zoom > 45.0f) zoom = 45.0f;
        }

    private:
        void updateCameraVectors() {
            // Calculate the new front vector
            Vector3D newFront;
            newFront.x = std::cos(yaw * M_PI / 180.0f) * std::cos(pitch * M_PI / 180.0f);
            newFront.y = std::sin(pitch * M_PI / 180.0f);
            newFront.z = std::sin(yaw * M_PI / 180.0f) * std::cos(pitch * M_PI / 180.0f);
            front = newFront.normalize();

            // Recalculate the right and up vectors
            right = front.cross(worldUp).normalize();
            up = right.cross(front).normalize();
        }
    };

    // Static instance for callbacks
    static SolarSystemApp* instance;

    // Window dimensions
    int width, height;

    // Time variables
    float lastFrame;
    float deltaTime;

    // Camera
    Camera camera;

    // Planets and materials
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<Planet> planets;
    std::vector<Fragment> fragments;

    // Sun properties
    float sunRadius;
    bool sunExpanding;
    bool supernovaTriggered;

    // Mouse tracking
    bool firstMouse;
    float lastX, lastY;

    // Initialize materials
    void initMaterials() {
        // Sun material
        // Sun material - Jaune
        materials.push_back(Material("Sun",
            Vector3D(0.3f, 0.3f, 0.0f),    // Jaune ambiant
            Vector3D(1.0f, 0.9f, 0.2f),    // Jaune vif diffus
            Vector3D(1.0f, 1.0f, 0.5f),    // Jaune clair spéculaire
            32.0f));

        // Mercury material
        materials.push_back(Material("Mercury",
            Vector3D(0.1f, 0.1f, 0.1f),
            Vector3D(0.5f, 0.5f, 0.5f),
            Vector3D(0.7f, 0.7f, 0.7f),
            16.0f));

        // Venus material
        materials.push_back(Material("Venus",
            Vector3D(0.1f, 0.1f, 0.1f),
            Vector3D(0.9f, 0.7f, 0.4f),
            Vector3D(0.8f, 0.8f, 0.8f),
            32.0f));

        // Earth material
        materials.push_back(Material("Earth",
            Vector3D(0.1f, 0.1f, 0.2f),
            Vector3D(0.0f, 0.5f, 1.0f),
            Vector3D(0.5f, 0.5f, 1.0f),
            64.0f));

        // Mars material
        materials.push_back(Material("Mars",
            Vector3D(0.1f, 0.05f, 0.05f),
            Vector3D(1.0f, 0.3f, 0.3f),
            Vector3D(0.7f, 0.5f, 0.5f),
            32.0f));

        // Jupiter material
        materials.push_back(Material("Jupiter",
            Vector3D(0.1f, 0.1f, 0.05f),
            Vector3D(0.9f, 0.6f, 0.4f),
            Vector3D(0.7f, 0.7f, 0.6f),
            16.0f));

        // Saturn material
        materials.push_back(Material("Saturn",
            Vector3D(0.1f, 0.1f, 0.05f),
            Vector3D(0.9f, 0.8f, 0.5f),
            Vector3D(0.7f, 0.7f, 0.6f),
            16.0f));

        // Uranus material
        materials.push_back(Material("Uranus",
            Vector3D(0.05f, 0.1f, 0.1f),
            Vector3D(0.6f, 0.8f, 0.9f),
            Vector3D(0.6f, 0.7f, 0.8f),
            32.0f));

        // Neptune material
        materials.push_back(Material("Neptune",
            Vector3D(0.05f, 0.05f, 0.1f),
            Vector3D(0.3f, 0.5f, 0.8f),
            Vector3D(0.5f, 0.6f, 0.9f),
            32.0f));

        // Fragment material
        materials.push_back(Material("Fragment",
            Vector3D(0.1f, 0.05f, 0.0f),
            Vector3D(1.0f, 0.7f, 0.2f),
            Vector3D(1.0f, 0.8f, 0.4f),
            8.0f));
    }

    // Initialize lights
    void initLights() {
        // Sun light (point light at the center)
        lights.push_back(Light(GL_LIGHT0, "SunLight", Vector3D(0.0f, 0.0f, 0.0f),
            Vector3D(0.3f, 0.3f, 0.3f),    // Increased ambient
            Vector3D(1.0f, 1.0f, 1.0f),    // Full diffuse
            Vector3D(1.0f, 1.0f, 1.0f)));  // Full specular

        // Secondary light to better illuminate the scene
        lights.push_back(Light(GL_LIGHT1, "SceneLight", Vector3D(0.0f, 30.0f, 0.0f),
            Vector3D(0.2f, 0.2f, 0.2f),    // Soft ambient
            Vector3D(0.7f, 0.7f, 0.7f),    // Strong diffuse
            Vector3D(0.5f, 0.5f, 0.5f)));  // Medium specular
    }

    // Initialize planets
    void initPlanets() {
        planets.push_back(Planet("Mercury", &materials[1], 8.0f, 0.5f, 4.0f, 0.02f));
        planets.push_back(Planet("Venus", &materials[2], 11.0f, 1.0f, 2.0f, 0.015f));
        planets.push_back(Planet("Earth", &materials[3], 15.0f, 1.2f, 1.0f, 0.01f));
        planets.push_back(Planet("Mars", &materials[4], 19.0f, 0.7f, 0.6f, 0.008f));
        planets.push_back(Planet("Jupiter", &materials[5], 24.0f, 3.0f, 0.3f, 0.005f));
        planets.push_back(Planet("Saturn", &materials[6], 30.0f, 2.5f, 0.2f, 0.004f));
        planets.push_back(Planet("Uranus", &materials[7], 35.0f, 2.0f, 0.15f, 0.003f));
        planets.push_back(Planet("Neptune", &materials[8], 40.0f, 2.0f, 0.1f, 0.002f));
    }

    // Setup OpenGL state
    void setupOpenGL() {
        // Enable depth testing
        glEnable(GL_DEPTH_TEST);

        // Enable lighting
        glEnable(GL_LIGHTING);

        // Enable normalize (for correct lighting with scaled objects)
        glEnable(GL_NORMALIZE);

        // Add global ambient light
        float globalAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);

        // Setup lights
        for (const Light& light : lights) {
            light.apply();
        }
    }

    // Update scene
    void updateScene() {
        // Update supernova effect
        if (sunExpanding && sunRadius < 30.0f) {
            sunRadius += deltaTime * 10.0f;
            for (Planet& p : planets) {
                if (p.active && p.position.magnitude() <= sunRadius) {
                    // Create fragments
                    std::vector<Vector3D> fragmentPositions = p.createFragmentPositions(50);

                    // Generate random fragments
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * M_PI);
                    std::uniform_real_distribution<float> elevationDist(0.0f, M_PI);
                    std::uniform_real_distribution<float> speedDist(5.0f, 15.0f);

                    for (const Vector3D& pos : fragmentPositions) {
                        float angle1 = angleDist(gen);
                        float angle2 = elevationDist(gen);
                        float speed = speedDist(gen);

                        float vx = speed * std::sin(angle2) * std::cos(angle1);
                        float vy = speed * std::cos(angle2);
                        float vz = speed * std::sin(angle2) * std::sin(angle1);

                        fragments.push_back(Fragment(pos, Vector3D(vx, vy, vz), p.radius * 0.1f, &materials[9]));
                    }

                    p.active = false;
                }
            }
            if (sunRadius >= 30.0f) {
                supernovaTriggered = true;
            }
        }

        // Update planets
        for (Planet& p : planets) {
            p.update(deltaTime);
        }

        // Update fragments
        for (Fragment& f : fragments) {
            f.update(deltaTime);
        }
    }

    // Render scene
    void renderScene() {
        // Clear the screen
        glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set up projection matrix
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(camera.getZoom(), (float)width / (float)height, 0.1f, 100.0f);

        // Set up view matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Apply camera transformations
        Vector3D cameraTarget = camera.position + camera.front;
        gluLookAt(
            camera.position.x, camera.position.y, camera.position.z,
            cameraTarget.x, cameraTarget.y, cameraTarget.z,
            camera.up.x, camera.up.y, camera.up.z
        );

        // Update light positions
        for (const Light& light : lights) {
            light.apply();
        }

        // Draw sun
        materials[0].apply();
        glPushMatrix();
        glColor3f(1.0f, 0.9f, 0.2f);
        glutSolidSphere(sunRadius, 32, 32);
        glPopMatrix();

        // Draw planets
        for (const Planet& p : planets) {
            p.drawOrbit();
            p.draw();
        }

        // Draw fragments
        for (const Fragment& f : fragments) {
            f.draw();
        }

        // Swap buffers
        glutSwapBuffers();
    }

    // Static callback wrappers
    static void displayCallback() {
        instance->display();
    }

    static void reshapeCallback(int width, int height) {
        instance->reshape(width, height);
    }

    static void keyboardCallback(unsigned char key, int x, int y) {
        instance->keyboard(key, x, y);
    }

    static void specialCallback(int key, int x, int y) {
        instance->special(key, x, y);
    }

    static void mouseCallback(int button, int state, int x, int y) {
        instance->mouse(button, state, x, y);
    }

    static void motionCallback(int x, int y) {
        instance->motion(x, y);
    }

    static void idleCallback() {
        instance->idle();
    }

    // Member callback implementations
    void display() {
        // Calculate delta time
        float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        // Update and render scene
        updateScene();
        renderScene();
    }

    void reshape(int w, int h) {
        width = w;
        height = h;
        glViewport(0, 0, width, height);
    }

    void keyboard(unsigned char key, int x, int y) {
        if (key == 27)  // ESC key
            exit(0);

        camera.processKeyboard(key, deltaTime);

        if (key == ' ')
            triggerSupernova();
    }

    void special(int key, int x, int y) {
        camera.processSpecialKeys(key, deltaTime);
    }

    void mouse(int button, int state, int x, int y) {
        if (button == 3 || button == 4) { // Mouse wheel
            if (state == GLUT_DOWN) {
                if (button == 3)
                    camera.processMouseScroll(-1);
                else
                    camera.processMouseScroll(1);
            }
        }
    }

    void motion(int x, int y) {
        if (firstMouse) {
            lastX = x;
            lastY = y;
            firstMouse = false;
        }

        float xoffset = x - lastX;
        float yoffset = lastY - y;

        lastX = x;
        lastY = y;

        camera.processMouseMovement(xoffset, yoffset);
    }

    void idle() {
        glutPostRedisplay();
    }

public:
    SolarSystemApp(int argc, char** argv, int windowWidth = 800, int windowHeight = 600)
        : width(windowWidth), height(windowHeight),
        lastFrame(0.0f), deltaTime(0.0f),
        camera(Vector3D(0.0f, 20.0f, 40.0f)),
        sunRadius(3.0f), sunExpanding(false), supernovaTriggered(false),
        firstMouse(true), lastX(windowWidth / 2), lastY(windowHeight / 2) {

        // Set static instance for callbacks
        instance = this;

        // Initialize GLUT
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
        glutInitWindowSize(width, height);
        glutCreateWindow("Solar System - Supernova (with Quaternions)");

        // Initialize GLEW
        GLenum err = glewInit();
        if (err != GLEW_OK) {
            std::cerr << "GLEW initialization error: " << glewGetErrorString(err) << std::endl;
            exit(-1);
        }

        // Print OpenGL version
        std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;

        // Initialize materials and scene
        initMaterials();
        initLights();
        initPlanets();
        setupOpenGL();

        // Register callbacks
        glutDisplayFunc(displayCallback);
        glutReshapeFunc(reshapeCallback);
        glutKeyboardFunc(keyboardCallback);
        glutSpecialFunc(specialCallback);
        glutMouseFunc(mouseCallback);
        glutMotionFunc(motionCallback);
        glutIdleFunc(idleCallback);
    }

    // Trigger supernova effect
    void triggerSupernova() {
        if (!supernovaTriggered) {
            sunExpanding = true;
        }
    }

    // Start the main loop
    void run() {
        glutMainLoop();
    }
};

// Initialize static instance
SolarSystemApp* SolarSystemApp::instance = nullptr;

// Main function
int main(int argc, char** argv) {
    // Create and run application
    SolarSystemApp app(argc, argv);
    app.run();

    return 0;
}
