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

    Vector3D operator+(const Vector3D& v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
    Vector3D operator-(const Vector3D& v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
    Vector3D operator*(float scalar) const { return Vector3D(x * scalar, y * scalar, z * scalar); }
    float dot(const Vector3D& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3D cross(const Vector3D& v) const {
        return Vector3D(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
        );
    }
    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }
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

    Vector4D operator+(const Vector4D& v) const { return Vector4D(x + v.x, y + v.y, z + v.z, w + v.w); }
    Vector4D operator-(const Vector4D& v) const { return Vector4D(x - v.x, y - v.y, z - v.z, w - v.w); }
    Vector4D operator*(float scalar) const { return Vector4D(x * scalar, y * scalar, z * scalar, w * scalar); }
    float dot(const Vector4D& v) const { return x * v.x + y * v.y + z * v.z + w * v.w; }
    float magnitude() const { return std::sqrt(x * x + y * y + z * z + w * w); }
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
                m[i][j] = (i == j) ? 1.0f : 0.0f;
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
    Vector3D operator*(const Vector3D& v) const {
        return Vector3D(
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }
    float determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    Matrix3x3 transpose() const {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }
    Matrix3x3 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
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
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
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
    Vector4D operator*(const Vector4D& v) const {
        return Vector4D(
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
                m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
        );
    }
    Matrix4x4 transpose() const {
        Matrix4x4 result;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                result.m[i][j] = m[j][i];
        return result;
    }
    static Matrix4x4 createTranslation(float x, float y, float z) {
        Matrix4x4 result;
        result.m[0][3] = x;
        result.m[1][3] = y;
        result.m[2][3] = z;
        return result;
    }
    static Matrix4x4 createScaling(float x, float y, float z) {
        Matrix4x4 result;
        result.m[0][0] = x;
        result.m[1][1] = y;
        result.m[2][2] = z;
        return result;
    }
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

    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
    Quaternion operator*(float scalar) const { return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar); }
    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }
    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    Quaternion normalize() const {
        float n = norm();
        if (n > 0) {
            return Quaternion(w / n, x / n, y / n, z / n);
        }
        return Quaternion(1, 0, 0, 0);
    }
    Quaternion inverse() const {
        float n = norm();
        n = n * n;
        if (n > 0) {
            float invN = 1.0f / n;
            return Quaternion(w * invN, -x * invN, -y * invN, -z * invN);
        }
        return Quaternion(1, 0, 0, 0);
    }
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
    static Quaternion fromRotationMatrix4x4(const Matrix4x4& m) {
        Matrix3x3 m3(
                m.m[0][0], m.m[0][1], m.m[0][2],
                m.m[1][0], m.m[1][1], m.m[1][2],
                m.m[2][0], m.m[2][1], m.m[2][2]
        );
        return fromRotationMatrix3x3(m3);
    }
    Vector3D rotateVector(const Vector3D& v) const {
        Quaternion vq(0, v.x, v.y, v.z);
        Quaternion result = (*this) * vq * conjugate();
        return Vector3D(result.x, result.y, result.z);
    }
    void toAxisAngle(Vector3D& axis, float& angle) const {
        Quaternion q = normalize();
        angle = 2.0f * std::acos(q.w);
        float s = std::sqrt(1.0f - q.w * q.w);
        if (s < 0.001f) {
            axis.x = q.x;
            axis.y = q.y;
            axis.z = q.z;
        } else {
            axis.x = q.x / s;
            axis.y = q.y / s;
            axis.z = q.z / s;
        }
    }
    static Quaternion fromTwoVectors(const Vector3D& from, const Vector3D& to) {
        Vector3D fromNorm = from.normalize();
        Vector3D toNorm = to.normalize();
        float dot = fromNorm.dot(toNorm);
        if (dot > 0.99999f) return Quaternion(1, 0, 0, 0);
        else if (dot < -0.99999f) {
            Vector3D axis = Vector3D(1, 0, 0).cross(fromNorm);
            if (axis.magnitude() < 0.00001f) axis = Vector3D(0, 1, 0).cross(fromNorm);
            return Quaternion::fromAxisAngle(M_PI, axis.x, axis.y, axis.z);
        }
        Vector3D axis = fromNorm.cross(toNorm).normalize();
        float angle = std::acos(dot);
        return Quaternion::fromAxisAngle(angle, axis.x, axis.y, axis.z);
    }
    static Quaternion createPivotRotation(const Vector3D& pivot, float angle, const Vector3D& axis) {
        return fromAxisAngle(angle, axis.x, axis.y, axis.z);
    }
};

// Main application class that encapsulates all functionality
class SolarSystemApp {
private:
    struct Shader {
        unsigned int ID;
        Shader() : ID(0) {}
        void load(const char* vertexPath, const char* fragmentPath) { ID = 1; }
        void use() {}
        void setInt(const std::string& name, int value) const {}
        void setFloat(const std::string& name, float value) const {}
        void setVec3(const std::string& name, const Vector3D& value) const {}
        void setMat4(const std::string& name, const Matrix4x4& mat) const {}
    };

    struct Material {
        std::string name;
        Vector3D ambient;
        Vector3D diffuse;
        Vector3D specular;
        float shininess;
        Material(const std::string& name, const Vector3D& ambient, const Vector3D& diffuse, const Vector3D& specular, float shininess)
                : name(name), ambient(ambient), diffuse(diffuse), specular(specular), shininess(shininess) {}
        void apply() const {
            GLfloat mat_ambient[] = { ambient.x, ambient.y, ambient.z, 1.0f };
            GLfloat mat_diffuse[] = { diffuse.x, diffuse.y, diffuse.z, 1.0f };
            GLfloat mat_specular[] = { specular.x, specular.y, specular.z, 1.0f };
            glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
            glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
            glMaterialf(GL_FRONT, GL_SHININESS, shininess);
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
        }
    };

    struct Light {
        enum LightType { DIRECTIONAL, POINT, SPOT };
        LightType type;
        std::string name;
        Vector3D position;
        Vector3D direction;
        Vector3D ambient;
        Vector3D diffuse;
        Vector3D specular;
        float constant;
        float linear;
        float quadratic;
        float cutOff;
        float outerCutOff;
        GLenum lightID;

        Light(GLenum lightID, const std::string& name, const Vector3D& position, const Vector3D& ambient, const Vector3D& diffuse, const Vector3D& specular)
                : type(POINT), lightID(lightID), name(name), position(position),
                  ambient(ambient), diffuse(diffuse), specular(specular),
                  constant(1.0f), linear(0.02f), quadratic(0.005f) {}
        void apply() const {
            GLfloat light_position[] = { position.x, position.y, position.z, 1.0f };
            GLfloat light_ambient[] = { ambient.x, ambient.y, ambient.z, 1.0f };
            GLfloat light_diffuse[] = { diffuse.x, diffuse.y, diffuse.z, 1.0f };
            GLfloat light_specular[] = { specular.x, specular.y, specular.z, 1.0f };
            glLightfv(lightID, GL_POSITION, light_position);
            glLightfv(lightID, GL_AMBIENT, light_ambient);
            glLightfv(lightID, GL_DIFFUSE, light_diffuse);
            glLightfv(lightID, GL_SPECULAR, light_specular);
            glLightf(lightID, GL_CONSTANT_ATTENUATION, constant);
            glLightf(lightID, GL_LINEAR_ATTENUATION, linear);
            glLightf(lightID, GL_QUADRATIC_ATTENUATION, quadratic);
            glEnable(lightID);
        }
    };

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
        Vector3D orbitAxis;
        const Material* material;
        std::vector<Planet> moons;

        Planet(const std::string& name, const Material* material, float orbRadius, float radius, float orbSpeed, float rotSpeed, Vector3D orbitAxis = Vector3D(0,1,0))
                : name(name), material(material),
                  orbitRadius(orbRadius), radius(radius),
                  orbitSpeed(orbSpeed), rotationSpeed(rotSpeed),
                  angle(0.0f), rotationAngle(0.0f), active(true),
                  position(Vector3D(0.0f, 0.0f, 0.0f)), orbitAxis(orbitAxis) {}

        void update(float deltaTime) {
            if (!active) return;
            angle += orbitSpeed * deltaTime;
            if (angle > 2 * M_PI) angle -= 2 * M_PI;
            rotationAngle += rotationSpeed * deltaTime;
            if (rotationAngle > 2 * M_PI) rotationAngle -= 2 * M_PI;
            Quaternion q = Quaternion::fromAxisAngle(angle, orbitAxis.x, orbitAxis.y, orbitAxis.z);
            Vector3D orbitPos(orbitRadius, 0.0f, 0.0f);
            Vector3D rotatedPos = q.rotateVector(orbitPos);
            position = rotatedPos;
            for(auto& moon : moons) {
                moon.update(deltaTime);
                moon.position = position + moon.position;
            }
        }
        // void drawOrbit() const {} // Orbite désactivée
        void draw() const {
            if (!active) return;
            material->apply();
            glPushMatrix();
            glTranslatef(position.x, position.y, position.z);
            Vector3D axis;
            float angleRad;
            Quaternion rotQuat = Quaternion::fromAxisAngle(rotationAngle, 0.0f, 1.0f, 0.0f);
            rotQuat.toAxisAngle(axis, angleRad);
            glRotatef(angleRad * 180.0f / M_PI, axis.x, axis.y, axis.z);
            glColor3f(material->diffuse.x, material->diffuse.y, material->diffuse.z);
            glutSolidSphere(radius, 32, 32);
            for (const auto& moon : moons) moon.draw();
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

    struct Fragment {
        Vector3D position;
        Vector3D velocity;
        float radius;
        const Material* material;
        Fragment(const Vector3D& pos, const Vector3D& vel, float r, const Material* mat)
                : position(pos), velocity(vel), radius(r), material(mat) {}
        void update(float deltaTime) {
            position.x += velocity.x * deltaTime;
            position.y += velocity.y * deltaTime;
            position.z += velocity.z * deltaTime;
        }
        void draw() const {
            material->apply();
            glPushMatrix();
            glTranslatef(position.x, position.y, position.z);
            glColor3f(material->diffuse.x, material->diffuse.y, material->diffuse.z);
            glutSolidSphere(radius, 8, 8);
            glPopMatrix();
        }
    };

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
                  movementSpeed(2.5f), mouseSensitivity(0.1f) { updateCameraVectors(); }
        Matrix4x4 getViewMatrix() const {
            return Matrix4x4::createLookAt(position, position + front, up);
        }
        float getZoom() const { return zoom; }
        void processKeyboard(unsigned char key, float deltaTime) {
            float velocity = movementSpeed * deltaTime;
            if (key == 'w') position = position + front * velocity;
            if (key == 's') position = position - front * velocity;
            if (key == 'a') position = position - right * velocity;
            if (key == 'd') position = position + right * velocity;
            if (key == '+') position.y += velocity;
            if (key == '-') position.y -= velocity;
        }
        void processSpecialKeys(int key, float deltaTime) {
            if (key == GLUT_KEY_UP) pitch += mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_DOWN) pitch -= mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_LEFT) yaw -= mouseSensitivity * 10.0f;
            if (key == GLUT_KEY_RIGHT) yaw += mouseSensitivity * 10.0f;
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;
            updateCameraVectors();
        }
        void processMouseMovement(float xoffset, float yoffset) {
            xoffset *= mouseSensitivity;
            yoffset *= mouseSensitivity;
            yaw += xoffset;
            pitch += yoffset;
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
            Vector3D newFront;
            newFront.x = std::cos(yaw * M_PI / 180.0f) * std::cos(pitch * M_PI / 180.0f);
            newFront.y = std::sin(pitch * M_PI / 180.0f);
            newFront.z = std::sin(yaw * M_PI / 180.0f) * std::cos(pitch * M_PI / 180.0f);
            front = newFront.normalize();
            right = front.cross(worldUp).normalize();
            up = right.cross(front).normalize();
        }
    };

    static SolarSystemApp* instance;
    int width, height;
    float lastFrame;
    float deltaTime;
    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<Planet> planets;
    std::vector<Fragment> fragments;
    float sunRadius;
    bool sunExpanding;
    bool supernovaTriggered;
    bool firstMouse;
    float lastX, lastY;

    void initMaterials() {
        materials.push_back(Material("Sun", Vector3D(0.3f, 0.3f, 0.0f), Vector3D(1.0f, 0.9f, 0.2f), Vector3D(1.0f, 1.0f, 0.5f), 32.0f));
        materials.push_back(Material("Mercury", Vector3D(0.1f, 0.1f, 0.1f), Vector3D(0.5f, 0.5f, 0.5f), Vector3D(0.7f, 0.7f, 0.7f), 16.0f));
        materials.push_back(Material("Venus", Vector3D(0.1f, 0.1f, 0.1f), Vector3D(0.9f, 0.7f, 0.4f), Vector3D(0.8f, 0.8f, 0.8f), 32.0f));
        materials.push_back(Material("Earth", Vector3D(0.1f, 0.1f, 0.2f), Vector3D(0.0f, 0.5f, 1.0f), Vector3D(0.5f, 0.5f, 1.0f), 64.0f));
        materials.push_back(Material("Mars", Vector3D(0.1f, 0.05f, 0.05f), Vector3D(1.0f, 0.3f, 0.3f), Vector3D(0.7f, 0.5f, 0.5f), 32.0f));
        materials.push_back(Material("Jupiter", Vector3D(0.1f, 0.1f, 0.05f), Vector3D(0.9f, 0.6f, 0.4f), Vector3D(0.7f, 0.7f, 0.6f), 16.0f));
        materials.push_back(Material("Saturn", Vector3D(0.1f, 0.1f, 0.05f), Vector3D(0.9f, 0.8f, 0.5f), Vector3D(0.7f, 0.7f, 0.6f), 16.0f));
        materials.push_back(Material("Uranus", Vector3D(0.05f, 0.1f, 0.1f), Vector3D(0.6f, 0.8f, 0.9f), Vector3D(0.6f, 0.7f, 0.8f), 32.0f));
        materials.push_back(Material("Neptune", Vector3D(0.05f, 0.05f, 0.1f), Vector3D(0.3f, 0.5f, 0.8f), Vector3D(0.5f, 0.6f, 0.9f), 32.0f));
        materials.push_back(Material("Fragment", Vector3D(0.1f, 0.05f, 0.0f), Vector3D(1.0f, 0.7f, 0.2f), Vector3D(1.0f, 0.8f, 0.4f), 8.0f));
        materials.push_back(Material("Moon", Vector3D(0.2f, 0.2f, 0.2f), Vector3D(0.6f, 0.6f, 0.6f), Vector3D(0.8f, 0.8f, 0.8f), 16.0f));
    }
    void initLights() {
        lights.push_back(Light(GL_LIGHT0, "SunLight", Vector3D(0.0f, 0.0f, 0.0f), Vector3D(0.3f, 0.3f, 0.3f), Vector3D(1.0f, 1.0f, 1.0f), Vector3D(1.0f, 1.0f, 1.0f)));
        lights.push_back(Light(GL_LIGHT1, "SceneLight", Vector3D(0.0f, 30.0f, 0.0f), Vector3D(0.2f, 0.2f, 0.2f), Vector3D(0.7f, 0.7f, 0.7f), Vector3D(0.5f, 0.5f, 0.5f)));
    }
    void initPlanets() {
        planets.push_back(Planet("Mercury", &materials[1], 8.0f, 0.5f, 4.0f, 0.02f));
        planets.push_back(Planet("Venus", &materials[2], 11.0f, 1.0f, 2.0f, 0.015f));
        Planet earth("Earth", &materials[3], 15.0f, 1.2f, 1.0f, 0.01f);
        Planet moon("Moon", &materials[10], 2.0f, 0.3f, 4.0f, 0.04f); // Axe d'orbite par défaut (autour de la Terre)
        earth.moons.push_back(moon);
        planets.push_back(earth);
        planets.push_back(Planet("Mars", &materials[4], 19.0f, 0.7f, 0.6f, 0.008f));
        planets.push_back(Planet("Jupiter", &materials[5], 24.0f, 3.0f, 0.3f, 0.005f));
        planets.push_back(Planet("Saturn", &materials[6], 30.0f, 2.5f, 0.2f, 0.004f));
        planets.push_back(Planet("Uranus", &materials[7], 35.0f, 2.0f, 0.15f, 0.003f));
        planets.push_back(Planet("Neptune", &materials[8], 40.0f, 2.0f, 0.1f, 0.002f));
    }
    void setupOpenGL() {
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_NORMALIZE);
        float globalAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
        for (const Light& light : lights) light.apply();
    }
    void updateScene() {
        if (sunExpanding && sunRadius < 30.0f) {
            sunRadius += deltaTime * 10.0f;
            for (Planet& p : planets) {
                if (p.active && p.position.magnitude() <= sunRadius) {
                    std::vector<Vector3D> fragmentPositions = p.createFragmentPositions(50);
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
            if (sunRadius >= 30.0f) supernovaTriggered = true;
        }
        for (Planet& p : planets) { p.update(deltaTime); }
        for (Fragment& f : fragments) { f.update(deltaTime); }
    }
    void renderScene() {
        glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(camera.getZoom(), (float)width / (float)height, 0.1f, 100.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        Vector3D cameraTarget = camera.position + camera.front;
        gluLookAt(camera.position.x, camera.position.y, camera.position.z, cameraTarget.x, cameraTarget.y, cameraTarget.z, camera.up.x, camera.up.y, camera.up.z);
        for (const Light& light : lights) { light.apply(); }
        materials[0].apply();
        glPushMatrix();
        glColor3f(1.0f, 0.9f, 0.2f);
        glutSolidSphere(sunRadius, 32, 32);
        glPopMatrix();
        for (const Planet& p : planets) { p.draw(); }
        for (const Fragment& f : fragments) { f.draw(); }
        glutSwapBuffers();
    }
    static void displayCallback() { instance->display(); }
    static void reshapeCallback(int width, int height) { instance->reshape(width, height); }
    static void keyboardCallback(unsigned char key, int x, int y) { instance->keyboard(key, x, y); }
    static void specialCallback(int key, int x, int y) { instance->special(key, x, y); }
    static void mouseCallback(int button, int state, int x, int y) { instance->mouse(button, state, x, y); }
    static void motionCallback(int x, int y) { instance->motion(x, y); }
    static void idleCallback() { instance->idle(); }
    void display() {
        float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;
        updateScene();
        renderScene();
    }
    void reshape(int w, int h) { width = w; height = h; glViewport(0, 0, width, height); }
    void keyboard(unsigned char key, int x, int y) {
        if (key == 27) exit(0);
        camera.processKeyboard(key, deltaTime);
        if (key == ' ') triggerSupernova();
    }
    void special(int key, int x, int y) { camera.processSpecialKeys(key, deltaTime); }
    void mouse(int button, int state, int x, int y) {
        if (button == 3 || button == 4) {
            if (state == GLUT_DOWN) {
                if (button == 3) camera.processMouseScroll(-1);
                else camera.processMouseScroll(1);
            }
        }
    }
    void motion(int x, int y) {
        if (firstMouse) { lastX = x; lastY = y; firstMouse = false; }
        float xoffset = x - lastX;
        float yoffset = lastY - y;
        lastX = x; lastY = y;
        camera.processMouseMovement(xoffset, yoffset);
    }
    void idle() { glutPostRedisplay(); }

public:
    SolarSystemApp(int argc, char** argv, int windowWidth = 800, int windowHeight = 600)
            : width(windowWidth), height(windowHeight), lastFrame(0.0f), deltaTime(0.0f),
              camera(Vector3D(0.0f, 20.0f, 40.0f)), sunRadius(3.0f), sunExpanding(false), supernovaTriggered(false),
              firstMouse(true), lastX(windowWidth / 2), lastY(windowHeight / 2) {
        instance = this;
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
        glutInitWindowSize(width, height);
        glutCreateWindow("Solar System - Supernova (with Quaternions and Moon)");
        std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
        initMaterials();
        initLights();
        initPlanets();
        setupOpenGL();
        glutDisplayFunc(displayCallback);
        glutReshapeFunc(reshapeCallback);
        glutKeyboardFunc(keyboardCallback);
        glutSpecialFunc(specialCallback);
        glutMouseFunc(mouseCallback);
        glutMotionFunc(motionCallback);
        glutIdleFunc(idleCallback);
    }
    void triggerSupernova() { if (!supernovaTriggered) { sunExpanding = true; } }
    void run() { glutMainLoop(); }
};
SolarSystemApp* SolarSystemApp::instance = nullptr;
int main(int argc, char** argv) {
    SolarSystemApp app(argc, argv);
    app.run();
    return 0;
}
