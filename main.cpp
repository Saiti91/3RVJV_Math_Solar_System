#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

template<typename T>
T clamp(T v, T lo, T hi) { return std::min(std::max(v, lo), hi); }

class Vector3D {
public:
    float x, y, z;
    Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vector3D operator+(const Vector3D& v) const { return { x + v.x, y + v.y, z + v.z }; }
    Vector3D operator-(const Vector3D& v) const { return { x - v.x, y - v.y, z - v.z }; }
    Vector3D operator*(float s) const { return { x * s, y * s, z * s }; }
    float dot(const Vector3D& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3D cross(const Vector3D& v) const {
        return { y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x };
    }
    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3D normalize() const {
        float m = magnitude();
        return (m > 1e-6f) ? Vector3D(x / m, y / m, z / m) : *this;
    }
};

class Quaternion {
public:
    float w, x, y, z;
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    Quaternion normalize() const {
        float n = std::sqrt(w * w + x * x + y * y + z * z);
        return (n > 1e-6f) ? Quaternion(w / n, x / n, y / n, z / n) : Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }
    static Quaternion fromAxisAngle(float angle, float ax, float ay, float az) {
        Vector3D A(ax, ay, az);
        A = A.normalize();
        float ha = angle * 0.5f;
        float s = std::sin(ha);
        return Quaternion(std::cos(ha), A.x * s, A.y * s, A.z * s).normalize();
    }
    Vector3D rotateVector(const Vector3D& v) const {
        Quaternion qn = this->normalize();
        Quaternion p(0.0f, v.x, v.y, v.z);
        Quaternion r = qn * p * qn.conjugate();
        return { r.x, r.y, r.z };
    }
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
    Quaternion conjugate() const { return { w, -x, -y, -z }; }
};

class Camera {
public:
    Vector3D pos, front, up, right, worldUp;
    float yaw, pitch, zoom;
    float speed, sens;
    Camera()
            : pos(0.0f, 20.0f, 40.0f), worldUp(0.0f, 1.0f, 0.0f), yaw(-90.0f), pitch(0.0f), zoom(45.0f), speed(5.0f), sens(0.1f) {
        update();
    }
    void update() {
        front = {
                std::cos(yaw * float(M_PI) / 180.0f) * std::cos(pitch * float(M_PI) / 180.0f),
                std::sin(pitch * float(M_PI) / 180.0f),
                std::sin(yaw * float(M_PI) / 180.0f) * std::cos(pitch * float(M_PI) / 180.0f)
        };
        front = front.normalize();
        if (front.magnitude() < 1e-6f) front = Vector3D(0.0f, 0.0f, -1.0f);
        right = front.cross(worldUp).normalize();
        up = right.cross(front).normalize();
    }
    void move(char key, float dt) {
        Vector3D v = front * (speed * dt);
        if (key == 'w') pos = pos + v;
        if (key == 's') pos = pos - v;
        if (key == 'a') pos = pos - right * (speed * dt);
        if (key == 'd') pos = pos + right * (speed * dt);
        if (key == '+') pos = pos + worldUp * (speed * dt);
        if (key == '-') pos = pos - worldUp * (speed * dt);
    }
    void rotate(float dx, float dy) {
        yaw += dx * sens;
        pitch += dy * sens;
        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
        update();
    }
    void zoomIn(float dy) { zoom = clamp(zoom - dy, 1.0f, 45.0f); }
};

class Planet {
public:
    std::string name;
    Vector3D orbitAxis;
    float orbitRadius, radius, orbitSpeed, angle;
    Vector3D position;
    Planet(std::string n, float orbR, float r, float oS, Vector3D axis = Vector3D(0.0f, 1.0f, 0.0f))
            : name(n), orbitRadius(orbR), radius(r), orbitSpeed(oS), angle(0.0f),
              orbitAxis(axis.normalize()), position(orbR, 0.0f, 0.0f) {}
    void update(float dt) {
        angle += orbitSpeed * dt;
        if (angle > 2 * float(M_PI)) angle -= 2 * float(M_PI);
        position = Quaternion::fromAxisAngle(angle, orbitAxis.x, orbitAxis.y, orbitAxis.z).rotateVector(Vector3D(orbitRadius, 0.0f, 0.0f));
    }
};

class SolarSystemApp {
public:
    static SolarSystemApp* instance;
    Camera camera;
    float lastF, dt;
    int w, h;
    std::vector<Planet> planets;
    bool firstMouse;
    float lastX, lastY;

    SolarSystemApp(int argc, char** argv)
            : lastF(0.0f), dt(0.0f), w(800), h(600), firstMouse(true), lastX(0.0f), lastY(0.0f) {
        planets.emplace_back("Mercury", 8.0f, 0.5f, 4.0f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Venus", 11.0f, 1.0f, 2.0f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Earth", 15.0f, 1.2f, 1.0f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Mars", 19.0f, 0.7f, 0.6f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Jupiter", 24.0f, 3.0f, 0.3f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Saturn", 30.0f, 2.5f, 0.2f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Uranus", 35.0f, 2.0f, 0.15f, Vector3D(0.0f, 1.0f, 0.0f));
        planets.emplace_back("Neptune", 40.0f, 2.0f, 0.1f, Vector3D(0.0f, 0.0f, 1.0f));

        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
        glutInitWindowSize(w, h);
        glutCreateWindow("Système Solaire (axe arbitraire)");

        GLenum err = glewInit();
        if (err != GLEW_OK) {
            std::cerr << "GLEW error: " << glewGetErrorString(err) << std::endl;
            exit(-1);
        }

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_NORMALIZE);
        glEnable(GL_LIGHT0);
        float amb[4] = { 0.2f,0.2f,0.2f,1.0f };
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

        instance = this;

        glutDisplayFunc([]() {
            SolarSystemApp& A = *SolarSystemApp::instance;
            float now = glutGet(GLUT_ELAPSED_TIME) * 0.001f;
            A.dt = now - A.lastF;
            A.lastF = now;
            for (auto& p : A.planets) p.update(A.dt);

            glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(A.camera.zoom, (float)A.w / (float)A.h, 0.1f, 100.0f);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            Vector3D eye = A.camera.pos;
            Vector3D ctr = A.camera.pos + A.camera.front;
            Vector3D up = A.camera.up;
            gluLookAt(eye.x, eye.y, eye.z, ctr.x, ctr.y, ctr.z, up.x, up.y, up.z);

            glColor3f(1.0f, 1.0f, 0.2f);
            glutSolidSphere(3.0f, 32, 32);

            for (auto& p : A.planets) {
                glPushMatrix();
                glTranslatef(p.position.x, p.position.y, p.position.z);
                glColor3f(1.0f, 1.0f, 1.0f);
                glutSolidSphere(p.radius, 16, 16);
                glPopMatrix();
            }
            glutSwapBuffers();
        });

        glutReshapeFunc([](int W, int H) {
            SolarSystemApp::instance->w = W;
            SolarSystemApp::instance->h = H;
            glViewport(0, 0, W, H);
        });

        glutKeyboardFunc([](unsigned char key, int x, int y) {
            if (key == 27) exit(0);
            SolarSystemApp::instance->camera.move(key, SolarSystemApp::instance->dt);
        });

        glutMotionFunc([](int x, int y) {
            SolarSystemApp& A = *SolarSystemApp::instance;
            if (A.firstMouse) {
                A.lastX = float(x); A.lastY = float(y);
                A.firstMouse = false;
            }
            float dx = float(x) - A.lastX, dy = A.lastY - float(y);
            A.camera.rotate(dx, dy);
            A.lastX = float(x); A.lastY = float(y);
        });

        glutMouseFunc([](int b, int s, int x, int y) {
            if ((b == 3 || b == 4) && s == GLUT_DOWN) {
                SolarSystemApp::instance->camera.zoomIn((b == 3) ? -1.0f : 1.0f);
            }
        });

        glutIdleFunc([]() { glutPostRedisplay(); });
    }

    void run() { glutMainLoop(); }
};

SolarSystemApp* SolarSystemApp::instance = nullptr;

int main(int argc, char** argv) {
    SolarSystemApp app(argc, argv);
    app.run();
    return 0;
}
