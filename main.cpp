#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>

#define _USE_MATH_DEFINES
#include <math.h>

//définition de la structure Quaternion pour la rotation
struct Quaternion {
    float w, x, y, z;

    Quaternion(float w_, float x_, float y_, float z_)
            : w(w_), x(x_), y(y_), z(z_) {}
// Fonction pour créer un quaternion à partir d'un angle et d'un axe
    static Quaternion fromAxisAngle(float angleRad, float axisX, float axisY, float axisZ) {
        float half = angleRad * 0.5f;
        float s = sinf(half);
        return Quaternion(cosf(half), axisX * s, axisY * s, axisZ * s);
    }
// Fonction pour normaliser le quaternion
    Quaternion multiply(const Quaternion& b) const {
        return Quaternion(
                w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w
        );
    }
// Fonction pour appliquer la rotation à un vecteur
    void rotateVector(float vx, float vy, float vz, float& rx, float& ry, float& rz) const {
        Quaternion v(0.0f, vx, vy, vz);
        Quaternion conj(w, -x, -y, -z);
        Quaternion result = this->multiply(v).multiply(conj);
        rx = result.x;
        ry = result.y;
        rz = result.z;
    }
};

// Définition de la structure Matrice pour les transformations
struct Matrice {
    float m[4][4];

    Matrice() {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] = (i == j) ? 1 : 0; // Matrice identité
    }
// Fonction pour appliquer une rotation autour de l'axe Y
    void translate(float tx, float ty, float tz) {
        m[3][0] += tx;
        m[3][1] += ty;
        m[3][2] += tz;
    }
// Fonction pour appliquer une rotation autour de l'axe Y
    void apply(float& x, float& y, float& z) const {
        float tx = m[0][0] * x + m[1][0] * y + m[2][0] * z + m[3][0];
        float ty = m[0][1] * x + m[1][1] * y + m[2][1] * z + m[3][1];
        float tz = m[0][2] * x + m[1][2] * y + m[2][2] * z + m[3][2];
        x = tx;
        y = ty;
        z = tz;
    }
};

struct Fragment {
    float x, y, z;
    float vx, vy, vz;
    float radius;

    Fragment(float x_, float y_, float z_, float vx_, float vy_, float vz_, float r)
            : x(x_), y(y_), z(z_), vx(vx_), vy(vy_), vz(vz_), radius(r) {}

    void update(float deltaTime) {
        x += vx * deltaTime;
        y += vy * deltaTime;
        z += vz * deltaTime;
    }

    void draw() const {
        glPushMatrix();
        glTranslatef(x, y, z);
        glutSolidSphere(radius, 8, 8);
        glPopMatrix();
    }
};

struct Planet {
    std::string name;
    float orbitRadius;
    float radius;
    float orbitSpeed;
    float angle;
    bool active;
    float x, y, z;

    Planet(std::string n, float orbR, float r, float speed)
            : name(n), orbitRadius(orbR), radius(r), orbitSpeed(speed), angle(0.0f), active(true), x(0), y(0), z(0) {}

    void update(float deltaTime) {
        if (!active) return;
        angle += orbitSpeed * deltaTime;
        if (angle > 2 * M_PI) angle -= 2 * M_PI;

        Quaternion q = Quaternion::fromAxisAngle(angle, 0.0f, 1.0f, 0.0f);
        q.rotateVector(orbitRadius, 0.0f, 0.0f, x, y, z);
    }

    void drawOrbit() const {
        if (!active) return;
        glColor3f(0.3f, 0.3f, 0.3f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 100; ++i) {
            float theta = (2.0f * M_PI * i) / 100;
            float ox = orbitRadius * cosf(theta);
            float oz = orbitRadius * sinf(theta);
            glVertex3f(ox, 0.0f, oz);
        }
        glEnd();
    }

    void draw() const {
        if (!active) return;
        glPushMatrix();
        glTranslatef(x, 0.0f, z);
        glutSolidSphere(radius, 32, 32);
        glPopMatrix();
    }

    void createFragments(std::vector<Fragment>& fragments) {
        for (int i = 0; i < 50; ++i) {
            float angle1 = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
            float angle2 = static_cast<float>(rand()) / RAND_MAX * M_PI;
            float speed = 10.0f + static_cast<float>(rand()) / RAND_MAX * 5.0f;
            float vx = speed * sinf(angle2) * cosf(angle1);
            float vy = speed * cosf(angle2);
            float vz = speed * sinf(angle2) * sinf(angle1);

            fragments.emplace_back(x, y, z, vx, vy, vz, radius * 0.1f);
        }
    }
};

struct Moon {
    float orbitRadius;
    float radius;
    float orbitSpeed;
    float angle;
    bool active;
    Matrice transformation;
    float x, y, z;

    Moon(float orbR, float r, float speed)
            : orbitRadius(orbR), radius(r), orbitSpeed(speed), angle(0.0f), active(true), x(0), y(0), z(0) {}

    void update(float deltaTime, float earthX, float earthZ) {
        if (!active) return;
        angle += orbitSpeed * deltaTime;
        if (angle > 2 * M_PI) angle -= 2 * M_PI;

        x = earthX + orbitRadius * cosf(angle);
        z = earthZ + orbitRadius * sinf(angle);
    }

    void draw() const {
        if (!active) return;

        Matrice localTransform = transformation;
        localTransform.translate(x, 0.0f, z);
        glPushMatrix();
        glMultMatrixf(&localTransform.m[0][0]);
        glutSolidSphere(radius, 16, 16);
        glPopMatrix();
    }
};

std::vector<Planet> planets;
Moon moon(3.0f, 0.5f, 1.5f);
std::vector<Fragment> fragments;
float lastTime = 0;
bool supernovaTriggered = false;
float sunRadius = 3.0f;
bool sunExpanding = false;

float cameraX = 0.0f, cameraY = 20.0f, cameraZ = 40.0f;
float cameraAngleX = 0.0f, cameraAngleY = 0.0f;

void keyboard(unsigned char key, int x, int y) {
    const float moveSpeed = 2.0f;

    switch (key) {
        case 'w': cameraZ -= moveSpeed; break;
        case 's': cameraZ += moveSpeed; break;
        case 'a': cameraX -= moveSpeed; break;
        case 'd': cameraX += moveSpeed; break;
        case '+': cameraY -= moveSpeed; break;
        case '-': cameraY += moveSpeed; break;
        case 27: exit(0);
        case ' ':
            if (!supernovaTriggered) {
                sunExpanding = true;
            }
            break;
    }
    glutPostRedisplay();
}

void specialKeys(int key, int x, int y) {
    const float angleSpeed = 2.0f;

    switch (key) {
        case GLUT_KEY_UP: cameraAngleX -= angleSpeed; break;
        case GLUT_KEY_DOWN: cameraAngleX += angleSpeed; break;
        case GLUT_KEY_LEFT: cameraAngleY -= angleSpeed; break;
        case GLUT_KEY_RIGHT: cameraAngleY += angleSpeed; break;
    }
    glutPostRedisplay();
}

void display() {
    float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glRotatef(cameraAngleX, 1, 0, 0);
    glRotatef(cameraAngleY, 0, 1, 0);
    // Positionnement de la caméra
    gluLookAt(cameraX, cameraY, cameraZ, 0, 0, 0, 0, 1, 0);

    GLfloat light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    // Positionnement de la lumière
    glDisable(GL_LIGHTING);
    if (sunExpanding && sunRadius < 30.0f) {
        sunRadius += deltaTime * 10.0f;
        for (Planet& p : planets) {
            if (p.active && sqrtf(p.x * p.x + p.z * p.z) <= sunRadius) {
                p.createFragments(fragments);
                p.active = false;
            }
        }
        if (sunRadius >= 30.0f) supernovaTriggered = true;
    }

    glColor3f(1.0f, 0.6f, 0.0f);
    glutSolidSphere(sunRadius, 32, 32);
    glEnable(GL_LIGHTING);

    for (Planet& p : planets) {
        p.update(deltaTime);
        glDisable(GL_LIGHTING);
        p.drawOrbit();
        glEnable(GL_LIGHTING);

        GLfloat mat_diffuse[4];
        if (p.name == "Mercury") mat_diffuse[0] = 0.5f, mat_diffuse[1] = 0.5f, mat_diffuse[2] = 0.5f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Venus") mat_diffuse[0] = 0.9f, mat_diffuse[1] = 0.7f, mat_diffuse[2] = 0.4f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Earth") {
            mat_diffuse[0] = 0.0f, mat_diffuse[1] = 0.5f, mat_diffuse[2] = 1.0f, mat_diffuse[3] = 1.0f;
            p.draw();

            moon.update(deltaTime, p.x, p.z);
            moon.draw();
        }
        else if (p.name == "Mars") mat_diffuse[0] = 1.0f, mat_diffuse[1] = 0.3f, mat_diffuse[2] = 0.3f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Jupiter") mat_diffuse[0] = 0.9f, mat_diffuse[1] = 0.6f, mat_diffuse[2] = 0.4f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Saturn") mat_diffuse[0] = 0.9f, mat_diffuse[1] = 0.8f, mat_diffuse[2] = 0.5f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Uranus") mat_diffuse[0] = 0.6f, mat_diffuse[1] = 0.8f, mat_diffuse[2] = 0.9f, mat_diffuse[3] = 1.0f;
        else if (p.name == "Neptune") mat_diffuse[0] = 0.3f, mat_diffuse[1] = 0.5f, mat_diffuse[2] = 0.8f, mat_diffuse[3] = 1.0f;
        else continue;

        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
        p.draw();
    }

    for (Fragment& f : fragments) {
        f.update(deltaTime);
        glColor3f(1.0f, 0.7f, 0.2f);
        f.draw();
    }

    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / h, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Solar System - Supernova Version");
    glEnable(GL_DEPTH_TEST);

    planets.emplace_back("Mercury", 12.0f, 0.5f, 4.0f); // Augmenté
    planets.emplace_back("Venus", 15.0f, 1.0f, 2.0f);    // Augmenté
    planets.emplace_back("Earth", 18.0f, 2.0f, 1.0f);    // Augmenté
    planets.emplace_back("Mars", 22.0f, 1.7f, 0.6f);     // Augmenté
    planets.emplace_back("Jupiter", 28.0f, 3.0f, 0.3f);  // Augmenté
    planets.emplace_back("Saturn", 35.0f, 2.5f, 0.2f);   // Augmenté
    planets.emplace_back("Uranus", 42.0f, 2.0f, 0.15f);  // Augmenté
    planets.emplace_back("Neptune", 50.0f, 2.0f, 0.1f);   // Augmenté

    Moon moon(5.0f, 0.5f, 1.5f); // Augmenté l'orbite de la Lune

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutMainLoop();
    return 0;
}