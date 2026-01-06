#include <iostream>
#include <cmath>

#define MAX_STEPS 16
#define MAX_DISTANCE 10.0f
#define EPSILON 0.01f

#define WIDTH 60
#define HEIGHT 25

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


using namespace std;

class Vector3 {
public:
    float x = 0, y = 0, z = 0;

    Vector3() = default;
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Vector + Vector
    Vector3 operator+(const Vector3& other) const {
        return {
                x + other.x,
                y + other.y,
                z + other.z
        };
    }

    // Vector * scalar
    Vector3 operator*(float scalar) const {
        return {
                x * scalar,
                y * scalar,
                z * scalar
        };
    }

    [[nodiscard]] float length() const {
        return sqrt(x*x + y*y + z*z);
    }

    [[nodiscard]] Vector3 normalize() const {
        float len = length();
        if (len == 0.0f) return *this;
        return { x / len, y / len, z / len };
    }
};

float donutSDF(Vector3 p, float R, float P, float a, float b, float c)
{
    // Precompute trig
    float ca = cos(a), sa = sin(a);
    float cb = cos(b), sb = sin(b);
    float cc = cos(c), sc = sin(c);

    // Rotate point (your exact equations)
    float X =
            (cc * cb) * p.x +
            (sc * cb) * p.y -
            sb * p.z;

    float Y =
            (cc * sb * sa - sc * ca) * p.x +
            (sc * sb * sa + cc * ca) * p.y +
            (cb * sa) * p.z;

    float Z =
            (cc * sb * ca + sc * sa) * p.x +
            (sc * sb * ca - cc * sa) * p.y +
            (cb * ca) * p.z;

    // Torus SDF
    float qx = sqrt(X * X + Y * Y) - R;
    return sqrt(qx * qx + Z * Z) - P;
}

float sceneSDF(Vector3 p, float a, float b, float c)
{
    return donutSDF(p, 1.44, 0.8, a, b, c);
}

Vector3 estimateNormal(Vector3 p, float a, float b, float c)
{
    const float e = 0.001f;

    float dx = sceneSDF({p.x + e, p.y, p.z}, a, b, c) -
               sceneSDF({p.x - e, p.y, p.z}, a, b, c);

    float dy = sceneSDF({p.x, p.y + e, p.z}, a, b, c) -
               sceneSDF({p.x, p.y - e, p.z}, a, b, c);

    float dz = sceneSDF({p.x, p.y, p.z + e}, a, b, c) -
               sceneSDF({p.x, p.y, p.z - e}, a, b, c);

    return Vector3(dx, dy, dz).normalize();
}


char distanceToChar(float d) {
    const char gradient[] = "@#%*+=-:. ";
    int levels = sizeof(gradient) - 1;

    float shadeDistance = 7.5f;
    float t = d / shadeDistance;

    t = min(max(t, 0.0f), 1.0f);
    int index = (int)(t * (levels - 1));
    return gradient[index];
}

char lightToChar(float light)
{
    const char gradient[] = "@#%*+=-:. ";
    int levels = sizeof(gradient) - 1;

    light = std::fmax(0.0f, std::fmin(light, 1.0f));

    // Invert so bright = solid character
    int index = (int)((1.0f - light) * (levels - 1));
    return gradient[index];
}

void lighting(Vector3 point, float a, float b, float c, float l){
    Vector3 normal = estimateNormal(point, a, b, c);

    // Directional light
    //Vector3 lightDir = Vector3(-1, 1, 1).normalize();

    Vector3 lightDir = Vector3(
            cos(l), 1.0f, sin(l)
    ).normalize();

    float ambient = 0.2f;
    float diffuse =
            ambient + (1.0f - ambient) * (
                    normal.x * lightDir.x +
                    normal.y * lightDir.y +
                    normal.z * lightDir.z
            );

    char ch = lightToChar(diffuse);
    cout << ch;
}

void distanceLighting(float distanceTraveled){
    char ch = distanceToChar(distanceTraveled);
    cout << ch;
}

void marchingRay(Vector3 origin, Vector3 direction, float a, float b, float c, float l) {
    float distanceTraveled = 0.0f;

    for (int i = 0; i < MAX_STEPS; i++) {
        Vector3 point = origin + direction * distanceTraveled;
        float distanceToSurface = sceneSDF(point, a ,b ,c);

        // Hit surface
        if (distanceToSurface < EPSILON) {
            //distanceLighting(distanceTraveled);
            lighting(point, a, b, c, l);
            return;
        }

        distanceTraveled += distanceToSurface;

        // Ray escaped
        if (distanceTraveled > MAX_DISTANCE) {
            cout << ' ';
            return;
        }
    }

    // No hit
    cout << ' ';
}

void rasterize(float a, float b, float c, float l)
{
    Vector3 cameraPos(-4, 0, 0);

    float aspect = (float)WIDTH / (float)HEIGHT;
    float fov = 1.45f; // controls zoom

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {

            // Normalized device coordinates (-1 to 1)
            float charAspect = 0.5f; // terminal pixels are tall
            float u = (2.0f * x / WIDTH - 1.0f) * aspect * charAspect;
            float v = 1.0f - 2.0f * y / HEIGHT;

            // Ray direction
            Vector3 rayDir(fov, v, u);
            rayDir = rayDir.normalize();

            marchingRay(cameraPos, rayDir, a, b, c, l);
        }
        cout << '\n';
    }
}


void completeClearScreen() {
    cout << "\033[2J\033[H";
}

void clearScreen() {
    cout << "\033[H";
}

void hideCursor() {
    cout << "\033[?25l";
}

void showCursor() {
    cout << "\033[?25h";
}

void animate()
{
    hideCursor();

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float l = 1.9f;

    float stepA = 0.08f;
    float stepB = 0.06f;
    float stepC = 0.05f;
    float stepL = 0.01f;

    while (true) {
        clearScreen();

        rasterize(a, b, c, l);

        a += stepA;
        b += stepB;
        c += stepC;
        l += stepL;

        if (a > 2 * M_PI) a -= 2 * M_PI;
        if (b > 2 * M_PI) b -= 2 * M_PI;
        if (c > 2 * M_PI) c -= 2 * M_PI;
        if (l > 2 * M_PI) l -= 2 * M_PI;

        // ~30 FPS
        cout.flush();
#ifdef _WIN32
        Sleep(11);
#else
        usleep(11000);
#endif
    }

    showCursor(); // (won’t reach, but good practice)
}



int main() {

    //rasterize(1.1,1.5,1.1);
    completeClearScreen();
    animate();

    return 0;
}
