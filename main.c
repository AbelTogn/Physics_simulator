
#include "raylib.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define GRAVITY 9.81f

#define PLATFORM_HALF_X 12.0f
#define PLATFORM_HALF_Z 12.0f
#define PLATFORM_THICKNESS 0.4f

#define MAX_VERTICES 128
#define MAX_TRIANGLES 256
#define MAX_EDGES 384
#define MAX_SOURCES 32

#define ELECTRIC_SCALE 18.0f
#define MAGNETIC_SCALE 5.0f

#define SIM_SUBSTEPS 8
#define CONTACT_PASSES 4
#define SLEEP_SPEED 0.025f

typedef struct {
    Vector3 vertices[MAX_VERTICES];
    int triangles[MAX_TRIANGLES][3];
    int edges[MAX_EDGES][2];
    int vertexCount;
    int triangleCount;
    int edgeCount;
    float radius;
} Shape;

typedef struct {
    Vector3 position;
    Vector3 velocity;
    Vector3 angularVelocity;
    Quaternion rotation;

    float mass;
    float charge;
    float restitution;
    float friction;

    int shapeType;
    Color color;
} Body;

typedef struct {
    Vector3 position;
    float charge;
    Vector3 magneticMoment;
    bool magnetic;
} FieldSource;

#define MAX_SCENE_OBJECTS 64

typedef enum {
    SCENE_WALL = 0,
    SCENE_RAMP = 1
} SceneObjectType;

typedef struct {
    SceneObjectType type;
    Vector3 position; /* centre du mur ou de la pente */
    float angle;      /* angle de la pente, en radians */
    Vector3 size;     /* dimensions propres à cet objet */
    float friction;
    float restitution;
    Color color;
} SceneObject;

typedef struct {
    Vector3 pivot;
    float length;
    float angle;
    float angularVelocity;
    float damping;
    float bobRadius;
    float mass;
    float restitution;
    Color color;
} Pendulum;

static SceneObject sceneObjects[MAX_SCENE_OBJECTS];
static int sceneObjectCount = 0;
static Pendulum pendulums[MAX_SCENE_OBJECTS];
static int pendulumCount = 0;

/* placementMode : 0 aucun, 1 charge +, -1 charge -, 2 magnetique,
   3 mur, 4 pente, 5 pendule */
static Shape shape;
static Body body;
static FieldSource sources[MAX_SOURCES];
static int sourceCount = 0;

static int placementMode = 0;

static bool objectPlacementMode = false;
static bool objectPlacementValid = false;
static Vector3 objectPlacementPosition = {0};
static float objectPlacementHeight = 0.02f;

static Camera3D camera = {0};

static bool paused = false;
static bool showHelp = true;
static bool showFields = true;
static bool wireframe = false;

static const float platformTop = 0.0f;


static Vector3 V(float x, float y, float z)
{
    return (Vector3){x, y, z};
}

static Vector3 add(Vector3 a, Vector3 b)
{
    return Vector3Add(a, b);
}

static Vector3 sub(Vector3 a, Vector3 b)
{
    return Vector3Subtract(a, b);
}

static Vector3 mul(Vector3 a, float s)
{
    return Vector3Scale(a, s);
}

static float dot(Vector3 a, Vector3 b)
{
    return Vector3DotProduct(a, b);
}

static Vector3 cross(Vector3 a, Vector3 b)
{
    return Vector3CrossProduct(a, b);
}

static float len(Vector3 v)
{
    return Vector3Length(v);
}

static Vector3 norm(Vector3 v)
{
    float l = len(v);

    if (l < 0.000001f)
        return V(0, 0, 0);

    return mul(v, 1.0f / l);
}

static float clampf(float x, float minimum, float maximum)
{
    if (x < minimum) return minimum;
    if (x > maximum) return maximum;
    return x;
}

static Vector3 rotateLocal(Vector3 v)
{
    return Vector3RotateByQuaternion(v, body.rotation);
}

static Vector3 worldVertex(int i)
{
    return add(body.position, rotateLocal(shape.vertices[i]));
}

static float inertia(void)
{
    return 0.4f * body.mass * shape.radius * shape.radius;
}


static void clearShape(void)
{
    memset(&shape, 0, sizeof(shape));
}

static void addVertex(float x, float y, float z)
{
    if (shape.vertexCount >= MAX_VERTICES)
        return;

    shape.vertices[shape.vertexCount++] = V(x, y, z);
}

static void addEdge(int a, int b)
{
    if (shape.edgeCount >= MAX_EDGES)
        return;

    if (a > b) {
        int t = a;
        a = b;
        b = t;
    }

    for (int i = 0; i < shape.edgeCount; i++) {
        if (shape.edges[i][0] == a &&
            shape.edges[i][1] == b)
            return;
    }

    shape.edges[shape.edgeCount][0] = a;
    shape.edges[shape.edgeCount][1] = b;
    shape.edgeCount++;
}

static void addTriangle(int a, int b, int c)
{
    if (shape.triangleCount >= MAX_TRIANGLES)
        return;

    shape.triangles[shape.triangleCount][0] = a;
    shape.triangles[shape.triangleCount][1] = b;
    shape.triangles[shape.triangleCount][2] = c;
    shape.triangleCount++;

    addEdge(a, b);
    addEdge(b, c);
    addEdge(c, a);
}

static void buildCube(void)
{
    clearShape();

    float s = 0.7f;

    addVertex(-s, -s, -s);
    addVertex( s, -s, -s);
    addVertex( s,  s, -s);
    addVertex(-s,  s, -s);
    addVertex(-s, -s,  s);
    addVertex( s, -s,  s);
    addVertex( s,  s,  s);
    addVertex(-s,  s,  s);

    int f[][3] = {
        {0,2,1}, {0,3,2},
        {4,5,6}, {4,6,7},
        {0,1,5}, {0,5,4},
        {1,2,6}, {1,6,5},
        {2,3,7}, {2,7,6},
        {3,0,4}, {3,4,7}
    };

    for (int i = 0; i < 12; i++)
        addTriangle(f[i][0], f[i][1], f[i][2]);

    shape.radius = sqrtf(3.0f) * s;
}

static void buildPyramid(void)
{
    clearShape();

    float s = 0.9f;

    addVertex(-s, -0.6f, -s);
    addVertex( s, -0.6f, -s);
    addVertex( s, -0.6f,  s);
    addVertex(-s, -0.6f,  s);
    addVertex(0, 1.0f, 0);

    addTriangle(0,2,1);
    addTriangle(0,3,2);
    addTriangle(0,1,4);
    addTriangle(1,2,4);
    addTriangle(2,3,4);
    addTriangle(3,0,4);

    shape.radius = 1.4f;
}

static void buildTetrahedron(void)
{
    clearShape();

    addVertex(0, 1.0f, 0);
    addVertex(-0.9f, -0.5f, 0.7f);
    addVertex(0.9f, -0.5f, 0.7f);
    addVertex(0, -0.5f, -0.9f);

    addTriangle(0,1,2);
    addTriangle(0,2,3);
    addTriangle(0,3,1);
    addTriangle(1,3,2);

    shape.radius = 1.2f;
}

static void buildOctahedron(void)
{
    clearShape();

    addVertex(0, 1.1f, 0);
    addVertex(0, -1.1f, 0);
    addVertex(-1, 0, 0);
    addVertex(1, 0, 0);
    addVertex(0, 0, -1);
    addVertex(0, 0, 1);

    int f[][3] = {
        {0,2,4}, {0,4,3}, {0,3,5}, {0,5,2},
        {1,4,2}, {1,3,4}, {1,5,3}, {1,2,5}
    };

    for (int i = 0; i < 8; i++)
        addTriangle(f[i][0], f[i][1], f[i][2]);

    shape.radius = 1.1f;
}

static void buildSphere(void)
{
    clearShape();

    const int rings = 8;
    const int segments = 12;
    const float r = 0.85f;

    for (int j = 0; j <= rings; j++) {
        float phi = PI * (float)j / (float)rings;

        for (int i = 0; i < segments; i++) {
            float theta = 2.0f * PI * (float)i / (float)segments;

            addVertex(
                r * sinf(phi) * cosf(theta),
                r * cosf(phi),
                r * sinf(phi) * sinf(theta)
            );
        }
    }

    for (int j = 0; j < rings; j++) {
        for (int i = 0; i < segments; i++) {
            int next = (i + 1) % segments;

            int a = j * segments + i;
            int b = j * segments + next;
            int c = (j + 1) * segments + i;
            int d = (j + 1) * segments + next;

            addTriangle(a, c, b);
            addTriangle(b, c, d);
        }
    }

    shape.radius = r;
}

static const char *shapeName(int type)
{
    switch (type) {
        case 0: return "Cube";
        case 1: return "Pyramide";
        case 2: return "Tetraedre";
        case 3: return "Octaedre";
        case 4: return "Sphere";
        default: return "Objet";
    }
}

static void buildShape(int type)
{
    switch (type) {
        case 0: buildCube(); break;
        case 1: buildPyramid(); break;
        case 2: buildTetrahedron(); break;
        case 3: buildOctahedron(); break;
        case 4: buildSphere(); break;
        default: buildCube(); break;
    }

    body.shapeType = type;
}

static float lowestLocalY(void)
{
    float lowest = shape.vertices[0].y;

    for (int i = 1; i < shape.vertexCount; i++) {
        if (shape.vertices[i].y < lowest)
            lowest = shape.vertices[i].y;
    }

    return lowest;
}

static float restingCenterHeight(void)
{
    if (body.shapeType == 4)
        return shape.radius;

    return -lowestLocalY();
}

static void resetBody(void)
{
    body.mass = 1.0f;
    body.charge = 1.0f;
    body.restitution = 0.12f;
    body.friction = 0.75f;
    body.color = ORANGE;

    body.velocity = V(0, 0, 0);
    body.angularVelocity = V(0, 0, 0);
    body.rotation = QuaternionIdentity();

    buildShape(body.shapeType);

    body.position = V(
        0,
        restingCenterHeight() + 0.02f,
        0
    );
}

static void resetCamera(void)
{
    camera.position = V(10, 9, 14);
    camera.target = V(0, 0, 0);
    camera.up = V(0, 1, 0);
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}


static void addSourceAt(Vector3 position, int type)
{
    if (sourceCount >= MAX_SOURCES)
        return;

    FieldSource s = {0};
    s.position = position;

    if (type == 1) {
        s.charge = 1.0f;
    } else if (type == -1) {
        s.charge = -1.0f;
    } else {
        s.magnetic = true;
        s.magneticMoment = V(0, 1, 0);
    }

    sources[sourceCount++] = s;
}

static void addSceneObjectAt(Vector3 groundPosition, int type)
{
    if (type == 3 || type == 4) {
        if (sceneObjectCount >= MAX_SCENE_OBJECTS) return;

        SceneObject obj = {0};
        obj.type = (type == 3) ? SCENE_WALL : SCENE_RAMP;
        obj.position = groundPosition;
        obj.friction = 0.65f;
        obj.restitution = 0.18f;
        if (obj.type == SCENE_WALL) {
            obj.size = V(5.0f, 4.0f, 0.3f);
            obj.position.y = obj.size.y * 0.5f;
            obj.color = (Color){130, 145, 165, 255};
        } else {
            obj.size = V(6.0f, 0.18f, 3.0f);
            obj.position.y = 1.5f;
            obj.angle = 25.0f * DEG2RAD;
            obj.color = (Color){190, 135, 75, 255};
        }
        sceneObjects[sceneObjectCount++] = obj;
    } else if (type == 5) {
        if (pendulumCount >= MAX_SCENE_OBJECTS) return;

        Pendulum p = {0};
        p.pivot = groundPosition;
        p.pivot.y += 4.0f;
        p.length = 2.5f;
        p.angle = 0.65f;
        p.angularVelocity = 0.0f;
        p.damping = 0.025f;
        p.bobRadius = 0.32f;
        p.mass = 1.0f;
        p.restitution = 0.72f;
        p.color = (Color){220, 75, 65, 255};
        pendulums[pendulumCount++] = p;
    }
}

static Vector3 electricFieldAt(Vector3 p)
{
    Vector3 field = V(0, 0, 0);

    for (int i = 0; i < sourceCount; i++) {
        FieldSource s = sources[i];

        if (s.magnetic || fabsf(s.charge) < 0.0001f)
            continue;

        Vector3 r = sub(p, s.position);
        float r2 = dot(r, r);

        if (r2 < 0.12f)
            r2 = 0.12f;

        float rLen = sqrtf(r2);
        float factor = ELECTRIC_SCALE * s.charge / (r2 * rLen);

        field = add(field, mul(r, factor));
    }

    return field;
}

static Vector3 magneticFieldAt(Vector3 p)
{
    Vector3 field = V(0, 0, 0);

    for (int i = 0; i < sourceCount; i++) {
        FieldSource s = sources[i];

        if (!s.magnetic)
            continue;

        Vector3 r = sub(p, s.position);
        float r2 = dot(r, r);

        if (r2 < 0.25f)
            r2 = 0.25f;

        float rLen = sqrtf(r2);
        Vector3 rh = mul(r, 1.0f / rLen);
        float mr = dot(s.magneticMoment, rh);

        Vector3 numerator = sub(
            mul(rh, 3.0f * mr),
            s.magneticMoment
        );

        field = add(
            field,
            mul(numerator, MAGNETIC_SCALE / (r2 * rLen))
        );
    }

    return field;
}

static Vector3 totalForce(void)
{
    Vector3 force = V(0, -GRAVITY * body.mass, 0);

    Vector3 e = electricFieldAt(body.position);
    Vector3 b = magneticFieldAt(body.position);

    force = add(force, mul(e, body.charge));

    /* Force de Lorentz : F = q(v x B). */
    force = add(
        force,
        mul(cross(body.velocity, b), body.charge)
    );

    return force;
}

static Ray mouseRayCompatible(void)
{
    Vector2 mouse = GetMousePosition();

    float width = (float)GetScreenWidth();
    float height = (float)GetScreenHeight();

    if (width <= 0.0f || height <= 0.0f) {
        Ray invalid = {camera.position, V(0, 0, 0)};
        return invalid;
    }

    Vector3 forward = norm(sub(camera.target, camera.position));
    Vector3 right = norm(cross(forward, camera.up));
    Vector3 up = norm(cross(right, forward));

    float ndcX = 2.0f * mouse.x / width - 1.0f;
    float ndcY = 1.0f - 2.0f * mouse.y / height;

    float halfFov = camera.fovy * DEG2RAD * 0.5f;
    float tanHalfFov = tanf(halfFov);
    float aspect = width / height;

    Vector3 direction = add(
        forward,
        add(
            mul(right, ndcX * tanHalfFov * aspect),
            mul(up, ndcY * tanHalfFov)
        )
    );

    Ray ray;
    ray.position = camera.position;
    ray.direction = norm(direction);

    return ray;
}

static bool pointOnPlatform(Vector3 p)
{
    return fabsf(p.x) <= PLATFORM_HALF_X &&
           fabsf(p.z) <= PLATFORM_HALF_Z;
}

static bool mousePositionOnPlatform(Vector3 *result)
{
    Ray ray = mouseRayCompatible();

    if (fabsf(ray.direction.y) < 0.00001f)
        return false;

    float t = (platformTop - ray.position.y) / ray.direction.y;

    if (t < 0.0f)
        return false;

    Vector3 p = add(ray.position, mul(ray.direction, t));

    if (!pointOnPlatform(p))
        return false;

    *result = p;
    return true;
}

static void updateCameraControls(void)
{
    Vector2 delta = GetMouseDelta();

    if (placementMode == 0 &&
        !objectPlacementMode &&
        IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {

        Vector3 offset = sub(camera.position, camera.target);
        float radius = len(offset);

        if (radius < 0.1f)
            radius = 0.1f;

        float yaw = atan2f(offset.x, offset.z);
        float pitch = asinf(clampf(offset.y / radius, -1.0f, 1.0f));

        yaw -= delta.x * 0.005f;
        pitch += delta.y * 0.005f;

        pitch = clampf(pitch, -1.45f, 1.45f);

        offset.x = radius * cosf(pitch) * sinf(yaw);
        offset.y = radius * sinf(pitch);
        offset.z = radius * cosf(pitch) * cosf(yaw);

        camera.position = add(camera.target, offset);
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector3 forward = norm(sub(camera.target, camera.position));
        Vector3 right = norm(cross(forward, camera.up));
        Vector3 up = norm(cross(right, forward));

        float scale = len(sub(camera.target, camera.position)) * 0.0025f;

        Vector3 movement = add(
            mul(right, -delta.x * scale),
            mul(up, delta.y * scale)
        );

        camera.position = add(camera.position, movement);
        camera.target = add(camera.target, movement);
    }

    float wheel = GetMouseWheelMove();

    if (objectPlacementMode && fabsf(wheel) > 0.001f) {
        objectPlacementHeight += wheel * 0.25f;
        objectPlacementHeight = clampf(
            objectPlacementHeight, 0.02f, 20.0f
        );
    } else if (fabsf(wheel) > 0.001f) {
        Vector3 offset = sub(camera.position, camera.target);
        float distance = len(offset) * powf(0.85f, wheel);

        distance = clampf(distance, 1.5f, 100.0f);

        camera.position = add(
            camera.target,
            mul(norm(offset), distance)
        );
    }

    if (IsKeyPressed(KEY_C))
        resetCamera();
}


static Vector3 velocityAtPoint(Vector3 point)
{
    Vector3 arm = sub(point, body.position);

    return add(
        body.velocity,
        cross(body.angularVelocity, arm)
    );
}

static void applyImpulse(Vector3 point, Vector3 impulse)
{
    body.velocity = add(
        body.velocity,
        mul(impulse, 1.0f / body.mass)
    );

    Vector3 arm = sub(point, body.position);
    Vector3 angularImpulse = cross(arm, impulse);

    float I = inertia();

    if (I > 0.0001f) {
        body.angularVelocity = add(
            body.angularVelocity,
            mul(angularImpulse, 1.0f / I)
        );
    }
}

static float effectiveMassDenominator(Vector3 arm, Vector3 direction)
{
    float I = inertia();

    Vector3 rCrossD = cross(arm, direction);
    float rotationalTerm = 0.0f;

    if (I > 0.0001f)
        rotationalTerm = dot(rCrossD, rCrossD) / I;

    return 1.0f / body.mass + rotationalTerm;
}

static void resolveContact(
    Vector3 point,
    Vector3 normal,
    float penetration,
    float dt,
    float supportShare
) {
    if (penetration > 0.0f) {
        body.position = add(
            body.position,
            mul(normal, penetration + 0.0002f)
        );
    }

    Vector3 arm = sub(point, body.position);
    Vector3 contactVelocity = velocityAtPoint(point);

    float vn = dot(contactVelocity, normal);
    float normalImpulseMagnitude = 0.0f;

    float normalDenominator =
        effectiveMassDenominator(arm, normal);

    if (vn < 0.0f && normalDenominator > 0.000001f) {
        normalImpulseMagnitude =
            -(1.0f + body.restitution) * vn / normalDenominator;

        applyImpulse(
            point,
            mul(normal, normalImpulseMagnitude)
        );
    }

    float supportImpulse =
        body.mass * GRAVITY * dt * supportShare;

    float frictionBudget = body.friction *
        (normalImpulseMagnitude + supportImpulse);

    contactVelocity = velocityAtPoint(point);

    Vector3 tangentVelocity = sub(
        contactVelocity,
        mul(normal, dot(contactVelocity, normal))
    );

    float tangentSpeed = len(tangentVelocity);

    if (tangentSpeed < 0.000001f || frictionBudget <= 0.0f)
        return;

    Vector3 tangentDirection = mul(
        tangentVelocity,
        1.0f / tangentSpeed
    );

    float tangentDenominator =
        effectiveMassDenominator(arm, tangentDirection);

    if (tangentDenominator < 0.000001f)
        return;

    float jt = -tangentSpeed / tangentDenominator;
    jt = clampf(jt, -frictionBudget, frictionBudget);

    applyImpulse(point, mul(tangentDirection, jt));
}

static void collideSphereWithPlatform(float dt)
{
    float r = shape.radius;
    float bottom = body.position.y - r;

    if (bottom >= platformTop)
        return;

    if (!pointOnPlatform(body.position))
        return;

    Vector3 contact = V(
        body.position.x,
        platformTop,
        body.position.z
    );

    resolveContact(
        contact,
        V(0, 1, 0),
        platformTop - bottom,
        dt,
        1.0f
    );
}

static void collideMeshWithPlatform(float dt)
{
    float supportShare = 1.0f / 4.0f;

    for (int pass = 0; pass < CONTACT_PASSES; pass++) {
        for (int i = 0; i < shape.vertexCount; i++) {
            Vector3 p = worldVertex(i);

            if (!pointOnPlatform(p))
                continue;

            if (p.y < platformTop) {
                Vector3 contact = V(p.x, platformTop, p.z);

                resolveContact(
                    contact,
                    V(0, 1, 0),
                    platformTop - p.y,
                    dt,
                    supportShare
                );
            }
        }
    }
}

static void collideWithSceneObjects(float dt)
{
    (void)dt;
    float r = shape.radius;

    for (int i = 0; i < sceneObjectCount; i++) {
        SceneObject *o = &sceneObjects[i];

        if (o->type == SCENE_WALL) {
            /* Collision sphere englobante contre un mur rectangulaire
               de longueur 5, d'epaisseur 0.3 et de hauteur 4. */
            Vector3 half = mul(o->size, 0.5f);
            Vector3 closest = V(
                clampf(body.position.x, o->position.x - half.x, o->position.x + half.x),
                clampf(body.position.y, o->position.y - half.y, o->position.y + half.y),
                clampf(body.position.z, o->position.z - half.z, o->position.z + half.z)
            );
            Vector3 delta = sub(body.position, closest);
            float distance = len(delta);

            if (distance < r && distance > 0.0001f) {
                Vector3 normal = mul(delta, 1.0f / distance);
                body.position = add(body.position, mul(normal, r - distance + 0.001f));
                float vn = dot(body.velocity, normal);
                if (vn < 0.0f)
                    body.velocity = sub(body.velocity, mul(normal, (1.0f + fminf(body.restitution, o->restitution)) * vn));
                Vector3 tangent = sub(body.velocity, mul(normal, dot(body.velocity, normal)));
                body.velocity = sub(body.velocity, mul(tangent, clampf(fminf(body.friction, o->friction) * 0.08f, 0.0f, 0.8f)));
            }
        } else {
            /* Pente de 6 x 3 m. La surface monte vers +x.
               On utilise la sphere englobante pour le contact. */
            float dx = body.position.x - o->position.x;
            float dz = body.position.z - o->position.z;
            if (fabsf(dx) > o->size.x * 0.5f + r || fabsf(dz) > o->size.z * 0.5f + r)
                continue;

            Vector3 normal = norm(V(sinf(o->angle), cosf(o->angle), 0.0f));
            float surfaceY = o->position.y - tanf(o->angle) * dx;
            Vector3 planePoint = V(body.position.x, surfaceY, body.position.z);
            float signedDistance = dot(sub(body.position, planePoint), normal);

            if (signedDistance < r) {
                float penetration = r - signedDistance;
                body.position = add(body.position, mul(normal, penetration + 0.001f));
                float vn = dot(body.velocity, normal);
                if (vn < 0.0f)
                    body.velocity = sub(body.velocity, mul(normal, (1.0f + fminf(body.restitution, o->restitution)) * vn));

                Vector3 tangent = sub(body.velocity, mul(normal, dot(body.velocity, normal)));
                float tangentSpeed = len(tangent);
                if (tangentSpeed > 0.0001f) {
                    float frictionFactor = clampf(fminf(body.friction, o->friction) * 0.8f * dt, 0.0f, 1.0f);
                    body.velocity = sub(body.velocity, mul(tangent, frictionFactor));
                }
            }
        }
    }
}

static void updatePendulums(float dt)
{
    for (int i = 0; i < pendulumCount; i++) {
        Pendulum *p = &pendulums[i];
        float angularAcceleration = -(GRAVITY / p->length) * sinf(p->angle)
                                    - p->damping * p->angularVelocity;
        p->angularVelocity += angularAcceleration * dt;
        p->angle += p->angularVelocity * dt;
    }
}

static Vector3 pendulumBobPosition(const Pendulum *p)
{
    return V(
        p->pivot.x + p->length * sinf(p->angle),
        p->pivot.y - p->length * cosf(p->angle),
        p->pivot.z
    );
}

/* Collision entre chaque boule de pendule et l'objet physique principal.
   La boule est traitée comme une sphère; le transfert angulaire du pendule
   est calculé autour de son pivot. */
static void collidePendulumsWithBody(void)
{
    float bodyRadius = shape.radius;

    for (int i = 0; i < pendulumCount; i++) {
        Pendulum *p = &pendulums[i];
        Vector3 bob = pendulumBobPosition(p);
        Vector3 delta = sub(body.position, bob);
        float distance = len(delta);
        float minDistance = bodyRadius + p->bobRadius;

        if (distance >= minDistance)
            continue;

        Vector3 normal = distance > 0.0001f
            ? mul(delta, 1.0f / distance) : V(1.0f, 0.0f, 0.0f);
        float penetration = minDistance - distance;

        /* On écarte l'objet principal; le pendule reste attaché à son pivot. */
        body.position = add(body.position, mul(normal, penetration + 0.001f));

        Vector3 bobVelocity = V(
            p->length * cosf(p->angle) * p->angularVelocity,
            p->length * sinf(p->angle) * p->angularVelocity,
            0.0f
        );
        Vector3 relativeVelocity = sub(body.velocity, bobVelocity);
        float normalSpeed = dot(relativeVelocity, normal);

        if (normalSpeed < 0.0f) {
            float e = fminf(body.restitution, p->restitution);
            float impulse = -(1.0f + e) * normalSpeed /
                            (1.0f / body.mass + 1.0f / p->mass);
            body.velocity = add(body.velocity, mul(normal, impulse / body.mass));

            /* Couple impulsionnel simplifié appliqué au pendule. */
            Vector3 impulseOnBob = mul(normal, -impulse);
            Vector3 arm = sub(bob, p->pivot);
            Vector3 torque = cross(arm, impulseOnBob);
            float inertia = p->mass * p->length * p->length;
            if (inertia > 0.0001f)
                p->angularVelocity += torque.z / inertia;
        }
    }
}

static void drawSceneObjects(void)
{
    for (int i = 0; i < sceneObjectCount; i++) {
        const SceneObject *o = &sceneObjects[i];
        if (o->type == SCENE_WALL) {
            DrawCube(o->position, o->size.x, o->size.y, o->size.z, o->color);
            DrawCubeWires(o->position, o->size.x, o->size.y, o->size.z, LIGHTGRAY);
        } else {
            /* Prisme incline : surface triangulaire, flancs et aretes. */
            float halfL = o->size.x * 0.5f, halfW = o->size.z * 0.5f, thick = o->size.y;
            float c = cosf(o->angle), s = sinf(o->angle);
            Vector3 top[4] = {
                V(o->position.x-halfL, o->position.y+s*halfL, o->position.z-halfW),
                V(o->position.x+halfL, o->position.y-s*halfL, o->position.z-halfW),
                V(o->position.x+halfL, o->position.y-s*halfL, o->position.z+halfW),
                V(o->position.x-halfL, o->position.y+s*halfL, o->position.z+halfW)
            };
            Vector3 bottom[4];
            for (int k = 0; k < 4; k++) bottom[k] = sub(top[k], V(s*thick, c*thick, 0));
            DrawTriangle3D(top[0], top[1], top[2], o->color);
            DrawTriangle3D(top[0], top[2], top[3], o->color);
            for (int k = 0; k < 4; k++) {
                int n = (k+1)%4;
                DrawTriangle3D(top[k], bottom[k], bottom[n], (Color){130, 85, 45, 255});
                DrawTriangle3D(top[k], bottom[n], top[n], (Color){130, 85, 45, 255});
                DrawLine3D(top[k], top[n], RAYWHITE);
            }
            (void)c;
        }
    }

    for (int i = 0; i < pendulumCount; i++) {
        const Pendulum *p = &pendulums[i];
        Vector3 bob = pendulumBobPosition(p);
        DrawCylinderEx(p->pivot, bob, 0.045f, 0.045f, 8, LIGHTGRAY);
        DrawSphere(p->pivot, 0.13f, DARKGRAY);
        DrawSphere(bob, p->bobRadius, p->color);
        DrawSphereWires(bob, p->bobRadius, 10, 10, RAYWHITE);
    }
}

static void collideWithPlatform(float dt)
{
    if (body.shapeType == 4)
        collideSphereWithPlatform(dt);
    else
        collideMeshWithPlatform(dt);
}

static void updateBody(float dt)
{
    Vector3 force = totalForce();
    Vector3 acceleration = mul(force, 1.0f / body.mass);

    body.velocity = add(body.velocity, mul(acceleration, dt));
    body.position = add(body.position, mul(body.velocity, dt));

    float omega = len(body.angularVelocity);

    if (omega > 0.00001f) {
        Quaternion dq = QuaternionFromAxisAngle(
            norm(body.angularVelocity),
            omega * dt
        );

        body.rotation = QuaternionNormalize(
            QuaternionMultiply(dq, body.rotation)
        );
    }

    collideWithPlatform(dt);
    collideWithSceneObjects(dt);

    if (len(body.velocity) < SLEEP_SPEED &&
        len(body.angularVelocity) < SLEEP_SPEED) {
        body.velocity = V(0, 0, 0);
        body.angularVelocity = V(0, 0, 0);
    }
}


static void beginObjectPlacement(void)
{
    objectPlacementMode = true;
    objectPlacementValid = false;

    objectPlacementHeight = 0.02f;
    placementMode = 0;

    body.velocity = V(0, 0, 0);
    body.angularVelocity = V(0, 0, 0);
}

static void cancelObjectPlacement(void)
{
    objectPlacementMode = false;
    objectPlacementValid = false;
}

static void confirmObjectPlacement(void)
{
    if (!objectPlacementMode || !objectPlacementValid)
        return;

    body.position = objectPlacementPosition;
    body.position.y += restingCenterHeight() + objectPlacementHeight;

    body.velocity = V(0, 0, 0);
    body.angularVelocity = V(0, 0, 0);
    body.rotation = QuaternionIdentity();

    
    paused = false;

    cancelObjectPlacement();
}


static void drawPlatform(void)
{
    DrawCube(
        V(0, -PLATFORM_THICKNESS / 2.0f, 0),
        PLATFORM_HALF_X * 2.0f,
        PLATFORM_THICKNESS,
        PLATFORM_HALF_Z * 2.0f,
        (Color){65, 72, 84, 255}
    );

    DrawCubeWires(
        V(0, -PLATFORM_THICKNESS / 2.0f, 0),
        PLATFORM_HALF_X * 2.0f,
        PLATFORM_THICKNESS,
        PLATFORM_HALF_Z * 2.0f,
        LIGHTGRAY
    );

    for (int x = -(int)PLATFORM_HALF_X;
         x <= (int)PLATFORM_HALF_X; x++) {
        DrawLine3D(
            V((float)x, 0.005f, -PLATFORM_HALF_Z),
            V((float)x, 0.005f, PLATFORM_HALF_Z),
            (Color){90, 98, 110, 255}
        );
    }

    for (int z = -(int)PLATFORM_HALF_Z;
         z <= (int)PLATFORM_HALF_Z; z++) {
        DrawLine3D(
            V(-PLATFORM_HALF_X, 0.005f, (float)z),
            V(PLATFORM_HALF_X, 0.005f, (float)z),
            (Color){90, 98, 110, 255}
        );
    }
}

static void drawArrow(
    Vector3 start,
    Vector3 direction,
    float scale,
    Color color
) {
    if (len(direction) < 0.00001f)
        return;

    Vector3 dir = norm(direction);
    Vector3 end = add(start, mul(dir, scale));

    DrawLine3D(start, end, color);

    Vector3 side = norm(cross(dir, V(0, 1, 0)));

    if (len(side) < 0.01f)
        side = norm(cross(dir, V(1, 0, 0)));

    Vector3 back = sub(end, mul(dir, scale * 0.2f));

    DrawLine3D(
        end,
        add(back, mul(side, scale * 0.08f)),
        color
    );

    DrawLine3D(
        end,
        sub(back, mul(side, scale * 0.08f)),
        color
    );
}

static void drawBodyAt(Vector3 position, Quaternion rotation)
{
    if (body.shapeType == 4) {
        DrawSphere(position, shape.radius, body.color);
        DrawSphereWires(
            position, shape.radius, 16, 12, DARKGRAY
        );

        Vector3 markerLocal = V(0, 0, shape.radius);
        Vector3 markerWorld = add(
            position,
            Vector3RotateByQuaternion(markerLocal, rotation)
        );

        DrawSphere(markerWorld, 0.09f, YELLOW);
        return;
    }

    if (!wireframe) {
        for (int i = 0; i < shape.triangleCount; i++) {
            Vector3 a = add(
                position,
                Vector3RotateByQuaternion(
                    shape.vertices[shape.triangles[i][0]], rotation
                )
            );

            Vector3 b = add(
                position,
                Vector3RotateByQuaternion(
                    shape.vertices[shape.triangles[i][1]], rotation
                )
            );

            Vector3 c = add(
                position,
                Vector3RotateByQuaternion(
                    shape.vertices[shape.triangles[i][2]], rotation
                )
            );

            DrawTriangle3D(a, b, c, body.color);
        }
    }

    for (int i = 0; i < shape.edgeCount; i++) {
        Vector3 a = add(
            position,
            Vector3RotateByQuaternion(
                shape.vertices[shape.edges[i][0]], rotation
            )
        );

        Vector3 b = add(
            position,
            Vector3RotateByQuaternion(
                shape.vertices[shape.edges[i][1]], rotation
            )
        );

        DrawLine3D(a, b, BLACK);
    }
}

static void drawBody(void)
{
    drawBodyAt(body.position, body.rotation);
}

static void drawSources(void)
{
    for (int i = 0; i < sourceCount; i++) {
        FieldSource s = sources[i];

        if (s.magnetic) {
            DrawSphere(s.position, 0.23f, BLUE);
            DrawSphereWires(s.position, 0.31f, 8, 8, SKYBLUE);

            drawArrow(
                s.position,
                s.magneticMoment,
                0.8f,
                SKYBLUE
            );
        } else {
            Color color = s.charge > 0.0f ? RED : SKYBLUE;

            DrawSphere(s.position, 0.23f, color);
            DrawSphereWires(s.position, 0.31f, 8, 8, WHITE);

            if (showFields) {
                for (int k = 0; k < 8; k++) {
                    float a = 2.0f * PI * (float)k / 8.0f;
                    Vector3 dir = V(cosf(a), 0.2f, sinf(a));

                    if (s.charge < 0.0f)
                        dir = mul(dir, -1.0f);

                    drawArrow(s.position, dir, 1.0f, color);
                }
            }
        }
    }

    if (showFields) {
        drawArrow(
            body.position,
            electricFieldAt(body.position),
            0.3f,
            ORANGE
        );

        drawArrow(
            body.position,
            magneticFieldAt(body.position),
            0.4f,
            SKYBLUE
        );
    }
}


static void drawHUD(void)
{
    DrawRectangle(10, 10, 405, 238, (Color){15, 18, 25, 220});

    DrawText(
        TextFormat("Objet : %s", shapeName(body.shapeType)),
        22, 20, 18, RAYWHITE
    );

    DrawText(
        TextFormat("Charge : %.1f", body.charge),
        22, 46, 17, ORANGE
    );

    DrawText(
        TextFormat("Sources : %d / %d", sourceCount, MAX_SOURCES),
        22, 70, 17, RAYWHITE
    );

    DrawText(
        TextFormat("Obstacles : %d | Pendules : %d", sceneObjectCount, pendulumCount),
        22, 214, 15, RAYWHITE
    );

    DrawText(
        TextFormat("Vitesse : %.2f m/s", len(body.velocity)),
        22, 94, 17, RAYWHITE
    );

    DrawText(
        TextFormat("Rotation : %.2f rad/s", len(body.angularVelocity)),
        22, 118, 17, RAYWHITE
    );

    DrawText(
        TextFormat("Frottement : %.2f", body.friction),
        22, 142, 17, RAYWHITE
    );

    DrawText(
        paused ? "PAUSE" : "Simulation active",
        22, 166, 17, paused ? YELLOW : GREEN
    );

    if (objectPlacementMode) {
        DrawText(
            objectPlacementValid
                ? "Placement : Entree pour valider"
                : "Placement : clic gauche sur le plateau",
            22, 192, 14, YELLOW
        );
    } else if (placementMode == 1) {
        DrawText("Placement : source electrique +",
                 22, 192, 14, RED);
    } else if (placementMode == -1) {
        DrawText("Placement : source electrique -",
                 22, 192, 14, SKYBLUE);
    } else if (placementMode == 2) {
        DrawText("Placement : source magnetique",
                 22, 192, 14, BLUE);
    } else if (placementMode == 3) {
        DrawText("Placement : mur (clic sur le plateau)",
                 22, 192, 14, YELLOW);
    } else if (placementMode == 4) {
        DrawText("Placement : pente (clic sur le plateau)",
                 22, 192, 14, YELLOW);
    } else if (placementMode == 5) {
        DrawText("Placement : pendule (clic sur le plateau)",
                 22, 192, 14, YELLOW);
    }

    DrawText(
        "Molette : zoom/hauteur | Gauche : orbite/placement",
        12, GetScreenHeight() - 25, 14, RAYWHITE
    );

    if (!showHelp)
        return;

    int x = GetScreenWidth() - 350;

    DrawRectangle(
        x - 10, 10, 340, 455,
        (Color){15, 18, 25, 220}
    );

    DrawText("COMMANDES", x, 20, 18, YELLOW);
    DrawText("1-5 : changer de forme", x, 48, 14, RAYWHITE);
    DrawText("P : placer l'objet librement", x, 70, 14, YELLOW);
    DrawText("Clic gauche : choisir la position", x, 92, 13, RAYWHITE);
    DrawText("Molette : regler la hauteur en mode P", x, 114, 13, RAYWHITE);
    DrawText("Entree : valider | Echap : annuler", x, 136, 13, RAYWHITE);
    DrawText("E : source electrique +", x, 162, 14, RED);
    DrawText("Q/A : source electrique -", x, 184, 14, SKYBLUE);
    DrawText("B : source magnetique", x, 206, 14, BLUE);
    DrawText("Clic gauche : placer la source", x, 228, 13, RAYWHITE);
    DrawText("Backspace : supprimer source", x, 250, 13, RAYWHITE);
    DrawText("+ / - : charge de l'objet", x, 272, 14, ORANGE);
    DrawText("F : afficher les champs", x, 294, 14, RAYWHITE);
    DrawText("W/Z : mode vide", x, 316, 14, RAYWHITE);
    DrawText("Espace : pause | R : reset", x, 338, 14, RAYWHITE);
    DrawText("H : aide | C : recentrer camera", x, 360, 13, RAYWHITE);
    DrawText("Fleches : pousser l'objet", x, 382, 13, RAYWHITE);
    DrawText("M / ; : placer un mur", x, 404, 14, (Color){170, 195, 225, 255});
    DrawText("N : placer une pente inclinee", x, 426, 14, (Color){225, 175, 115, 255});
    DrawText("J : ajouter un pendule", x, 448, 14, (Color){240, 120, 110, 255});
}

/* --------------------------------------------------
   Programme principal
   -------------------------------------------------- */

int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);

    InitWindow(
        1280,
        800,
        "Physique - Collisions et champs electromagnetiques"
    );

    SetTargetFPS(60);

    body.shapeType = 0;

    resetBody();
    resetCamera();

    while (!WindowShouldClose()) {
        float frameTime = GetFrameTime();

        updateCameraControls();

        if (IsKeyPressed(KEY_SPACE))
            paused = !paused;

        if (IsKeyPressed(KEY_H))
            showHelp = !showHelp;

        if (IsKeyPressed(KEY_F))
            showFields = !showFields;

        /* Raylib utilise les positions de touches QWERTY : Z sur AZERTY
           occupe la position de W. On accepte donc les deux. */
        if (IsKeyPressed(KEY_W) || IsKeyPressed(KEY_Z))
            wireframe = !wireframe;

        if (IsKeyPressed(KEY_ESCAPE)) {
            placementMode = 0;
            cancelObjectPlacement();
        }

        if (IsKeyPressed(KEY_P)) {
            if (objectPlacementMode) {
                cancelObjectPlacement();
            } else {
                beginObjectPlacement();
            }
        }

        if (IsKeyPressed(KEY_E) && !objectPlacementMode)
            placementMode = 1;

        /* Sur un clavier AZERTY, la touche physique QWERTY est etiquetee A. */
        if ((IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_A)) &&
            !objectPlacementMode)
            placementMode = -1;

        if (IsKeyPressed(KEY_B) && !objectPlacementMode)
            placementMode = 2;

        /* Sur AZERTY, M se trouve a la position de ; en QWERTY. */
        if ((IsKeyPressed(KEY_M) || IsKeyPressed(KEY_SEMICOLON)) &&
            !objectPlacementMode)
            placementMode = 3;

        if (IsKeyPressed(KEY_N) && !objectPlacementMode)
            placementMode = 4;

        if (IsKeyPressed(KEY_J) && !objectPlacementMode)
            placementMode = 5;

        /* Placement libre : le clic choisit la projection sur le plan. */
        if (objectPlacementMode &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector3 p;

            if (mousePositionOnPlatform(&p)) {
                objectPlacementPosition = p;
                objectPlacementValid = true;
            }
        }

        if (objectPlacementMode && IsKeyPressed(KEY_ENTER))
            confirmObjectPlacement();

        /* Placement des sources, murs, pentes et pendules par clic. */
        if (!objectPlacementMode &&
            placementMode != 0 &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {

            Vector3 position;
            if (mousePositionOnPlatform(&position)) {
                if (placementMode == 1 || placementMode == -1 || placementMode == 2) {
                    position.y = platformTop + 0.28f;
                    addSourceAt(position, placementMode);
                } else {
                    addSceneObjectAt(position, placementMode);
                }
                placementMode = 0;
            }
        }

        if (IsKeyPressed(KEY_BACKSPACE) && sourceCount > 0)
            sourceCount--;

        if (IsKeyPressed(KEY_EQUAL) ||
            IsKeyPressed(KEY_KP_ADD)) {
            body.charge += 0.5f;
            body.charge = clampf(body.charge, -10.0f, 10.0f);
        }

        if (IsKeyPressed(KEY_MINUS) ||
            IsKeyPressed(KEY_KP_SUBTRACT)) {
            body.charge -= 0.5f;
            body.charge = clampf(body.charge, -10.0f, 10.0f);
        }

        if (IsKeyPressed(KEY_ONE)) {
            body.shapeType = 0;
            resetBody();
        }

        if (IsKeyPressed(KEY_TWO)) {
            body.shapeType = 1;
            resetBody();
        }

        if (IsKeyPressed(KEY_THREE)) {
            body.shapeType = 2;
            resetBody();
        }

        if (IsKeyPressed(KEY_FOUR)) {
            body.shapeType = 3;
            resetBody();
        }

        if (IsKeyPressed(KEY_FIVE)) {
            body.shapeType = 4;
            resetBody();
        }

        if (IsKeyPressed(KEY_R)) {
            resetBody();
            sourceCount = 0;
            sceneObjectCount = 0;
            pendulumCount = 0;
            placementMode = 0;
            cancelObjectPlacement();
        }

        if (!paused && !objectPlacementMode) {
            if (IsKeyDown(KEY_UP))
                body.velocity.z -= 5.0f * frameTime;

            if (IsKeyDown(KEY_DOWN))
                body.velocity.z += 5.0f * frameTime;

            if (IsKeyDown(KEY_LEFT))
                body.velocity.x -= 5.0f * frameTime;

            if (IsKeyDown(KEY_RIGHT))
                body.velocity.x += 5.0f * frameTime;

            int substeps = SIM_SUBSTEPS;
            float dt = frameTime / (float)substeps;

            for (int i = 0; i < substeps; i++) {
                updateBody(dt);
                updatePendulums(dt);
                collidePendulumsWithBody();
            }
        }

        /* Actualise le point de placement sous le curseur. */
        if (objectPlacementMode) {
            Vector3 p;

            if (mousePositionOnPlatform(&p)) {
                objectPlacementPosition = p;
                objectPlacementValid = true;
            } else {
                objectPlacementValid = false;
            }
        }

        BeginDrawing();
        ClearBackground((Color){28, 32, 42, 255});

        BeginMode3D(camera);

        drawPlatform();
        drawSceneObjects();
        drawSources();
        drawBody();

        if (objectPlacementMode && objectPlacementValid) {
            Vector3 previewPosition = objectPlacementPosition;

            previewPosition.y +=
                restingCenterHeight() + objectPlacementHeight;

            Color previewColor = (Color){100, 220, 140, 180};

            drawBodyAt(
                previewPosition,
                QuaternionIdentity()
            );

            DrawSphereWires(
                previewPosition,
                0.12f,
                8,
                8,
                previewColor
            );

            DrawLine3D(
                objectPlacementPosition,
                previewPosition,
                YELLOW
            );
        }

        if (!objectPlacementMode && placementMode != 0) {
            Vector3 p;

            if (mousePositionOnPlatform(&p)) {
                p.y += 0.06f;

                Color c = placementMode == 2 ? BLUE :
                          placementMode == 1 ? RED : SKYBLUE;

                DrawSphereWires(p, 0.3f, 8, 8, c);
            }
        }

        EndMode3D();

        drawHUD();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}