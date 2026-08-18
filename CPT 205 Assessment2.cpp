#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#define PI 3.14
using namespace std;
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

bool highView = false;
float fw_ang = 0;
float fw_speed = 0.1f;

float fw_lightPhase = 0;
float fw_lightSpeed = 0.15f;

GLfloat emmission[] = { 0.4, 0.3, 0.1 };
GLfloat no_emmission[] = { 0.0, 0.0, 0.0 };

// camera
float fltFOV = 80;
float eyeX = 0.0;
float eyeY = 0.0;
float eyeZ = -2000.0;
float centerX = 0.0;
float centerY = 0.0;
float centerZ = 0.0;
float fltXUp = 0.0;
float fltYUp = 1.0;
float fltZUp = 0.0;
// Camera movement range
const float CAM_MIN_X = -2500.0f;
const float CAM_MAX_X = 2400.0f;

const float CAM_MIN_Y = 0.0f;
const float CAM_MAX_Y = 1500.0f;

const float CAM_MIN_Z = -2000.0f;
const float CAM_MAX_Z = 2500.0f;

GLfloat move_speed = 20;
GLfloat eye_rotation = 90;
GLfloat eye_updown = 0;
float zoomSpeed = 40;
// world
int daytime = 0;
int day = 0;

// speed
int updatespeed = 16;
int intWinWidth = 1600, intWinHeight = 800;

// Draw a line of bitmap text on the screen (in screen coordinates)
void drawScreenText(float x, float y, const char* text)
{
    if (!text) return;

    // Save current projection/modelview matrix and OpenGL states
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, intWinWidth, 0, intWinHeight);  // Set 2D orthographic projection

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Save current enable states
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.0f, 1.0f, 1.0f);

    // Raster position: screen coordinates
    glRasterPos2f(x, y);

    // Output characters one by one
    const char* p = text;
    while (*p)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
        ++p;
    }

    // Restore states
    glPopAttrib();
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
}

// =======================
// Two "high-speed train" tracks
// =======================
const float TRAIN_MIN_X = -3500.0f;
const float TRAIN_MAX_X = 3500.0f;

struct TrainPath {
    float x;
    float z;
    float yGround;
    float yAir;
    float speed;
    int   dir;       // +1: left -> right,  -1: right -> left
};

TrainPath g_trainPaths[2];  // Two tracks in total → 4 trains


// ============================
//  Ground cars + hover cars
// ============================
const int NUM_GROUND_CARS = 12;  // 12 ground cars
const int NUM_HOVER_CARS = 6;    // 6 hover cars

struct CarMover {
    float x;
    float y;
    float z;
    float speed;
    int   dir;   // +1: left -> right,  -1: right -> left
};

CarMover g_groundCars[NUM_GROUND_CARS];

// Hover car
struct HoverCar {
    float angle;
    float radius;
    float y;
    float speed;
};

HoverCar g_hoverCars[NUM_HOVER_CARS];

struct PlatformInfo {
    float x, y, z;
    float h;
};
PlatformInfo gPlatforms[6];

struct TreeInfo {
    float x;
    float y;
    float z;
    float scale;
};
std::vector<TreeInfo> gTrees;
const int TEX_TREE = 22;

// ============================
//  Floating high-speed train parameters
// ============================
float trainX = 500.0f;
float trainY = 80.0f;
float trainZ = 0.0f;
float trainCarLen = 60.0f;
float trainCarHeight = 10.0f;
float trainCarWidth = 8.0f;
int   trainCarCount = 4;
float trainspeed = 20.0f;


// =================== Robot following camera related ===================
bool  g_robotVisible = true;
bool  g_robotWalking = false;
float g_robotDistFromCam = 150.0f;
float g_robotBaseY = 0.0f;
float g_robotWalkPhase = 0.0f;
float orbitAngle = 0.0f;
float headRotate = 0.0f;
float leftUpperArmRotate = 20.0f;
float leftLowerArmRotate = -10.0f;
float rightUpperArmRotate = -20.0f;
float rightLowerArmRotate = 10.0f;
float leftUpperLegRotate = 5.0f;
float leftLowerLegRotate = -5.0f;
float rightUpperLegRotate = -5.0f;
float rightLowerLegRotate = 5.0f;

// ============================
// Figure-8 drone formation parameters
// ============================
const int NUM_FLY_DRONES = 10;
const float BEZ_TOWER_X = 1800.0f;
const float BEZ_TOWER_Z = 500.0f;
const float CYL_TOWER_X = -2400.0f;
const float CYL_TOWER_Z = 1300.0f;
const float DRONE_FLY_Y = 800.0f;

float dronePhase = 0.0f;
float droneSpeed = 0.01f;

// ============================
// Vehicle visibility / movement switches
// ============================
bool g_trainVisible = false;
bool g_trainMoving = false;

bool g_carVisible = false;
bool g_carMoving = false;

bool g_droneVisible = false;
bool g_droneMoving = false;


//-------------------------------------------------basic paint------------------------------------
void clampCamera()
{
    // limit eye
    if (eyeX < CAM_MIN_X) eyeX = CAM_MIN_X;
    if (eyeX > CAM_MAX_X) eyeX = CAM_MAX_X;

    if (eyeY < CAM_MIN_Y) eyeY = CAM_MIN_Y;
    if (eyeY > CAM_MAX_Y) eyeY = CAM_MAX_Y;

    if (eyeZ < CAM_MIN_Z) eyeZ = CAM_MIN_Z;
    if (eyeZ > CAM_MAX_Z) eyeZ = CAM_MAX_Z;

    if (centerX < CAM_MIN_X) centerX = CAM_MIN_X;
    if (centerX > CAM_MAX_X) centerX = CAM_MAX_X;

    if (centerY < CAM_MIN_Y) centerY = CAM_MIN_Y;
    if (centerY > CAM_MAX_Y) centerY = CAM_MAX_Y;

    if (centerZ < CAM_MIN_Z) centerZ = CAM_MIN_Z;
    if (centerZ > CAM_MAX_Z) centerZ = CAM_MAX_Z;
}


void glColorRGB(int r, int g, int b) {
    glColor3f(r / 255.0f, g / 255.0f, b / 255.0f);
}

// CSG helpers
void inside(void a(), void b(), GLenum face, GLenum test)
{
    glEnable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glCullFace(face);
    a();
    glDepthMask(GL_FALSE);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 0, 0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    glCullFace(GL_BACK);
    b();
    glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
    glCullFace(GL_FRONT);
    b();
    glDepthMask(GL_TRUE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glStencilFunc(test, 0, 1);
    glDisable(GL_DEPTH_TEST);
    glCullFace(face);
    a();
    glDisable(GL_STENCIL_TEST);
}

void fixup(void a())
{
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDepthFunc(GL_ALWAYS);
    a();
    glDepthFunc(GL_LESS);
}


//-------------------------------------------- Texture --------------------------------------------------

// texture struct
struct image {
    GLint imageWidth;
    GLint imageHeight;
    GLint pixelLength;
};

image loadTexture[27];
vector<GLubyte*> p;

void ReadImage(const char path[256], GLint& imagewidth, GLint& imageheight, GLint& pixellength) {
    GLubyte* pixeldata;
    FILE* pfile;
    fopen_s(&pfile, path, "rb");
    if (!pfile) exit(0);

    fseek(pfile, 0x0012, SEEK_SET);
    fread(&imagewidth, sizeof(imagewidth), 1, pfile);
    fread(&imageheight, sizeof(imageheight), 1, pfile);

    pixellength = imagewidth * 3;
    while (pixellength % 4 != 0) pixellength++;

    pixellength *= imageheight;
    pixeldata = (GLubyte*)malloc(pixellength);
    if (!pixeldata) exit(0);

    fseek(pfile, 0x0036, SEEK_SET);
    fread(pixeldata, pixellength, 1, pfile);
    p.push_back(pixeldata);
    fclose(pfile);
}

GLuint texture[27];
void textureinit() {
    const char* pictures[27] = {
        "texture/train.bmp",
        "texture/sea.bmp",
        "texture/rainsky2.bmp",
        "texture/sunnysky.bmp",
        "texture/starrysky.bmp",
        "texture/IFC.bmp",
        "texture/IFCHead.bmp",
        "texture/CB1.bmp",
        "texture/CB2.bmp",
        "texture/city1.bmp",
        "texture/city2.bmp",
        "texture/city3.bmp",
        "texture/city4.bmp",
        "texture/city5.bmp",
        "texture/city6.bmp",
        "texture/wall.bmp",
        "texture/ball1.bmp",
        "texture/ball2.bmp",

        "texture/head.bmp",   
        "texture/limb.bmp",   
        "texture/torso.bmp",  
        "texture/joint.bmp",  
        "texture/tree.bmp",   
        "texture/1.bmp",
        "texture/2.bmp",
        "texture/3.bmp",
        "texture/4.bmp"
    };

    for (int i = 0; i < 27; i++) {
        ReadImage(
            pictures[i],
            loadTexture[i].imageWidth,
            loadTexture[i].imageHeight,
            loadTexture[i].pixelLength
        );

        glGenTextures(1, &texture[i]);
        glBindTexture(GL_TEXTURE_2D, texture[i]);

        // =========================
        // BGR -> RGB
        // =========================
        if (i != TEX_TREE) {
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGB,
                loadTexture[i].imageWidth,
                loadTexture[i].imageHeight,
                0,
                GL_BGR_EXT, GL_UNSIGNED_BYTE, p[i]
            );
        }
        // =========================
        //  RGBA
        // =========================
        else {
            int w = loadTexture[i].imageWidth;
            int h = loadTexture[i].imageHeight;
            int pixelCount = w * h;

            GLubyte* src = p[i];                    
            GLubyte* rgba = new GLubyte[pixelCount * 4];

            GLubyte keyB = src[0];
            GLubyte keyG = src[1];
            GLubyte keyR = src[2];

            for (int k = 0; k < pixelCount; ++k) {
                int si = k * 3;
                int di = k * 4;

                GLubyte b = src[si + 0];
                GLubyte g = src[si + 1];
                GLubyte r = src[si + 2];

                rgba[di + 0] = r;
                rgba[di + 1] = g;
                rgba[di + 2] = b;

                // Check if it is the background color
                bool isKey =
                    (abs((int)r - (int)keyR) < 5) &&
                    (abs((int)g - (int)keyG) < 5) &&
                    (abs((int)b - (int)keyB) < 5);

                rgba[di + 3] = isKey ? 0 : 255;   // Transparent background, everything else opaque
            }

            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGBA,
                w, h,
                0,
                GL_RGBA, GL_UNSIGNED_BYTE, rgba
            );

            delete[] rgba;
        }

       // Public parameters
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }
}

//---------------------------------------------- Sky ---------------------------------
void drawball_tx(int textureindex, int radius) {
    glBindTexture(GL_TEXTURE_2D, texture[textureindex]);
    GLUquadric* obj = gluNewQuadric();
    gluQuadricDrawStyle(obj, GLU_FILL);
    gluQuadricNormals(obj, GLU_SMOOTH);
    gluQuadricTexture(obj, GL_TRUE);

    gluQuadricOrientation(obj, GLU_INSIDE);

    gluSphere(obj, radius, 80, 80);
    gluDeleteQuadric(obj);
}

float backgroundrotate = 0;
float skytransparency = 0;

void skyball(int textureindex, int radius) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPushMatrix();
    glScalef(60, 60, 60);
    glRotatef(90, 1, 0, 0);
    glRotatef(backgroundrotate, 0, 0, 1);
    drawball_tx(textureindex, radius);
    glPopMatrix();
    glDisable(GL_BLEND);
}


void skychange() {
    glColor4f(1, 1, 1, 0.5f - skytransparency);
    skyball(4, 70); 
    glColor4f(1, 1, 1, 0.5f + skytransparency);
    skyball(3, 60); 
}


//-------------------------------- Sea ------------------------------------------------------

int wave = 0;
void drawsquare_tx(int textureindex, int zoom) {
    glBindTexture(GL_TEXTURE_2D, texture[textureindex]);
    glBegin(GL_QUADS);
    glTexCoord2f(1, 0); glVertex3f(-2.0 * zoom, 0, -1.0 * zoom);
    glTexCoord2f(0, 0); glVertex3f(-2.0 * zoom, 0, 1.0 * zoom);
    glTexCoord2f(0, 1); glVertex3f(0, 0, 1.0 * zoom);
    glTexCoord2f(1, 1); glVertex3f(0, 0, -1.0 * zoom);
    glEnd();
}
void lake() {
    glColor4f(1, 1, 1, 0.5);

    int start = -3000;  
    int end = 3000;  
    int step = 600;    
    float scale = 300;  

    for (int a = start; a <= end; a += step) {
        for (int b = start; b <= end; b += step) {

            glPushMatrix();
            glTranslatef(a, -10, b + wave);
            glScalef(scale, 10, scale);
            drawsquare_tx(1, 1);
            glPopMatrix();
        }
    }
}

void texturedisplay() {
    glEnable(GL_TEXTURE_2D);
    lake();
    skychange();
    glDisable(GL_TEXTURE_2D);
}

//---------------------------------- Geometry for background ------------------------------------
void drawland(float w, float h, float d)
{
    glPushMatrix();
    glScalef(w, h, d);   
    glutSolidCube(1.0f);
    glPopMatrix();
}

void greenland() {

    // Record the original state
    GLboolean lightingWasOn = glIsEnabled(GL_LIGHTING);
    GLboolean colorMaterialWasOn = glIsEnabled(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glColor3f(0.08f, 0.35f, 0.08f);

    glPushMatrix();
    glTranslatef(-2700.0f, 0.0f, 1000.0f);
    drawland(1500.0f, 5.0f, 2000.0f);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(2300.0f, 0.0f, 500.0f);
    drawland(1500.0f, 5.0f, 2000.0f);
    glPopMatrix();

    // Restore the original lighting / color and material state
    if (lightingWasOn)
        glEnable(GL_LIGHTING);
    else
        glDisable(GL_LIGHTING);

    if (colorMaterialWasOn)
        glEnable(GL_COLOR_MATERIAL);
    else
        glDisable(GL_COLOR_MATERIAL);
}

float randf(float a, float b) {
    return a + (b - a) * (rand() / (float)RAND_MAX);
}

void initForest()
{
    gTrees.clear();
    const float islandTopY = 2.5f;  

    {
        float cx = -2800.0f;
        float cz = 1000.0f;
        float halfW = 1500.0f * 0.5f;
        float halfD = 2000.0f * 0.5f;

        float margin = 90.0f;

        for (int i = 0; i < 60; ++i) {
            TreeInfo t;
            t.x = randf(cx - halfW + margin, cx + halfW - margin);
            t.z = randf(cz - halfD + margin, cz + halfD - margin);
            t.y = islandTopY;                
            t.scale = randf(60.0f, 110.0f);
            gTrees.push_back(t);
        }
    }

    {
        float cx = 2300.0f;
        float cz = 500.0f;
        float halfW = 1500.0f * 0.5f;
        float halfD = 2000.0f * 0.5f;
        float margin = 90.0f;

        for (int i = 0; i < 60; ++i) {
            TreeInfo t;
            t.x = randf(cx - halfW + margin, cx + halfW - margin);
            t.z = randf(cz - halfD + margin, cz + halfD - margin);
            t.y = islandTopY;                 
            t.scale = randf(60.0f, 110.0f);
            gTrees.push_back(t);
        }
    }
}

void drawForest()
{
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);

    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[TEX_TREE]);

    // Alpha test: discard pixels whose alpha = 0 after color-keying tree.bmp
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.1f);

    // Enable blending to soften the edges
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);   // Do not tint the texture

    // ============================
    // Read camera orientation from MODELVIEW matrix
    // ============================
    float mv[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);

    // Camera "right" direction X and Z components in world space
    float rightX = mv[0];
    float rightZ = mv[8];

    // Normalize to avoid unexpected scaling
    float len = sqrtf(rightX * rightX + rightZ * rightZ);
    float rX = 1.0f, rZ = 0.0f;      // Use default direction if len is too small
    if (len > 1e-4f) {
        rX = rightX / len;
        rZ = rightZ / len;
    }

    // Make the quad's +Z normal always face opposite to the viewing direction (billboarding)
    GLfloat billboardMat[16] = {
        rX,   0.0f,  rZ,   0.0f,
        0.0f, 1.0f,  0.0f, 0.0f,
       -rZ,   0.0f,  rX,   0.0f,
        0.0f, 0.0f,  0.0f, 1.0f
    };

    for (const auto& t : gTrees) {
        glPushMatrix();

        // Place the tree at its world position (standing on the sand)
        glTranslatef(t.x, t.y, t.z);

        // Multiply by billboard matrix → make the quad face the camera
        glMultMatrixf(billboardMat);

        // Scale to different sizes
        glScalef(t.scale, t.scale, t.scale);

        // Draw a vertical quad (in X-Y plane, normal along +Z)
        float w = 0.5f;
        float h = 1.0f;

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-w, 0.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(w, 0.0f, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(w, h, 0.0f);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-w, h, 0.0f);
        glEnd();

        glPopMatrix();
    }

    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glPopAttrib();
}



// =====================================================
//   Future City Main Tower
// =====================================================

// 1D quadratic Bézier: controls inner edge distance (half-width) from the center
float bezierQuad(float t, float p0, float p1, float p2)
{
    float u = 1.0f - t;
    // B(t) = (1-t)^2 p0 + 2(1-t)t p1 + t^2 p2
    return u * u * p0 + 2.0f * u * t * p1 + t * t * p2;
}

void drawBezierBuildingBody()
{
    // ===== Basic parameters =====
    const int   layers = 55;
    const float layerH = 10.0f;
    const float depth = 40.0f;
    const float outerHalf = 200.0f;

    // ===== Quadratic Bézier control points for inner edge =====
    float innerBottom = 70.0f;
    float innerMid = 65.0f;
    float innerTop = 0.0f;
    glColorRGB(60, 70, 255);

    // ----------------- Main tower layers -----------------
    for (int i = 0; i < layers; ++i)
    {
        float t = (i + 0.5f) / (float)layers;
        float innerHalf = bezierQuad(t, innerBottom, innerMid, innerTop);
        float blockWidth = outerHalf - innerHalf;
        float yCenter = (i + 0.5f) * layerH;

        // Left block: [-outerHalf, -innerHalf]
        {
            float leftCenterX = -(outerHalf + innerHalf) * 0.5f;
            glPushMatrix();
            glTranslatef(leftCenterX, yCenter, 0.0f);
            glScalef(blockWidth, layerH, depth);
            glutSolidCube(1.0f);
            glPopMatrix();
        }

        // Right block: [innerHalf, outerHalf]
        {
            float rightCenterX = (outerHalf + innerHalf) * 0.5f;
            glPushMatrix();
            glTranslatef(rightCenterX, yCenter, 0.0f);
            glScalef(blockWidth, layerH, depth);
            glutSolidCube(1.0f);
            glPopMatrix();
        }
    }

    // ----------------- Top beams -----------------
    {
        int topBeams = 5;
        float beamH = layerH;
        float yStart = layers * layerH;

        for (int b = 0; b < topBeams; ++b)
        {
            float yCenter = yStart + (b + 0.5f) * beamH;

            glPushMatrix();
            glTranslatef(0.0f, yCenter, 0.0f);       // Span left and right
            glScalef(2.0f * outerHalf, beamH, depth);
            glutSolidCube(1.0f);
            glPopMatrix();
        }
    }
}

void drawBezierBuilding()
{
    glPushMatrix();

    // ------------ First pass: normal filled color rendering ------------
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    drawBezierBuildingBody();

    // ------------ Second pass: outline using glutWireCube ------------
    glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_CULL_FACE);      // Do not cull back faces in wireframe to keep outline complete

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(3.0f);
    glColorRGB(60, 70, 255);

    // ==== Re-declare parameters from drawBezierBuildingBody for wire drawing ====
    const int   layers = 55;
    const float layerH = 10.0f;
    const float depth = 40.0f;
    const float outerHalf = 200.0f;

    float innerBottom = 70.0f;
    float innerMid = 65.0f;
    float innerTop = 0.0f;

    // Each wireframe block is slightly larger than solid to avoid Z-fighting
    const float SCALE_EPS = 1.02f;

    // ---------- Wireframe for main layers ----------
    for (int i = 0; i < layers; ++i)
    {
        float t = (i + 0.5f) / (float)layers;
        float innerHalf = bezierQuad(t, innerBottom, innerMid, innerTop);
        float blockWidth = outerHalf - innerHalf;
        float yCenter = (i + 0.5f) * layerH;

        // Left block: [-outerHalf, -innerHalf]
        {
            float leftCenterX = -(outerHalf + innerHalf) * 0.5f;
            glPushMatrix();
            glTranslatef(leftCenterX, yCenter, 0.0f);
            glScalef(blockWidth * SCALE_EPS,
                layerH * SCALE_EPS,
                depth * SCALE_EPS);
            glutWireCube(1.0f);
            glPopMatrix();
        }

        // Right block: [innerHalf, outerHalf]
        {
            float rightCenterX = (outerHalf + innerHalf) * 0.5f;
            glPushMatrix();
            glTranslatef(rightCenterX, yCenter, 0.0f);
            glScalef(blockWidth * SCALE_EPS,
                layerH * SCALE_EPS,
                depth * SCALE_EPS);
            glutWireCube(1.0f);
            glPopMatrix();
        }
    }

    // ---------- Wireframe for top beams ----------
    {
        int   topBeams = 5;
        float beamH = layerH;
        float yStart = layers * layerH;

        for (int b = 0; b < topBeams; ++b)
        {
            float yCenter = yStart + (b + 0.5f) * beamH;

            glPushMatrix();
            glTranslatef(0.0f, yCenter, 0.0f);  // Span left and right
            glScalef(2.0f * outerHalf * SCALE_EPS,
                beamH * SCALE_EPS,
                depth * SCALE_EPS);
            glutWireCube(1.0f);
            glPopMatrix();
        }
    }

    glPopAttrib();   // Restore line/wireframe/enable states
    glPopMatrix();   // Restore overall matrix
}




void DrawCutCylinder_Solid(float radius, float height, float cutDeg, int slices,
    GLuint texSide, GLuint texTop)
{
    float rad = cutDeg * PI / 180.0f;
    float cosA = cos(rad);
    float sinA = sin(rad);

    // ==================================================
    // Side wall with texture
    // ==================================================
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texSide);

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; i++)
    {
        float t = (float)i / slices;
        float ang = t * 2.0f * PI;

        float x = cos(ang);
        float z = sin(ang);

        float yTop = height + radius * z * sinA;
        float zTop = radius * z * cosA;

        glNormal3f(x, 0, z);

        glTexCoord2f(t, 0);
        glVertex3f(radius * x, 0, radius * z);

        glTexCoord2f(t, 1);
        glVertex3f(radius * x, yTop, zTop);
    }
    glEnd();

    glDisable(GL_TEXTURE_2D);

    // ==================================================
    // ⭐ Bottom (no texture)
    // ==================================================
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, -1, 0);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= slices; i++)
    {
        float a = 2 * PI * i / slices;
        glVertex3f(radius * cos(a), 0, radius * sin(a));
    }
    glEnd();

    // ==================================================
    // ⭐ Top elliptical cap with texture (texTop)
    // ==================================================
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texTop);

    glBegin(GL_TRIANGLE_FAN);

    // Fixed normal direction
    glNormal3f(0, cosA, sinA);

    // Center point
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f(0, height, 0);

    for (int i = 0; i <= slices; i++)
    {
        float ang = 2 * PI * i / slices;
        float x = cos(ang);
        float z = sin(ang);

        float yTop = height + radius * z * sinA;
        float zTop = radius * z * cosA;

        // Top texture UV (circular mapping)
        float u = 0.5f + x * 0.5f;
        float v = 0.5f + z * 0.5f;

        glTexCoord2f(u, v);
        glVertex3f(radius * x, yTop, zTop);
    }

    glEnd();
    glDisable(GL_TEXTURE_2D);
}



// =========================== Ferris Wheel Module ===============================
// Support base (white steel structure)
void FW_DrawBase()
{
    glPushMatrix();
    glColor3f(1.0f, 1.0f, 1.0f); 

    // front right
    glPushMatrix();
    glTranslatef(250.0f, 0.0f, 50.0f);
    glRotatef(90.0f, 1, 0, 0);
    glRotatef(30.0f, 0, 1, 0);
    glScalef(40.0f, 40.0f, 1000.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // front left
    glPushMatrix();
    glTranslatef(-250.0f, 0.0f, 50.0f);
    glRotatef(90.0f, 1, 0, 0);
    glRotatef(-30.0f, 0, 1, 0);
    glScalef(40.0f, 40.0f, 1000.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // rear supports
    glPushMatrix();
    glTranslatef(230.0f, 0.0f, -40.0f);
    glRotatef(90.0f, 1, 0, 0);
    glRotatef(25.0f, 0, 1, 0);
    glScalef(30.0f, 30.0f, 900.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-230.0f, 0.0f, -40.0f);
    glRotatef(90.0f, 1, 0, 0);
    glRotatef(-25.0f, 0, 1, 0);
    glScalef(30.0f, 30.0f, 900.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // central vertical column
    glPushMatrix();
    glTranslatef(0.0f, 380.0f, 0.0f);
    glRotatef(90.0f, 1, 0, 0);
    glScalef(30.0f, 60.0f, 30.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    glPopMatrix();
}

// Dynamic neon light ring
void FW_DrawLights()
{
    glPushMatrix();
    glTranslatef(0.0f, 420.0f, 0.0f);   // wheel center height
    glRotatef(fw_ang, 0, 0, 1);         // rotation of the whole wheel
    for (int i = 0; i < 24; i++) {      // more light strips for a more gorgeous look
        glRotatef(360.0f / 24.0f, 0, 0, 1);
        for (int j = 0; j < 18; j++) {  // number of bulbs on each strip
            // t controls the phase of the flowing lights
            float t = (i * 0.35f + j * 0.18f + fw_lightPhase);
            float r = 0.7f + 0.3f * sin(t);
            float g = 0.7f + 0.3f * sin(t + 2.0f);
            float b = 0.9f + 0.3f * sin(t + 4.0f);
            glPushMatrix();
            glTranslatef(0.0f, j * 20.0f, 0.0f);
            glColor3f(r, g, b);
            glutSolidSphere(8.0f, 16, 16);
            glPopMatrix();
        }
    }
    glPopMatrix();
}


// ====================== Full drone model used as Ferris wheel cabins ======================
void FW_DrawDroneFull()
{
    glPushMatrix();

    glScalef(0.55f, 0.55f, 0.55f);

    // —— Body —— //
    glColor3f(1, 1, 1);
    glPushMatrix();
    glScalef(0.7, 0.5, 1.5);
    glColor3f(0.5, 0.5, 0.5);
    glutSolidCube(0.7);
    glPopMatrix();

    // —— Four arms —— //
    glColor3f(1, 1, 1);
    for (int k = 0; k < 4; k++)
    {
        glPushMatrix();
        glRotatef(45 + 90 * k, 0, 1, 0);
        glutSolidCylinder(0.08, 1, 50, 50);
        glPopMatrix();
    }

    // —— Four vertical supports —— //
    glColor3f(0.6, 0.6, 0.6);
    float s = sqrt(2) / 2.0f;
    float offsets[4][2] = {
        { s,  s},
        {-s,  s},
        {-s, -s},
        { s, -s}
    };

    for (int i = 0; i < 4; i++)
    {
        glPushMatrix();
        glTranslatef(offsets[i][0], 0.15, offsets[i][1]);
        glRotatef(90, 1, 0, 0);
        glutSolidCylinder(0.08, 0.3, 50, 50);
        glPopMatrix();
    }

    // —— Rotating propellers —— //
    glColor3f(1, 1, 1);
    for (int k = 0; k < 4; k++)
    {
        glPushMatrix();
        // 1️⃣ First, same as arms: rotate around Y axis to each direction
        glRotatef(45 + k * 90, 0, 1, 0);

        // 2️⃣ Then move along the arm direction to the end of the arm
        glTranslatef(0.0f, 0.0f, 1.0f);   

        // 3️⃣ Propeller self rotation
        glRotatef(fw_ang * 5, 0, 1, 0);

        // 4️⃣ Draw cross-shaped blades (around current origin)
        glBegin(GL_LINES);
        glVertex3f(0.3f, 0.15f, 0.0f);
        glVertex3f(-0.3f, 0.15f, 0.0f);
        glVertex3f(0.0f, 0.15f, 0.3f);
        glVertex3f(0.0f, 0.15f, -0.3f);
        glEnd();

        glPopMatrix();
    }
    glPopMatrix();
}


// Wheel skeleton
void FW_DrawWheel()
{
    glPushMatrix();
    glTranslatef(0.0f, 420.0f, 0.0f);
    glRotatef(fw_ang, 0, 0, 1);

    glColor3f(1.0f, 1.0f, 1.0f);

    // Outer ring: arrange drones in a circle
    int segs = 24;
    float radiusOuter = 320.0f;

    float offset = 7.5f * PI / 180.0f;  

    for (int i = 0; i < segs; i++) {

        float theta = 2.0f * PI * i / segs + offset;   
        float x = radiusOuter * cos(theta);
        float y = radiusOuter * sin(theta);


        glPushMatrix();

        // Drone position
        glTranslatef(x, y, 0.0f);

        // Make drone face the tangential direction of the circle
        glRotatef(theta * 180.0f / PI + 90, 0, 0, 1);

        glScalef(30, 30, 30);

        FW_DrawDroneFull();

        glPopMatrix();
    }


    // Spokes
    int spokeNum = 12;
    for (int i = 0; i < spokeNum; i++) {
        glPushMatrix();
        glRotatef(360.0f / spokeNum * i, 0, 0, 1);
        glTranslatef(0.0f, radiusOuter / 2.0f, 0.0f);
        glScalef(8.0f, radiusOuter, 8.0f);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    // Central hub
    glPushMatrix();
    glColor3f(0.95f, 0.95f, 0.98f);
    glutSolidSphere(35.0f, 24, 24);
    glPopMatrix();

    glPopMatrix();
}

// Decide whether it's day or night according to daytime
bool isDayTime()
{
    // Same theta as in sun_moon()
    double theta = 2 * PI * daytime / 2400.0;
    double sy = sin(theta);   // sun's y (height)

    // If sy > 0, the sun is above the horizon → daytime
    return sy > 0;
}

bool isNightTime()
{
    return !isDayTime();
}

void DrawFerrisWheel()
{
    glPushMatrix();

    // Base is always white steel structure
    FW_DrawBase();

    // Wheel body
    FW_DrawWheel();

    // Turn on neon lights only at night
    if (isNightTime()) {
        FW_DrawLights();   // Night view: colorful flowing lights
    }
    glPopMatrix();
}

// Compute position and orientation on a figure-8 path
void getDroneFigure8Pos(float t, float& x, float& y, float& z, float& yawDeg)
{
    // Direction vector u connecting the two towers
    float ax = BEZ_TOWER_X;
    float az = BEZ_TOWER_Z;
    float bx = CYL_TOWER_X;
    float bz = CYL_TOWER_Z;

    // Midpoint (center of the figure-8 is between the two towers)
    float cx = 0.5f * (ax + bx);
    float cz = 0.5f * (az + bz);

    float ux = bx - ax;
    float uz = bz - az;
    float len = sqrtf(ux * ux + uz * uz);
    if (len < 1e-3f) len = 1.0f;
    ux /= len;
    uz /= len;

    // Perpendicular direction v (also in the XZ plane)
    float vx = -uz;
    float vz = ux;

    // Two radii of the figure-8 in the u/v plane
    float R_long = len * 0.60f;    // Radius along the tower direction (larger)
    float R_short = R_long * 0.5f; // Radius in the perpendicular direction (smaller)

    // Use Gerono lemniscate: x = a sin t, y = a sin t cos t
    float f = sinf(t);             // Along the u direction
    float g = sinf(t) * cosf(t);   // Along the v direction (forming the 8-shape)

    x = cx + ux * (R_long * f) + vx * (R_short * g);
    z = cz + uz * (R_long * f) + vz * (R_short * g);
    y = DRONE_FLY_Y;

    // Compute tangent direction to determine drone heading
    float dfdt = cosf(t);          // f'(t) = cos t
    float dgdt = cosf(2.0f * t);   // g(t) = sin t cos t = 0.5 sin(2t) → g'(t) = cos(2t)

    float tx = ux * (R_long * dfdt) + vx * (R_short * dgdt);
    float tz = uz * (R_long * dfdt) + vz * (R_short * dgdt);

    yawDeg = atan2f(tx, tz) * 180.0f / PI;  // Face along the velocity direction
}

// Ten scaled-up drones flying in a figure-8 between the two towers
void drawFlyingDrones()
{
    const float TWO_PI = 2.0f * PI;
    float phaseStep = TWO_PI / NUM_FLY_DRONES;

    for (int i = 0; i < NUM_FLY_DRONES; ++i)
    {
        float t = dronePhase + i * phaseStep;

        float x, y, z, yaw;
        getDroneFigure8Pos(t, x, y, z, yaw);

        glPushMatrix();
        glTranslatef(x, y, z);
        // Drone faces tangential direction
        glRotatef(yaw, 0.0f, 1.0f, 0.0f);
        glScalef(60.0f, 60.0f, 60.0f);
        FW_DrawDroneFull();

        glPopMatrix();
    }
}

// =====================
// Trapezoid
// =====================
void drawTrapezoid(GLfloat topWidth, GLfloat bottomWidth, GLfloat height, GLfloat depth) {
    glTranslatef(0.0f, -1.0f, 0.0f);

    glPushMatrix();
    glutSolidCube(topWidth);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(topWidth / 2, -topWidth * 0.2f, 0.0f);
    glRotatef(135.0f, 0.0f, 0.0f, 1.0f);
    glutSolidCube(topWidth);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-topWidth / 2, -topWidth * 0.2f, 0.0f);
    glRotatef(135.0f, 0.0f, 0.0f, 1.0f);
    glutSolidCube(topWidth);
    glPopMatrix();
}

// =====================
// CB
// =====================
void drawCB()
{
    GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);
    if (cullWasOn) glDisable(GL_CULL_FACE);


    glPushMatrix();
    glColor3f(0.0f, 0.0f, 0.0f);
    drawTrapezoid(6.0f, 1.5f, 1.0f, 1.0f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    glTranslatef(0.0f, 2.0f, 0.0f);
    drawTrapezoid(4.0f, 1.5f, 1.0f, 1.0f);
    glPopMatrix();


    float L = 2.3f;   // Half side length
    float H = 2.3f;   // Half height

    GLuint texFront = texture[7];
    GLuint texSide = texture[8];

    glPushMatrix();
    glTranslatef(0.0f, 4.0f, 0.0f);

    glEnable(GL_TEXTURE_2D);
    glColor3f(1, 1, 1);

    // ---------- Front wall (+Z) ----------
    glBindTexture(GL_TEXTURE_2D, texFront);
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-L, -H, L);
    glTexCoord2f(1, 0); glVertex3f(L, -H, L);
    glTexCoord2f(1, 1); glVertex3f(L, H, L);
    glTexCoord2f(0, 1); glVertex3f(-L, H, L);
    glEnd();

    // ---------- Back wall (-Z) ----------
    glBindTexture(GL_TEXTURE_2D, texFront);
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(L, -H, -L);
    glTexCoord2f(1, 0); glVertex3f(-L, -H, -L);
    glTexCoord2f(1, 1); glVertex3f(-L, H, -L);
    glTexCoord2f(0, 1); glVertex3f(L, H, -L);
    glEnd();

    // ---------- Right wall (+X) ----------
    glBindTexture(GL_TEXTURE_2D, texSide);
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(L, -H, -L);
    glTexCoord2f(1, 0); glVertex3f(L, -H, L);
    glTexCoord2f(1, 1); glVertex3f(L, H, L);
    glTexCoord2f(0, 1); glVertex3f(L, H, -L);
    glEnd();

    // ---------- Left wall (-X) ----------
    glBindTexture(GL_TEXTURE_2D, texSide);
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0, 0); glVertex3f(-L, -H, L);
    glTexCoord2f(1, 0); glVertex3f(-L, -H, -L);
    glTexCoord2f(1, 1); glVertex3f(-L, H, -L);
    glTexCoord2f(0, 1); glVertex3f(-L, H, L);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    // ---------- Top / bottom faces ----------
    glColor3f(0.65f, 0.65f, 0.65f);
    glBegin(GL_QUADS);

    // Top
    glNormal3f(0, 1, 0);
    glVertex3f(-L, H, -L);
    glVertex3f(L, H, -L);
    glVertex3f(L, H, L);
    glVertex3f(-L, H, L);

    // Bottom
    glNormal3f(0, -1, 0);
    glVertex3f(-L, -H, L);
    glVertex3f(L, -H, L);
    glVertex3f(L, -H, -L);
    glVertex3f(-L, -H, -L);

    glEnd();

    glPopMatrix();


    glPushMatrix();
    glTranslatef(0.0f, 4.0f, 0.0f);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    glColor4f(0.5f, 0.65f, 0.82f, 0.35f);
    glutSolidCube(4.5f);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();

    if (cullWasOn) glEnable(GL_CULL_FACE);
}


void drawGlassTower(float w, float h, float d, int sideTex)
{
    GLboolean wasColor = glIsEnabled(GL_COLOR_MATERIAL);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    float x = w * 0.5f;
    float y = h * 0.5f;
    float z = d * 0.5f;

    // Gray material used for top and bottom faces
    GLfloat amb[] = { 0.7f, 0.7f, 0.7f, 1.0f };
    GLfloat diff[] = { 0.7f, 0.7f, 0.7f, 1.0f };
    GLfloat spec[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);

    // Four side faces with texture (repeated along height) =
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[sideTex]);

    // Allow texture to repeat
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    float repeat = h / 4.0f;   // Larger h → larger repeat → texture will not be stretched


    // ----------------- Front side -----------------
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0);       glVertex3f(-x, -y, z);
    glTexCoord2f(1, 0);       glVertex3f(x, -y, z);
    glTexCoord2f(1, repeat);      glVertex3f(x, y, z);
    glTexCoord2f(0, repeat);      glVertex3f(-x, y, z);
    glEnd();

    // ----------------- Back side -----------------
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0);       glVertex3f(x, -y, -z);
    glTexCoord2f(1, 0);       glVertex3f(-x, -y, -z);
    glTexCoord2f(1, repeat);      glVertex3f(-x, y, -z);
    glTexCoord2f(0, repeat);      glVertex3f(x, y, -z);
    glEnd();

    // ----------------- Left side -----------------
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0, 0);       glVertex3f(-x, -y, -z);
    glTexCoord2f(1, 0);       glVertex3f(-x, -y, z);
    glTexCoord2f(1, repeat);      glVertex3f(-x, y, z);
    glTexCoord2f(0, repeat);      glVertex3f(-x, y, -z);
    glEnd();

    // ----------------- Right side -----------------
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glTexCoord2f(0, 0);       glVertex3f(x, -y, z);
    glTexCoord2f(1, 0);       glVertex3f(x, -y, -z);
    glTexCoord2f(1, repeat);      glVertex3f(x, y, -z);
    glTexCoord2f(0, repeat);      glVertex3f(x, y, z);
    glEnd();

    glDisable(GL_TEXTURE_2D);



    //  Top face
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-x, y, -z);
    glVertex3f(x, y, -z);
    glVertex3f(x, y, z);
    glVertex3f(-x, y, z);
    glEnd();

    //  Bottom face
    glBegin(GL_QUADS);
    glNormal3f(0, -1, 0);
    glVertex3f(-x, -y, z);
    glVertex3f(x, -y, z);
    glVertex3f(x, -y, -z);
    glVertex3f(-x, -y, -z);
    glEnd();

    if (wasColor) glEnable(GL_COLOR_MATERIAL);
}



// Frustum platform
void drawPlatform(int texID)
{
    glDisable(GL_CULL_FACE);
    GLboolean wasColorMat = glIsEnabled(GL_COLOR_MATERIAL);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    float top = 1.0f;
    float bottom = 0.6f;
    float h = 5.0f;
    float yTop = h * 0.5f;
    float yBot = -h * 0.5f;

    // Fixed gray material
    GLfloat amb[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat diff[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat spec[] = { 0.1f, 0.1f, 0.1f, 1.0f };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);



    // Side faces
    glBegin(GL_QUADS);

    // Front
    glNormal3f(0, 0, 1);
    glVertex3f(-top, yTop, top);
    glVertex3f(top, yTop, top);
    glVertex3f(bottom, yBot, bottom);
    glVertex3f(-bottom, yBot, bottom);

    // Back
    glNormal3f(0, 0, -1);
    glVertex3f(-top, yTop, -top);
    glVertex3f(-bottom, yBot, -bottom);
    glVertex3f(bottom, yBot, -bottom);
    glVertex3f(top, yTop, -top);

    // Left
    glNormal3f(-1, 0, 0);
    glVertex3f(-top, yTop, -top);
    glVertex3f(-top, yTop, top);
    glVertex3f(-bottom, yBot, bottom);
    glVertex3f(-bottom, yBot, -bottom);

    // Right
    glNormal3f(1, 0, 0);
    glVertex3f(top, yTop, top);
    glVertex3f(top, yTop, -top);
    glVertex3f(bottom, yBot, -bottom);
    glVertex3f(bottom, yBot, bottom);

    glEnd();

    // Top face with texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[texID]);

    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);

    glTexCoord2f(0, 0); glVertex3f(-top, yTop, -top);
    glTexCoord2f(1, 0); glVertex3f(top, yTop, -top);
    glTexCoord2f(1, 1); glVertex3f(top, yTop, top);
    glTexCoord2f(0, 1); glVertex3f(-top, yTop, top);

    glEnd();

    glDisable(GL_TEXTURE_2D);

    // Bottom face
    glBegin(GL_QUADS);
    glNormal3f(0, -1, 0);
    glVertex3f(-bottom, yBot, -bottom);
    glVertex3f(bottom, yBot, -bottom);
    glVertex3f(bottom, yBot, bottom);
    glVertex3f(-bottom, yBot, bottom);
    glEnd();
    if (wasColorMat) glEnable(GL_COLOR_MATERIAL);
}


void drawSimpleCylinder(float radius, float h, int slices = 16)
{
    const float TWO_PI = 6.28318530718f;

    // Side surface
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i)
    {
        float ang = (TWO_PI * i) / slices;
        float x = cosf(ang);
        float z = sinf(ang);

        glNormal3f(x, 0.0f, z);
        glVertex3f(radius * x, 0.0f, radius * z);
        glVertex3f(radius * x, h, radius * z);
    }
    glEnd();
}

// Billboard:
void drawBillboardLocal(float poleH, float poleR,
    float boardW, float boardH,
    int texId)
{
    drawSimpleCylinder(poleR, poleH);

    float halfW = boardW * 0.5f;
    float halfH = boardH * 0.5f;
    float boardT = 0.4f;
    float halfT = boardT * 0.5f;

    float yBottom = poleH;
    float yCenter = yBottom + halfH;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[texId]);
    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);
    // Front face  Z+
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-halfW, yCenter - halfH, halfT);
    glTexCoord2f(1, 0); glVertex3f(halfW, yCenter - halfH, halfT);
    glTexCoord2f(1, 1); glVertex3f(halfW, yCenter + halfH, halfT);
    glTexCoord2f(0, 1); glVertex3f(-halfW, yCenter + halfH, halfT);

    // Back face  Z-
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(halfW, yCenter - halfH, -halfT);
    glTexCoord2f(1, 0); glVertex3f(-halfW, yCenter - halfH, -halfT);
    glTexCoord2f(1, 1); glVertex3f(-halfW, yCenter + halfH, -halfT);
    glTexCoord2f(0, 1); glVertex3f(halfW, yCenter + halfH, -halfT);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    glColor3f(0.8f, 0.8f, 0.85f);

    glBegin(GL_QUADS);
    // Left side  X-
    glNormal3f(-1, 0, 0);
    glVertex3f(-halfW, yCenter - halfH, -halfT);
    glVertex3f(-halfW, yCenter - halfH, halfT);
    glVertex3f(-halfW, yCenter + halfH, halfT);
    glVertex3f(-halfW, yCenter + halfH, -halfT);

    // Right side  X+
    glNormal3f(1, 0, 0);
    glVertex3f(halfW, yCenter - halfH, halfT);
    glVertex3f(halfW, yCenter - halfH, -halfT);
    glVertex3f(halfW, yCenter + halfH, -halfT);
    glVertex3f(halfW, yCenter + halfH, halfT);

    // Top side  Y+
    glNormal3f(0, 1, 0);
    glVertex3f(-halfW, yCenter + halfH, -halfT);
    glVertex3f(halfW, yCenter + halfH, -halfT);
    glVertex3f(halfW, yCenter + halfH, halfT);
    glVertex3f(-halfW, yCenter + halfH, halfT);

    // Bottom side  Y-
    glNormal3f(0, -1, 0);
    glVertex3f(-halfW, yCenter - halfH, halfT);
    glVertex3f(halfW, yCenter - halfH, halfT);
    glVertex3f(halfW, yCenter - halfH, -halfT);
    glVertex3f(-halfW, yCenter - halfH, -halfT);
    glEnd();
}


void drawThreeTowersPattern(int patternId, float baseY, float hBase)
{
    const float platformTopY = baseY + 2.5f;
    float gap = 0.8f;

    // Default sizes for three towers
    float hA = hBase * 0.8f;
    float hB = hBase * 1.2f;
    float hC = hBase * 1.6f;

    float wA = 3.0f, dA = 3.0f;
    float wB = 4.0f, dB = 3.5f;
    float wC = 3.2f, dC = 4.0f;

    switch (patternId)
    {

    case 100:
    {
        const int N = 9;

        float rInner = 4.0f;
        float rOuter = 7.0f;

        float posX[N] = {
            0.0f,
            0.0f,  0.0f,
           -rInner,  rInner,
           -rOuter,  rOuter,
           -rOuter,  rOuter
        };

        float posZ[N] = {
            0.0f,
            rInner, -rInner,
            0.0f,   0.0f,
            rOuter, rOuter,
           -rOuter,-rOuter
        };

        // Three height levels
        float heights[N];
        heights[0] = hBase * 1.7f;
        for (int i = 1; i <= 4; ++i)
            heights[i] = hBase * 1.25f;
        for (int i = 5; i < N; ++i)
            heights[i] = hBase * 0.9f;

        float wCenter = 3.6f, dCenter = 3.6f;
        float wInner = 3.0f, dInner = 3.0f;
        float wOuter = 2.4f, dOuter = 2.4f;

        // -------- First draw 9 towers --------
        for (int i = 0; i < N; ++i)
        {
            float hi = heights[i];
            float wi, di;

            if (i == 0)          wi = wCenter, di = dCenter;
            else if (i <= 4)     wi = wInner, di = dInner;
            else                 wi = wOuter, di = dOuter;

            glPushMatrix();
            glTranslatef(posX[i], platformTopY + hi * 0.5f, posZ[i]);
            drawGlassTower(wi, hi, di, 15);
            glPopMatrix();
        }

        // -------- Insert billboards between each pair of outer towers --------
        int outerIdx[4] = { 5, 6, 8, 7 }; // Loop around once

        // 4 billboards using 4 textures
        int boardTexList[4] = { 23,24,25,26 };

        float poleH = hBase * 0.35f;
        float poleR = 0.25f;
        float boardW = 3.0f;
        float boardH = 1.6f;

        for (int k = 0; k < 4; ++k)
        {
            int i1 = outerIdx[k];
            int i2 = outerIdx[(k + 1) % 4];

            // Midpoint of two outer towers
            float mx = 0.5f * (posX[i1] + posX[i2]);
            float mz = 0.5f * (posZ[i1] + posZ[i2]);

            float r = sqrtf(mx * mx + mz * mz);
            if (r > 0.001f)
            {
                float extra = 0.8f;
                float scale = (r + extra) / r;
                mx *= scale;
                mz *= scale;
            }

            float baseY = platformTopY;

            float angleRad = atan2f(mz, mx);
            float angleDeg = angleRad * 180.0f / PI;

            glPushMatrix();
            glTranslatef(mx, baseY, mz);
            glRotatef(90.0f - angleDeg, 0.0f, 1.0f, 0.0f);

            // ✅ The k-th billboard uses the k-th texture
            int texId = boardTexList[k];
            drawBillboardLocal(poleH, poleR, boardW, boardH, texId);

            glPopMatrix();
        }

        break;
    }
    default:
        break;
    }
}

// Single floating platform with buildings
void drawBuilding(float x, float y, float z, float height, int patternId, int platformTex)
{
    glPushMatrix();

    glTranslatef(x, y, z);
    glScalef(5.0f, 5.0f, 5.0f);

    float platformW = 10.0f;

    glPushMatrix();
    glTranslatef(0, 0, 0);
    glScalef(platformW, 1, platformW);
    drawPlatform(platformTex);
    glPopMatrix();

    drawThreeTowersPattern(patternId, 0, height);

    glPopMatrix();
}

void drawOrbitBalls(float platformX, float platformY, float platformZ)
{
    float orbitRadius = 80.0f;
    float ballRadius = 5.0f;

    // Turn off lighting
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glColor3f(1, 1, 1);

    // Ball 1
    float rad1 = orbitAngle * PI / 180.0f;
    glPushMatrix();
    glTranslatef(platformX + orbitRadius * cos(rad1),
        platformY,
        platformZ + orbitRadius * sin(rad1));
    drawball_tx(16, ballRadius);
    glPopMatrix();

    // Ball 2 (opposite side)
    float rad2 = (orbitAngle + 180.0f) * PI / 180.0f;
    glPushMatrix();
    glTranslatef(platformX + orbitRadius * cos(rad2),
        platformY,
        platformZ + orbitRadius * sin(rad2));
    drawball_tx(17, ballRadius);
    glPopMatrix();

    // Restore state
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
}

void initFloatingPlatforms()
{
    float R = 320.0f;

    float phi = 40.0f * PI / 180.0f;
    float ringR = R * sin(phi);
    float ringY = R * cos(phi);

    float heights[6] = { 12.0f, 15.0f, 18.0f, 14.0f, 20.0f, 16.0f };

    int N = 6;
    for (int i = 0; i < N; ++i)
    {
        float theta = 2.0f * PI * i / (float)N;

        gPlatforms[i].x = ringR * cos(theta);
        gPlatforms[i].z = ringR * sin(theta);
        gPlatforms[i].y = ringY;
        gPlatforms[i].h = heights[i];
    }
}

void drawFloatingCity()
{
    int texMap[6] = { 9, 10, 11, 12, 13, 14 };

    for (int i = 0; i < 6; i++)
    {
        int patternUsed = 100;

        drawBuilding(
            gPlatforms[i].x,
            gPlatforms[i].y,
            gPlatforms[i].z,
            gPlatforms[i].h,
            patternUsed,
            texMap[i]
        );

        // Orbiting spheres that revolve with the platform around the center
        float orbitCenterY = gPlatforms[i].y + 100.0f;
        drawOrbitBalls(gPlatforms[i].x, orbitCenterY, gPlatforms[i].z);
    }
}

// Tech-style wireframe tower
void drawTechWireTower(float w, float h, float d)
{
    // Save original OpenGL states for later restore
    GLboolean wasLighting = glIsEnabled(GL_LIGHTING);
    GLboolean wasTex = glIsEnabled(GL_TEXTURE_2D);
    GLboolean wasBlend = glIsEnabled(GL_BLEND);
    GLboolean wasColorMat = glIsEnabled(GL_COLOR_MATERIAL);

    // For wireframe tower we use glColor directly, not material
    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);  
    glLineWidth(2.0f);

    float x = w * 0.5f;
    float y = h * 0.5f;
    float z = d * 0.5f;

    auto drawBox6Faces = [&]()
        {
            glBegin(GL_QUADS);
            // Front face Z+
            glVertex3f(-x, -y, z);
            glVertex3f(x, -y, z);
            glVertex3f(x, y, z);
            glVertex3f(-x, y, z);

            // Back face Z-
            glVertex3f(x, -y, -z);
            glVertex3f(-x, -y, -z);
            glVertex3f(-x, y, -z);
            glVertex3f(x, y, -z);

            // Left face X-
            glVertex3f(-x, -y, -z);
            glVertex3f(-x, -y, z);
            glVertex3f(-x, y, z);
            glVertex3f(-x, y, -z);

            // Right face X+
            glVertex3f(x, -y, z);
            glVertex3f(x, -y, -z);
            glVertex3f(x, y, -z);
            glVertex3f(x, y, z);

            // Top face Y+
            glVertex3f(-x, y, -z);
            glVertex3f(x, y, -z);
            glVertex3f(x, y, z);
            glVertex3f(-x, y, z);

            // Bottom face Y-
            glVertex3f(-x, -y, z);
            glVertex3f(x, -y, z);
            glVertex3f(x, -y, -z);
            glVertex3f(-x, -y, -z);
            glEnd();
        };

    // Neon wireframe skeleton: multiple stacked wireframe layers
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Switch wireframe color based on day/night
    if (isNightTime()) {
        glColor3f(0.30f, 0.00f, 0.45f);  
    }
    else {
        glColor3f(0.2f, 0.9f, 1.0f);
    }

    // Many "floor" wireframes along height
    int slices = 24;             
    float dy = h / slices;

    // Multiple rectangle loops: one rectangle per level
    for (int i = 0; i <= slices; ++i)
    {
        float yy = -y + dy * i;

        glBegin(GL_LINE_LOOP);
        glVertex3f(-x, yy, -z);
        glVertex3f(x, yy, -z);
        glVertex3f(x, yy, z);
        glVertex3f(-x, yy, z);
        glEnd();
    }

    glBegin(GL_LINES);
    // Front-left vertical edge
    glVertex3f(-x, -y, z);
    glVertex3f(-x, y, z);
    // Front-right vertical edge
    glVertex3f(x, -y, z);
    glVertex3f(x, y, z);
    // Back-left vertical edge
    glVertex3f(-x, -y, -z);
    glVertex3f(-x, y, -z);
    // Back-right vertical edge
    glVertex3f(x, -y, -z);
    glVertex3f(x, y, -z);
    glEnd();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4f(0.05f, 0.25f, 0.45f, 0.90f); // Slightly darker to highlight the wireframe

    drawBox6Faces();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Restore states
    if (!wasBlend)    glDisable(GL_BLEND);
    if (!wasLighting) glDisable(GL_LIGHTING); else glEnable(GL_LIGHTING);
    if (wasTex)       glEnable(GL_TEXTURE_2D); else glDisable(GL_TEXTURE_2D);
    if (wasColorMat)  glEnable(GL_COLOR_MATERIAL); else glDisable(GL_COLOR_MATERIAL);
}


// Three wireframe towers arranged in a diagonal line
void drawThreeAlignedTechTowers(
    float baseX, float baseY, float baseZ,
    float angleDeg,
    float spacing,
    float w, float h, float d
)
{
    float rad = angleDeg * PI / 180.0f;
    float dirX = cos(rad);
    float dirZ = sin(rad);

    for (int i = -1; i <= 1; ++i)
    {
        float offset = spacing * i;

        float cx = baseX + offset * dirX;
        float cz = baseZ + offset * dirZ;
        float cy = baseY + h * 0.5f; 

        glPushMatrix();
        glTranslatef(cx, cy, cz);
        drawTechWireTower(w, h, d);

        glPopMatrix();
    }
}

void drawBezierSideClusters()
{
    const float towerX = 1800.0f;
    const float towerZ = 500.0f;

    const float groundY = 0.0f;
    const float sideOffsetX = 600.0f;

    const float towerW = 80.0f;
    const float towerH = 400.0f;
    const float towerD = 80.0f;

    const float spacing = 150.0f;

    // Original left/right positions (before rotation)
    float Lx = towerX - sideOffsetX;
    float Lz = towerZ;

    float Rx = towerX + sideOffsetX;
    float Rz = towerZ;

    // Rotate 90° counterclockwise (around Bezier tower)
    float angle90_cos = 0.0f;
    float angle90_sin = 1.0f;

    auto rot90 = [&](float x, float z, float& outX, float& outZ)
        {
            float dx = x - towerX;
            float dz = z - towerZ;

            outX = towerX + (-dz);
            outZ = towerZ + (dx);
        };

    float Lx2, Lz2, Rx2, Rz2;
    rot90(Lx, Lz, Lx2, Lz2);
    rot90(Rx, Rz, Rx2, Rz2);

    // Direction angles also rotated by 90°
    float leftDir = 45.0f + 90.0f;   
    float rightDir = -45.0f + 90.0f;  

    drawThreeAlignedTechTowers(
        Lx2, groundY, Lz2,
        leftDir,
        spacing,
        towerW, towerH, towerD
    );

    drawThreeAlignedTechTowers(
        Rx2, groundY, Rz2,
        rightDir,
        spacing,
        towerW, towerH, towerD
    );
}


// Initialize high-speed trains + cars
void initCarsAndTrains()
{
    // Initialize two high-speed train tracks = 4 trains
    float trainZ1 = 1450.0f;
    float trainZ2 = -450.0f;
    float groundY = -300.0f;
    float airOffset = 1000.0f;

    // Track 1: left → right
    g_trainPaths[0].x = TRAIN_MIN_X;
    g_trainPaths[0].z = trainZ1;
    g_trainPaths[0].yGround = groundY;
    g_trainPaths[0].yAir = groundY + airOffset;
    g_trainPaths[0].speed = 25.0f;
    g_trainPaths[0].dir = +1;      // move from left to right

    // Track 2: right → left
    g_trainPaths[1].x = TRAIN_MAX_X;
    g_trainPaths[1].z = trainZ2;
    g_trainPaths[1].yGround = groundY;
    g_trainPaths[1].yAir = groundY + airOffset;
    g_trainPaths[1].speed = 25.0f;
    g_trainPaths[1].dir = -1;      // move from right to left

    // Ground cars: 12 cars, all have different z in [-200, 2000]
    float zMin = -200.0f;
    float zMax = 2000.0f;
    float zStep = (zMax - zMin) / (NUM_GROUND_CARS - 1);

    float baseSpeed = 12.0f;   // base speed
    float speedStep = 1.5f;    // add a bit for each car so speeds differ

    for (int i = 0; i < NUM_GROUND_CARS; ++i) {
        float z = zMin + zStep * i;        // evenly distributed in [-200, 2000]
        float speed = baseSpeed + speedStep * (i % 6);

        if (i < 6) {
            // First 6 cars: left -> right, starting from left
            g_groundCars[i].x = TRAIN_MIN_X - 200.0f * i;
            g_groundCars[i].dir = +1;
        }
        else {
            // Last 6 cars: right -> left, starting from right
            int j = i - 6;
            g_groundCars[i].x = TRAIN_MAX_X + 200.0f * j;
            g_groundCars[i].dir = -1;
        }
        g_groundCars[i].y = 0;
        g_groundCars[i].z = z;
        g_groundCars[i].speed = speed;
    }

    // =======================
    // Hover cars: 6 cars, moving along a circle in the sky
    // =======================
    float hoverY = 3200.0f;
    float hoverRadius = 4600.0f;

    for (int i = 0; i < NUM_HOVER_CARS; ++i) {
        g_hoverCars[i].radius = hoverRadius;
        g_hoverCars[i].y = hoverY;

        // Evenly distributed along the circle
        g_hoverCars[i].angle = 2.0f * PI * i / NUM_HOVER_CARS;

        // Each car has slightly different angular speed to look more natural
        g_hoverCars[i].speed = 0.01f;
    }

}

float flameTime = 0.0f;
void drawHoverFlame(float x, float y, float z, float scale)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale, scale);

    float flicker = sin(flameTime * 18.0f) * 0.25f;
    float swirl = sin(flameTime * 10.0f) * 0.18f;

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // Core flame
    glColor4f(1.0f, 0.85f, 0.35f, 0.60f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= 360; i += 30)
    {
        float rad = i * 3.14159f / 180.0f;
        glVertex3f(
            sin(rad) * 0.12f + swirl * 0.2f,
            -0.75f - flicker,
            cos(rad) * 0.12f - swirl * 0.2f
        );
    }
    glEnd();

    // Middle flame layer
    glColor4f(1.0f, 0.35f, 0.02f, 0.75f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= 360; i += 30)
    {
        float rad = i * 3.14159f / 180.0f;
        glVertex3f(
            sin(rad) * (0.55f + swirl * 0.35f),
            -1.85f - flicker * 1.5f,
            cos(rad) * (0.55f + swirl * 0.35f)
        );
    }
    glEnd();

    // Outer red flame fog (further enhancing red glow)
    glColor4f(1.0f, 0.07f, 0.0f, 0.45f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= 360; i += 30)
    {
        float rad = i * 3.14159f / 180.0f;
        glVertex3f(
            sin(rad) * (1.20f + swirl * 0.40f),
            -2.70f - flicker * 2.2f,
            cos(rad) * (1.20f + swirl * 0.40f)
        );
    }
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glPopMatrix();
}


void drawCar(GLfloat x, GLfloat y, GLfloat z)
{
    glPushMatrix();
    glTranslatef(x, y + 0.2f, z);

    // Main car body (streamlined)
    glPushMatrix();
    glColor3f(0.1f, 0.15f, 0.6f);
    glTranslatef(1.0f, 0.6f, 0.0f);
    glScalef(2.6f, 0.8f, 1.4f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Aerodynamic sharp front nose
    glPushMatrix();
    glColor3f(0.12f, 0.18f, 0.75f);
    glBegin(GL_TRIANGLES);
    glVertex3f(-0.3f, 0.4f, 0.7f);
    glVertex3f(-0.3f, 0.4f, -0.7f);
    glVertex3f(0.9f, 0.8f, 0.0f);
    glEnd();
    glPopMatrix();

    // Front windshield (slanted)
    glPushMatrix();
    glColor4f(0.3f, 0.7f, 1.0f, 0.8f);
    glTranslatef(0.0f, 1.1f, 0.0f);
    glRotatef(60, 0, 0, 1);
    glScalef(0.9f, 0.02f, 1.2f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Rear windshield (reverse slope)
    glPushMatrix();
    glColor4f(0.3f, 0.7f, 1.0f, 0.8f);
    glTranslatef(2.0f, 1.05f, 0.0f);
    glRotatef(-70, 0, 0, 1);
    glScalef(0.9f, 0.02f, 1.2f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Top panoramic glass (connect front and rear glass)
    glPushMatrix();
    glColor4f(0.3f, 0.8f, 1.0f, 0.6f);
    glTranslatef(1.0f, 1.45f, 0.0f);
    glScalef(1.7f, 0.03f, 1.25f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Tail light
    glPushMatrix();
    glColor3f(1.0f, 0.1f, 0.1f);
    glTranslatef(2.5f, 0.6f, 0.0f);
    glScalef(0.1f, 0.1f, 1.3f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Wheels
    auto drawWheel = [](float wx, float wz)
        {
            glPushMatrix();
            glTranslatef(wx, 0.2f, wz);

            // Outer ring
            glColor3f(0, 0, 0);
            glutSolidTorus(0.15, 0.35, 20, 20);

            // Inner bright hub
            glColor3f(0.2f, 0.9f, 1.0f);
            glutSolidCylinder(0.25f, 0.1f, 20, 20);

            glPopMatrix();
        };

    drawWheel(0.2f, 0.75f);
    drawWheel(0.2f, -0.75f);
    drawWheel(2.2f, 0.75f);
    drawWheel(2.2f, -0.75f);

    // Tech metal plate on the roof
    glPushMatrix();
    glColor3f(0.5f, 0.8f, 1.0f);
    glBegin(GL_TRIANGLES);
    glVertex3f(1.2f, 1.25f, -0.4f);
    glVertex3f(1.8f, 1.25f, -0.4f);
    glVertex3f(1.5f, 1.55f, 0.0f);
    glEnd();
    glPopMatrix();

    // Hover flame thrusters (under the car)
    glPushMatrix();
    glTranslatef(0, 0.25f, 0);

    // Front-left
    drawHoverFlame(0.4f, 0.0f, 0.6f, 0.6f);
    // Rear-left
    drawHoverFlame(1.8f, 0.0f, 0.6f, 0.7f);

    // Front-right
    drawHoverFlame(0.4f, 0.0f, -0.6f, 0.6f);
    // Rear-right
    drawHoverFlame(1.8f, 0.0f, -0.6f, 0.7f);

    glPopMatrix();

    glPopMatrix();
}

// 12 ground cars
void drawGroundCars()
{
    for (int i = 0; i < NUM_GROUND_CARS; ++i) {
        CarMover& c = g_groundCars[i];

        glPushMatrix();
        glTranslatef(c.x, c.y, c.z);

        // If dir < 0, car front faces -X
        if (c.dir < 0) {
            glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
        }

        glScalef(50.0f, 50.0f, 50.0f);
        drawCar(0.0f, 0.0f, 0.0f);
        glPopMatrix();
    }
}

// 6 hover cars: circular orbit
void drawHoverCars()
{
    for (int i = 0; i < NUM_HOVER_CARS; ++i) {
        HoverCar& h = g_hoverCars[i];

        float cx = h.radius * cos(h.angle);
        float cz = h.radius * sin(h.angle);
        float cy = h.y;

        glPushMatrix();
        glTranslatef(cx, cy, cz);
        glScalef(80.0f, 80.0f, 80.0f);
        drawCar(0.0f, 0.0f, 0.0f);
        glPopMatrix();
    }
}

// Cubic exhaust flame (emitted along -X direction from baseX)

void setupMaterial(float red, float green, float blue) {

    // Set material properties for the object
    GLfloat material_ambient[] = { red, green, blue, 1.0 };
    GLfloat material_diffuse[] = { red, green, blue, 1.0 };
    GLfloat material_specular[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat material_shininess[] = { 50.0 };

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, material_shininess);
}

void drawCube(float X_length, float Y_length, float Z_length, float centerX, float centerY, float centerZ, float xAngle, float yAngle, float zAngle, float red, float green, float blue, GLuint Texture) {
    setupMaterial(red, green, blue);

    float halfWidth = X_length / 2.0;
    float halfHeight = Y_length / 2.0;
    float halfDepth = Z_length / 2.0;

    // Calculate the vertices of the parallelepiped based on the center position
    float vertices[8][3] = {
        {centerX - halfWidth, centerY - halfHeight, centerZ + halfDepth},
        {centerX + halfWidth, centerY - halfHeight, centerZ + halfDepth},
        {centerX + halfWidth, centerY + halfHeight, centerZ + halfDepth},
        {centerX - halfWidth, centerY + halfHeight, centerZ + halfDepth},
        {centerX - halfWidth, centerY - halfHeight, centerZ - halfDepth},
        {centerX - halfWidth, centerY + halfHeight, centerZ - halfDepth},
        {centerX + halfWidth, centerY + halfHeight, centerZ - halfDepth},
        {centerX + halfWidth, centerY - halfHeight, centerZ - halfDepth}
    };

    // Apply rotation around x-axis
    for (int i = 0; i < 8; i++) {
        float y = vertices[i][1] - centerY;
        float z = vertices[i][2] - centerZ;

        // Rotate the points around the x-axis
        float rotatedY = y * cos(xAngle) - z * sin(xAngle);
        float rotatedZ = y * sin(xAngle) + z * cos(xAngle);

        // Update the rotated vertices
        vertices[i][1] = rotatedY + centerY;
        vertices[i][2] = rotatedZ + centerZ;
    }

    // Apply rotation around y-axis
    for (int i = 0; i < 8; i++) {
        float x = vertices[i][0] - centerX;
        float z = vertices[i][2] - centerZ;

        // Rotate the points around the y-axis
        float rotatedX = x * cos(yAngle) + z * sin(yAngle);
        float rotatedZ = -x * sin(yAngle) + z * cos(yAngle);

        // Update the rotated vertices
        vertices[i][0] = rotatedX + centerX;
        vertices[i][2] = rotatedZ + centerZ;
    }

    // Apply rotation around z-axis
    for (int i = 0; i < 8; i++) {
        float x = vertices[i][0] - centerX;
        float y = vertices[i][1] - centerY;

        // Rotate the points around the z-axis
        float rotatedX = x * cos(zAngle) - y * sin(zAngle);
        float rotatedY = x * sin(zAngle) + y * cos(zAngle);

        // Update the rotated vertices
        vertices[i][0] = rotatedX + centerX;
        vertices[i][1] = rotatedY + centerY;
    }

    // Draw colored faces of the parallelepiped
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, Texture);
    glBegin(GL_QUADS);

    // Front face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[0]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[1]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[2]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[3]);

    // Back face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[4]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[5]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[6]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[7]);

    // Top face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[3]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[2]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[6]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[5]);

    // Bottom face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[0]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[4]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[7]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[1]);

    // Right face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[1]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[7]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[6]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[2]);

    // Left face
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(vertices[0]);
    glTexCoord2f(1.0, 0.0);
    glVertex3fv(vertices[3]);
    glTexCoord2f(1.0, 1.0);
    glVertex3fv(vertices[5]);
    glTexCoord2f(0.0, 1.0);
    glVertex3fv(vertices[4]);

    glEnd();
    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void drawEZCube(float width, float height, float depth, float centerX, float centerY, float centerZ, float red, float green, float blue, GLuint texture) {
    drawCube(width, height, depth, centerX, centerY, centerZ, 0.0, 0.0, 0.0, red, green, blue, texture);
}

void drawCubeJetTrail(float baseX, float baseY, float baseZ, float fireSize, float intensity)
{
    if (intensity <= 0.0f) return;

    float disappearDistance = fireSize * intensity * 8.0f;

    for (float i = 1.0f; i >= 0.0f; i -= 0.05f)
    {
        float alpha = 0.7f * i;
        float r = 1.0f;
        float g = 0.5f * i;
        float b = 0.0f;

        float s = fireSize * 2 * i;
        float cx = baseX + disappearDistance * (i - 1.0f);
        float cy = baseY;
        float cz = baseZ;

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4f(r, g, b, alpha);
        drawEZCube(s, s, s, cx, cy, cz, r, g, b, 0);

        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }
}
// Single train car (middle regular car)
void drawTrainCar(float centerX, float centerY, float centerZ)
{
    float L = trainCarLen * 0.5f;
    float H = trainCarHeight * 0.5f;
    float W = trainCarWidth * 0.5f;

    glPushMatrix();
    glTranslatef(centerX, centerY, centerZ);

    setupMaterial(0.9f, 0.9f, 0.9f);

    //-----------------------------------
    // Z front and back sides with texture (window side)
    //-----------------------------------
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glColor3f(1, 1, 1);

    //  Front side z = +W 
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-L, -H, W);
    glTexCoord2f(1, 0); glVertex3f(L, -H, W);
    glTexCoord2f(1, 1); glVertex3f(L, H, W);
    glTexCoord2f(0, 1); glVertex3f(-L, H, W);
    glEnd();

    // Back side z = -W
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(L, -H, -W);
    glTexCoord2f(1, 0); glVertex3f(-L, -H, -W);
    glTexCoord2f(1, 1); glVertex3f(-L, H, -W);
    glTexCoord2f(0, 1); glVertex3f(L, H, -W);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Other 4 faces: plain metallic color
    glColor3f(0.9, 0.9, 0.9);

    // Right side x = +L
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0);
    glVertex3f(L, -H, -W);
    glVertex3f(L, -H, W);
    glVertex3f(L, H, W);
    glVertex3f(L, H, -W);
    glEnd();

    // Left side x = -L
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glVertex3f(-L, -H, W);
    glVertex3f(-L, -H, -W);
    glVertex3f(-L, H, -W);
    glVertex3f(-L, H, W);
    glEnd();

    // Top y = +H
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-L, H, -W);
    glVertex3f(L, H, -W);
    glVertex3f(L, H, W);
    glVertex3f(-L, H, W);
    glEnd();

    // Bottom y = -H
    glBegin(GL_QUADS);
    glNormal3f(0, -1, 0);
    glVertex3f(-L, -H, -W);
    glVertex3f(L, -H, -W);
    glVertex3f(L, -H, W);
    glVertex3f(-L, -H, W);
    glEnd();

    glPopMatrix();
}


// High-speed train streamlined head (ln(t) upper arc + flat bottom)
void drawSmoothHeadShape(float scaleX, float scaleY, float scaleZ)
{
    glPushMatrix();
    glScalef(scaleX, scaleY, scaleZ);
    glColor3f(0.65f, 0.65f, 0.65f);
    const int slices = 60;
    const float length = 1.0f;
    const float halfW = 1.0f;
    const float baseY = -1.0f;

    const float lnC = 2.5f;
    const float lnNorm = 1.0f / logf(1.0f + lnC);

    for (int i = 0; i < slices; ++i)
    {
        float t1 = (float)i / slices;
        float t2 = (float)(i + 1) / slices;

        float x1 = t1 * length;
        float x2 = t2 * length;

        float f1 = logf(1.0f + lnC * t1) * lnNorm;
        float f2 = logf(1.0f + lnC * t2) * lnNorm;

        float h1 = baseY + 2.0f * f1;
        float h2 = baseY + 2.0f * f2;

        float w1 = halfW * (0.25f + 0.75f * t1);
        float w2 = halfW * (0.25f + 0.75f * t2);

        // ---- Right side ----
        glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glVertex3f(x1, baseY, w1);
        glVertex3f(x2, baseY, w2);
        glVertex3f(x2, h2, w2);
        glVertex3f(x1, h1, w1);
        glEnd();

        // ---- Left side ----
        glBegin(GL_QUADS);
        glNormal3f(0, 0, -1);
        glVertex3f(x1, h1, -w1);
        glVertex3f(x2, h2, -w2);
        glVertex3f(x2, baseY, -w2);
        glVertex3f(x1, baseY, -w1);
        glEnd();

        // ---- Top ----
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(x1, h1, -w1);
        glVertex3f(x2, h2, -w2);
        glVertex3f(x2, h2, w2);
        glVertex3f(x1, h1, w1);
        glEnd();
    }

    // ---- Front semicircular cap ----
    glBegin(GL_TRIANGLES);
    glNormal3f(1, 0, 0);
    glVertex3f(1.0f, baseY, 0);
    glVertex3f(1.0f, baseY + 0.4f, 0.35f);
    glVertex3f(1.0f, baseY + 0.4f, -0.35f);
    glEnd();

    glPopMatrix();
}

void drawTrainHead(float centerX, float centerY, float centerZ)
{
    float halfLen = trainCarLen / 2.0f;
    float halfH = trainCarHeight / 2.0f;
    float halfW = trainCarWidth / 2.0f;

    drawTrainCar(centerX, centerY, centerZ);

    glPushMatrix();

    glTranslatef(centerX + 2.8 * halfLen, centerY, centerZ);

    glRotatef(180, 0, 1, 0);

    float scaleX = trainCarLen * 0.9f;
    float scaleY = trainCarHeight * 0.5f;
    float scaleZ = halfW;

    setupMaterial(0.15f, 0.15f, 0.17f);
    drawSmoothHeadShape(scaleX, scaleY, scaleZ);

    glPopMatrix();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}



// ============================
// Dual thrusters at the train tail
//   Using cube jet trail (similar to the plane)
// ============================
void drawTrainThrusters(float tailCenterX, float tailCenterY, float tailCenterZ)
{
    float halfLen = trainCarLen / 2.0f;
    float halfW = trainCarWidth / 2.0f;

    // Thruster positions: at the rear of the last car, one on each side
    float baseX = tailCenterX - halfLen - 2.0f;   // slightly extended backwards
    float baseY = tailCenterY;
    float offsetZ = halfW * 0.6f;

    // Use plane speed to map a visual intensity so when the plane accelerates the train also looks more dramatic
    float intensity = 0.5f + fabs(trainspeed) / 40.0f;

    // Two nozzles (left and right)
    drawCubeJetTrail(baseX, baseY, tailCenterZ + offsetZ, halfW * 0.8f, intensity);
    drawCubeJetTrail(baseX, baseY, tailCenterZ - offsetZ, halfW * 0.8f, intensity);
}

// ============================
// Whole train (in world coordinates)
//   Head faces +X, cars follow along -X
// ============================
void drawTrainWorld()
{
    // Train head center at (trainX, trainY, trainZ)
    float headX = trainX;
    float headY = trainY;
    float headZ = trainZ;

    // Train head
    drawTrainHead(headX, headY, headZ);

    // Remaining cars: arranged along -X
    float gap = 0.0f;  // Small gap between cars
    float lastCarCenterX = headX;

    for (int i = 1; i < trainCarCount; ++i)
    {
        float cx = headX - i * (trainCarLen + gap);
        float cy = headY;
        float cz = headZ;

        drawTrainCar(cx, cy, cz);
        lastCarCenterX = cx;
    }

    // Add thrusters behind the last car
    drawTrainThrusters(lastCarCenterX, headY, headZ);
}



// Only texture the front (Z+) and back (Z-) faces, no texture on the other four faces
void drawCubeFrontBackTex(int texIndex, float sx, float sy, float sz)
{
    glPushMatrix();
    glScalef(sx, sy, sz);

    // ---------- Front & Back: textured ----------
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[texIndex]);
    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);
    // Front (Z+)
    glNormal3f(0, 0, 1);
    glTexCoord2f(0, 0); glVertex3f(-0.5f, -0.5f, 0.5f);
    glTexCoord2f(1, 0); glVertex3f(0.5f, -0.5f, 0.5f);
    glTexCoord2f(1, 1); glVertex3f(0.5f, 0.5f, 0.5f);
    glTexCoord2f(0, 1); glVertex3f(-0.5f, 0.5f, 0.5f);

    // Back (Z-)
    glNormal3f(0, 0, -1);
    glTexCoord2f(0, 0); glVertex3f(0.5f, -0.5f, -0.5f);
    glTexCoord2f(1, 0); glVertex3f(-0.5f, -0.5f, -0.5f);
    glTexCoord2f(1, 1); glVertex3f(-0.5f, 0.5f, -0.5f);
    glTexCoord2f(0, 1); glVertex3f(0.5f, 0.5f, -0.5f);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    // ---------- Left / Right / Top / Bottom: plain color, no texture ----------
    glColor3f(0.85f, 0.85f, 0.9f);

    glBegin(GL_QUADS);
    // Left  (X-)
    glNormal3f(-1, 0, 0);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f(-0.5f, 0.5f, 0.5f);
    glVertex3f(-0.5f, 0.5f, -0.5f);

    // Right (X+)
    glNormal3f(1, 0, 0);
    glVertex3f(0.5f, -0.5f, 0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, 0.5f, -0.5f);
    glVertex3f(0.5f, 0.5f, 0.5f);

    // Top   (Y+)
    glNormal3f(0, 1, 0);
    glVertex3f(-0.5f, 0.5f, 0.5f);
    glVertex3f(0.5f, 0.5f, 0.5f);
    glVertex3f(0.5f, 0.5f, -0.5f);
    glVertex3f(-0.5f, 0.5f, -0.5f);

    // Bottom(Y-)
    glNormal3f(0, -1, 0);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, 0.5f);
    glVertex3f(-0.5f, -0.5f, 0.5f);
    glEnd();

    glPopMatrix();
}

// =================== Robot parts drawing ===================
void drawJoint(float r = 0.17f)
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[21]);
    glColor3f(1.0f, 1.0f, 1.0f);

    GLUquadric* q = gluNewQuadric();
    gluQuadricDrawStyle(q, GLU_FILL);
    gluQuadricNormals(q, GLU_SMOOTH);
    gluQuadricTexture(q, GL_TRUE);
    gluQuadricOrientation(q, GLU_OUTSIDE);

    gluSphere(q, r, 40, 40);
    gluDeleteQuadric(q);

    glDisable(GL_TEXTURE_2D);
}

void drawNeck()
{
    drawCubeFrontBackTex(19, 0.35f, 0.5f, 0.35f);
}

void drawUpperArm()
{
    drawCubeFrontBackTex(19, 0.45f, 1.2f, 0.45f);
}

void drawLowerArm()
{
    drawCubeFrontBackTex(19, 0.42f, 1.0f, 0.42f);
}

void drawUpperLeg()
{
    drawCubeFrontBackTex(19, 0.6f, 1.5f, 0.6f);
}

void drawLowerLeg()
{
    drawCubeFrontBackTex(19, 0.5f, 1.5f, 0.5f);
}


void drawTorso()
{
    drawCubeFrontBackTex(20, 1.4f, 2.2f, 0.8f);
}


// =================== Robot hierarchical model ===================
void drawRobot()
{
    glPushMatrix();
    glColor3f(0.85f, 0.85f, 0.9f);

    // ===== TORSO (root) =====
    drawTorso();

    // ===== HEAD + NECK =====
    glPushMatrix();
    glTranslatef(0.0f, 1.1f, 0.0f);  // Move up to top of torso

    // Neck (center here, half height 0.25)
    glPushMatrix();
    drawNeck();
    glPopMatrix();

    // Head: above the neck
    glTranslatef(0.0f, 0.5f, 0.0f);  // Head center height ≈ 1.6
    glRotatef(headRotate, 0, 1, 0);

    // Use a cube of side length 1 (equal to diameter of original sphere of radius 0.5)
    drawCubeFrontBackTex(18, 1.0f, 1.0f, 1.0f);

    glPopMatrix();


    // ================== Left arm ==================
    glPushMatrix();
    // Shoulder position: slightly below torso top and slightly outward
    glTranslatef(-0.9f, 0.6f, 0.0f);
    drawJoint(0.17f);                 // Shoulder joint sphere

    // Upper arm
    glRotatef(leftUpperArmRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.6f, 0.0f);  // Upper arm center: half length 0.6
    drawUpperArm();

    // Elbow
    glTranslatef(0.0f, -0.6f, 0.0f);  // Move down to end of upper arm
    drawJoint(0.15f);

    // Lower arm
    glRotatef(leftLowerArmRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.5f, 0.0f);  // Lower arm half length 0.5
    drawLowerArm();
    glPopMatrix();


    // ================== Right arm ==================
    glPushMatrix();
    glTranslatef(0.9f, 0.6f, 0.0f);
    drawJoint(0.17f);

    glRotatef(rightUpperArmRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.6f, 0.0f);
    drawUpperArm();

    glTranslatef(0.0f, -0.6f, 0.0f);
    drawJoint(0.15f);

    glRotatef(rightLowerArmRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.5f, 0.0f);
    drawLowerArm();
    glPopMatrix();


    // ================== Left leg ==================
    glPushMatrix();
    // Hip joint: near bottom of torso, slightly outward
    glTranslatef(-0.5f, -1.0f, 0.0f);
    drawJoint(0.20f);

    glRotatef(leftUpperLegRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.75f, 0.0f);  // Upper leg center
    drawUpperLeg();

    glTranslatef(0.0f, -0.75f, 0.0f);  // Knee position
    drawJoint(0.18f);

    glRotatef(leftLowerLegRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.75f, 0.0f);  // Lower leg center
    drawLowerLeg();
    glPopMatrix();


    // ================== Right leg ==================
    glPushMatrix();
    glTranslatef(0.5f, -1.0f, 0.0f);
    drawJoint(0.20f);

    glRotatef(rightUpperLegRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.75f, 0.0f);
    drawUpperLeg();

    glTranslatef(0.0f, -0.75f, 0.0f);
    drawJoint(0.18f);

    glRotatef(rightLowerLegRotate, 1, 0, 0);
    glTranslatef(0.0f, -0.75f, 0.0f);
    drawLowerLeg();
    glPopMatrix();


    glPopMatrix();
}

// =================== Wrapper: draw robot following the camera ===================
// Make the robot always stand in front of the camera and stay at screen center
void drawRobotFollowCamera()
{
    if (!g_robotVisible) return;

    glPushMatrix();

    // Compute camera view direction vector (from eye to center)
    float dirX = centerX - eyeX;
    float dirY = centerY - eyeY;
    float dirZ = centerZ - eyeZ;

    float len = sqrtf(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (len < 1e-3f) len = 1.0f;
    dirX /= len;
    dirY /= len;
    dirZ /= len;

    // Robot position = camera position + view direction * distance
    float dist = g_robotDistFromCam;
    float rx = eyeX + dirX * dist;
    float ry = eyeY + dirY * dist;
    float rz = eyeZ + dirZ * dist;

    glTranslatef(rx, ry, rz);

    // Make robot face along the camera view direction
    float yawDeg = atan2f(dirX, dirZ) * 180.0f / PI;
    glRotatef(yawDeg, 0.0f, 1.0f, 0.0f);

    // Scale robot to suitable size
    glScalef(5.0f, 5.0f, 5.0f);
    glTranslatef(0, 2, 0);  // Move it into view center

    // Draw actual robot model
    drawRobot();

    glPopMatrix();
}


//---------------------------------------- Sun / Moon  ------------------------------------------

int sun_moonz = -100;
void sunlight(GLfloat x, GLfloat y, GLfloat z) {
    GLfloat light_position[] = { x, y, z, 0 };
    GLfloat light_diffuse[] = { 251 / 255.0f,188 / 255.0f,81 / 255.0f, 0.5 };
    GLfloat ambient[] = { 0.4,0.4,0.4,1 };
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
}

void moonlight(GLfloat x, GLfloat y, GLfloat z) {
    GLfloat pos[] = { x,y,z,0 };
    GLfloat diff[] = { 102 / 255.0f,153 / 255.0f,255 / 255.0f, 0.5 };
    GLfloat amb[] = { 0.2,0.2,0.2,1 };
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diff);
    glLightfv(GL_LIGHT1, GL_AMBIENT, amb);
    glLightfv(GL_LIGHT1, GL_POSITION, pos);
}

void sun_moon() {
    int baseR = 1500;

    double theta = 2 * PI * daytime / 2400.0;
    double sx = baseR * cos(theta);
    double sy = baseR * sin(theta);

    glMaterialfv(GL_FRONT, GL_EMISSION, emmission);
    glPushMatrix();
    glTranslatef(sx, sy, sun_moonz);
    glColorRGB(255, 255, 204);
    glutSolidSphere(50, 20, 20);
    glPopMatrix();

    double theta2 = theta + PI;
    double mx = baseR * cos(theta2);
    double my = baseR * sin(theta2);

    glPushMatrix();
    glTranslatef(mx, my, sun_moonz);
    glColorRGB(255, 153, 51);
    glutSolidSphere(30, 20, 20);
    glPopMatrix();
    glMaterialfv(GL_FRONT, GL_EMISSION, no_emmission);

    if (sy > 0) {
        glEnable(GL_LIGHT0);
        sunlight(sx, sy, sun_moonz);
    }
    else {
        glDisable(GL_LIGHT0);
    }

    if (my > 0) {
        glEnable(GL_LIGHT1);
        moonlight(mx, my, sun_moonz);
    }
    else {
        glDisable(GL_LIGHT1);
    }

    glDisable(GL_FOG);
}

//---------------------------------------------- Update ----------------------------------------------

bool skyup = true;

void update(int value) {
    fw_ang += fw_speed;          
    fw_lightPhase += fw_lightSpeed; 

    daytime++;
    if (daytime == 2400) { daytime = 0; day++; }

    wave += 2;
    if (wave >= 600) wave = 0;

    orbitAngle += 0.8f;                 
    if (orbitAngle > 360.0f) orbitAngle -= 360.0f;
    // sky rotation
    backgroundrotate += 0.2;
    if (backgroundrotate >= 360) backgroundrotate = 0;

    // sky transparency fade
    if (skytransparency >= 0.9999) skyup = false;
    else if (skytransparency <= -0.9999) skyup = true;

    if (skyup) skytransparency += 1.0f / 600.0f;
    else skytransparency -= 1.0f / 600.0f;

    if (g_droneMoving) {
        dronePhase += droneSpeed;
        if (dronePhase > 2.0f * PI) dronePhase -= 2.0f * PI;
    }

    if (g_trainMoving) {
        for (int i = 0; i < 2; ++i) {
            TrainPath& p = g_trainPaths[i];

            p.x += p.speed * p.dir;

            if (p.dir > 0 && p.x > TRAIN_MAX_X) {
                p.x = TRAIN_MAX_X;
                p.dir = -1;
            }
            else if (p.dir < 0 && p.x < TRAIN_MIN_X) {
                p.x = TRAIN_MIN_X;
                p.dir = +1;
            }
        }
    }

    float minX = -3500;
    float maxX = 3000;
    if (g_carMoving) {
        // car
        for (int i = 0; i < NUM_GROUND_CARS; ++i) {
            CarMover& c = g_groundCars[i];

            c.x += c.speed * c.dir;

            if (c.dir > 0 && c.x > maxX) {
                c.x = maxX;
                c.dir = -1;
            }
            else if (c.dir < 0 && c.x < minX) {
                c.x = minX;
                c.dir = +1;
            }
        }
        for (int i = 0; i < NUM_HOVER_CARS; ++i) {
            HoverCar& h = g_hoverCars[i];
            h.angle += h.speed;

            if (h.angle > 2.0f * PI) h.angle -= 2.0f * PI;
            if (h.angle < 0.0f)      h.angle += 2.0f * PI;
        }
    }

       // Robot walking animation
    if (g_robotWalking) {
        g_robotWalkPhase += 0.15f;         
        if (g_robotWalkPhase > 2.0f * PI) {
            g_robotWalkPhase -= 2.0f * PI;
        }

        // Legs swing forward and backward (opposite phase)
        float legSwing = 25.0f * sinf(g_robotWalkPhase);
        float kneeSwing = 15.0f * sinf(2.0f * g_robotWalkPhase);
        float armSwing = 20.0f * sinf(g_robotWalkPhase + PI); // Arms opposite to legs

        leftUpperLegRotate = legSwing;
        rightUpperLegRotate = -legSwing;

        leftLowerLegRotate = max(0.0f, kneeSwing);   // Knees only bend forward
        rightLowerLegRotate = max(0.0f, -kneeSwing);

        leftUpperArmRotate = -armSwing;
        rightUpperArmRotate = armSwing;

        leftLowerArmRotate = 0.0f;
        rightLowerArmRotate = 0.0f;

        headRotate = 5.0f * sinf(g_robotWalkPhase * 0.5f);  // Head slightly turns left/right
    }
    else {
        // Default idle pose
        headRotate = 0.0f;
        leftUpperArmRotate = 20.0f;
        leftLowerArmRotate = -10.0f;
        rightUpperArmRotate = -20.0f;
        rightLowerArmRotate = 10.0f;
        leftUpperLegRotate = 5.0f;
        leftLowerLegRotate = -5.0f;
        rightUpperLegRotate = -5.0f;
        rightLowerLegRotate = 5.0f;
    }

    glutPostRedisplay();
    glutTimerFunc(updatespeed, update, 0);
}



//---------------------------------------------- Input ----------------------------------------------
// Update center position from eye_rotation / eye_updown and eye position
void updateCameraCenterFromRotation()
{
    float yawRad = eye_rotation * PI / 180.0f;   
    float pitchRad = eye_updown * PI / 180.0f;   

    float dirX = cosf(pitchRad) * sinf(yawRad);
    float dirY = sinf(pitchRad);
    float dirZ = cosf(pitchRad) * cosf(yawRad);

    centerX = eyeX + dirX;
    centerY = eyeY + dirY;
    centerZ = eyeZ + dirZ;
}
//---------------------------------------------- Input ----------------------------------------------

void KeyboardDisplay(unsigned char key, int x, int y)
{
    if (key == '1') {
        g_trainMoving = !g_trainMoving;        // Train start/stop
        std::cout << "Train moving: " << g_trainMoving << std::endl;
    }
    if (key == '2') {
        g_trainVisible = !g_trainVisible;      // Train show/hide
        std::cout << "Train visible: " << g_trainVisible << std::endl;
    }
    if (key == '3') {
        g_carMoving = !g_carMoving;            // Car start/stop
        std::cout << "Car moving: " << g_carMoving << std::endl;
    }
    if (key == '4') {
        g_carVisible = !g_carVisible;          // Car show/hide
        std::cout << "Car visible: " << g_carVisible << std::endl;
    }
    if (key == '5') {
        g_droneMoving = !g_droneMoving;        // Drones start/stop
        std::cout << "Drone moving: " << g_droneMoving << std::endl;
    }
    if (key == '6') {
        g_droneVisible = !g_droneVisible;      // Drones show/hide
        std::cout << "Drone visible: " << g_droneVisible << std::endl;
    }

    // ============================ Look up / down ============================
    if (key == 's' || key == 'S') {
        eye_updown -= 2;
        if (!highView) updateCameraCenterFromRotation();  
    }
    if (key == 'w' || key == 'W') {
        eye_updown += 2;
        if (!highView) updateCameraCenterFromRotation();   
    }

    // ============================ Yaw left / right ============================
    if (key == 'a' || key == 'A') {
        eye_rotation -= 2;
        if (!highView) updateCameraCenterFromRotation();   
    }
    if (key == 'd' || key == 'D') {
        eye_rotation += 2;
        if (!highView) updateCameraCenterFromRotation();  
    }

    // ============================ Move up / down ============================
    if (key == 'q' || key == 'Q') {
        if (eyeY < 1500) eyeY += 5;
        if (!highView) updateCameraCenterFromRotation();  
    }
    if (key == 'e' || key == 'E') {
        if (eyeY > -200) eyeY -= 5;
        if (!highView) updateCameraCenterFromRotation();   
    }

    // ============================ High-altitude view toggle ============================
    if (key == 'h' || key == 'H') {
        highView = !highView;

        if (highView) {
            // Enter high-altitude view
            eyeX = 2350.0f;
            eyeY = 400.0f;
            eyeZ = -2100.0f;

            eye_rotation = 135.0f;       
            updateCameraCenterFromRotation();   
        }
        else {
            // Return to normal view
            eyeX = 0.0f;
            eyeY = 0.0f;
            eyeZ = -2600.0f;

            eye_rotation = 90.0f;           
            updateCameraCenterFromRotation();   
        }
    }

    clampCamera();
    centerX = cos(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeX;
    centerY = sin(eye_updown / 180 * PI) + eyeY;
    centerZ = sin(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeZ;

    glutPostRedisplay();
}


void specialKeyboardKeys(int key, int xx, int yy) {
    switch (key)
    {
    case GLUT_KEY_LEFT:
        eyeX += move_speed * sin(eye_rotation / 180 * PI);
        eyeZ -= move_speed * cos(eye_rotation / 180 * PI);
        centerX += move_speed * sin(eye_rotation / 180 * PI);
        centerZ -= move_speed * cos(eye_rotation / 180 * PI);
        break;

    case GLUT_KEY_RIGHT:
        eyeX -= move_speed * sin(eye_rotation / 180 * PI);
        eyeZ += move_speed * cos(eye_rotation / 180 * PI);
        centerX -= move_speed * sin(eye_rotation / 180 * PI);
        centerZ += move_speed * cos(eye_rotation / 180 * PI);
        break;

    case GLUT_KEY_DOWN:
        eyeX -= move_speed * cos(eye_rotation / 180 * PI);
        eyeZ -= move_speed * sin(eye_rotation / 180 * PI);
        centerX -= move_speed * cos(eye_rotation / 180 * PI);
        centerZ -= move_speed * sin(eye_rotation / 180 * PI);
        break;

    case GLUT_KEY_UP:
        eyeX += move_speed * cos(eye_rotation / 180 * PI);
        eyeZ += move_speed * sin(eye_rotation / 180 * PI);
        centerX += move_speed * cos(eye_rotation / 180 * PI);
        centerZ += move_speed * sin(eye_rotation / 180 * PI);
        break;
    }
    clampCamera();
    centerX = cos(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeX;
    centerY = sin(eye_updown / 180 * PI) + eyeY;
    centerZ = sin(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeZ;

    glutPostRedisplay();
}

void MouseControl(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        // Left button: toggle robot walking on/off
        if (button == GLUT_LEFT_BUTTON)
        {
            g_robotWalking = !g_robotWalking;
            std::cout << "Robot walking: " << g_robotWalking << std::endl;
        }

        // Right button: toggle robot visibility
        if (button == GLUT_RIGHT_BUTTON)
        {
            g_robotVisible = !g_robotVisible;
            std::cout << "Robot visible: " << g_robotVisible << std::endl;
        }
    }
}

//---------------------------------------------- Rendering ----------------------------------------------

void reshapeWindow(GLint w, GLint h)
{
    intWinWidth = w;
    intWinHeight = h;
    glViewport(0, 0, w, h);
}

void displayObject()
{
    // Projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fltFOV, (float)intWinWidth / intWinHeight, 0.1, 20000);

    // Camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, fltXUp, fltYUp, fltZUp);

    // Clear buffers
    glClearColor(153 / 255.0, 204 / 255.0, 255 / 255.0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glPushMatrix();
    texturedisplay();
    glPopMatrix();

    glPushMatrix();
    greenland();
    glPopMatrix();

    glPushMatrix();
    drawForest();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(1800.0f, 0.0f, 500.0f); 
    glRotatef(90.0f, 0, 1, 0);          
    glScalef(1.2, 1.5, 1.6);            
    drawBezierBuilding();
    glPopMatrix();
    drawBezierSideClusters();

    glPushMatrix();
    glTranslatef(-2400, 0, 1300);
    glScalef(3, 45, 3);

    glDisable(GL_CULL_FACE);

    glColor3f(1.0, 1.0, 1.0);
    DrawCutCylinder_Solid(20, 30, 10, 80, texture[5], texture[6]);
    glEnable(GL_CULL_FACE);
    glPopMatrix();


    glPushMatrix();
    glTranslatef(-2500, 0, 500);
    DrawFerrisWheel();
    glPopMatrix();

    if (g_droneVisible) {
        glPushMatrix();
        drawFlyingDrones();
        glPopMatrix();
    }

    glDisable(GL_CULL_FACE);

    if (g_trainVisible) {
        for (int i = 0; i < 2; ++i) {
            TrainPath& p = g_trainPaths[i];

            // ---- Ground trains ----
            glPushMatrix();
            glTranslatef(p.x, p.yGround, p.z);

            // dir < 0: moving from right to left → flip model
            if (p.dir < 0) {
                glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
            }

            // Double size (as requested before)
            glScalef(4.0f, 4.0f, 4.0f);
            drawTrainWorld();
            glPopMatrix();

            // ---- Corresponding airborne trains ----
            glPushMatrix();
            glTranslatef(p.x, p.yAir, p.z);

            if (p.dir < 0) {
                glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
            }

            glScalef(4.0f, 4.0f, 4.0f);
            drawTrainWorld();
            glPopMatrix();
        }
    }
    glEnable(GL_CULL_FACE);


    flameTime += 0.02f;  // Keep flame animation time
    if (g_carVisible) {
        glPushMatrix();
        glScalef(0.5f, 0.5f, 0.5f);

        drawGroundCars();   // 12 cars on the ground
        drawHoverCars();    // 6 hovering cars in the air

        glPopMatrix();
    }

    glPushMatrix();
    glTranslatef(-2300, 0, -2000);
    glScalef(100, 100, 100);
    drawCB();
    glPopMatrix();

    glPushMatrix();
    drawRobotFollowCamera();
    glPopMatrix();

    glPushMatrix();
    sun_moon();
    glPopMatrix();


    glPushMatrix();
    glScalef(5, 5, 5);
    drawFloatingCity();
    glPopMatrix();

    // Number keys
    drawScreenText(10.0f, 160.0f, "1: Train start/stop   2: Train show/hide");
    drawScreenText(10.0f, 140.0f, "3: Cars start/stop    4: Cars show/hide");
    drawScreenText(10.0f, 120.0f, "5: Drones start/stop  6: Drones show/hide");

    // Camera rotation & pitch
    drawScreenText(10.0f, 100.0f, "W/S: Look up/down     A/D: Rotate view");

    // Camera move up/down & high view
    drawScreenText(10.0f, 80.0f, "Q/E: Move camera up/down   H: Toggle high view");

    // Movement with arrow keys
    drawScreenText(10.0f, 60.0f, "Arrows: Move (forward/back/strafe)");

    // Mouse control
    drawScreenText(10.0f, 40.0f, "Left mouse button: Robot walk on/off ");
    drawScreenText(10.0f, 20.0f, "Right mouse button: Robot show/hide");
    glutSwapBuffers();
}


//---------------------------------------------- Init ----------------------------------------------

void init() {
    glEnable(GL_LIGHTING);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
    glEnable(GL_MULTISAMPLE);
    textureinit();
    initFloatingPlatforms();
    initForest();
    initCarsAndTrains();
    centerX = cos(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeX;
    centerY = sin(eye_updown / 180 * PI) + eyeY;
    centerZ = sin(eye_rotation / 180 * PI) * cos(eye_updown / 180 * PI) + eyeZ;
}

//---------------------------------------------- Main ----------------------------------------------

int main(int argc, char** argv)
{
    srand(time(NULL));
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(50, 50);
    glutInitWindowSize(1600, 800);
    glutCreateWindow("futuristic cityscape");
    init();

    glutDisplayFunc(displayObject);
    glutReshapeFunc(reshapeWindow);
    glutKeyboardFunc(KeyboardDisplay);
    glutSpecialFunc(specialKeyboardKeys);
    glutMouseFunc(MouseControl);
    glutTimerFunc(0, update, 0);

    glutMainLoop();
}
