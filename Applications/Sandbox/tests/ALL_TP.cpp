#include <Unitest/Unitest.h>
#include <Unitest/TestMacro.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <iostream>
#include <random>
#include <cstdlib>

#include "NKLogger/NkLog.h"
#include "NKMath/NKMath.h"
#include "Mat4d.h" 
#include "Quat.h" 
#include "NKImage.h" 

using namespace NkMath;

// paramètres image
const int W = 500, H = 500;
NkImage framebuffer(W, H);

// RNG
std::mt19937 gen(1337);
std::uniform_real_distribution<double> rnd(-8.0, 8.0);

// géométrie cube
std::vector<Vec4d> cubeVerts = {
    {-0.6,-0.6,-0.6,1}, {0.6,-0.6,-0.6,1},
    {0.6, 0.6,-0.6,1}, {-0.6, 0.6,-0.6,1},
    {-0.6,-0.6, 0.6,1}, {0.6,-0.6, 0.6,1},
    {0.6, 0.6, 0.6,1}, {-0.6, 0.6, 0.6,1}
};

// indices des arêtes
std::vector<Vec2d> cubeEdges = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
};

// caméra
Vec3d cam{0,1.2,3.2}, look{0,0,0}, upVec{0,1,0};
Mat4d viewM = LookAt(cam, look, upVec);
Mat4d projM = Perspective(55.0, double(W)/H, 0.1, 100.0);

// ---------- TP1 ----------
TEST_CASE(FloatBasics, InspectValues) {
    inspectFloat(0.2f);
    inspectFloat(2.0f);
    inspectFloat(1.0f / 0.0f);
    inspectFloat(std::sqrt(-2.0f));
    inspectFloat(-0.0f);
    inspectFloat(0.0f);
    inspectFloat(std::numeric_limits<float>::denorm_min());
}

// ---------- TP2 ----------
TEST_CASE(FloatPrecisionStudy, AccuracyTests) {
    float a, b;
    std::vector<float> vec;

    std::vector<float> data(800000, 0.125f);

    a = std::accumulate(data.begin(), data.end(), 0.0f);
    b = kahanSum(data);

    logger.Info("Accumulate: {0} | Kahan: {1}", a, b);

    vec = {1e7f, 1e7f, 3.0f, 4.0f};
    logger.Info("Var naive: {0} | Var Welford: {1}",
                varianceNaive(vec), varianceWelford(vec));

    logger.Info("Eps loop: {0} | Eps std: {1}",
                epsilonMachine(), std::numeric_limits<float>::epsilon());
}

// ---------- TP3 ----------
TEST_CASE(FloatValidationSuite, FloatChecks) {

    ASSERT_TRUE(!isFiniteValid(std::numeric_limits<float>::quiet_NaN()));
    ASSERT_TRUE(!isFiniteValid(std::numeric_limits<float>::infinity()));
    ASSERT_TRUE(isFiniteValid(2.0f));

    ASSERT_TRUE(nearlyZero(1e-7f, 1e-6f));
    ASSERT_TRUE(!nearlyZero(1e-4f, 1e-6f));

    ASSERT_TRUE(approxEq(1.0f, 1.00001f, 1e-4f));
    ASSERT_TRUE(!approxEq(1.0f, 1.2f, 1e-2f));

    std::vector<float> v(2000, 0.05f);
    float k = kahanSum(v);
    ASSERT_TRUE(approxEq(k, 100.0f, 1e-2f));
}

// ---------- TP4 ----------
TEST_CASE(Vec2dCore, VectorOps2D) {

    ASSERT_TRUE(Dot({2,0}, {0,2}) == 0.0);
    ASSERT_TRUE(Dot({2,3}, {2,3}) == 13.0);

    ASSERT_TRUE(Cross2D({1,0}, {0,1}) > 0);

    Vec2d a{4,3};
    Vec2d n = a.Normalized();
    ASSERT_TRUE(std::fabs(n.Norm() - 1.0) < kEps);

    a = {9, 8};
    ASSERT_TRUE(a[0] == 9.0);
    ASSERT_TRUE(a[1] == 8.0);

    static_assert(sizeof(Vec2d) == 16, "Vec2d size mismatch");
}

// ---------- TP5 ----------
TEST_CASE(Vec3dOrthonormal, GramSchmidtProcess) {

    Vec3d x{1,0,0}, y{0,1,0};
    ASSERT_TRUE(ApproxVec(Cross(x,y), {0,0,1}));

    for(int i=0;i<5;i++){
        Vec3d a{rnd(gen), rnd(gen), rnd(gen)};
        Vec3d b{rnd(gen), rnd(gen), rnd(gen)};

        Vec3d u = a.Normalized();
        Vec3d v = (b - Project(b,u)).Normalized();

        ASSERT_TRUE(approxEq(Dot(u,v), 0.0));
    }
}

// ---------- TP6 ----------
TEST_CASE(SimpleProjection, CubeDraw) {

    std::vector<Vec2d> projPts;

    for(auto p : cubeVerts){
        p.z += 2.5;
        projPts.push_back(ProjectPoint(p));
    }

    for(auto e : cubeEdges)
        framebuffer.DrawLine(
            (int)projPts[e.x].x, (int)projPts[e.x].y,
            (int)projPts[e.y].x, (int)projPts[e.y].y
        );

    framebuffer.SavePPM("cube_simple.ppm");
}

// ---------- TP7 ----------
TEST_CASE(MatrixOps, InversionTest) {

    Mat4d m, inv;

    for(int t=0;t<5;t++){
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                m(i,j) = rnd(gen);

        if(Inverse(m, inv))
            ASSERT_TRUE(ApproxMat(m*inv, Mat4d::Identity(), 1e-8f));
    }
}

// ---------- TP8 ----------
TEST_CASE(CubeAnimation, RotatingFrames) {

    for(int f=0; f<6; f++){
        framebuffer = NkImage(W,H);

        Mat4d R = Mat4d::RotateAxis(upVec, f * 0.4);
        std::vector<Vec3d> screen;

        for(auto v : cubeVerts){
            Vec4d p = projM * (viewM * (R * v));
            screen.push_back(ProjectToScreen(p,W,H));
        }

        for(auto e : cubeEdges)
            framebuffer.DrawLine(
                (int)screen[e.x].x, (int)screen[e.x].y,
                (int)screen[e.y].x, (int)screen[e.y].y, 255
            );

        framebuffer.SavePPM("anim_"+std::to_string(f)+".ppm");
    }
}

// ---------- TP9 ----------
TEST_CASE(TRSCheck, DecompositionTest) {

    for(int i=0;i<10;i++){
        Vec3d T{rnd(gen), rnd(gen), rnd(gen)};
        Vec3d R{rnd(gen), rnd(gen), rnd(gen)};
        Vec3d S{rnd(gen)+5, rnd(gen)+5, rnd(gen)+5};

        Mat4d M = TRS(T,R,S);

        Vec3d t2,r2,s2;
        DecomposeTRS(M,t2,r2,s2);

        ASSERT_TRUE(ApproxVec(T,t2));
        ASSERT_TRUE(ApproxVec(S,s2));
    }
}

// ---------- TP10 ----------
TEST_CASE(QuatTests, QuaternionOps) {

    Quat q = FromAxisAngle({0,1,0}, NKENTSEU_PI_DOUBLE/2.0);
    Vec3d res = Rotate(q, {1,0,0});

    ASSERT_TRUE(std::fabs(res.z + 1.0) < kEps);

    for(int i=0;i<10;i++){
        Quat a{rnd(gen), rnd(gen), rnd(gen), rnd(gen)};
        a = a.Normalized();
        Quat b = a.Inverse();

        ASSERT_TRUE(ApproxQuat(a*b, Quat::Identity(), 1e-4f));
    }
}

// ---------- TP11 ----------
TEST_CASE(SlerpAnim, QuaternionInterpolation) {

    Quat q1 = FromAxisAngle({0,1,0}, 0);
    Quat q2 = FromAxisAngle({0,1,0}, NKENTSEU_PI_DOUBLE);

    for(int f=0; f<30; f++){
        double t = f / 29.0;

        Quat q = Slerp(q1,q2,t);
        Mat4d R = FromRT(ToMat3(q), {0,0,0});

        framebuffer = NkImage(W,H);
        std::vector<Vec3d> screen;

        for(auto v : cubeVerts){
            Vec4d p = projM * (viewM * (R * v));
            screen.push_back(ProjectToScreen(p,W,H));
        }

        for(auto e : cubeEdges)
            framebuffer.DrawLine(
                (int)screen[e.x].x,(int)screen[e.x].y,
                (int)screen[e.y].x,(int)screen[e.y].y,255
            );

        framebuffer.SavePPM("slerp_"+std::to_string(f)+".ppm");
    }
}