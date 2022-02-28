// Karl Yerkes
// 2022-01-20

#include "al/app/al_App.hpp" //header
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"

using namespace al; //library

#include <fstream> //header 
#include <vector>
using namespace std;

Vec3f randomVec3f(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
}
string slurp(string fileName);  // forward declaration

struct AlloApp : App {
  Parameter pointSize{"/pointSize", "", 1.0, "", 0.0, 2.0}; //label GUI
  Parameter timeStep{"/timeStep", "", 0.1, "", 0.01, 0.6};
  Parameter gravConst{"/gravConst", "", 0.1, "", 0.01, 0.99}; //label GUI
  Parameter dragFactor{"/dragFactor", "", 0.1, "", 0.01, 0.99};
  Parameter maxAcceler{"/maxAcceler", "", 6.0, "", 0.01, 5.00};


  ShaderProgram pointShader;

  //  simulation state
  Mesh mesh;  // position *is inside the mesh* mesh.vertices() are the positions
  vector<Vec3f> velocity;
  vector<Vec3f> acceleration;
  vector<float> mass;

  void onInit() override {
    // set up GUI
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto &gui = GUIdomain->newGUI();
    gui.add(pointSize);  // add parameter to GUI
    gui.add(timeStep);   // add parameter to GUI
    gui.add(gravConst);   // add parameter to GUI
    gui.add(dragFactor);   // add parameter to GUI
    gui.add(maxAcceler);   // add parameter to GUI

  }

  void onCreate() override {

    // compile shaders
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    // set initial conditions of the simulation
    //

    // c++11 "lambda" function
    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };

    mesh.primitive(Mesh::POINTS);
    // does 1000 work on your system? how many can you make before you get a low
    // frame rate? do you need to use <1000?
    for (int _ = 0; _ < 1000; _++) {
    //  mesh.vertex(randomVec3f(5));
      mesh.vertex(rnd::uniformS(3), rnd::uniformS(3), 0.0);
      mesh.color(randomColor());

      // float m = rnd::uniform(3.0, 0.5);
      float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);


      // using a simplified volume/size relationship
      mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      //velocity.push_back(randomVec3f(0.1));
      velocity.push_back(Vec3f(0, 0, rnd::uniformS()));
      acceleration.push_back(randomVec3f(1));
      
    }

    nav().pos(0, 0, 10);
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze) return;

    // ignore the real dt and set the time step;
    dt = timeStep;

    // Calculate forces

    // XXX you put code here that calculates gravitational forces
    // These are pair-wise. Each unique pairing of two particles
    // These are equal but opposite: A exerts a force on B while B exerts that
    // same amount of force on A (but in the opposite direction!) Use a nested
    // for loop to visit each pair once The time complexity is O(n*n)
    //
    // drag
    for (int i = 0; i < velocity.size(); i++) {
      for (int n = i + 1; n < velocity.size(); n++) {
        Vec3f p_dist = mesh.vertices()[i] - mesh.vertices()[n];
        
        float rad = p_dist.mag();

        float gravity = gravConst/(rad*rad);

        acceleration[i] = (gravity) + acceleration[i];
        acceleration[n] = -acceleration[i];              
      }   
    }

    // Vec3f has lots of operations you might use...
    // • +=
    // • -=
    // • .normalize()
    // • .normalize(float scale)
    // • .mag()
    // • .magSqr()
    // • .dot(Vec3f f)
    // • .cross(Vec3f f)

    // Integration
    //
    vector<Vec3f> &position(mesh.vertices());
    for (int i = 0; i < velocity.size(); i++) {
      // "semi-implicit" Euler integration
      velocity[i] += acceleration[i] / mass[i] * dt;
      position[i] += velocity[i] * dt;

      // Explicit (or "forward") Euler integration would look like this:
      // position[i] += velocity[i] * dt;
      // velocity[i] += acceleration[i] / mass[i] * dt;
    }

    // clear all accelerations (IMPORTANT!!)
    for (auto &a : acceleration) a.zero();

/*     for (int i = 0; i < velocity.size(); i++) {
      float distance = (mesh.vertices()[i] - nav().pos()).mag();
      if (distance > 10) {
         cout << "particle:" << i << "is" << distance << "away" << end1;
      }
    } */
  }

  bool onKeyDown(const Keyboard &k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == '1') {
      // introduce some "random" forces
      for (int i = 0; i < velocity.size(); i++) {
        // F = ma
        acceleration[i] = randomVec3f(1) / mass[i];
      }
    }

    return true;
  }

  void onDraw(Graphics &g) override {
    g.clear(0.3);
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(mesh);
  }
};

int main() {
  AlloApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}

string slurp(string fileName) {
  fstream file(fileName);
  string returnValue = "";
  while (file.good()) {
    string line;
    getline(file, line);
    returnValue += line + "\n";
  }
  return returnValue;
}