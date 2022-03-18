// Iason Paterakis, Nefeli Manoudaki
// 02/15/2022
// adapted from Chaotic System // Karl Yerkes // MAT201B W2022
//
#include <cstdio>  // for printing to stdout

#include "Gamma/Analysis.h"
#include "Gamma/Effects.h"
#include "Gamma/Envelope.h"
#include "Gamma/Oscillator.h"
#include "al/app/al_App.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/io/al_AudioIO.hpp"
#include "al/math/al_Random.hpp"
#include "al/scene/al_DistributedScene.hpp"
#include "al/scene/al_PolySynth.hpp"
#include "al/scene/al_SynthSequencer.hpp"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_Parameter.hpp"

using namespace gam;
using namespace al;

template <typename T>
T mtof(T m) {
  return 440 * pow(2, (m - 69) / 12);
}

template <typename T>
T dbtoa(T db) {
  return pow(10, db / 20);
}

//[scale 0. 0. 127. -23. 400.] like Max object
inline float scale(float value, float low, float high, float Low, float High) {
  return Low + (High - Low) * ((value - low) / (high - low));
}

// This is the Attractor system

struct MyApp : public DistributedApp {  // use simple app if not distributed
  static const int P = 15;
  static const int D = 10;

  PresetHandler presetHandler{"presets"};

  Parameter p[P]{
      {"N", "p", 10000, 0, 20000},    // p[0] = N     | (simulation steps)
      {"h", "p", 0.01, 0, 0.018},     // p[1] = h     | (simulation time step)
      {"x0", "p", 0, -D, D},          // p[2] = x0    | initial
      {"y0", "p", 0.1, -D, D},        // p[3] = y0    | conditions
      {"z0", "p", 0, -D, D},          // p[4] = z0    |
      {"rho", "p", 28, 0, 56},        // p[5] = rho   | simulation
      {"sigma", "p", 10, 0, 20},      // p[6] = sigma | parameters
      {"beta", "p", 8.0f / 3, 0, 4},  // p[7] = beta  |
      {"a", "p", 5, -D, 60},          // p[8] = a     |
      {"b", "p", -10, -D, D},         // p[9] = b     |
      {"c", "p", -10, -D, D},         // p[10] = c    |
      {"d", "p", -10, -D, D},         // p[11] = d    |
      {"e", "p", -10, -D, D},         // p[12] = e    |
      {"o", "p", -10, -D, D},         // p[13] = o    |
      {"g", "p", -10, -D, D},         // p[14] = g    |
  };

  Parameter width{"Width", 0.07, 0, 0.2};
  Parameter gain{"Gain", -90, -90, 0};
  ShaderProgram point_shader;

  Mesh system;
  Mesh point;
  bool light{false};  // switch light

  double a{0};

  int selectmode = 0;

  void recompute() {
    system.reset();
    system.primitive(Mesh::LINE_STRIP);  // defines the nature of the drawn
                                         // line?

    system.vertex(p[2], p[3], p[4]);

    for (int i = 0; i < (int)p[0]; i++) {  // draw a point for each iteration
      Vec3f _(system.vertices().back());

      if (selectmode == 0) {
        // Allotsucs, based on the Den Tsucs Attractor by Paul Bourke
        float a(p[8]), c(p[10]), e(p[12]), o(p[14]);
        float h(p[1]);
        Vec3f f((a * (_.y - _.x)) + (_.x * _.z),  //
                (o * _.y) - (_.x * _.z),          //
                (c * _.z) + (_.x * _.y) - (e * pow(_.x, 2)));
        system.vertex(_ + h * f);
        // the line above is Euler's method! */
      }

      if (selectmode == 1) {
        // Lorenz
        float rho(p[5]), sigma(p[6]), beta(p[7]), h(p[1]);
        Vec3f f(sigma * (_.y - _.x),      //
                _.x * (rho - _.z) - _.y,  //
                _.x * _.y - beta * _.z);
        system.vertex(_ + h * f);
        // the line above is Euler's method!
      }

      if (selectmode == 2) {
        // Allorenz
        float rho(p[5]), sigma(p[6]), beta(p[7]), h(p[1]);  // a, b, c
        Vec3f f(_.y * _.z,          // main equation
                rho * (_.x - _.y),  // main equation
                sigma - beta * _.x * _.y - (1 - beta) * pow(_.x, 2));  // main
        system.vertex(_ + h * f);
        // the line above is Euler's method!
      }

      if (selectmode == 3) {
        // Allorenz. Based on the Chen - Lee Attractor
        float a(p[8]), b(p[9]), d(p[11]), h(p[1]);  // a, c, d, e, f
        Vec3f f((a * _.x) - (_.y * _.z), (_.y * b) + (_.x * _.z),
                (d * _.z) + (_.x * _.y / 3));

        system.vertex(_ + h * f);
        // the line above is Euler's method!
      }
    }

    system.ribbonize(width, true);
    system.primitive(Mesh::TRIANGLE_STRIP);
    system.generateNormals();
  }

  void onCreate() override {
    recompute();
    nav().pos(0, 0, 10);
  }

  void onDraw(Graphics& g) override {
    g.clear(0);

    // draw system
    g.depthTesting(light);
    g.lighting(light);
    g.blendTrans();
    g.color(1);
    g.scale(0.1);
    g.draw(system);

    // draw red ends
    g.pointSize(8);
    Mesh m{Mesh::POINTS};
    m.vertex(system.vertices()[0]);
    m.vertex(system.vertices().back());
    g.color(1, 0, 0);
    g.draw(m);
  }

  bool onMouseDrag(Mouse const&) override {
    recompute();
    return true;
  }

  void onInit() override {
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto& gui = GUIdomain->newGUI();
    gui.add(presetHandler);
    for (auto& e : p) {
      gui.add(e);
      parameterServer() << e;
      presetHandler << e;
    }
    presetHandler << width;
    gui.add(width);
    presetHandler << gain;
    gui.add(gain);
    presetHandler.recallPresetSynchronous(7);  // initial condition on startup, how to make autocue?
    // interpolate between 2 presets instead of recall?
    // presetHandler.setInterpolatedPreset(1, 8, 0); //initial condition on
    // startup, how to make autocue? interpolate between 2 presets instead of
    // recall?
  }

  bool onKeyDown(const Keyboard& k) override {
    if (k.key() == ' ') {
      light = !light;
    }

    // XXX ky ~ this code needed to me moved into the onKeyDown function
    //
    if (isPrimary() && k.key() == '0') {
      selectmode = 0;  // XXX ky ~ corrected; was 1, made it 0
    }
    if (isPrimary() && k.key() == '1') {
      selectmode = 1;
    }
    if (isPrimary() && k.key() == '2') {
      selectmode = 2;
    }
    if (isPrimary() && k.key() == '3') {
      selectmode = 3;
    }
    return false;
  }

  gam::Sine<> carrier;
  gam::Sine<> modulator;

  void onSound(AudioIOData& io) override {
    auto hertz = [](al::Parameter& p) -> float {
      return mtof(scale(p.get(), p.min(), p.max(), 0, 127));
    };
    while (io()) {
      modulator.freq(hertz(p[10]));
      carrier.freq(hertz(p[8]) + hertz(p[12]) * modulator());
      float v = carrier();
      io.out(0) = io.out(1) = v * dbtoa(gain.get());
    }
  }
  void onAnimate(double) override { recompute(); }

  DistributedScene scene{
      TimeMasterMode::TIME_MASTER_CPU};  // remove if app is not distributed
};

int main() {
  MyApp app;
  // app.configureAudio(48000, 512, 2, 0);
  app.configureAudio(44100., 1024, 2, 0);
  Domain::master().spu(app.audioIO().framesPerSecond());
  app.start();
}