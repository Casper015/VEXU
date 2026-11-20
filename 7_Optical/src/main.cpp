/*----------------------------------------------------------------------------*/
/*  Optical Sensor test                                                       */
/*                                                                            */
/*  Shows every live reading of the V5 Optical Sensor on the Brain screen:    */
/*    hue, brightness, color(), a hue-based color name, near-object flag,     */
/*    raw RGB + clear channel, LED power, integration time and the number     */
/*    of objectDetected / objectLost events.                                  */
/*                                                                            */
/*  Touch the LEFT half of the screen  -> cycle LED power  (0/25/50/100 %)    */
/*  Touch the RIGHT half of the screen -> cycle integration time (ms)         */
/*                                                                            */
/*  Set the port in include/robot-config.h (OPTICAL_PORT).                    */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

static const int LED_LEVELS[] = {0, 25, 50, 100};
static const int LED_LEVEL_COUNT = sizeof(LED_LEVELS) / sizeof(LED_LEVELS[0]);
static const double INTEGRATION_TIMES[] = {5, 20, 50, 100};
static const int INTEGRATION_COUNT =
    sizeof(INTEGRATION_TIMES) / sizeof(INTEGRATION_TIMES[0]);

static volatile int ledIndex = 2;         // start at 50 %
static volatile int integrationIndex = 1; // start at 20 ms
static volatile int detectCount = 0;
static volatile int lostCount = 0;

static void applyLed() {
  int level = LED_LEVELS[ledIndex];
  if (level == 0) {
    Optical1.setLight(ledState::off);
  } else {
    Optical1.setLight(ledState::on);
    Optical1.setLightPower(level, percent);
  }
}

static void applyIntegrationTime() {
  Optical1.integrationTime(INTEGRATION_TIMES[integrationIndex]);
}

static void onObjectDetected() { detectCount = detectCount + 1; }
static void onObjectLost() { lostCount = lostCount + 1; }

static void onScreenPressed() {
  if (Brain.Screen.xPosition() < 240) {
    ledIndex = (ledIndex + 1) % LED_LEVEL_COUNT;
    applyLed();
  } else {
    integrationIndex = (integrationIndex + 1) % INTEGRATION_COUNT;
    applyIntegrationTime();
  }
}

// Name of the vex::color returned by Optical1.color().
static const char *sensorColorName(const color &c) {
  uint32_t v = c.rgb();
  if (v == color::red.rgb()) return "RED";
  if (v == color::orange.rgb()) return "ORANGE";
  if (v == color::yellow.rgb()) return "YELLOW";
  if (v == color::green.rgb()) return "GREEN";
  if (v == color::cyan.rgb()) return "CYAN";
  if (v == color::blue.rgb()) return "BLUE";
  if (v == color::purple.rgb()) return "PURPLE";
  if (v == color::white.rgb()) return "WHITE";
  if (v == color::black.rgb()) return "BLACK";
  if (c.isTransparent()) return "NONE";
  return "OTHER";
}

// Our own classification from the raw hue angle (0-359 degrees).
static const char *hueBandName(double hue) {
  if (hue < 15 || hue >= 340) return "RED";
  if (hue < 45) return "ORANGE";
  if (hue < 75) return "YELLOW";
  if (hue < 150) return "GREEN";
  if (hue < 195) return "CYAN";
  if (hue < 260) return "BLUE";
  return "PURPLE";
}

static void drawSwatch(int x, int y, int w, int h, const char *label,
                       const color &fill) {
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFillColor(color::black);
  Brain.Screen.printAt(x, y - 4, false, "%s", label);
  Brain.Screen.drawRectangle(x, y, w, h, fill);
}

static void drawDisconnected() {
  Brain.Screen.clearScreen(color::black);
  Brain.Screen.setFont(mono30);
  Brain.Screen.setPenColor(color::red);
  Brain.Screen.printAt(20, 80, false, "Optical NOT detected");
  Brain.Screen.setFont(mono20);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(20, 120, false, "Expected on PORT %d",
                       (int)OPTICAL_PORT + 1);
  Brain.Screen.printAt(20, 145, false, "Check cable / OPTICAL_PORT in");
  Brain.Screen.printAt(20, 170, false, "include/robot-config.h");
  Brain.Screen.render();
}

int main() {
  vexcodeInit();

  // Wait a moment for the sensor to enumerate after power up.
  wait(200, msec);

  applyLed();
  applyIntegrationTime();
  Optical1.objectDetected(onObjectDetected);
  Optical1.objectLost(onObjectLost);
  Brain.Screen.pressed(onScreenPressed);

  bool wasInstalled = false;

  while (true) {
    if (!Optical1.installed()) {
      wasInstalled = false;
      drawDisconnected();
      wait(100, msec);
      continue;
    }
    if (!wasInstalled) {
      // Re-apply settings if the sensor was (re)plugged while running.
      applyLed();
      applyIntegrationTime();
      wasInstalled = true;
    }

    double hue = Optical1.hue();
    double bright = Optical1.brightness();
    color sensed = Optical1.color();
    bool near = Optical1.isNearObject();
    optical::rgbc raw = Optical1.getRgb();
    double integration = Optical1.integrationTime();

    Brain.Screen.clearScreen(color::black);
    Brain.Screen.setFont(mono20);
    Brain.Screen.setFillColor(color::black);

    const int x = 10;
    int y = 22;
    const int dy = 21;

    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(x, y, false, "OPTICAL  PORT %d", (int)OPTICAL_PORT + 1);
    y += dy + 4;

    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(x, y, false, "Hue:      %6.1f deg", hue);
    y += dy;
    Brain.Screen.printAt(x, y, false, "Bright:   %6.1f %%", bright);
    y += dy;
    Brain.Screen.printAt(x, y, false, "color():  %s", sensorColorName(sensed));
    y += dy;
    Brain.Screen.printAt(x, y, false, "Hue band: %s", hueBandName(hue));
    y += dy;

    Brain.Screen.setPenColor(near ? color::green : color::red);
    Brain.Screen.printAt(x, y, false, "Near obj: %s", near ? "YES" : "NO");
    y += dy;

    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(x, y, false, "R:%6.0f  G:%6.0f", raw.red, raw.green);
    y += dy;
    Brain.Screen.printAt(x, y, false, "B:%6.0f  C:%6.0f", raw.blue, raw.clear);
    y += dy;
    Brain.Screen.printAt(x, y, false, "LED:%3d%%  Int:%4.0fms",
                         LED_LEVELS[ledIndex], integration);
    y += dy;
    Brain.Screen.printAt(x, y, false, "Found:%3d  Lost:%3d", (int)detectCount,
                         (int)lostCount);

    Brain.Screen.setPenColor(color(0x808080));
    Brain.Screen.printAt(x, 236, false, "Tap L: LED   Tap R: Int");

    // Right side: colour swatches.
    color hueColor;
    hueColor.hsv((uint32_t)hue % 360, 1.0, 1.0);
    drawSwatch(300, 30, 170, 85, "color()", sensed);
    drawSwatch(300, 145, 170, 85, "hue", hueColor);

    Brain.Screen.render();
    wait(50, msec);
  }
}
