#include "vex.h"

using namespace vex;

// True while orientCup() is running, so a second press cannot start another.
static bool busy = false;

/*---------------------------------------------------------------------------*/
/*                              Sensor readings                              */
/*---------------------------------------------------------------------------*/

// vex::optical has no saturation() or numeric proximity, so these two come
// from the V5 device API directly.
static double sensorSaturation() {
  return vexDeviceOpticalSatGet(vexDeviceGetByIndex(OpticalSensor.index()));
}

static int sensorProximity() {
  return vexDeviceOpticalProximityGet(vexDeviceGetByIndex(OpticalSensor.index()));
}

static void applySensorSettings() {
  OpticalSensor.setLight(ledState::on);
  OpticalSensor.setLightPower((int32_t)Config::cupLedPercent, percent);
  OpticalSensor.integrationTime(Config::cupIntegrationMs);
}

double cupReading() {
  switch (Config::cupDetectBy) {
    case 0:  return sensorSaturation();
    case 1:  return OpticalSensor.brightness();
    default: return sensorProximity();
  }
}

static const char* detectByName() {
  switch (Config::cupDetectBy) {
    case 0:  return "sat";
    case 1:  return "bri";
    default: return "prox";
  }
}

/*---------------------------------------------------------------------------*/
/*                                 Deciding                                  */
/*---------------------------------------------------------------------------*/

CupSide cupSideNow() {
  if (!OpticalSensor.installed()) return CupSide::unknown;

  const double opaqueT = Config::cupOpaqueThreshold;
  const double clearT = Config::cupClearThreshold;
  if (opaqueT == clearT) return CupSide::unknown;  // not calibrated yet

  double reading = cupReading();
  if (opaqueT > clearT) {
    // Opaque end gives the larger reading.
    if (reading >= opaqueT) return CupSide::opaque;
    if (reading <= clearT) return CupSide::clear;
  } else {
    // Clear end gives the larger reading.
    if (reading <= opaqueT) return CupSide::opaque;
    if (reading >= clearT) return CupSide::clear;
  }
  return CupSide::unknown;  // in the gap between the two thresholds
}

CupSide readCupSide() {
  CupSide last = CupSide::unknown;
  int agreeing = 0;
  uint32_t startMs = Brain.Timer.time();

  while (Brain.Timer.time() - startMs <= Config::cupDetectTimeoutMs) {
    CupSide now = cupSideNow();
    if (now != CupSide::unknown && now == last) {
      agreeing++;
    } else {
      agreeing = (now == CupSide::unknown) ? 0 : 1;
    }
    last = now;

    if (agreeing >= Config::cupAgreeingReads) return now;

    // Wait for a fresh sample before reading again.
    wait(Config::cupIntegrationMs < 10 ? 10 : Config::cupIntegrationMs, msec);
  }
  return CupSide::unknown;
}

/*---------------------------------------------------------------------------*/
/*                                  Acting                                   */
/*---------------------------------------------------------------------------*/

static bool orientCupOnce() {
  applySensorSettings();

  const CupSide wanted =
      Config::cupWantOpaqueAtSensor ? CupSide::opaque : CupSide::clear;

  CupSide seen = readCupSide();
  if (seen == CupSide::unknown) return false;  // no cup, or cannot tell
  if (seen == wanted) return true;             // already the right way round

  // Wrong way round: turn the table, then check again.
  CascadeMotor.setTimeout((int32_t)Config::cupFlipTimeoutMs, msec);
  CascadeMotor.spinFor(forward,
                         Config::cupFlipDegrees * Config::cupTableGearRatio,
                         degrees, Config::cupFlipSpeedPercent,
                         velocityUnits::pct, true);

  return readCupSide() == wanted;
}

bool cupBusy() { return busy; }

bool orientCup() {
  if (!Config::enableCupOrienter || busy) return false;

  busy = true;
  bool ok = orientCupOnce();
  busy = false;
  return ok;
}

// Right arrow in driver control. Button events run in their own thread, so the
// drive loop keeps running while the table turns.
static void onCupButton() {
  if (!orientCup()) {
    Controller1.rumble("-");  // could not decide, or could not fix it
  }
}

void cupInit() {
  if (!Config::enableCupOrienter) return;

  applySensorSettings();
  CascadeMotor.setStopping(hold);
  Controller1.ButtonRight.pressed(onCupButton);
}

/*---------------------------------------------------------------------------*/
/*                               Debug display                               */
/*---------------------------------------------------------------------------*/

void cupDebugRow(int row) {
  Brain.Screen.setCursor(row, 1);
  if (!OpticalSensor.installed()) {
    Brain.Screen.print("CUP  optical port %2d NONE                        ",
                       (int)OpticalSensor.index() + 1);
    return;
  }

  const char* side = "?";
  switch (cupSideNow()) {
    case CupSide::opaque: side = "OPAQUE"; break;
    case CupSide::clear:  side = "CLEAR"; break;
    default:              side = "?"; break;
  }

  // hue, saturation, brightness, proximity, then which one is in use and
  // the decision it gives with the current thresholds.
  Brain.Screen.print("CUP h%3d s%5.2f b%5.1f p%3d  by %s: %-6s   ",
                     (int)OpticalSensor.hue(), sensorSaturation(),
                     OpticalSensor.brightness(), sensorProximity(),
                     detectByName(), side);
}
