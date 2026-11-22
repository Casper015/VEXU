#ifndef CUP_H
#define CUP_H

// Cup orienter: an Optical Sensor looks at one end of the cup sitting on the
// cascade and decides whether it sees the opaque half or the clear half.
// If the cup is the wrong way round, the cascade motor turns it.
// Every number that tunes this is in config.h ("Cup orienter").

enum class CupSide { unknown, opaque, clear };

// Call once at start-up: sensor LED, integration time, A-button trigger.
void cupInit();

// The single reading chosen by Config::cupDetectBy, as the sensor reports it.
double cupReading();

// What the sensor sees right now (one reading, no waiting).
CupSide cupSideNow();

// Wait until Config::cupAgreeingReads readings in a row agree, or give up
// after Config::cupDetectTimeoutMs and return CupSide::unknown.
CupSide readCupSide();

// Read the cup and turn it if it is the wrong way round. Blocks while it
// works (up to detect timeout + flip timeout). Returns true if the cup ends
// up the right way round, false if it could not decide or could not fix it.
// Safe to call from autonomous. In driver control it runs from the Right arrow.
bool orientCup();

// True while orientCup() is turning the cascade. Driver control must leave
// the cascade motor alone during that time.
bool cupBusy();

// Draw one Brain-screen line with the live readings and the decision.
void cupDebugRow(int row);

#endif // CUP_H
