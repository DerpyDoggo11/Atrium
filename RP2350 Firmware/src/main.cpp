
#include <Arduino.h>
#include "pins.h"
#include "gnss.h"
#include "hmi.h"
#include <SerialPIO.h>
#include <math.h>

#ifndef PISAT_LINK_HW_UART
#define PISAT_LINK_HW_UART 1
#endif

#if defined(PISAT_BASE)
// ---------------------------------------------------------------------------
//  BASE STATION  --  position-domain differential ("poor-man's DGPS").
//  The base sits at a fixed spot. It surveys-in a reference position (or uses a
//  known one), then each second sends its position ERROR (measured - reference)
//  to the rover as a $PSDG sentence over the link. The rover subtracts it to
//  cancel the common-mode error. Meter-level, not cm RTK, but needs no RTCM-base
//  firmware, no PC and no internet. See RTK_SETUP_INSTRUCTIONS.txt sec. 13.
// ---------------------------------------------------------------------------
static const uint32_t ESP_LINK_BAUD = 115200;
#if PISAT_LINK_HW_UART
static Stream& EspOut = Serial1;                       // GP0 TX ("ESP32" header)
#else
static SerialPIO EspOutPio(PIN_J6[0] /* GP12 TX */, NOPIN);
static Stream& EspOut = EspOutPio;
#endif

// Reference position: a KNOWN surveyed point, or survey-in (average our own fix).
// To use a known point, build with e.g.
//   -DBASE_REF_LAT=48.7597082 -DBASE_REF_LON=-122.3620117
#ifndef BASE_REF_LAT
#define BASE_REF_LAT 0.0
#endif
#ifndef BASE_REF_LON
#define BASE_REF_LON 0.0
#endif
static const int SURVEY_SAMPLES = 60;      // survey-in: 1 sample/s -> ~60 s

static double   refLat = BASE_REF_LAT, refLon = BASE_REF_LON;
static bool     refLocked = (BASE_REF_LAT != 0.0 || BASE_REF_LON != 0.0);
static double   sumLat = 0, sumLon = 0;
static uint32_t sampleN = 0;
static uint32_t corrOut = 0;

// $PSDG,<valid>,<dlat>,<dlon>,<sats>,<fixQ>*hh  -- our position error, degrees.
// The base is stationary, so smooth its measured position (~5 s EMA) before
// differencing -- that strips the base's own receiver noise/multipath out of
// the correction instead of injecting it into the rover. Only send when the
// base fix is healthy (enough sats, low HDOP).
static bool bsmInit = false; static double bsmLat = 0, bsmLon = 0;
static void sendCorrection() {
    const GnssFix& f = Gnss::fix();
    if (Gnss::hasFix()) {
        if (!bsmInit) { bsmLat = f.latitude; bsmLon = f.longitude; bsmInit = true; }
        else { bsmLat += 0.2 * (f.latitude - bsmLat); bsmLon += 0.2 * (f.longitude - bsmLon); }
    }
    int valid = (refLocked && Gnss::hasFix() && f.satellites >= 5 && f.hdop < 3.0f) ? 1 : 0;
    double dlat = valid ? (bsmLat - refLat) : 0.0;
    double dlon = valid ? (bsmLon - refLon) : 0.0;
    char body[112];
    snprintf(body, sizeof(body), "PSDG,%d,%.7f,%.7f,%u,%u,%.7f,%.7f",
             valid, dlat, dlon, f.satellites, f.fixQuality, bsmLat, bsmLon);
    uint8_t sum = 0; for (const char* p = body; *p; p++) sum ^= (uint8_t)*p;
    EspOut.printf("$%s*%02X\r\n", body, sum);
    if (valid) corrOut++;
}

void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Power::begin();
    Gnss::begin();
#if PISAT_LINK_HW_UART
    Serial1.setTX(0); Serial1.setRX(1); Serial1.setFIFOSize(512); Serial1.begin(ESP_LINK_BAUD);
#else
    EspOutPio.begin(ESP_LINK_BAUD);
#endif
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    Serial.println("=== PiSat RTK BASE -- differential (RP2350A) ===");
    if (refLocked)
        Serial.printf("Using KNOWN reference %.7f, %.7f\n", refLat, refLon);
    else
        Serial.printf("Survey-in: averaging position ~%d s after first fix.\n", SURVEY_SAMPLES);
    Serial.println("Sending $PSDG corrections to the ESP32-C3. Type 'help'.");
    Hmi::beep(60);
}

void loop() {
    Gnss::poll();

    static char line[32]; static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (n) {
                line[n] = 0;
                if (!strcmp(line, "help"))
                    Serial.println("gps | sats | ver | stat | nmea | survey | gnss_reset");
                else if (!strcmp(line, "gps") || !strcmp(line, "sats"))
                    Gnss::printStatus();
                else if (!strcmp(line, "ver")) {
                    Gnss::setRawDebug(true);
                    Gnss::sendCommand("PSTMGETSWVER,255");
                    Serial.println("(sent PSTMGETSWVER; reply echoes -- 'nmea' to stop)");
                }
                else if (!strcmp(line, "stat")) {
                    const GnssFix& gf = Gnss::fix();
                    Serial.printf("corrections sent %lu  vbat %.2fV  sats %u  fix q%u\n",
                                  (unsigned long)corrOut, Power::batteryVolts(),
                                  gf.satellites, gf.fixQuality);
                    if (refLocked) Serial.printf("  ref LOCKED %.7f, %.7f\n", refLat, refLon);
                    else Serial.printf("  survey-in %lu/%d samples...\n",
                                       (unsigned long)sampleN, SURVEY_SAMPLES);
                }
                else if (!strcmp(line, "nmea")) {
                    bool on = !Gnss::rawDebug();
                    Gnss::setRawDebug(on);
                    Serial.printf("raw NMEA echo %s\n", on ? "ON" : "OFF");
                }
                else if (!strcmp(line, "survey")) {         // restart survey-in
                    refLocked = (BASE_REF_LAT != 0.0 || BASE_REF_LON != 0.0);
                    sumLat = sumLon = 0; sampleN = 0;
                    Serial.println(refLocked ? "using known ref (survey n/a)"
                                             : "survey-in restarted");
                }
                else if (!strcmp(line, "raw on"))  { Gnss::rawMeasurements(true);  Serial.println("raw measurements ON (for RTKLIB)"); }
                else if (!strcmp(line, "raw off")) { Gnss::rawMeasurements(false); Serial.println("raw measurements OFF"); }
                else if (!strcmp(line, "gnss_reset")) { Gnss::reset(); Serial.println("gnss reset"); }
                n = 0;
            }
        } else if (n < sizeof(line) - 1) line[n++] = c;
    }

    // 1 Hz: survey-in sample + send correction + LEDs
    static uint32_t tSec = 0;
    if (millis() - tSec >= 1000) {
        tSec = millis();
        const GnssFix& f = Gnss::fix();

        if (!refLocked && Gnss::hasFix()) {
            sumLat += f.latitude; sumLon += f.longitude; sampleN++;
            if (sampleN >= (uint32_t)SURVEY_SAMPLES) {
                refLat = sumLat / sampleN; refLon = sumLon / sampleN;
                refLocked = true;
                Serial.printf("survey-in DONE: ref %.7f, %.7f (%lu samples)\n",
                              refLat, refLon, (unsigned long)sampleN);
                Hmi::beep(120);
            }
        }

        sendCorrection();

        if (refLocked && Gnss::hasFix()) Hmi::ledA(true);
        else Hmi::blink(PIN_LED_A, 250);        // surveying / no fix -> blink
        if (Power::batteryVolts() < Power::VBAT_WARN) Hmi::blink(PIN_LED_B, 125);
    }
}

void setup1() {}
void loop1() {}

// ============================================================================
#elif defined(PISAT_CLIENT)
// ---------------------------------------------------------------------------
//  CLIENT PiSat  --  a second GNSS only (no motors). Reads its Teseo and sends
//  its position as $GP2 over the link to its own ESP32-C3, which relays it over
//  WiFi to the MAIN rover's command server. The main rover averages the two
//  fixes to cut L1 noise. See RTK_SETUP_INSTRUCTIONS.txt.
// ---------------------------------------------------------------------------
static const uint32_t ESP_LINK_BAUD = 115200;
#if PISAT_LINK_HW_UART
static Stream& EspOut = Serial1;                       // GP0 TX ("ESP32" header)
#else
static SerialPIO EspOutPio(PIN_J6[0] /* GP12 TX */, NOPIN);
static Stream& EspOut = EspOutPio;
#endif

// $GP2,<lat>,<lon>,<hdop>,<sats>,<fixQ>*hh -- our position for the main rover.
static uint32_t gp2Out = 0;
static void sendGps2() {
    const GnssFix& f = Gnss::fix();
    bool ok = Gnss::hasFix();
    char body[80];
    snprintf(body, sizeof(body), "GP2,%.7f,%.7f,%.2f,%u,%u",
             ok ? f.latitude : 0.0, ok ? f.longitude : 0.0,
             f.hdop, f.satellites, f.fixQuality);
    uint8_t sum = 0; for (const char* p = body; *p; p++) sum ^= (uint8_t)*p;
    EspOut.printf("$%s*%02X\r\n", body, sum);
    if (ok) gp2Out++;
}

void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Power::begin();
    Gnss::begin();
#if PISAT_LINK_HW_UART
    Serial1.setTX(0); Serial1.setRX(1); Serial1.setFIFOSize(512); Serial1.begin(ESP_LINK_BAUD);
#else
    EspOutPio.begin(ESP_LINK_BAUD);
#endif
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    Serial.println("=== PiSat CLIENT -- 2nd GPS ($GP2) (RP2350A) ===");
    Serial.println("Sending $GP2 position to the ESP32-C3. Type 'help'.");
    Hmi::beep(60);
}

void loop() {
    Gnss::poll();

    static char line[32]; static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (n) {
                line[n] = 0;
                if      (!strcmp(line, "help")) Serial.println("gps | sats | stat | nmea | gnss_reset");
                else if (!strcmp(line, "gps") || !strcmp(line, "sats")) Gnss::printStatus();
                else if (!strcmp(line, "stat")) {
                    const GnssFix& gf = Gnss::fix();
                    Serial.printf("$GP2 sent %lu  sats %u  fix q%u  hdop %.2f\n",
                                  (unsigned long)gp2Out, gf.satellites, gf.fixQuality, gf.hdop);
                }
                else if (!strcmp(line, "nmea")) { bool on = !Gnss::rawDebug(); Gnss::setRawDebug(on); Serial.printf("raw NMEA %s\n", on?"ON":"OFF"); }
                else if (!strcmp(line, "gnss_reset")) { Gnss::reset(); Serial.println("gnss reset"); }
                n = 0;
            }
        } else if (n < sizeof(line) - 1) line[n++] = c;
    }

    // 5 Hz position out + fix LED.
    static uint32_t tOut = 0;
    if (millis() - tOut >= 200) {
        tOut = millis();
        sendGps2();
        if (Gnss::hasFix()) Hmi::ledA(true); else Hmi::blink(PIN_LED_A, 250);
    }
}

void setup1() {}
void loop1() {}

// ============================================================================
#else
// ---------------------------------------------------------------------------
//  ROVER  --  GNSS + RTK + pathfinding + two ZS-X111H motor controllers.
//
//  The motors are wired to the PiSat's own breakout GPIO (below); all steering
//  decisions run here. The rover's external ESP32-C3 is a transparent WiFi
//  bridge: it pushes RTCM corrections (from the base, over WiFi) into GP13, and
//  relays our telemetry text from GP12 out to the Processing client.
// ---------------------------------------------------------------------------

// ---- external C3 bridge link -----------------------------------------------
// Default: hardware UART Serial1 on the onboard "ESP32" header (GP0 TX, GP1 RX).
// Set -DPISAT_LINK_HW_UART=0 to use the J6 soft-UART on GP12/GP13 instead.
static const uint32_t ESP_LINK_BAUD = 115200;
#if PISAT_LINK_HW_UART
static Stream& link = Serial1;
#else
static SerialPIO EspLinkIO(PIN_J6[0] /* GP12 TX telemetry out */,
                          PIN_J6[1] /* GP13 RX RTCM in */);
static Stream& link = EspLinkIO;
#endif

// ---- ZS-X111H motor controllers, wired to the PiSat breakout ---------------
// Four wires per wheel: P (PWM speed, out), ZF (rotation/direction, out),
// BRAKE (out), M (speed-feedback pulse, in). The C3 link now lives on the
// "ESP32" header (GP0/GP1), so J6 GP12/GP13 are free for the LEFT wheel.
// RIGHT wheel (ZS-X111H #1)
static const uint8_t M_PWM_R   = 20;   // J5 -> P     (PWM speed)         GPIO 20
static const uint8_t M_SPEED_R = 21;   // J5 <- M     (speed feedback)    GPIO 21
static const uint8_t M_BRAKE_R = 22;   // J5 -> BRAKE                     GPIO 22
static const uint8_t M_DIR_R   = 23;   // J5 -> ZF    (rotation)          GPIO 23
// LEFT wheel (ZS-X111H #2)
static const uint8_t M_PWM_L   = 15;   // J6 -> P     (PWM speed)         GPIO 15
static const uint8_t M_SPEED_L = 19;   // J6 <- M     (speed feedback)    GPIO 19
static const uint8_t M_BRAKE_L = 13;   // J6 -> BRAKE                     GPIO 13
static const uint8_t M_DIR_L   = 12;   // J6 -> ZF    (rotation)          GPIO 12

// LEFT brake/rotation use GP12/GP13, free only because the link is on the
// hardware UART. If you switch the link back to the J6 soft-UART, they clash.
#if !PISAT_LINK_HW_UART
#error "LEFT motor uses GP12/GP13, which the soft-UART link (PISAT_LINK_HW_UART=0) also needs. Keep the HW-UART link, or move M_BRAKE_L/M_DIR_L to other free pins (e.g. GP14/GP24)."
#endif

// DIR level that makes each wheel drive the robot FORWARD. Flip a constant here
// (rather than swapping motor phase wires) if a wheel spins the wrong way.
static const int DIR_R_FORWARD = HIGH;
static const int DIR_L_FORWARD = HIGH;
static const int BRAKE_ON  = HIGH;     // ZS-X111H brake engages on high
static const int BRAKE_OFF = LOW;

// Wheel-speed feedback: the ZS-X111H M pin pulses as the wheel turns. We count
// edges and report the pulse frequency (Hz) in telemetry. Divide by your
// motor's pulses-per-revolution to get RPM.
static volatile uint32_t s_pulsesR = 0, s_pulsesL = 0;
static float g_spdHzR = 0, g_spdHzL = 0;
static void isrSpeedR() { s_pulsesR++; }
static void isrSpeedL() { s_pulsesL++; }

// ---- PATH + BOUNDARY (RAM, pushed from the planner/runtime over WiFi) -------
// $WPT,<i>,<n>,<lat>,<lon> and $BND,<i>,<n>,<lat>,<lon> fill these; they persist
// until replaced. The rover follows g_wp[] and STOPS if it leaves g_bnd[].
struct LatLon { double lat; double lon; };
static const int MAX_WP = 64, MAX_BND = 64;
static LatLon g_wp[MAX_WP];   static int g_wpCount  = 0;
static LatLon g_bnd[MAX_BND]; static int g_bndCount = 0;

// ---- TUNING ----------------------------------------------------------------
static float MAX_PWM = 40;                         // 0..255, base wheel speed
static const float WAYPOINT_RADIUS_M = 1.5f;       // waypoint reached within (m)
static const float MIN_SPEED_FOR_COURSE = 0.4f;    // m/s below which course is bad
static const float PIVOT_DEG = 70.0f;              // |err| beyond this -> pivot in place
// PID on heading error (degrees) -> steering command in [-1, 1]
static float KP = 0.020f, KI = 0.0f, KD = 0.004f;

// ---- STATE -----------------------------------------------------------------
static float currentCourse = 0, currentSpeed = 0;
static float targetBearing = 0, headingError = 0;
static float lastPwmR = 0, lastPwmL = 0;
static int   currentSegmentIndex = 0;
static bool  pathCompleted = false;
static bool  hadFix = false;
static bool  g_hold = false;        // safety hold (brakes motors)
static float g_vbat = 0.0f;

// PID + manual-control state
static float    pidI = 0, pidPrevErr = 0;
static uint32_t pidPrevMs = 0;
static bool     manualMode = false;          // WASD override from the visualiser
static float    manThrottle = 0, manTurn = 0;// both in [-1, 1]
static uint32_t manLastMs = 0;
static const uint32_t MAN_TIMEOUT_MS = 600;  // no manual cmd this long -> stop

// ---- differential correction from the base ($PSDG) -------------------------
// The base sends its position error (measured - reference). We subtract it from
// our own fix to cancel the common-mode error -> better, more repeatable position.
static bool     corrValid  = false;   // a fresh, valid correction is available
static bool     diffOn     = false;   // APPLY the differential? default OFF -- on
                                      // L1 it often adds noise (base off = better).
static double   corrDLat   = 0.0;     // base error, degrees latitude
static double   corrDLon   = 0.0;     // base error, degrees longitude
static double   baseLat = 0, baseLon = 0;  // base's own GPS position (for the map dot)
static bool     haveBasePos = false;
static uint8_t  corrBaseSats = 0, corrBaseFixQ = 0;
static uint32_t lastRtcmMs = 0;       // ms of last valid correction (link health)
static const uint32_t CORR_TIMEOUT_MS = 5000;  // correction stale after this

// Second GPS: a client PiSat (GNSS only) sends its position as $GP2 over the
// link. We average the two fixes (inverse-HDOP weighted) to beat down L1 noise.
static double   g2_lat = 0, g2_lon = 0;
static float    g2_hdop = 99.0f; static uint8_t g2_sats = 0;
static uint32_t g2_ms = 0;
static const uint32_t GP2_TIMEOUT_MS      = 2000;  // 2nd GPS stale after this
static const double   GP2_MAX_DISAGREE_M  = 8.0;   // beyond this the two GPS
                                                   // disagree -> trust lower HDOP

// Calibration bias: a constant lat/lon shift added to the fix to align it with a
// known truth point. Set live via $CAL,<dlat>,<dlon> from the visualiser; $CAL,0,0
// clears. Persists in RAM until reboot (re-send after a power cycle).
static double calDLat = 0.0, calDLon = 0.0;

// Corrected position actually used for navigation + telemetry.
static double g_lat = 0.0, g_lon = 0.0;

// local metre frame (equirectangular, origin = first valid fix)
static bool   originSet = false;
static double originLat = 0, originLon = 0;
static double mPerDegLat = 111320.0, mPerDegLon = 111320.0;

struct Vec2 { double e; double n; };

static void setOrigin(double lat, double lon) {
    originLat = lat; originLon = lon;
    mPerDegLat = 111320.0;
    mPerDegLon = 111320.0 * cos(lat * M_PI / 180.0);
    originSet = true;
}
static Vec2 toLocal(double lat, double lon) {
    return { (lon - originLon) * mPerDegLon, (lat - originLat) * mPerDegLat };
}

// ---- UTILS -----------------------------------------------------------------
static float normalizeAngle(float a) {          // -> [-180, 180)
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}
static float bearingBetween(const Vec2& a, const Vec2& b) {
    float deg = (float)(atan2(b.e - a.e, b.n - a.n) * 180.0 / M_PI);
    if (deg < 0) deg += 360.0f;
    return deg;
}
static double dist2(const Vec2& a, const Vec2& b) {
    double dE = b.e - a.e, dN = b.n - a.n;
    return dE * dE + dN * dN;
}
static bool isInsidePolygon(const Vec2& p, const LatLon poly[], int count) {
    bool inside = false;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        Vec2 pi = toLocal(poly[i].lat, poly[i].lon);
        Vec2 pj = toLocal(poly[j].lat, poly[j].lon);
        if (((pi.n > p.n) != (pj.n > p.n)) &&
            (p.e < (pj.e - pi.e) * (p.n - pi.n) / (pj.n - pi.n) + pi.e))
            inside = !inside;
    }
    return inside;
}

// ---- MOTORS ----------------------------------------------------------------
// The proven MowerBot V2 drove the ZS-X11H with the PWM pin ONLY -- it never
// touched DIR or BRAKE. Driving BRAKE holds the controller stopped (dead motors
// even with correct wiring + LED on). So by default we match MowerBot: PWM-only,
// forward-only, DIR/BRAKE left hi-Z. Set -DMOTOR_USE_DIR=1 only after you have
// verified your board's DIR/BRAKE polarity and want reverse/pivot.
#ifndef MOTOR_USE_DIR
#define MOTOR_USE_DIR 0
#endif
static void motorsInit() {
    analogWriteFreq(1000);
    analogWriteRange(255);
    pinMode(M_PWM_R, OUTPUT); pinMode(M_PWM_L, OUTPUT);
    analogWrite(M_PWM_R, 0);  analogWrite(M_PWM_L, 0);
#if MOTOR_USE_DIR
    pinMode(M_DIR_R, OUTPUT); pinMode(M_BRAKE_R, OUTPUT);
    pinMode(M_DIR_L, OUTPUT); pinMode(M_BRAKE_L, OUTPUT);
    digitalWrite(M_DIR_R, DIR_R_FORWARD); digitalWrite(M_DIR_L, DIR_L_FORWARD);
    digitalWrite(M_BRAKE_R, BRAKE_OFF);   digitalWrite(M_BRAKE_L, BRAKE_OFF);
#else
    pinMode(M_DIR_R, INPUT); pinMode(M_BRAKE_R, INPUT);   // hi-Z: leave the
    pinMode(M_DIR_L, INPUT); pinMode(M_BRAKE_L, INPUT);   // controller at its defaults, like MowerBot
#endif
    pinMode(M_SPEED_R, INPUT_PULLUP);
    pinMode(M_SPEED_L, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M_SPEED_R), isrSpeedR, FALLING);
    attachInterrupt(digitalPinToInterrupt(M_SPEED_L), isrSpeedL, FALLING);
}

// Convert the pulse counts to a frequency (Hz) over a ~250 ms window.
static void updateWheelSpeeds() {
    static uint32_t prevMs = 0, prevR = 0, prevL = 0;
    uint32_t now = millis(), dt = now - prevMs;
    if (dt >= 250) {
        uint32_t cR = s_pulsesR, cL = s_pulsesL;
        g_spdHzR = (cR - prevR) * 1000.0f / dt;
        g_spdHzL = (cL - prevL) * 1000.0f / dt;
        prevR = cR; prevL = cL; prevMs = now;
    }
}
static void motorsStop() {
    lastPwmR = lastPwmL = 0;
    analogWrite(M_PWM_R, 0);  analogWrite(M_PWM_L, 0);
#if MOTOR_USE_DIR
    digitalWrite(M_BRAKE_R, BRAKE_OFF);  digitalWrite(M_BRAKE_L, BRAKE_OFF);
#endif
}
static void motorsBrake() {
    lastPwmR = lastPwmL = 0;
    analogWrite(M_PWM_R, 0);  analogWrite(M_PWM_L, 0);
#if MOTOR_USE_DIR
    digitalWrite(M_BRAKE_R, BRAKE_ON);   digitalWrite(M_BRAKE_L, BRAKE_ON);
#endif
}
// Differential drive from throttle + steer (both in [-1, 1]). +steer = turn right.
static void driveDiff(float throttle, float steer) {
    float base = throttle * MAX_PWM, d = steer * MAX_PWM;
#if MOTOR_USE_DIR
    float r = constrain(base - d, -MAX_PWM, MAX_PWM);   // signed: reverse allowed
    float l = constrain(base + d, -MAX_PWM, MAX_PWM);
    digitalWrite(M_BRAKE_R, BRAKE_OFF); digitalWrite(M_DIR_R, (r>=0)?DIR_R_FORWARD:!DIR_R_FORWARD);
    digitalWrite(M_BRAKE_L, BRAKE_OFF); digitalWrite(M_DIR_L, (l>=0)?DIR_L_FORWARD:!DIR_L_FORWARD);
    analogWrite(M_PWM_R, (int)fabsf(r)); analogWrite(M_PWM_L, (int)fabsf(l));
#else
    float r = constrain(base - d, 0, MAX_PWM);          // MowerBot: forward-only, PWM only
    float l = constrain(base + d, 0, MAX_PWM);
    analogWrite(M_PWM_R, (int)r);  analogWrite(M_PWM_L, (int)l);
#endif
    lastPwmR = r; lastPwmL = l;
}

// ---- PATHFINDING (go-to-point sequencing + PID steering) -------------------
// currentSegmentIndex = the waypoint we are currently driving AT (starts at 0,
// so we head for g_wp[0] first). Within WAYPOINT_RADIUS_M it is "reached":
// advance to the next. Processing check-marks every wp with index < that.
// Done once the last waypoint is reached.
static void pathfind() {
    if (g_wpCount < 1)   { motorsStop();  return; }
    if (pathCompleted)   { motorsBrake(); return; }
    if (!Gnss::hasFix()) { motorsStop();  pidI = 0; return; }

    Vec2 pos = toLocal(g_lat, g_lon);   // differentially-corrected position

    if (g_bndCount >= 3 && !isInsidePolygon(pos, g_bnd, g_bndCount)) {
        Serial.println("LEFT boundary zone -> emergency stop");
        motorsBrake(); Hmi::beep(300); pathCompleted = true; return;
    }

    // Reached the current target waypoint? mark it and step to the next.
    Vec2 target = toLocal(g_wp[currentSegmentIndex].lat, g_wp[currentSegmentIndex].lon);
    if (dist2(pos, target) < WAYPOINT_RADIUS_M * WAYPOINT_RADIUS_M) {
        Serial.printf("Waypoint %d reached\n", currentSegmentIndex);
        Hmi::beep(80);
        currentSegmentIndex++;
        if (currentSegmentIndex >= g_wpCount) {
            Serial.println("All waypoints reached -> path complete");
            motorsBrake(); Hmi::beep(200); pathCompleted = true;
        }
        return;                         // re-aim at the new target next loop
    }

    targetBearing = bearingBetween(pos, target);

    // Heading error (GPS course is only trustworthy while moving).
    bool reliable = currentSpeed >= MIN_SPEED_FOR_COURSE;
    headingError = reliable ? normalizeAngle(targetBearing - currentCourse) : 0.0f;

    // PID -> steering in [-1, 1].
    uint32_t now = millis();
    float dt = pidPrevMs ? (now - pidPrevMs) / 1000.0f : 0.1f;
    if (dt <= 0) dt = 0.1f;
    pidPrevMs = now;
    pidI = constrain(pidI + headingError * dt, -400.0f, 400.0f);   // anti-windup
    float deriv = (headingError - pidPrevErr) / dt;
    pidPrevErr = headingError;
    float steer = constrain(KP * headingError + KI * pidI + KD * deriv, -1.0f, 1.0f);

    // Full throttle unless we must pivot in place for a big heading error.
    float throttle = (fabsf(headingError) >= PIVOT_DEG) ? 0.0f : 1.0f;
    driveDiff(throttle, steer);
}

// ---- Inbound link lines over the link (from base + from the visualiser) -----
//   $PSDG,<valid>,<dlat>,<dlon>,<sats>,<fixQ>   base differential correction
//   $WPT,<i>,<n>,<lat>,<lon>                     waypoint i of n (path)
//   $BND,<i>,<n>,<lat>,<lon>                     boundary vertex i of n
//   $MAN,<throttle>,<turn>                       manual WASD drive ([-1,1] each)
//   $AUTO                                        resume autonomous path-following
//   $HOLD[,<0|1>]                                brake hold (toggle or set)
//   $PCLR                                        clear path + boundary
// All are "<body>*HH" XOR-checksummed. linkFields() verifies + splits.
static uint8_t linkFields(char* s, char* f[], uint8_t maxF) {
    char* star = strchr(s, '*');
    if (!star || strlen(star) < 3) return 0;
    uint8_t sum = 0;
    for (char* p = s + 1; p < star; p++) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)strtol(star + 1, nullptr, 16)) return 0;   // bad checksum
    uint8_t n = 0; f[n++] = s;
    for (char* p = s; *p && n < maxF; p++) {
        if (*p == ',')      { *p = '\0'; f[n++] = p + 1; }
        else if (*p == '*') { *p = '\0'; break; }
    }
    return n;
}

static void dispatchLine(char* s) {
    char* f[8];
    uint8_t n = linkFields(s, f, 8);
    if (n < 1) return;

    if (!strncmp(f[0], "$PSDG", 5) && n >= 6) {
        double dla = atof(f[2]), dlo = atof(f[3]);
        // sanity: a differential offset > ~100 m is a base glitch -> ignore
        if (atoi(f[1]) == 1 && fabs(dla) < 0.001 && fabs(dlo) < 0.001) {
            corrDLat = dla; corrDLon = dlo;
            corrBaseSats = (uint8_t)atoi(f[4]); corrBaseFixQ = (uint8_t)atoi(f[5]);
            corrValid = true; lastRtcmMs = millis();
            if (n >= 8) { baseLat = atof(f[6]); baseLon = atof(f[7]); haveBasePos = true; }
        }
    }
    else if (!strncmp(f[0], "$WPT", 4) && n >= 5) {
        int i = atoi(f[1]), cnt = atoi(f[2]);
        if (cnt < 0 || cnt > MAX_WP) return;
        if (i == 0) g_wpCount = 0;                      // first of a new set
        if (i >= 0 && i < MAX_WP) {
            g_wp[i].lat = atof(f[3]); g_wp[i].lon = atof(f[4]);
            if (i + 1 > g_wpCount) g_wpCount = i + 1;
        }
        if (i == cnt - 1) { currentSegmentIndex = 0; pathCompleted = false; }
    }
    else if (!strncmp(f[0], "$BND", 4) && n >= 5) {
        int i = atoi(f[1]), cnt = atoi(f[2]);
        if (cnt < 0 || cnt > MAX_BND) return;
        if (i == 0) g_bndCount = 0;
        if (i >= 0 && i < MAX_BND) {
            g_bnd[i].lat = atof(f[3]); g_bnd[i].lon = atof(f[4]);
            if (i + 1 > g_bndCount) g_bndCount = i + 1;
        }
    }
    else if (!strncmp(f[0], "$MAN", 4) && n >= 3) {
        manThrottle = constrain((float)atof(f[1]), -1.0f, 1.0f);
        manTurn     = constrain((float)atof(f[2]), -1.0f, 1.0f);
        manualMode  = true; manLastMs = millis();
    }
    else if (!strncmp(f[0], "$GP2", 4) && n >= 4) {     // second GPS position
        double la = atof(f[1]), lo = atof(f[2]);
        if (la != 0.0 && lo != 0.0) {                   // 0,0 = no fix on client
            g2_lat = la; g2_lon = lo;
            g2_hdop = (float)atof(f[3]);
            g2_sats = (n >= 5) ? (uint8_t)atoi(f[4]) : 0;
            g2_ms = millis();
        }
    }
    else if (!strncmp(f[0], "$CAL", 4) && n >= 3) { calDLat = atof(f[1]); calDLon = atof(f[2]); }
    else if (!strncmp(f[0], "$SPD", 4) && n >= 2) { MAX_PWM = constrain((float)atof(f[1]), 0.0f, 255.0f); }
    else if (!strncmp(f[0], "$AUTO", 5)) { manualMode = false; pidI = 0; pidPrevErr = 0; }
    else if (!strncmp(f[0], "$HOLD", 5)) { g_hold = (n >= 2) ? (atoi(f[1]) != 0) : !g_hold; }
    else if (!strncmp(f[0], "$PCLR", 5)) { g_wpCount = 0; g_bndCount = 0; currentSegmentIndex = 0; pathCompleted = false; }
}

// The link carries BOTH raw RTCM3 (from a base or an NTRIP client, binary,
// starts 0xD3) AND our "$..." command sentences. This byte state machine demuxes
// them: complete RTCM3 frames are pushed straight into the GNSS (on-chip DGPS/
// RTK), and "$" lines are dispatched as commands.
static char    s_linkBuf[96]; static uint8_t  s_linkLen = 0;
static uint8_t s_rtcm[1029];  static uint16_t s_rtcmIdx = 0, s_rtcmLen = 0;
static uint8_t s_lstate = 0;  // 0 idle, 1 RTCM3 frame, 2 "$" line
static void pumpLink() {
    while (link.available()) {
        uint8_t c = (uint8_t)link.read();
        if (s_lstate == 0) {
            if (c == 0xD3) { s_rtcm[0]=c; s_rtcmIdx=1; s_rtcmLen=0; s_lstate=1; }
            else if (c == '$') { s_linkBuf[0]='$'; s_linkLen=1; s_lstate=2; }
        } else if (s_lstate == 1) {                        // RTCM3 frame
            s_rtcm[s_rtcmIdx++] = c;
            if (s_rtcmIdx == 3) {
                uint16_t pl = ((s_rtcm[1] & 0x03) << 8) | s_rtcm[2];
                if (pl > 1023) { s_lstate = 0; continue; }
                s_rtcmLen = pl + 6;                        // 3 hdr + payload + 3 CRC
            }
            if (s_rtcmLen && s_rtcmIdx >= s_rtcmLen) { Gnss::write(s_rtcm, s_rtcmLen); s_lstate = 0; }
            else if (s_rtcmIdx >= sizeof(s_rtcm)) s_lstate = 0;
        } else {                                           // "$" command line
            if (c == '\r' || c == '\n') {
                s_linkBuf[s_linkLen] = '\0';
                if (s_linkLen > 4) dispatchLine(s_linkBuf);
                s_lstate = 0;
            } else if (s_linkLen < sizeof(s_linkBuf) - 1) s_linkBuf[s_linkLen++] = (char)c;
            else s_lstate = 0;
        }
    }
}

// Apply the base correction to our raw fix (if fresh), then SPEED-ADAPTIVE
// smoothing: heavy EMA when nearly stationary (hides L1 multipath jitter),
// light when moving (keeps path-following responsive). Set SMOOTH_MIN=1 to
// disable. This reduces visible drift; it does not add true accuracy.
static const float SMOOTH_MIN = 0.08f;   // stationary: strong smoothing
static const float SMOOTH_MAX = 0.5f;    // >=1 m/s: light smoothing
static const float GPS_JUMP_M = 5.0f;    // reject fixes that leap further than
                                         // this in one cycle (beyond what our
                                         // speed allows) -- L1 multipath spikes
static const uint8_t GPS_RELOCK_N = 5;   // after this many rejects in a row, the
                                         // "spike" is really where we are now ->
                                         // re-lock (handles a real reposition)
static bool   sm_init = false;
static double sm_lat = 0, sm_lon = 0;
static uint32_t sm_ms = 0;
static uint32_t g_rejects = 0;           // outliers dropped (telemetry-worthy)
static uint8_t  sm_rejStreak = 0;        // consecutive rejects

static void applyCorrection() {
    const GnssFix& f = Gnss::fix();
    bool fresh = diffOn && corrValid && (millis() - lastRtcmMs) < CORR_TIMEOUT_MS;
    double clat = f.latitude  - (fresh ? corrDLat : 0.0) + calDLat;
    double clon = f.longitude - (fresh ? corrDLon : 0.0) + calDLon;

    // Fuse the 2nd GPS if fresh: inverse-HDOP weighted mean when the two agree,
    // else trust whichever has the better HDOP. Halves independent L1 noise.
    // Same calibration bias applies to the client (assumed common-site error).
    if (Gnss::hasFix() && g2_lat != 0.0 && (millis() - g2_ms) < GP2_TIMEOUT_MS) {
        double g2la = g2_lat + calDLat, g2lo = g2_lon + calDLon;
        double dN = (g2la - clat) * 111320.0;
        double dE = (g2lo - clon) * 111320.0 * cos(clat * M_PI / 180.0);
        double disagree = sqrt(dN * dN + dE * dE);
        if (disagree <= GP2_MAX_DISAGREE_M) {
            float w1 = 1.0f / max(0.5f, f.hdop), w2 = 1.0f / max(0.5f, g2_hdop);
            clat = (clat * w1 + g2la * w2) / (w1 + w2);
            clon = (clon * w1 + g2lo * w2) / (w1 + w2);
        } else if (g2_hdop < f.hdop) { clat = g2la; clon = g2lo; }
    }

    if (!Gnss::hasFix()) { g_lat = clat; g_lon = clon; return; }
    if (!sm_init) { sm_lat = clat; sm_lon = clon; sm_init = true; sm_ms = millis(); }
    else {
        // Outlier reject: how far did the raw fix move vs. how far we could have
        // gone (speed * dt + margin)? A spike beyond that is bad data -> drop it.
        uint32_t now = millis();
        float dt = (now - sm_ms) / 1000.0f; if (dt <= 0) dt = 0.1f;
        double dN = (clat - sm_lat) * 111320.0;
        double dE = (clon - sm_lon) * 111320.0 * cos(clat * M_PI / 180.0);
        double jump = sqrt(dN * dN + dE * dE);
        double budget = f.speed_mps * dt + GPS_JUMP_M;
        if (jump > budget) {
            g_rejects++;
            if (++sm_rejStreak < GPS_RELOCK_N) return;   // transient spike -> drop
            sm_lat = clat; sm_lon = clon;                // sustained -> re-lock
            sm_rejStreak = 0; sm_ms = now;
            g_lat = sm_lat; g_lon = sm_lon; return;
        }
        sm_rejStreak = 0; sm_ms = now;
        float a = constrain(SMOOTH_MIN + f.speed_mps * (SMOOTH_MAX - SMOOTH_MIN),
                            SMOOTH_MIN, SMOOTH_MAX);
        sm_lat += a * (clat - sm_lat);
        sm_lon += a * (clon - sm_lon);
    }
    g_lat = sm_lat; g_lon = sm_lon;
}

// ---- TELEMETRY out: text -> GP12 -> C3 bridge -> Processing -----------------
// One-time-ish header (path/boundary/origin) resent periodically so a late
// Processing client still gets it; then live T lines at 10 Hz.
// Append an XOR checksum + newline so a corrupted line is REJECTED by Processing
// (a flipped digit no longer plots a wild point). Checksum = XOR of all body
// chars, printed as 2 hex digits: "<body>*HH\n".
static void sendLine(const char* body) {
    uint8_t sum = 0;
    for (const char* p = body; *p; p++) sum ^= (uint8_t)*p;
    link.printf("%s*%02X\n", body, sum);
}
static void sendPathHeader() {
    char b[64];
    snprintf(b, sizeof(b), "M,%.0f,%.2f,%d,%d", MAX_PWM, WAYPOINT_RADIUS_M,
             g_wpCount, g_bndCount); sendLine(b);
    for (int i = 0; i < g_wpCount; i++) {
        snprintf(b, sizeof(b), "W,%d,%.7f,%.7f", i, g_wp[i].lat, g_wp[i].lon);
        sendLine(b);
    }
    for (int i = 0; i < g_bndCount; i++) {
        snprintf(b, sizeof(b), "B,%d,%.7f,%.7f", i, g_bnd[i].lat, g_bnd[i].lon);
        sendLine(b);
    }
    if (originSet) { snprintf(b, sizeof(b), "O,%.7f,%.7f", originLat, originLon); sendLine(b); }
    if (haveBasePos) { snprintf(b, sizeof(b), "BP,%.7f,%.7f", baseLat, baseLon); sendLine(b); }
}
static void sendTelemetry() {
    const GnssFix& f = Gnss::fix();
    uint8_t fq = Gnss::hasFix() ? f.fixQuality : 0;
    unsigned long age = lastRtcmMs ? (millis() - lastRtcmMs) : 999999;
    int mode = g_hold ? 2 : (manualMode ? 1 : 0);   // 0 auto, 1 manual, 2 hold
    // ...,<corrAgeMs>,<pwmR>,<pwmL>,<spdHzR>,<spdHzL>,<mode>,<vbat>,<maxPwm>
    char t[220];
    snprintf(t, sizeof(t),
        "T,%lu,%u,%.7f,%.7f,%.1f,%.2f,%u,%.1f,%d,%.1f,%.1f,%d,%lu,%.0f,%.0f,%.1f,%.1f,%d,%.2f,%.0f",
        (unsigned long)millis(), fq, g_lat, g_lon,
        currentCourse, currentSpeed, f.satellites, f.hdop,
        currentSegmentIndex, targetBearing, headingError, pathCompleted ? 1 : 0,
        age, lastPwmR, lastPwmL, g_spdHzR, g_spdHzL, mode, g_vbat, MAX_PWM);
    sendLine(t);
}

// ---- DRIVE CONTROL (10 Hz): hold > manual (WASD) > autonomous path ----------
static void driveControl() {
    if (g_hold) { motorsBrake(); return; }
    if (manualMode) {
        if (millis() - manLastMs < MAN_TIMEOUT_MS) driveDiff(manThrottle, manTurn);
        else motorsStop();          // manual cmds stopped -> coast (stay in manual)
        return;
    }
    pathfind();
}

// ---- USB console -----------------------------------------------------------
static void handleCommand(char* cmd) {
    const GnssFix& f = Gnss::fix();
    if (!strcmp(cmd, "help")) {
        Serial.println("gps|sats|ver|aux|stat|path|spd <n>|hold|manual|auto|diff on|off|gnss_reset|nmea|ant on|off");
    }
    else if (!strncmp(cmd, "spd ", 4)) { MAX_PWM = constrain((float)atof(cmd+4), 0.0f, 255.0f); Serial.printf("MAX_PWM = %.0f\n", MAX_PWM); }
    else if (!strcmp(cmd, "manual")) { manualMode = true; manLastMs = millis(); Serial.println("manual mode (drive with WASD / $MAN)"); }
    else if (!strcmp(cmd, "path"))
        Serial.printf("waypoints %d  boundary %d  seg %d  mode %s\n",
                      g_wpCount, g_bndCount, currentSegmentIndex,
                      g_hold ? "HOLD" : (manualMode ? "MANUAL" : "AUTO"));
    else if (!strcmp(cmd, "auto")) { manualMode = false; pidI = 0; Serial.println("autonomous mode"); }
    else if (!strcmp(cmd, "diff on"))  { diffOn = true;  Serial.println("differential ON (base correction applied)"); }
    else if (!strcmp(cmd, "diff off")) { diffOn = false; Serial.println("differential OFF (raw GNSS)"); }
    else if (!strcmp(cmd, "ver")) {
        Gnss::setRawDebug(true);
        Gnss::sendCommand("PSTMGETSWVER,255");
        Serial.println("(sent PSTMGETSWVER; $PSTMVER reply will echo -- 'nmea' to stop)");
    }
    else if (!strcmp(cmd, "hold")) {
        g_hold = !g_hold;
        Serial.printf("motor HOLD %s\n", g_hold ? "ON (braked)" : "OFF");
    }
    else if (!strcmp(cmd, "gps") || !strcmp(cmd, "sats")) {
        Gnss::printStatus();
    }
    else if (!strcmp(cmd, "aux"))
        Serial.printf("J4: A1 %.3fV  A2 %.3fV  A3 %.3fV\n",
                      Power::readAux(0), Power::readAux(1), Power::readAux(2));
    else if (!strcmp(cmd, "stat")) {
        unsigned long cAge = lastRtcmMs ? (millis() - lastRtcmMs) : 999999UL;
        bool cFresh = corrValid && cAge < CORR_TIMEOUT_MS;
        Serial.printf("vbat %.2f V  usb %d  hold %d  seg %d  fix %d q%u  "
                      "pwm R%.0f/L%.0f  spd R%.1f/L%.1f Hz\n",
                      g_vbat, (int)Power::usbPresent(), (int)g_hold,
                      currentSegmentIndex, (int)Gnss::hasFix(), f.fixQuality,
                      lastPwmR, lastPwmL, g_spdHzR, g_spdHzL);
        Serial.printf("  diff-corr: %s  age %lums  dLat %.7f dLon %.7f  base(sats %u q%u)\n",
                      cFresh ? "APPLIED" : "none", cAge, corrDLat, corrDLon,
                      corrBaseSats, corrBaseFixQ);
    }
    else if (!strcmp(cmd, "gnss_reset")) { Gnss::reset(); Gnss::configureRover(); Serial.println("gnss reset"); }
    else if (!strcmp(cmd, "nmea")) {
        bool on = !Gnss::rawDebug();
        Gnss::setRawDebug(on);
        Serial.printf("raw NMEA echo %s\n", on ? "ON" : "OFF");
    }
    else if (!strcmp(cmd, "ant on"))  { Gnss::antennaPower(true);  Serial.println("ant on"); }
    else if (!strcmp(cmd, "ant off")) { Gnss::antennaPower(false); Serial.println("ant off"); }
    else if (!strcmp(cmd, "raw on"))  { Gnss::rawMeasurements(true);  Serial.println("raw measurements ON (for RTKLIB)"); }
    else if (!strcmp(cmd, "raw off")) { Gnss::rawMeasurements(false); Serial.println("raw measurements OFF"); }
}

// ---- SETUP / LOOP ----------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Power::begin();
    Gnss::begin();
    Gnss::configureRover();     // accept RTCM3 in, report RTK quality in GGA

    motorsInit();   // owns GP12/13/15/19/20/21/22/23 (both ZS-X111H)
    motorsStop();

#if PISAT_LINK_HW_UART
    Serial1.setTX(0); Serial1.setRX(1); Serial1.setFIFOSize(512); Serial1.begin(ESP_LINK_BAUD);
#else
    EspLinkIO.begin(ESP_LINK_BAUD);   // GP12 TX / GP13 RX soft-UART
#endif

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    Serial.println("=== PiSat RTK ROVER (RP2350A) ===");
    Serial.printf("VBAT: %.2f V\n", Power::batteryVolts());
    Serial.println("Path/boundary arrive over WiFi from the planner/visualiser.");
    Serial.println("Type 'help' for commands, 'hold' to brake/release motors.");
    Hmi::beep(60);
}

void loop() {
    Gnss::poll();
    pumpLink();             // base $PSDG corrections + path/boundary + WASD manual
    applyCorrection();      // raw fix - base error -> g_lat / g_lon

    static char line[64]; static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (n) { line[n] = 0; handleCommand(line); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = c;
    }

    // capture course/speed and set the local-frame origin on first fix
    const GnssFix& f = Gnss::fix();
    if (Gnss::hasFix()) {
        currentCourse = f.course_deg;
        currentSpeed  = f.speed_mps;
        if (!originSet) setOrigin(g_lat, g_lon);   // corrected position
    }
    if (!hadFix && Gnss::hasFix()) { hadFix = true; Serial.println("GPS fix acquired"); Hmi::beep(120); }

    updateWheelSpeeds();

    static uint32_t tLast = 0, tHdr = 0;
    if (millis() - tLast >= 100) {          // 10 Hz control + telemetry
        tLast = millis();
        g_vbat = Power::batteryVolts();

        driveControl();     // hold > manual (WASD) > autonomous path (PID)

        if (millis() - tHdr >= 2000) { tHdr = millis(); sendPathHeader(); }
        sendTelemetry();

        if (Gnss::hasFix()) Hmi::ledA(true); else Hmi::blink(PIN_LED_A, 500);
        Hmi::ledB(!g_hold);
        // NOTE: no vbat cutoff -- the VBAT ADC floats (0-5V without a battery),
        // so it is telemetry-only and never gates the motors.
    }
}

void setup1() {}
void loop1() {}

#endif
