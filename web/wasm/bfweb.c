// SPDX-License-Identifier: GPL-3.0-or-later
// drone-duels: Betaflight in the page. The harness of cudaflight's SITL_LOCKSTEP target built
// to WebAssembly (scripts/setup-web.sh copies this file into the checkout's
// src/platform/SIMULATOR/; scripts/cudaflight-web.patch builds it in place of the golden-trace
// harness). The page's worker (web/src/sim/firmware.ts) is the other side of every function
// here.
//
// One module instance is one flight controller, on the virtual clock of sitl_lockstep.c: it
// runs only inside bfw_step. Sensors are set by direct call, as stock SITL's updateState()
// sets them from a simulator's packet (sitl.c, the Gazebo bridge's conventions, which the
// game's native bridge, drone_duels.bridges.betaflight, is held to): the same scaling, the
// same synthetic magnetic field, the same barometer. The eeprom is in memory: a fresh one
// boots on Betaflight's defaults, bfw_cli types settings in and saves them, and
// bfw_eeprom/bfw_eeprom_size let the page carry the image into the instance that flies (a
// setting takes effect at boot, as on a board).
//
// The UARTs are rings in memory: bfw_serial_write is what a peripheral sends the flight
// controller, bfw_serial_read what it has sent back. MSP and MSP DisplayPort run over them
// exactly as over stock SITL's TCP ports.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/printf_serial.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "drivers/compass/compass_virtual.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"

#include "fc/init.h"
#include "fc/runtime_config.h"

#include "io/gps_virtual.h"
#include "io/serial.h"

#include "sitl_lockstep.h"

#define BFW_EXPORT(name) __attribute__((used, export_name(#name)))

// The instancer's per-instance offset (sitl_lockstep_instance.c, not built here): one
// instance a module, so none.
uint64_t __bf_delta = 0;

// ===========================================================================
// UARTs
// ===========================================================================

#define BFW_RX_SIZE 4096    // to the flight controller
#define BFW_TX_SIZE 32768   // from it: a whole HD OSD screen is a few kilobytes

typedef struct {
    serialPort_t port;
    bool open;
    uint8_t rx[BFW_RX_SIZE];
    uint8_t tx[BFW_TX_SIZE];
} bfwPort_t;

static bfwPort_t ports[SERIAL_PORT_COUNT];

static uint32_t used(uint32_t head, uint32_t tail, uint32_t size)
{
    return head >= tail ? head - tail : size + head - tail;
}

static void bfwWrite(serialPort_t *instance, uint8_t ch)
{
    serialPort_t *p = instance;
    const uint32_t next = (p->txBufferHead + 1) % p->txBufferSize;
    if (next == p->txBufferTail) {
        return; // nobody reading: the oldest stays, as a full UART drops
    }
    p->txBuffer[p->txBufferHead] = ch;
    p->txBufferHead = next;
}

static uint32_t bfwRxWaiting(const serialPort_t *p)
{
    return used(p->rxBufferHead, p->rxBufferTail, p->rxBufferSize);
}

static uint32_t bfwTxFree(const serialPort_t *p)
{
    return p->txBufferSize - 1 - used(p->txBufferHead, p->txBufferTail, p->txBufferSize);
}

static uint8_t bfwRead(serialPort_t *p)
{
    const uint8_t ch = p->rxBuffer[p->rxBufferTail];
    p->rxBufferTail = (p->rxBufferTail + 1) % p->rxBufferSize;
    return ch;
}

static void bfwSetBaudRate(serialPort_t *p, uint32_t baudRate)
{
    p->baudRate = baudRate;
}

// The firmware's side is done with a byte once it is in the ring (a wire has no queue).
static bool bfwTxEmpty(const serialPort_t *p)
{
    UNUSED(p);
    return true;
}

static void bfwSetMode(serialPort_t *p, portMode_e mode)
{
    p->mode = mode;
}

static void bfwSetCtrlLineStateCb(serialPort_t *p, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(p);
    UNUSED(cb);
    UNUSED(context);
}

static void bfwSetBaudRateCb(serialPort_t *p, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(p);
    UNUSED(cb);
    UNUSED(context);
}

static void bfwWriteBuf(serialPort_t *p, const void *data, int count)
{
    const uint8_t *bytes = data;
    for (int i = 0; i < count; i++) {
        bfwWrite(p, bytes[i]);
    }
}

static void bfwBeginWrite(serialPort_t *p)
{
    UNUSED(p);
}

static void bfwEndWrite(serialPort_t *p)
{
    UNUSED(p);
}

static const struct serialPortVTable bfwVTable = {
    .serialWrite = bfwWrite,
    .serialTotalRxWaiting = bfwRxWaiting,
    .serialTotalTxFree = bfwTxFree,
    .serialRead = bfwRead,
    .serialSetBaudRate = bfwSetBaudRate,
    .isSerialTransmitBufferEmpty = bfwTxEmpty,
    .setMode = bfwSetMode,
    .setCtrlLineStateCb = bfwSetCtrlLineStateCb,
    .setBaudRateCb = bfwSetBaudRateCb,
    .writeBuf = bfwWriteBuf,
    .beginWrite = bfwBeginWrite,
    .endWrite = bfwEndWrite,
};

serialPort_t *serTcpOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    const int id = findSerialPortIndexByIdentifier(identifier);
    if (id < 0 || id >= (int)ARRAYLEN(ports)) {
        return NULL;
    }
    bfwPort_t *s = &ports[id];
    memset(&s->port, 0, sizeof(s->port));
    s->open = true;
    s->port.vTable = &bfwVTable;
    s->port.identifier = identifier;
    s->port.rxBuffer = s->rx;
    s->port.txBuffer = s->tx;
    s->port.rxBufferSize = BFW_RX_SIZE;
    s->port.txBufferSize = BFW_TX_SIZE;
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.baudRate = baudRate;
    s->port.mode = mode;
    s->port.options = options;
    return &s->port;
}

// ===========================================================================
// The page's side
// ===========================================================================

// Bytes and text cross here: the page writes what it sends before the call, and reads what
// a call returns after it.
#define BFW_IO_SIZE 65536
static uint8_t io[BFW_IO_SIZE];
static uint16_t rc[BFL_MAX_RC_CHANNELS];
static float motors[4];

BFW_EXPORT(bfw_io) uint8_t *bfw_io(void)
{
    return io;
}

BFW_EXPORT(bfw_io_size) uint32_t bfw_io_size(void)
{
    return BFW_IO_SIZE;
}

// Boot: the firmware's own init, on the eeprom as it is in memory (zeroed: its defaults).
BFW_EXPORT(bfw_boot) void bfw_boot(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    bflSetEepromPath(NULL);
    bflCliSetEcho(false);
#if SERIAL_PORT_COUNT > 0
    printfSerialInit();
#endif
    systemInit();
    initPhase1();
    initPhase2();
    initPhase3();
}

BFW_EXPORT(bfw_eeprom) uint8_t *bfw_eeprom(void)
{
    return bflEepromBuffer();
}

BFW_EXPORT(bfw_eeprom_size) uint32_t bfw_eeprom_size(void)
{
    return bflEepromSize();
}

static void cliLine(const char *line)
{
    // Only what went wrong: a clean line says nothing worth a log.
    if (strstr(line, "###ERROR") != NULL) {
        printf("[cli] %s\n", line);
    }
}

// Type the `len` bytes of CLI text in bfw_io in, through the CLI's own parser, and save.
// Returns how many lines it rejected; the first is then in bfw_io, NUL-ended.
BFW_EXPORT(bfw_cli) uint32_t bfw_cli(uint32_t len)
{
    if (len > BFW_IO_SIZE) {
        len = BFW_IO_SIZE;
    }
    uint32_t cliLen = 0;
    char *cli = bflCliSanitizeDump((const char *)io, len, &cliLen);
    if (!cli) {
        return 1;
    }
    bflCliSetSink(cliLine);
    bflCliExec(cli, cliLen);
    bflCliSetSink(NULL);
    free(cli);
    const uint32_t errors = bflCliErrorCount();
    if (errors) {
        snprintf((char *)io, BFW_IO_SIZE, "%s", bflCliFirstError());
    }
    return errors;
}

// The firmware on by `us` µs of its own clock.
BFW_EXPORT(bfw_step) void bfw_step(uint32_t us)
{
    bflStepUs(us);
}

BFW_EXPORT(bfw_micros) double bfw_micros(void)
{
    return (double)bflMicros();
}

#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)
#define RAD2DEG (180.0 / M_PI)

static int16_t counts(double v)
{
    return (int16_t)constrain(v, -32767, 32767);
}

// The inertial sensors, from the physics' true state in ArduPilot's frames (body forward,
// right, down): the rates p, q, r in rad/s and the specific force in m/s² (level and still:
// fz = -9.80665). Betaflight's virtual gyro and accelerometer read forward, left, up, as
// sitl.c's updateState leaves a Gazebo packet.
BFW_EXPORT(bfw_set_imu) void bfw_set_imu(double p, double q, double r, double fx, double fy, double fz)
{
    virtualAccSet(virtualAccDev, counts(fx * ACC_SCALE), counts(-fy * ACC_SCALE), counts(-fz * ACC_SCALE));
    virtualGyroSet(virtualGyroDev, counts(p * GYRO_SCALE * RAD2DEG), counts(-q * GYRO_SCALE * RAD2DEG),
                   counts(-r * GYRO_SCALE * RAD2DEG));
}

// The barometer and the compass, from the height above sea level (m) and the true attitude
// (body forward-right-down to north-east-down, w x y z): the standard atmosphere and the
// synthetic field of sitl.c (60° inclination, 2° east declination, 4096 counts).
BFW_EXPORT(bfw_set_air) void bfw_set_air(double alt, double qw, double qx, double qy, double qz)
{
    const int32_t pressure = (int32_t)(101325.0 * pow(1.0 - 2.25577e-5 * alt, 5.25588));
    virtualBaroSet(pressure, 2500);

    // Body forward-left-up to north-west-up: a half turn about x either side.
    const float w = qw, x = qx, y = -qy, z = -qz;
    static const float fieldN = 2046.8f;
    static const float fieldW = -71.5f;
    static const float fieldU = -3547.2f;
    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    virtualMagSet(lrintf(r00 * fieldN + r10 * fieldW + r20 * fieldU),
                  lrintf(r01 * fieldN + r11 * fieldW + r21 * fieldU),
                  lrintf(r02 * fieldN + r12 * fieldW + r22 * fieldU));
}

// The GPS: degrees, m above sea level, and the velocity north, east, down, m/s.
BFW_EXPORT(bfw_set_gps) void bfw_set_gps(double lat, double lon, double alt, double vn, double ve, double vd)
{
    const double speed = sqrt(vn * vn + ve * ve);
    const double speed3D = sqrt(vn * vn + ve * ve + vd * vd);
    double course = atan2(ve, vn) * RAD2DEG;
    if (course < 0.0) {
        course += 360.0;
    }
    setVirtualGPS(lat, lon, alt, speed, speed3D, course, vn, ve, vd);
}

// The receiver: the page writes up to 16 pulse widths (µs) at bfw_rc, then says how many.
BFW_EXPORT(bfw_rc) uint16_t *bfw_rc(void)
{
    return rc;
}

BFW_EXPORT(bfw_set_rc) void bfw_set_rc(uint32_t count)
{
    bflSetRc(rc, count > BFL_MAX_RC_CHANNELS ? BFL_MAX_RC_CHANNELS : count);
}

// The motors' outputs as the mixer wrote them, µs (1000 stopped, 2000 full), in Betaflight's
// own order (quad X: rear right, front right, rear left, front left).
BFW_EXPORT(bfw_motors) float *bfw_motors(void)
{
    bflGetMotorsPwm(motors, 4);
    return motors;
}

BFW_EXPORT(bfw_armed) uint32_t bfw_armed(void)
{
    return bflIsArmed() ? 1 : 0;
}

BFW_EXPORT(bfw_arming_disabled) uint32_t bfw_arming_disabled(void)
{
    return (uint32_t)getArmingDisableFlags();
}

// To UART `uart` (1 is UART1): the `len` bytes in bfw_io. Returns how many it took.
BFW_EXPORT(bfw_serial_write) uint32_t bfw_serial_write(uint32_t uart, uint32_t len)
{
    const int id = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1 + uart - 1);
    if (id < 0 || id >= (int)ARRAYLEN(ports) || !ports[id].open) {
        return 0;
    }
    serialPort_t *p = &ports[id].port;
    uint32_t n = 0;
    for (; n < len && n < BFW_IO_SIZE; n++) {
        if (p->rxCallback) {
            p->rxCallback(io[n], p->rxCallbackData);
            continue;
        }
        const uint32_t next = (p->rxBufferHead + 1) % p->rxBufferSize;
        if (next == p->rxBufferTail) {
            break;
        }
        p->rxBuffer[p->rxBufferHead] = io[n];
        p->rxBufferHead = next;
    }
    return n;
}

// From UART `uart`: what the firmware has sent since the last read, into bfw_io. Returns
// how many bytes.
BFW_EXPORT(bfw_serial_read) uint32_t bfw_serial_read(uint32_t uart)
{
    const int id = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1 + uart - 1);
    if (id < 0 || id >= (int)ARRAYLEN(ports) || !ports[id].open) {
        return 0;
    }
    serialPort_t *p = &ports[id].port;
    uint32_t n = 0;
    while (p->txBufferTail != p->txBufferHead && n < BFW_IO_SIZE) {
        io[n++] = p->txBuffer[p->txBufferTail];
        p->txBufferTail = (p->txBufferTail + 1) % p->txBufferSize;
    }
    return n;
}
