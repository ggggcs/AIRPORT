#ifndef CLAMP_CONTROL_H
#define CLAMP_CONTROL_H
#include "main.h"

/*
 * Closed-loop endpoint configuration, in millimetres.
 *
 * Change these four values and rebuild to ship a fixed installation.  Keep a
 * value at 0 until that endpoint has been measured; an axis with a zero
 * endpoint remains unavailable to GRIP/RELEASE after boot.  The network layer
 * can replace them at runtime through Clamp_SetStart/Clamp_SetEnd or the
 * CLAMP_SET_START/CLAMP_SET_END commands below.
 */
/* Verified against the sensor display: 2754 counts = 275.4 mm. */
#ifndef LR400_COUNTS_PER_MM
#define LR400_COUNTS_PER_MM 10U
#endif
#define CLAMP_AXIS1_START_MM 260.0f
#define CLAMP_AXIS1_END_MM   130.0f
#define CLAMP_AXIS2_START_MM 260.0f
#define CLAMP_AXIS2_END_MM   130.0f
#define CLAMP_DEFAULT_TOLERANCE_MM 0.5f
#define CLAMP_CLOSE_DIR 0U /* Verified: DIR=0 is the closing direction. */
#define CLAMP_MAX_PULSE_HZ 20000U /* Driver input limit, not a guaranteed motor speed. */
#define CLAMP_CRUISE_HZ 5000U /* Verified motion; 20kHz direct start did not move. */
#define CLAMP_MOVE_TIMEOUT_MS 180000U /* Long screw travel at measured speed. */

/* Internal positions and ClampCommand.value remain sensor counts to avoid loss
 * of precision. Application/network code should use the millimetre APIs below.
 * Call commands and getters from the main loop only (queue network commands). */
typedef enum { CLAMP_STOP, CLAMP_JOG, CLAMP_CALIBRATE, CLAMP_SET_START,
    CLAMP_SET_END, CLAMP_TEACH_START, CLAMP_TEACH_END, CLAMP_GRIP,
    CLAMP_RELEASE, CLAMP_CLEAR, CLAMP_TOLERANCE, CLAMP_UNTEACH } ClampCommandType;
typedef struct {
    ClampCommandType type;
    uint8_t axis; /* 1/2; 0 means both for STOP/GRIP/RELEASE/CLEAR only */
    uint32_t value, speed; /* CLAMP_JOG API: pulse count and Hz; console converts degrees. */
    int32_t sign;
} ClampCommand;
typedef struct {
    uint32_t distance, sample_tick, sequence, start, end, target, tolerance;
    uint32_t motion_tick, progress_tick, progress_distance;
    uint8_t valid, endpoints, calibrated, close_level;
    int8_t close_sign, expected_sign;
    uint8_t state; /* 0 idle, 1 jog, 2 positioning, 3 fault */
    const char *fault;
    uint8_t fault_axis; /* Source axis when a fault stops both motors. */
    uint32_t fault_distance, fault_target; /* Frozen trigger sample, counts. */
    uint32_t samples[5];
    uint8_t sample_count, sample_index;
} ClampAxis;

void Clamp_Init(void);
void Clamp_Task(void);
void Clamp_Sensor(uint8_t axis, uint32_t raw, uint8_t valid);
int Clamp_Execute(const ClampCommand *cmd); /* 1 accepted, 0 rejected */
/* Runtime API for Ethernet/ROS: absolute distances in mm, including decimals.
 * Return 1 on success, 0 on invalid values, unconfirmed scale or moving axis. */
int Clamp_SetStart(uint8_t axis, float mm);
int Clamp_SetEnd(uint8_t axis, float mm);
int Clamp_SetEndpoints(uint8_t axis, float start_mm, float end_mm);
int Clamp_MmToCounts(float mm, uint32_t *counts);
float Clamp_CountsToMm(uint32_t counts); /* NAN if scale is unconfirmed. */
const ClampAxis *Clamp_Get(uint8_t axis);
void Clamp_StopAll(const char *reason);
void Clamp_PulseIRQ(void);
#endif
