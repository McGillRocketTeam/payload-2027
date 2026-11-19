#ifndef SIM_SIM_LOG_H_
#define SIM_SIM_LOG_H_

/*
 * Run log: one CSV per test run with everything the simulator did, so a
 * failing or surprising run can be read back (or plotted) afterwards.
 *
 *   sim_log_open(".pio/sim/full_mission.csv");
 *   ... run ...
 *   sim_log_close();      also prints a per-source summary to stdout
 *
 * Columns:  t_s, source, kind, name, value, detail
 *   t_s     sim (true) time in seconds, 12 decimals (picoseconds)
 *   source  device or pin name ("csac1", "gps", "sd", ...)
 *   kind    what happened:
 *             fault    a transfer hit an injected fault (name = mode)
 *             inject   a fault was injected or cleared
 *             power    power on/off
 *             state    a device state change (lock, fix, status, ...)
 *             edge     a pin edge (name = rising/falling), if pin logging is on
 *             truth    ground truth, e.g. an oscillator's real time error
 *                      at its PPS edge (value in seconds)
 *             error    something the firmware did wrong or that was lost
 *                      (UART overrun, missed IMU samples, bad command, ...)
 *             cmd      a command a device received from the firmware
 *   value   number for that row (0 if none)
 *   detail  free text
 *
 * Logging is off until sim_log_open(); every call is cheap when it is off.
 */

#include <stdbool.h>
#include <stdint.h>

bool sim_log_open(const char *path);
void sim_log_close(void);
bool sim_log_is_open(void);

/* Pin edges are many (9 PPS sources = 18 rows/s); off by default. */
void sim_log_edges(bool enable);
bool sim_log_edges_enabled(void);

void sim_log(const char *source, const char *kind, const char *name, double value,
             const char *fmt, ...) __attribute__((format(printf, 5, 6)));

/* Count of "fault" / "error" rows per source, for asserts and the summary. */
uint32_t sim_log_count(const char *source, const char *kind);

#endif /* SIM_SIM_LOG_H_ */
