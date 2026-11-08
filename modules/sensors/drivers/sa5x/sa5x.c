#include "sensors/drivers/sa5x/sa5x.h"

size_t sa5x_build_telemetry_request(uint8_t *buf, size_t max)
{
    /* TODO(workshop) */
    (void)buf;
    (void)max;
    return 0;
}

size_t sa5x_build_discipline_command(bool enable, uint8_t *buf, size_t max)
{
    /* TODO(workshop) */
    (void)enable;
    (void)buf;
    (void)max;
    return 0;
}

bool sa5x_parse_telemetry(const char *line, size_t len, sa5x_telemetry_t *out)
{
    /* TODO(workshop) */
    (void)line;
    (void)len;
    (void)out;
    return false;
}
