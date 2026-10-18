#include "gateway.h"

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    gateway_run();
}
