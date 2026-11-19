#include "car_logic.h"
#include "central_plan.h"
#include "lbr.h"

/*
 * Central Controller (ESP32-C5, CAN node 6): logger, time master, GPS, LoRa,
 * live links, cockpit link, and the car logic. Everything is the Central role
 * (lbrlib/components/lbr_central); this car adds its logic tasks (car/logic).
 */

LBR_CENTRAL_MAIN(&lbr_plan_central, .logic = &car_logic);
