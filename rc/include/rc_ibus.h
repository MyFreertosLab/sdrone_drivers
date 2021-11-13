/*
 * rc_ibus.h
 *
 *  Created on: 8 nov 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_RC_INCLUDE_RC_IBUS_H_
#define COMPONENTS_DRIVERS_RC_INCLUDE_RC_IBUS_H_

#include <esp_err.h>
#include <rc.h>

#define RC_PROTOCOL_IBUS

esp_err_t rc_ibus_init(rc_handle_t rc_handle);
esp_err_t rc_ibus_start(rc_handle_t rc_handle);
esp_err_t rc_ibus_stop(rc_handle_t rc_handle);



#endif /* COMPONENTS_DRIVERS_RC_INCLUDE_RC_IBUS_H_ */
