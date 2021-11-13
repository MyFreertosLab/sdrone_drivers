/*
 * rc_ppm.h
 *
 *  Created on: 8 nov 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_RC_INCLUDE_RC_PPM_H_
#define COMPONENTS_DRIVERS_RC_INCLUDE_RC_PPM_H_

#include <esp_err.h>
#include <rc.h>

#define RC_PROTOCOL_PPM
esp_err_t rc_ppm_init(rc_handle_t rc_handle);
esp_err_t rc_ppm_start(rc_handle_t rc_handle);
esp_err_t rc_ppm_stop(rc_handle_t rc_handle);



#endif /* COMPONENTS_DRIVERS_RC_INCLUDE_RC_PPM_H_ */
