/*
 * moteurs.h
 *
 *  PWM control for ESC (TIM1_CH1, PA8) and steering servo (TIM1_CH4, PA11).
 *  Constants and math from CoVAPSy_moteurs.h (Anthony Juton, May 2023).
 */
#include "tim.h"
#include <stdint.h>

#ifndef INC_MOTEURS_H_
#define INC_MOTEURS_H_

//constantes propulsion
#define V_MAX_SOFT 2.0
#define V_MAX_HARD 8.0
#define PROP_REPOS 1500
#define PROP_POINT_MORT 1560
#define PROP_POINT_MORT_NEG 1440
#define PROP_MAX 1850

// Direction constants, calibrated (steering_calibration.py). Center 1630us.
// Left at lower us, right at higher us, so GAUCHE < DROITE keeps +degrees = left.
// Stops ~1300 left / ~1900 right; symmetric +-260us about center.
#define DIR_ANGLE_MAX 18.0
#define DIR_BUTEE_DROITE 1890
#define DIR_BUTEE_GAUCHE 1370
#define DIR_MILIEU 1630

void Propulsion_init(void);
void Direction_init(void);
void set_direction_degres(float angle_degre);
void set_vitesse_m_s(float vitesse_m_s);
void recule(void);

#endif /* INC_MOTEURS_H_ */
