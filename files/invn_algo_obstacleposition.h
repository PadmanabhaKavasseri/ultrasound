// SPDX-License-Identifier: Apache-2.0
/*
	Copyright (C) 2020 InvenSense Corporation, All Rights Reserved.
*/

#ifndef _INVN_ALGO_OBSTACLEPOSITION_H_
#define _INVN_ALGO_OBSTACLEPOSITION_H_


#include "invn/common/invn_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INVN_OBSTACLE_POSITION_DATA_STRUCTURE_SIZE_ERROR		(127)
#define INVN_OBSTACLE_POSITION_PROCESS_SUCCESS					(0)
#define INVN_OBSTACLE_POSITION_PROCESS_NOT_READY				(1)
#define INVN_OBSTACLE_POSITION_PROCESS_ERROR					(2)

/** \defgroup InvnAlgoObstaclePosition InvnAlgoObstaclePosition
 *  \brief Range finder algorithm that estimates the position (X,Y) of object using ultrasound sensor. Algorithm input is raw ultrasound echo (IQ data)
 */


#define NB_SAMPLE_ECHO 130

#define MAX_OBJECT 3
#define NB_SENSOR 4


#define	INVN_CH_MASK 1
#define	INVN_QUAT_AG_MASK 2

#define INVN_OBSTACLE_POSITION_DATA_STRUCTURE_SIZE 2600//1350

/*! \struct InvnObstaclePositionInput
 * InvnAlgoObstaclePosition input structure (raw data) \ingroup InvnAlgoObstaclePosition
 */
typedef struct
{
	uint64_t time;				/*!< time in us */
	float qAG[4];				/*!< quaternion AG from IMU - Not used yet */
	uint16_t nbr_samples;		/*!< number of magnitude samples, e.g. nbr_samples = length(iq_buffer)/2 */
	int16_t *iq_buffer;	        /*!< buffer to iq data,  iq_buffer[2*i] = I[i], iq_buffer[2*i+1] = Q[i], */
	uint16_t nbr_samples_skip;  /*!< Number of magnitude samples skipped from original IQ data from sensor */
	uint8_t	sensor_ID_Tx;		/*!< index of transmitting sensor from 0 to 3 */
	uint8_t	sensor_ID_Rx;		/*!< index of receiving sensor */
	uint8_t mask;				/*!< sensor updated flag, 1 for ultrasound, 2 for IMU  */
} InvnAlgoObstaclePositionInput;


/*! \struct InvnObstaclePositionOutput
 * InvnAlgoObstaclePosition output structure \ingroup InvnAlgoObstaclePosition
 */
typedef struct
{
	uint64_t time;								/*!< time in us */
	int16_t output_position[MAX_OBJECT * 3];	/*!< vector of X,Y,Z position of each possible object detected by the algorithm, in the frame defined by sensor_position. If no object X=Y=Z=0.   */
} InvnAlgoObstaclePositionOutput;


/*! \struct InvnAlgoObstaclePositionConfig
 * InvnAlgoObstaclePosition configuration structure (algo and data related settings) \ingroup InvnAlgoObstaclePosition
 */
typedef struct
{
	/*!< Sensor configuration - link to firmware and sensor localization on target */
	//float range_to_mm[NB_SENSOR];			/*!< range conversion factor for each sensor in mm, default 7.7mm (CH101) */
	uint32_t sensor_FOP[NB_SENSOR];         /*!< sensor FOP in Hz */
	uint32_t pre_trigger_time;	      		/*!< length of the rx_pre_trigger in µs. Set 0 if pre_trigger is disabled, typical value for CH101 is 600 µs. */
	uint16_t pulse_length;                  /*!< pulse length in number of cycles, default ch101 is 30 cycles */
	int16_t sensor_position[NB_SENSOR][3];	/*!< sensor coordinates (X,Y,Z) in the robot frame (in mm). These coordinates determine the position of detected targets*/
	/*!< Algorithm configuration - parameters */
	uint16_t group_distance_criteria;		/*!< group distance criteria, in mm, should be set regarding the distance between sensor and FOV, default 200 */
	uint16_t min_scaling_factor;			/*!< minimal accepted scaling factor for adaptative threshold function, default 68 fxp value, scaling_factor_fxp = (scaling_factor_flt*175) + 50 */
	uint16_t max_scaling_factor;			/*!< maximal accepted scaling factor for adaptative threshold function, default 400 fxp value, scaling_factor_fxp = (scaling_factor_flt*175) + 50  */
	uint16_t noise_amplitude; 				/*!< noise amplitude at 80cm, should be set depending on sensor/horn used, default 500 for CH101-PN653, 200 for CH101-PN702*/
	uint16_t peak_threshold_decay;			/*!< define the decay model of the find_peak threshold, default 4000 */
	uint8_t	ringdown_index;					/*!< index of end of ringdown, default 18 (CH101) */
} InvnAlgoObstaclePositionConfig;

/*!
 * \brief Get library version x.y.z-suffix
 * \return library version as string
 * \ingroup InvnAlgoObstaclePosition
 */
const char * invn_algo_obstacleposition_version(void);

/*!
 * \brief Initializes algorithms with config parameters and reset algorithm states.
 * \param[in,out] algo_data_structure memory allocated for algorithm internal state of size INVN_OBSTACLE_POSITION_DATA_STRUCTURE_SIZE bytes.
 * \param[in] algorithm config structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \ingroup InvnAlgoObstaclePosition
 */
int8_t invn_algo_obstacleposition_init(void *algo_data_structure, const InvnAlgoObstaclePositionConfig *config);


/*!
 * \brief Performs algorithm computation.
 * \param[in,out] algo_data_structure memory allocated for algorithm internal state of size INVN_OBSTACLE_POSITION_DATA_STRUCTURE_SIZE bytes.
 * \param[in] input algorithm input structure
 * \param[out] output algorithm output structure
 * \return process success indicator.
 * \retval INVN_OBSTACLE_POSITION_PROCESS_SUCCESS Success and new position processed
 * \retval INVN_OBSTACLE_POSITION_PROCESS_NOT_READY Data processed but position was not updated yet
 * \retval INVN_OBSTACLE_POSITION_PROCESS_ERROR The pair Tx/Rx was not recognized, no data was processed
 * \ingroup InvnAlgoObstaclePosition
 */
int8_t invn_algo_obstacleposition_process(void *algo_data_structure, const InvnAlgoObstaclePositionInput *input, InvnAlgoObstaclePositionOutput *output);

/*!
 * \brief Output config structure with default values. The config includes hardware and algorithm configuration.
 * \return success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup InvnAlgoObstaclePosition
 */
int8_t invn_algo_obstacleposition_generate_default_config(InvnAlgoObstaclePositionConfig *config);

#ifdef __cplusplus
}
#endif




#endif
