// SPDX-License-Identifier: Apache-2.0
/*
	Copyright (C) 2020 InvenSense Corporation, All Rights Reserved.
*/

#ifndef _INVN_ALGO_CLIFF_DETECTION_H_
#define _INVN_ALGO_CLIFF_DETECTION_H_


#include "invn/common/invn_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INVN_CLIFF_DETECTION_DATA_STRUCTURE_SIZE      892

#define INVN_CLIFF_DETECTION_CLIFF_RESULT_FLOOR       0
#define INVN_CLIFF_DETECTION_CLIFF_RESULT_UNKNOWN     1
#define INVN_CLIFF_DETECTION_CLIFF_RESULT_CLIFF       2

#define INVN_CLIFF_DETECTION_FLOORTYPE_RESULT_SOFT    0
#define INVN_CLIFF_DETECTION_FLOORTYPE_RESULT_UNKNOWN 1
#define INVN_CLIFF_DETECTION_FLOORTYPE_RESULT_HARD    2

/** \defgroup InvnAlgoCliffDetection InvnAlgoCliffDetection
*   \brief cliff detection algorithm that detects floor distance change events based on a bi-sensor detector in pitch-catch mode.
*/


/*! \struct InvnCliffDetectionInput
 * InvnAlgoCliffDetection input structure (raw data) \ingroup InvnAlgoCliffDetection
*/
typedef struct
{
	uint64_t time;			/*!< time in us */
	int16_t *iq_buffer;		/*!< buffer to iq data,  iq_buffer[2*i] = I[i], iq_buffer[2*i+1] = Q[i], */
	uint16_t nbr_samples;	/*!< Number of range samples of the IQ data */
	int8_t Tx;				/*!< Index of transmitting sensor */
	int8_t Rx;				/*!< Index of receiving sensor */
} InvnAlgoCliffDetectionInput;


/*! \struct InvnCliffDetectionOutput
 * InvnAlgoCliffDetection output structure \ingroup InvnAlgoCliffDetection
 */
typedef struct
{
  uint64_t time;						/*!< time in us */
  uint32_t sum_floor_type_metric;		/*!< sum of floor type metric avalaible in floor_type_time_window */
  uint32_t floor_type_threshold;		/*!< threshold applied on the sum_floor_type_metric, Soft(0) if metric < threshold, Hard(2) otherwise  */
  uint16_t cliff_range_idx;				/*!< cliff height in range sample index. Value >= length of the IQ data length means the cliff height is beyond the measurable range*/
  int8_t Tx;							/*!< Index of transmitting sensor */
  int8_t Rx;							/*!< Index of receiving sensor */
  uint8_t cliff_detection;				/*!< detection result: floor (0), ambiguous (1) or cliff (2) */
  uint8_t floor_type;					/*!< Floor type: soft (0), ambiguous (1) or hard (2) */
} InvnAlgoCliffDetectionOutput;


/*! \struct InvnAlgoCliffDetectionConfig
 * InvnAlgoCliffDetection configuration structure (algo and data related settings) \ingroup InvnAlgoCliffDetection
 */
typedef struct
{
  int32_t thres_no_peak_classification;			/*!< no peak classification threshold: the tolerance of the std of the forward derivative at floor_location_min */
  int32_t threshold_soft_floor;					/*!< threshold on floor type detection metric, below which soft floor will likely be detected*/
  int32_t threshold_hard_floor;					/*!< threshold on floor type detection metric, above which hard floor will likely be detected*/
  int32_t soft_floor_amplitude_rampup;			/*!< threshold on soft floor amplitude ramp-up rate, below which the detection threshold will be adjusted for soft floor*/
  int32_t hard_floor_amplitude_rampup;			/*!< threshold on hard floor amplitude ramp-up rate, above which the detection threshold will be adjusted for hard floor*/
  uint16_t floor_location_min;					/*!< min peak location corresponding to the floor */
  uint16_t floor_location_max;					/*!< max peak location corresponding to the floor */
  uint16_t floor_amplitude_rampup_location;		/*!< range index where the amplitude of floor peak starts to ramp up */
  uint16_t threshold_peak_amplitude_min;		/*!< threshold of the minimum peak amplitude */
  uint16_t threshold_mag_long_range_cliff;		/*!< minimum peak amplitude to validate a long-range cliff*/
  uint16_t threshold_mag_short_range_cliff;		/*!< minimum peak amplitude to validate a short-range cliff*/
  uint8_t time_filter_length;					/*!< length of filter in time to smooth out the instantaneous detection result*/
  uint8_t floor_type_time_window;				/*!< length of floor type analysis time window*/
} InvnAlgoCliffDetectionConfig;

/*!
 * \brief Get library version x.y.z-suffix
 * \return library version as string
 * \ingroup InvnAlgoFloorType
 */
const char * invn_algo_cliff_detection_version(void);

/*!
 * \brief Fill config structure fields with default algorithm configuration. Some parameters are related to hardware setup and can be adjusted afterwards.
 * \param[in,out]  algorithm config structure
 * \retval 0 Success
 * \ingroup InvnAlgoCliffDetection
 */
int8_t invn_algo_cliff_detection_generate_default_config(InvnAlgoCliffDetectionConfig *config);

/*!
 * \brief Initializes algorithms with config parameters and reset algorithm states.
 * \algo_data_structure[out] memory allocated for algorithm internal state of size INVN_CLIFF_DETECTION_DATA_STRUCTURE_SIZE bytes.
 * \config[in] algo init parameters structure. This structure should be filled before calling the init.
 * \return initialization success indicator.
 * \retval 0 Success
 * \ingroup InvnAlgoCliffDetection
 */
int8_t invn_algo_cliff_detection_init(void *algo_data_structure, const InvnAlgoCliffDetectionConfig *config);


/*!
 * \brief Performs algorithm computation.
 * \algo_data_structure[out] memory allocated for algorithm internal state of size INVN_CLIFF_DETECTION_DATA_STRUCTURE_SIZE bytes.
 * \param[in]  input algorithm input structure
 * \param[out] output algorithm output structure
 * \retval 0 Success
* \ingroup InvnAlgoCliffDetection
 */
int8_t invn_algo_cliff_detection_process(void *algo_data_structure, const InvnAlgoCliffDetectionInput *input, InvnAlgoCliffDetectionOutput *output);

#ifdef __cplusplus
}
#endif




#endif
