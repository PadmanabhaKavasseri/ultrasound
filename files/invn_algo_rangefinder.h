/*
$License:
	Copyright (C) 2020 InvenSense Corporation, All Rights Reserved.
$
*/

#ifndef _INVN_ALGO_RANGEFINDER_H_
#define _INVN_ALGO_RANGEFINDER_H_


#include "invn/common/invn_types.h"
#include <string.h>

#define INVN_RANGEFINDER_DATA_STRUCTURE_SIZE_ERROR    (127) /*! Error returned if allocated memory is small */
#define INVN_RANGEFINDER_TxRx_ERROR                   (1)   /*! Error returned if tx rx link change */
#define INVN_RANGEFINDER_DATA_STRUCTURE_SIZE          (284) /*! size of memory allocation required for algo state */
#define INVN_RANGEFINDER_STATUS_NOT_DETECTED          (0)   /*! status value of range detection */
#define INVN_RANGEFINDER_STATUS_DETECTED              (1)   /*! status value of range detection */
#define INVN_RANGEFINDER_STATUS_PREDICTED             (2)   /*! status value of range detection */

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup InvnAlgoRangeFinder InvnAlgoRangeFinder
 *  \brief Range finder algorithm that estimates the distance of the closest object using ultrasound sensor. Algorithm input is raw ultrasound echo
 */

/*! \struct InvnRangeFinderInput
 * InvnAlgoRangeFinder input structure (raw data) \ingroup InvnAlgoRangeFinder
 */
typedef struct {

	uint64_t 					time;         /*!< time in us */
	int16_t  					*iq_buffer;   /*!< buffer to iq data,  iq_buffer[2*i] = I[i], iq_buffer[2*i+1] = Q[i], */
	uint16_t 					nbr_samples;  /*!< Number of magnitude samples, e.g. nbr_samples = length(iq_buffer)/2 */
	uint8_t	 					Tx;           /*!< Index of transmitting sensor */
	uint8_t	 					Rx;           /*!< Index of receiving sensor */

} InvnAlgoRangeFinderInput;


/*! \struct InvnRangeFinderOutput
 * InvnAlgoRangeFinder output structure \ingroup InvnAlgoRangeFinder
 */
typedef struct {
	uint64_t 					time;                /*!< time in us */
	uint32_t					distance_to_object;  /*!< distance to object in mm */
	uint32_t					magnitude_of_object; /*!< magnitude of the echo of the object detected*/
	uint8_t 					range_status;        /*!< object detected in the FOV, flag 0: no object, 1: object detected , 2: object predicted only */
	uint8_t  					Tx;	                 /*!< Index of transmitting sensor */
	uint8_t  					Rx;                  /*!< Index of receiving sensor */
	uint8_t  					update_data;         /*!< Flag that indicated the algorithm used input data */

} InvnAlgoRangeFinderOutput;


/*! \struct InvnAlgoRangeFinderConfig
 * InvnAlgoRangeFinder configuration structure (algo and data related settings) \ingroup InvnAlgoRangeFinder
 */
typedef struct {
	uint32_t					range_to_mm;                /*!< range conversion factor for each sensor in 1/100 mm, default 770 (CH101) */
	uint32_t					min_scaling_factor;         /*!< min scaling factor, default value 137 */
	uint32_t 					max_scaling_factor;         /*!< max scaling factor, default value 400 */
	uint32_t 					noise_amplitude;            /*!< noise amplitude, default value 500 */
	uint16_t 					sizeData;                   /*!< number of samples in i/q data per sensor, default 130 (CH101) */
	uint16_t					delta_pre_trigger_time;     /*!< delta pre-trigger time in us, corresponding to the delta time between the firmware pre-trigger time and the equivalent time of number of sample considered as pre-triggered */
	uint8_t						ringdown_index;             /*!< ringdown index of the sensor, default value 19 */
	uint8_t						maintain_distance;          /*!< nb of iteration where distance can be extrapolated if object is not detected in the echo, default = 3 */

} InvnAlgoRangeFinderConfig;


/*!
 * \brief Get library version x.y.z-suffix
 * \return library version as string
 * \ingroup InvnAlgoRangeFinder
 */
const char *invn_algo_rangefinder_version(void);
/*!
 * \brief Initializes algorithms with config parameters and reset algorithm states.
 * \algo_data_structure[in] memory allocated for algorithm internal state of size INVN_RANGEFINDER_DATA_STRUCTURE_SIZE bytes. Pointer sould be aligned with platform requirements (e.g. 32 bit).
 * \config[in] algo init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup InvnAlgoRangeFinder
 */
int8_t invn_algo_rangefinder_init(void *algo_data_structure, const InvnAlgoRangeFinderConfig *config);


/*!
 * \brief Performs algorithm computation.
 * \algo_data_structure[in] memory allocated for algorithm internal state of size INVN_RANGEFINDER_DATA_STRUCTURE_SIZE bytes.
 * \param[in]  input algorithm input structure
 * \param[out] output algorithm output structure
 * \retval 0 Success
 * \retval 1 error invalid Rx,Tx
 * \ingroup InvnAlgoRangeFinder
 */
int8_t invn_algo_rangefinder_process(void *algo_data_structure, const InvnAlgoRangeFinderInput *input, InvnAlgoRangeFinderOutput *output);

/*!
 * \brief Output config structure with default values. The config includes hardware and algorithm configuration.
 * \param[in,out]  algorithm config structure
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup InvnAlgoRangeFinder
 */
int8_t invn_algo_rangefinder_generate_default_config(InvnAlgoRangeFinderConfig *config);

#ifdef __cplusplus
}
#endif




#endif
