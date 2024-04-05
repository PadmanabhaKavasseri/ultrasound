// SPDX-License-Identifier: Apache-2.0
/*
	Copyright (C) 2020 InvenSense Corporation, All Rights Reserved.
*/

#ifndef _INVN_ALGO_RANGEFINDER_H_
#define _INVN_ALGO_RANGEFINDER_H_


#include "invn/common/invn_types.h"
#include <string.h>

#define INVN_RANGEFINDER_DATA_STRUCTURE_SIZE_ERROR    (150) /*! Error returned if allocated memory is small */
#define INVN_RANGEFINDER_TxRx_ERROR                   (1)   /*! Error returned if tx rx link change */
#define INVN_RANGEFINDER_DATA_STRUCTURE_SIZE          (228) /*! size of memory allocation required for algo state */
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
typedef struct
{
    uint64_t                    time;              /*!< time in us */
    int16_t                     *iq_buffer;        /*!< buffer to iq data,  iq_buffer[2*i] = I[i], iq_buffer[2*i+1] = Q[i], */
    uint16_t                    nbr_samples;       /*!< Number of magnitude samples, e.g. nbr_samples = length(iq_buffer)/2 */
    uint16_t                    nbr_samples_skip;  /*!< Number of magnitude samples skipped from original IQ data from sensor */
    uint8_t                     Tx;                /*!< Index of transmitting sensor */
    uint8_t                     Rx;                /*!< Index of receiving sensor */

} InvnAlgoRangeFinderInput;


/*! \struct InvnRangeFinderOutput
 * InvnAlgoRangeFinder output structure \ingroup InvnAlgoRangeFinder
 */
typedef struct
{
    uint64_t                    time;                 /*!< time in us */
    uint32_t                    distance_to_object;   /*!< distance to object in mm */
    uint32_t                    magnitude_of_echo;    /*!< magnitude of the echo of the object detected*/
    uint16_t                    nbr_samples;          /*!< Number of magnitude samples, e.g. nbr_samples = length(iq_buffer)/2 */
    uint8_t                     range_status;         /*!< object detected in the FOV, flag 0: no object, 1: object detected , 2: object predicted only */
    uint8_t                     Tx;                   /*!< Index of transmitting sensor */
    uint8_t                     Rx;                   /*!< Index of receiving sensor */
    uint8_t                     update_data;          /*!< Flag that indicated the algorithm used input data */

} InvnAlgoRangeFinderOutput;


/*! \struct InvnAlgoRangeFinderConfig
 * InvnAlgoRangeFinder configuration structure (algo and data related settings) \ingroup InvnAlgoRangeFinder
 */
typedef struct
{
    uint32_t                    sensor_FOP;                 /*!< sensor FOP in Hz */
    uint32_t                    min_scaling_factor;         /*!< min scaling factor, default value 137 fxp value, scaling_factor_fxp = (scaling_factor_flt*175) + 50 */
    uint32_t                    max_scaling_factor;         /*!< max scaling factor, default value 400 fxp value, scaling_factor_fxp = (scaling_factor_flt*175) + 50 */
    uint32_t                    noise_amplitude;            /*!< noise amplitude at 80cm, should be set depending on the sensor/horn used, default value 500 for ch101-PN653, 200 for CH101-PN702 */
    uint16_t                    pre_trigger_time;           /*!< pre-trigger time in us, only when Tx != Rx, set 0 else */
    uint16_t                    inter_sensor_distance_mm;   /*!< specific to couple of Rx/Tx different, distance between the center of sensors in mm*/
    uint16_t                    pulse_length;               /*!< pulse length in number of cycles, default ch101 is 30 cycles */
    uint8_t                     predict_distance_len;       /*!< nb of iteration where distance can be extrapolated if object is not detected in the echo, default = 3 */
} InvnAlgoRangeFinderConfig;


/*!
 * \brief Get library version x.y.z-suffix
 * \return library version as string
 * \ingroup InvnAlgoRangeFinder
 */
const char * invn_algo_rangefinder_version(void);
/*!
 * \brief Initializes algorithms with config parameters and reset algorithm states.
 * \algo_data_structure[in] memory allocated for algorithm internal state of size INVN_RANGEFINDER_DATA_STRUCTURE_SIZE bytes. Pointer sould be aligned with platform requirements (e.g. 32 bit).
 * \config[in] algo init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup InvnAlgoRangeFinder
 */
int8_t invn_algo_rangefinder_init(void * algo_data_structure, const InvnAlgoRangeFinderConfig *config);


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

/*!
 * \brief Performs convertion from time to number of sample
 * \param[in]  input time_us: time in us that need to be converted to number of sample
 * \param[in]  sensor_fop: the operating frequency of the sensor
 * \retval converted number of samples
 * \ingroup InvnAlgoRangeFinder
 */
uint16_t invn_algo_rangefinder_convert_time_to_nbr_samples( uint32_t time_us, uint32_t sensor_fop);


/*!
 * \brief Performs convertion from distance to number of sample
 * \param[in]  distance in mm: distance in mm that need to be converted to number of sample
 * \param[in]  sensor_fop: the operating frequency of the sensor
 * \retval converted number of samples
 * \ingroup InvnAlgoRangeFinder
 */
uint16_t invn_algo_rangefinder_convert_distance_to_nbr_samples( uint32_t distance_mm, uint32_t sensor_fop);

#ifdef __cplusplus
}
#endif




#endif
