// SPDX-License-Identifier: Apache-2.0
/*
	Copyright (C) 2020 InvenSense Corporation, All Rights Reserved.
*/

#ifndef _INVN_ALGO_FLOOR_TYPE_FXP_H_
#define _INVN_ALGO_FLOOR_TYPE_FXP_H_


#include "invn/common/invn_types.h"

#define INVN_FLOOR_TYPE_DATA_STRUCTURE_SIZE                     (712)      /*!< Algo memory size need to store internal state  */
#define INVN_FLOORTYPE_FXP_INPUT_TYPE_IQ_DATA                     (1)      /*!< bit mask to indicate raw input type is IQ data  */
#define INVN_FLOORTYPE_FXP_INPUT_TYPE_MAGNITUDE_DATA           (1<<1)      /*!< bit mask to indicate raw input type is magnitude data  */

#define INVN_FLOOR_TYPE_FXP_ENABLE_RANGE_MASK                     (1)      /*!< bit field mask to enable/disable range output  */
#define INVN_FLOOR_TYPE_FXP_ENABLE_METRIC_RGDWNCOMP_MASK       (1<<1)      /*!< bit field mask to enable/disable metric magnitude
                                                                                with ringdown compensation */
#define INVN_FLOOR_TYPE_FXP_ENABLE_ALL                        (  INVN_FLOOR_TYPE_FXP_ENABLE_RANGE_MASK \
                                                               | INVN_FLOOR_TYPE_FXP_ENABLE_METRIC_RGDWNCOMP_MASK)      /*!< bit field mask to enable/disable all outputs */


#define INVN_ALGO_FLOORTYPE_SPPED_OF_SOUND_MM_PER_SEC	     (343000)      /*!< Typical speed of sound in mm/sec at 25degC */
#define INVN_ALGO_FLOORTYPE_TYPICAL_OPERATION_FREQUENCY	     (178000)      /*!< Typical CH101 operation frequency in Hz */
#define INVN_ALGO_FLOORTYPE_INVALID_RANGE_VALUE	                 (-1)      /*!< Range value when no target detected */

#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_SUCCESS                                (0)      /*!< Return value succes for auto configuration */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_BELOW_FLOOR_DISTANCE_MIN       (1)      /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_ABOVE_FLOOR_DISTANCE_MAX       (1<<1)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_BELOW_DECIMATION_MIN           (1<<2)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_ABOVE_DECIMATION_MAX           (1<<3)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_DATA_START_IDX_HIGH            (1<<4)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_LOW_RINGDOWN_SAMPLE_RESOLUTION (1<<5)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_LOW_FLOOR_SAMPLE_RESOLUTION    (1<<6)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_BELOW_FOP_MIN                  (1<<7)   /*!< Return bit field warning of generated configuration  */
#define INVN_FLOORTYPE_FXP_RETURN_CONFIG_WARNING_ABOVE_FOP_MAX                  (1<<8)   /*!< Return bit field warning of generated configuration  */


#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup InvnAlgoFloorType InvnAlgoFloorType
 *  \brief floor type algorithm that estimates the soft/hard floor type. Algorithm input is raw ultrasound echo.
 *         The application should allocate memory for algorithm data_structure of size INVN_FLOOR_TYPE_DATA_STRUCTURE_SIZE.
 *         All API are reentrant to enable multiple instance of the algorithm. Every instance requires a separate data_structure memory allocation.
 *  \warning no warning
 */

/*! \struct InvnAlgoFloorTypeFxp_IQData
* Input data buffer supporting either IQ data pointer or magnitude data pointer \ingroup InvnAlgoFloorType
*/
typedef union {
    int16_t  *iq;       /*!< buffer pointer to iq data,  iq[2*i] = I[i], iq[2*i+1] = Q[i]. */
    uint16_t *magn;	    /*!< buffer pointer to magnitude data,  magn[i] = sqrt(I[i]*I[i] + Q[i]*Q[i]). */
} InvnAlgoFloorTypeFxp_IQData;

/*! \struct InvnAlgoFloorTypeFxpInput
 * InvnAlgoFloorType input structure (raw data) \ingroup InvnAlgoFloorType
 */
typedef struct
{
    uint64_t  time;                         /*!< Time in us */
    InvnAlgoFloorTypeFxp_IQData  buffer;    /*!< Buffer pointer to iq data or magnitude data */
    uint16_t  nbr_samples;                  /*!< Number of magnitude samples, e.g. nbr_samples = length(iq_buffer)/2 or nbr_samples = length(magn_buffer) */
    uint16_t mask;                          /*!< Bit field mask to specify updated input data type, e.g IQ data or magnitude data */
} InvnAlgoFloorTypeFxpInput;

/*! \struct InvnFloorTypeOutput
 * InvnAlgoFloorType output structure
 * \ingroup InvnAlgoFloorType
 */
typedef struct
{
	uint64_t time;      /*!< time in us */
	int32_t metric;     /*!< floortype metic value */
	int32_t threshold;  /*!< decision threshold */
	int16_t range;      /*!< floor range in mm (-1 if no target detected) */
	uint16_t range_amp; /*!< floor echo magnitude */
	uint8_t floor_type; /*!< floortype class id, 0 soft, 1 hard */
	uint8_t range_status; /*!< range status, 0 no target detected, 1 target detected, 2 target predicted */
} InvnAlgoFloorTypeFxpOutput;


/*! \struct InvnAlgoFloorTypeConfig
 * InvnAlgoFloorType configuration structure (algo and data related settings).
 * Call invn_algo_floor_type_fxp_generate_default_config to fill up structure with default recommended values.
 * Values can be adjusted according to sensor mounting, and raw data read (start and decimation).
 * \ingroup InvnAlgoFloorType
 */
typedef struct
{
    uint32_t range_resolution_q15;   /*!< Range resolution in mm/IQ sample in q15 (typical = 2mm/sample * pow(2, 15) )
                                         e.g. resolution = ~2mm/IQ sample when decimation = 1, speed of sound = 343m/s, fop = 178000 Hz */
    int16_t range_offset_mm;         /*!< Distance offset added to range related to compensate for pulse duration and
                                         index of start reading IQ or magnitude data.*/
    uint16_t floor_start_idx;        /*!< Start index of floor reflection window  */
    uint16_t floor_len;              /*!< Size in samples of floor reflection window, includes 1st floor echo  */
    uint16_t ringdown_start_idx;     /*!< Start index of window before floor reflection */
    uint16_t ringdown_len;           /*!< Size in samples of window before floor reflection window  */

    uint16_t len_window;             /*!< Size in samples of floor reflection window, includes all floor echos (1st, 2nd, ...) */

    int16_t threshold;               /*!< Decision threshold applied to metric output */
    int16_t threshold_hyst;          /*!< Decision threshold hysteresis */
    uint8_t cordic_iteration;        /*!< Cordic number of iterartion, is used when input iq_buffer is provided */

	uint16_t enable_mask;            /*!< Bit fielf mask to enable/disable features, e.g.
                                          enable range output: enable_mask |= INVN_FLOOR_TYPE_FXP_ENABLE_RANGE_MASK
                                          disable range output: enable_mask &= ~INVN_FLOOR_TYPE_FXP_ENABLE_RANGE_MASK */

    uint8_t metric_forgetting_factor_log2; /*!< low pass filter forgetting factor of output metric in log2, e.g. 0-> no filtering, default: 4-> (0.0625)*/
} InvnAlgoFloorTypeFxpConfig;

/*!
 * \brief Get library version x.y.z-suffix
 * \return library version as string
 * \ingroup InvnAlgoFloorType
 */
const char * invn_algo_floor_type_fxp_version(void);

/*!
 * \brief Fill config structure fields with default algorithm configuration.
 * \param[in] floor_distance_mm : shortest distance between sensor and floor in [mm]. Note, sensor FOV should be empty (void) below this range.
 * \param[in] data_start_read_idx : Start reading index of raw IQ data or Magnitude data. Recommended is to start reading from 8th sample.
 * \param[in] data_decimation : Decimation factor applied to raw IQ data or Magnitude data. Decimation=1 gives highest resolution, but increase data stream time.
 * \param[in] fop_hz : Sensor operation frequency (typical value for CH101 module is 178000Hz).
 * \param[out] config : algo configuration structure pointer.
 * \retval 0 Success
 * \retval (1<<0) Floor distance is below minimum supported value
 * \retval (1<<1) Floor distance is beyond maximum supported value
 * \retval (1<<2) Decimation value is below minimum value = 1
 * \retval (1<<3) Decimation value is above maximum value = 4
 * \retval (1<<4) Not enough ringdown sample resolustion, start index is high
 * \retval (1<<5) Not enough ringdown sample resolustion, decimation value is large
 * \retval (1<<6) Not enough floor distance or high decimation value for ringdown window
 * \retval (1<<7) fop_hz below lower bound of operation frequency
 * \retval (1<<8) fop_hz above upper bound of operation frequency
 * \ingroup InvnAlgoFloorType
 */
int16_t invn_algo_floor_type_fxp_generate_default_config(  const uint16_t floor_distance_mm
                                                         , const uint16_t data_start_read_idx
                                                         , const uint16_t data_decimation
                                                         , const uint32_t fop_hz
                                                         , InvnAlgoFloorTypeFxpConfig *config);

/*!
 * \brief Initializes algorithms with config parameters and reset algorithm states.
 * \algo_data_structure[out] memory allocated for algorithm internal state of size INVN_FLOOR_TYPE_DATA_STRUCTURE_SIZE bytes.
 * \config[in] algo init parameters structure. This structure should be filled before calling the init.
 * \retval 0 Success
 * \ingroup InvnAlgoFloorType
 */
int8_t invn_algo_floor_type_fxp_init(void *algo_data_structure, const InvnAlgoFloorTypeFxpConfig *config);


/*!
 * \brief Performs algorithm computation.
 * \algo_data_structure[out] memory allocated for algorithm internal state of size INVN_FLOOR_TYPE_DATA_STRUCTURE_SIZE bytes.
 * \param[in]  input : algorithm input
 * \param[out] output : algorithm output
 * \retval 0 Success
 * \ingroup InvnAlgoFloorType
 */
int8_t invn_algo_floor_type_fxp_process(void *algo_data_structure, const InvnAlgoFloorTypeFxpInput *input, InvnAlgoFloorTypeFxpOutput *output);

#ifdef __cplusplus
}
#endif




#endif
