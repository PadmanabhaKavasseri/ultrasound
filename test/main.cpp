#include <iostream>

#include <poll.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include<errno.h>

#include "invn_algo_rangefinder.h"
#include "invn_algo_floor_type_fxp.h"
#include "invn_algo_cliff_detection.h"
#include "invn_algo_obstacleposition.h"

#define DEV_NUM_BOUNDARY 3
#define TX_RX_MODE   0x10
#define RX_ONLY_MODE   0x20

long long orig_buffer[260];

int sensor_connected[] = {0, 0, 0, 0, 0, 0};
uint32_t op_freq[] = {0, 0, 0, 0, 0, 0};
static unsigned do_cliff=0, do_floor_type=0, do_obstacle_detect=0, do_range_finder=0;


/*! \struct InvnRangeFinder
 * InvnRangeFinder data structure that store internal algorithm state.
 * The struct below shows one method to align the data buffer pointer to
 * 32 bit for 32bit MCU. Other methods can be used to align memory using
 * malloc or attribute((aligned, 4))
 */
union InvnRangeFinder {
	uint8_t data[INVN_RANGEFINDER_DATA_STRUCTURE_SIZE];
	uint32_t data32;
} InvnRangeFinder;

/*! \struct InvnFloorType
 * InvnFloorType data structure that store internal algorithm state.
 * The struct below shows one method to align the data buffer pointer to 32 bit
 * for 32bit MCU. Other methods can be used to align memory using malloc or
 * attribute((aligned, 4))
 */
union InvnFloorType {
	uint8_t data[INVN_FLOOR_TYPE_DATA_STRUCTURE_SIZE];
	uint32_t data32;
} InvnFloorType;

/*! \struct InvnCliffDetection
 * InvnCliffDetection data structure that store internal algorithm state.
 * The struct below shows one method to align the data buffer pointer to 32
 * bit for 32bit MCU. Other methods can be used to align memory using malloc
 * or attribute((aligned, 4))
 */
union InvnCliffDetection {
	uint8_t data[INVN_CLIFF_DETECTION_DATA_STRUCTURE_SIZE];
	uint32_t data32;
} InvnCliffDetection;

#define CHIRP_NAME		"ch101"
#define IIO_DIR		"/sys/bus/iio/devices/"
#define FIRMWARE_PATH		"/usr/share/tdk/"
#define MAX_SYSFS_NAME_LEN	(100)
#define MAX_NUM_SAMPLES     450
#define CH101_DEFAULT_FW       2
#define CH201_DEFAULT_FW       5

#define VER_MAJOR (0)
#define VER_MINOR (7)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MAX_CH_IIO_BUFFER 256

static const char  *const fw_names[] = {
"v39.hex",
"v39_IQ_debug-02.hex",
"ch101_gpr_rxopt_v41b.bin",
"ch101_gpr_rxopt_v41b-IQ_Debug.bin",
"ch201_old.bin",
"ch201_gprmt_v10a.bin",
"ch201_gprmt_v10a-IQ_Debug.bin",
};

static char *firmware_path;

static char sysfs_path[MAX_SYSFS_NAME_LEN] = {0};
static char dev_path[MAX_SYSFS_NAME_LEN] = {0};
static char sensor_connection[6];
char mode[6];
static uint16_t floor_distance_mm = 33;



extern "C" {
	    void check_sensor_connection(void);
		int inv_load_dmp(char *dmp_path, int dmp_version, const char *dmp_firmware_path);
		void print_help(void);
		int process_sysfs_request(char *data);	
}



int main(int argc, char *argv[]){
	std::cout << "Hello" << std::endl;
	FILE *fp;
	// FILE *log_fp;
	char *buffer;
	char file_name[100];
	int i;
	// int j;
	// int bytes;
	// int total_bytes;
	// int target_bytes;
	// int scan_bytes;
	// long long timestamp, last_timestamp;
	int counter;
	int num_sensors;
	int ready;
	int index;
	int nfds, num_open_fds;
	struct pollfd pfds[1];
	int dur, sample, freq;
	char *log_file;
	int c, fp_writes;
	int load_firmware_flag;
	unsigned int retry = 0;

	buffer = (char *)orig_buffer;
	printf("\n\nTDK-Robotics-RB5-chx01-app-%d.%d\n\n",
		VER_MAJOR, VER_MINOR);
	printf("RangeFinder version: %s\n", invn_algo_rangefinder_version());
	printf("Cliff detection version %s\n",
				invn_algo_cliff_detection_version());
	printf("Floor type detection %s\n", invn_algo_floor_type_fxp_version());
	printf("Obstacle position %s\n", invn_algo_obstacleposition_version());

	// get absolute IIO path & build MPU's sysfs paths
	if (process_sysfs_request(sysfs_path) < 0) {
		printf("Cannot find %s sysfs path\n", CHIRP_NAME);
		exit(0);
	}

	printf("%s sysfs path: %s, dev path=%s\n",
		CHIRP_NAME, sysfs_path, dev_path);

	dur = 10;
	sample = 80;
	freq = 5;
	log_file = "/usr/chirp.csv";
	load_firmware_flag = 1;

	opterr = 0;
	printf("options, log file=%s, frequency=%d, samples=%d, duration=%d seconds\n",
	log_file, freq, sample, dur);

	printf("firmware load=%d\n", load_firmware_flag);
	if (load_firmware_flag) {
		if (inv_load_dmp(sysfs_path,
			CH101_DEFAULT_FW, FIRMWARE_PATH) != 0) {
			printf("CH101 firmware fail\n");
			return -EINVAL;
		}

		if (inv_load_dmp(sysfs_path,
			CH201_DEFAULT_FW, FIRMWARE_PATH) != 0) {
			printf("CH201 firmware fail\n");
			return -EINVAL;
		}
	}

	index = 0;
	counter = freq*dur;
	for (i = 0; i < 6; i++) {
		snprintf(file_name, 100, "%s/in_positionrelative%d_raw",
			sysfs_path, i+18);
		fp = fopen(file_name, "w+");
		if (fp == NULL) {
			printf("error opening %s\n", file_name);
			exit(0);
		} else {
			printf("open %s OK, with %d\n", file_name, sample);
		}
		fprintf(fp, "%d", sample);
		fscanf(fp, "%d", &op_freq[i]);
		fclose(fp);
		if (op_freq[i])
			sensor_connected[i] = 1;
		else
			sensor_connected[i] = 0;
	}
	for (i = 3; i < 6; i++)
		printf("%d, %d\n", sensor_connected[i], op_freq[i]);

	for (i = 0; i < 3; i++)
		printf("%d, %d\n", sensor_connected[i], op_freq[i]);

	return 0;

}
