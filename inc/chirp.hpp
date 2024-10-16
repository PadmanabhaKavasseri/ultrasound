#ifndef CHIRP_HPP
#define CHIRP_HPP

//C includes
#include <poll.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


#include <vector>
#include "invn_algo_rangefinder.h"

long long orig_buffer[32];
int sensor_connected[] = {0, 0, 0, 0, 0, 0};
uint32_t op_freq[] = {0, 0, 0, 0, 0, 0};
static InvnAlgoRangeFinderInput inputs;
static InvnAlgoRangeFinderOutput outputs;
static InvnAlgoRangeFinderConfig algo_config;
/*! \struct InvnRangeFinder
 * InvnRangeFinder data structure that store internal algorithm state.
 * The struct below shows one method to align the data buffer pointer
 * to 32 bit for 32bit MCU.
 * Other methods can be used to align memory using malloc or
 * attribute((aligned, 4))
 */
union InvnRangeFinder {
        uint8_t data[INVN_RANGEFINDER_DATA_STRUCTURE_SIZE];
        uint32_t data32;
};


static union InvnRangeFinder algo;
#define CHIRP_NAME              "ch101"
#define IIO_DIR         "/sys/bus/iio/devices/"
#define FIRMWARE_PATH           "/home/chirp_cpp/firmware/"
#define MAX_SYSFS_NAME_LEN      (200)
#define MAX_NUM_SAMPLES     450
#define CH101_DEFAULT_FW       2
#define CH201_DEFAULT_FW       5
#define VER_MAJOR (0)
#define VER_MINOR (3)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
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
FILE *fp;
FILE *log_fp;
int dur, freq;
//is updated by splitfunc1 and used in main
int scan_bytes;
int num_sensors;
char file_name[100];

class Chirp {
public:
    std::vector<int> getDataOnce();
    int init();
private:
    std::string dev_path_ = "/dev/iio:device0";
    std::string sysfs_path_ = "/sys/bus/iio/devices/iio:device0";


};


#endif // CHIRP_HPP