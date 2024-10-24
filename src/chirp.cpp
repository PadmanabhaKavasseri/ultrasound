#include "chirp.hpp"

#include <iostream>

std::string firmware_file_ = "sys/bus/iio/devices/iio:device0/misc_bin_dmp_firmware_vers";

static int init_algo(int samples, int16_t * iq_buffer)
{
  int res;
  // Nominal config
  algo_config.ringdown_index = 19;
  algo_config.maintain_distance = 3;
  algo_config.min_scaling_factor = 137;
  algo_config.max_scaling_factor = 400;
  algo_config.noise_amplitude = 500;
  algo_config.sizeData = samples;
  algo_config.range_to_mm = 770;
  inputs.nbr_samples = samples;
  inputs.iq_buffer = iq_buffer;
  res = invn_algo_rangefinder_init(&algo, &algo_config);
  return res;
}

int inv_load_dmp(char * dmp_path, int dmp_version, const char * dmp_firmware_path)
{
  int result = 0;
  int bytesWritten = 0;
  FILE *fp, *fp_read;
  int write_size, len;
  char firmware_file[200];

  char bin_file[200];
  char bank[256];
  const char * p;
  int fw_names_size = ARRAY_SIZE(fw_names);
  snprintf(firmware_file, 100, "%s/misc_bin_dmp_firmware_vers", dmp_path);
  fp = fopen(firmware_file, "wb");
  if (fp == NULL) {
    fclose(fp_read);
    return -EINVAL;
  }
  if (dmp_version != 0xff)
    p = static_cast<const char *>(fw_names[dmp_version]);
  else
    p = static_cast<const char *>(firmware_path);
  write_size = strlen(p);
  if (write_size > 30)
    write_size = 30;
  bytesWritten = fwrite(p, 1, write_size, fp);
  if (bytesWritten != write_size) {
    std::cout << "bytes written (" << bytesWritten << ") not match requested length (" << write_size
              << ")\n";
    result = -1;
  }
  fclose(fp);

  if (dmp_version != 0xff) {
    if (result == 0) {
      std::cout << "dmp firmware " << fw_names[dmp_version] << " written to " << firmware_file_
                << "\n";
    }
    if (dmp_version >= fw_names_size) {
      return -EINVAL;
    }
    snprintf(bin_file, 200, "%s%s", dmp_firmware_path, fw_names[dmp_version]);
  } else {
    if (result == 0) {
      std::cout << "dmp firmware " << firmware_path << " written to " << firmware_file_ << "\n";
    }

    snprintf(bin_file, 100, "%s%s", dmp_firmware_path, firmware_path);
  }
  fp_read = fopen(bin_file, "rb");
  std::cout << "Bin File: " << bin_file << "\n";
  if (fp_read == NULL) {
    std::cout << "dmp firmware file " << bin_file << " open fail\n";
    return -EINVAL;
  }
  fseek(fp_read, 0L, SEEK_END);
  len = ftell(fp_read);
  fseek(fp_read, 0L, SEEK_SET);
  snprintf(firmware_file, 100, "%s/misc_bin_dmp_firmware", dmp_path);
  fp = fopen(firmware_file, "wb");
  if (fp == NULL) {
    std::cout << "dmp sysfs file " << firmware_file_ << " open fail\n";
    fclose(fp_read);
    return -EINVAL;
  }
  while (len > 0) {
    if (len > 256)
      write_size = 256;
    else
      write_size = len;
    bytesWritten = fread(bank, 1, write_size, fp_read);
    bytesWritten = fwrite(bank, 1, write_size, fp);
    if (bytesWritten != write_size) {
      std::cout << "bytes written (" << bytesWritten << ") no match requested length ("
                << write_size << ")\n";
      result = -1;
    }
    len -= write_size;
  }
  fclose(fp);
  fclose(fp_read);
  return result;
}

int check_sensor_connection(void)
{
  int i;
  FILE * fp;
  char file_name[100];
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/in_positionrelative%d_raw", sysfs_path, i + 18);
    fp = fopen(file_name, "rt");
    if (fp == NULL) {
      std::cout << "error opening " << file_name << "\n";
      exit(0);
    }
    fscanf(fp, "%d", &op_freq[i]);
    fclose(fp);
    if (op_freq[i])
      sensor_connected[i] = 1;
    else
      sensor_connected[i] = 0;
    std::cout << sensor_connected[i] << ", " << op_freq[i] << "\n";
  }
  return 0;
}
/**
 * find_type_by_name() - function to match top level types by name
 * @name: top level type instance name
 * @type: the type of top level instance being sort
 *
 * Typical types this is used for are device and trigger.
 **/
int find_type_by_name(const char * name, const char * type)
{
  const struct dirent * ent;
  int number, numstrlen;
  FILE * nameFile;
  DIR * dp;
  char thisname[MAX_SYSFS_NAME_LEN];
  char filename[MAX_SYSFS_NAME_LEN];
  size_t filename_sz = 0;
  int ret = 0;
  int status = -1;
  dp = opendir(IIO_DIR);
  if (dp == NULL) {
    std::cout << "No industrialio devices available\n";
    return -EINVAL;
  }
  while (ent = readdir(dp), ent != NULL) {
    if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0 &&
        strlen(ent->d_name) > strlen(type) && strncmp(ent->d_name, type, strlen(type)) == 0) {
      std::cout << "name: " << ent->d_name << "\n";
      numstrlen = sscanf(ent->d_name + strlen(type), "%d", &number);
      filename_sz = strlen(IIO_DIR) + strlen(type) + numstrlen + 6;
      /* verify the next character is not a colon */
      if (strncmp(ent->d_name + strlen(type) + numstrlen, ":", 1) != 0) {
        snprintf(filename, filename_sz, "%s%s%d/name", IIO_DIR, type, number);
        nameFile = fopen(filename, "r");
        if (!nameFile)
          continue;
        ret = fscanf(nameFile, "%s", thisname);
        fclose(nameFile);
        if (ret == 1 && strcmp(name, thisname) == 0) {
          status = number;
          closedir(dp);
          return status;
        }
      }
    }
  }
}

static int process_sysfs_request(char * data)
{
  int dev_num = find_type_by_name(CHIRP_NAME, "iio:device");
  if (dev_num < 0)
    return -EINVAL;
  snprintf(data, 100, IIO_DIR "iio:device%d", dev_num);
  snprintf(dev_path, 100, "/dev/iio:device%d", dev_num);

  std::cout << "data: " << data << std::endl;
  std::cout << "dev path: " << dev_path << std::endl;
  return 0;
}

int switch_streaming(int on)
{
  int i;
  FILE * fp;
  char file_name[100];
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/scan_elements/in_proximity%d_en", sysfs_path, i);
    fp = fopen(file_name, "wt");
    if (fp == NULL) {
      printf("error opening %s\n", file_name);
      exit(0);
    } else {
    }
    fprintf(fp, "%d", sensor_connected[i] & on);
    fclose(fp);
  }
  // add 6 for distance
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/scan_elements/in_distance%d_en", sysfs_path, i + 6);
    fp = fopen(file_name, "wt");
    if (fp == NULL) {
      printf("error opening %s\n", file_name);
      exit(0);
    }
    fprintf(fp, "%d", sensor_connected[i] & on);
    fclose(fp);
  }
  // add 12 for intensity
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/scan_elements/in_intensity%d_en", sysfs_path, i + 12);
    fp = fopen(file_name, "wt");
    if (fp == NULL) {
      printf("error opening %s\n", file_name);
      exit(0);
    }
    fprintf(fp, "%d", sensor_connected[i] & on);
    fclose(fp);
  }
  // add 18 for intensity(rx/tx id)
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/scan_elements/in_positionrelative%d_en", sysfs_path, i + 18);
    fp = fopen(file_name, "wt");
    if (fp == NULL) {
      printf("error opening %s\n", file_name);
      exit(0);
    }
    fprintf(fp, "%d", sensor_connected[i] & on);
    fclose(fp);
  }
  snprintf(file_name, 100, "%s/scan_elements/in_timestamp_en", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening proximity\n");
    exit(0);
  }
  fprintf(fp, "%d", on);
  fclose(fp);
  snprintf(file_name, 100, "%s/buffer/length", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening length\n");
    exit(0);
  }
  fprintf(fp, "%d", 2000);
  fclose(fp);
  snprintf(file_name, 100, "%s/buffer/watermark", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening watermark\n");
    exit(0);
  }
  fprintf(fp, "%d", 1);
  fclose(fp);
  snprintf(file_name, 100, "%s/buffer/enable", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening master enable\n");
    exit(0);
  } else {
    printf("open master enable OK\n");
  }
  fprintf(fp, "%d", 1 & on);
  fclose(fp);
}
int16_t I[6][MAX_NUM_SAMPLES], Q[6][MAX_NUM_SAMPLES];
int16_t iq_buffer[MAX_NUM_SAMPLES * 2];
float sample_to_mm[6];
#define CH_SPEEDOFSOUND_MPS 343
unsigned short distance[6], amplitude[6];
char mode[6];
void print_header(int sample, int frequency, FILE * fp)
{
  int i, j;
  float pos;
  for (i = 0; i < 6; i++) {
    if (op_freq[i]) {
      sample_to_mm[i] = (sample * CH_SPEEDOFSOUND_MPS * 8 * 1000) / (op_freq[i] * 2);
    }
  }
  fprintf(fp, "# Chirp Microsystems redswallow Data Log\n");
  fprintf(fp, "# sample rate:, %d S/s\n", frequency * sample);
  fprintf(fp, "# Decimation factor:, 1\n");
  fprintf(fp, "# Content: iq\n");
  fprintf(fp, "# Sensors ID:, ");
  for (i = 0; i < 6; i++) {
    if (sensor_connected[i])
      fprintf(fp, "%d, ", i);
  }
  fprintf(fp, "\n# Sensors FOP:, ");
  for (i = 0; i < 6; i++) {
    if (sensor_connected[i])
      fprintf(fp, "%d, ", op_freq[i]);
  }
  fprintf(fp, "Hz\n");
  fprintf(fp, "# Sensors NB Samples:,");
  for (i = 0; i < 6; i++) {
    if (sensor_connected[i])
      fprintf(fp, "%d, ", sample);
  }
  fprintf(fp, "\n# Sensors NB First samples skipped:, 0, 0\n");
  for (j = 0; j < 6; j++) {
    if (sensor_connected[j]) {
      fprintf(fp, "# time [s],tx_id,rx_id, range [cm],intensity [a.u.], target_detected, ");
      pos = 0;
      for (i = 0; i < sample; i++) {
        pos += sample_to_mm[j];
        fprintf(fp, "i_data_%3.1f, ", pos / 1000.0);
      }
      pos = 0;
      for (i = 0; i < sample; i++) {
        pos += sample_to_mm[j];
        fprintf(fp, "q_data_%3.1f, ", pos / 1000.0);
      }
      fprintf(fp, "\n");
    }
  }
}
static void print_help(void)
{
  std::cout << "Usage:\n";
  std::cout << "-h: print this help\n";
  std::cout << "-d x: duration, unit in seconds. default: 10 seconds\n";
  std::cout << "-s x: number of samples. default: 40 samples\n";
  std::cout << "-f x: sampling frequency. Default: 5 Hz\n";
  std::cout << "-n: not loading firmware. Default will load firmware\n";
  std::cout << "-l string: output logging file name. Default: \"/usr/chirp.csv\"\n";
}
void log_data(int num_sensors, int sample, FILE * log_fp, long long last_timestamp)
{
  int dev_num, i, j;
  for (dev_num = 0; dev_num < num_sensors; dev_num++) {
    // only CH101 and only in RX_TX mode, we call algo to calculate
    if (((mode[dev_num] == 0x20) || (mode[dev_num] == 0x10)) && (sensor_connection[dev_num] < 3)) {
      for (i = 0; i < sample; i++) {
        iq_buffer[2 * i] = I[dev_num][i];
        iq_buffer[2 * i + 1] = Q[dev_num][i];
      }
      invn_algo_rangefinder_process(&algo, &inputs, &outputs);
      distance[dev_num] = outputs.distance_to_object;
      amplitude[dev_num] = outputs.magnitude_of_object;
    }
  }
  for (dev_num = 0; dev_num < num_sensors; dev_num++) {
    if (mode[dev_num] == 0) {
      std::cout << "mode 0 here\n";
      break;
    }
    fprintf(log_fp, "%f, ", last_timestamp / 1000000000.0);
    // TX_RX mode
    if (mode[dev_num] == 0x10) {
      fprintf(log_fp, "%d, ", sensor_connection[dev_num]);
      fprintf(log_fp, "%d, ", sensor_connection[dev_num]);
    }
    // RX only mode.
    if (mode[dev_num] == 0x20) {
      for (j = 0; j < num_sensors; j++) {
        if ((mode[j] == 0x10) && (sensor_connection[j] < 3)) {
          fprintf(log_fp, "%d, ", sensor_connection[j]);
        }
      }
      fprintf(log_fp, "%d, ", sensor_connection[dev_num]);
    }
    fprintf(log_fp, "%d, ", distance[dev_num] / 10);
    fprintf(log_fp, "%d, ", amplitude[dev_num]);
    if ((distance[dev_num] == 0xFFFF) || (distance[dev_num] == 0)) {
      fprintf(log_fp, "%d, ", 0);
    } else {
      fprintf(log_fp, "%d, ", 1);
    }
    if (sensor_connection[dev_num] < 3) {
      for (i = 0; i < sample; i++) {
        fprintf(log_fp, "%d, ", I[dev_num][i]);
      }
      for (i = 0; i < sample; i++) {
        fprintf(log_fp, "%d, ", Q[dev_num][i]);
      }
    }
    fprintf(log_fp, "\n");
  }
}

void Chirp::setCounter(int counter)
{
  // counter controls how many times it will run.
  snprintf(file_name, 100, "%s/calibbias", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening %s\n", file_name);
    exit(0);
  }
  fprintf(fp, "%d", counter);
  fclose(fp);
}

void Chirp::setFreq(int freq)
{
  snprintf(file_name, 100, "%s/sampling_frequency", sysfs_path);
  fp = fopen(file_name, "wt");
  if (fp == NULL) {
    printf("error opening %s\n", file_name);
    exit(0);
  }
  fprintf(fp, "%d", freq);
  fclose(fp);
}

int Chirp::init()
{
  int i;
  long long timestamp, last_timestamp;
  int counter;
  int index;
  int num_open_fds;
  char * log_file;
  int c;
  int fp_writes;
  printf("TDK-Robotics-RB5-chx01-app-%d.%d\n", VER_MAJOR, VER_MINOR);
  // get absolute IIO path & build MPU's sysfs paths
  if (process_sysfs_request(sysfs_path) < 0) {
    printf("Cannot find %s sysfs path\n", CHIRP_NAME);
    exit(0);
  }
  printf("%s sysfs path: %s, dev path=%s\n", CHIRP_NAME, sysfs_path, dev_path);
  dur = 1;
  freq = 1;
  log_file = "/usr/chirp.csv";
  if (inv_load_dmp(sysfs_path, CH101_DEFAULT_FW, FIRMWARE_PATH) != 0) {
    printf("CH101 firmware fail\n");
    return -EINVAL;
  }
  if (inv_load_dmp(sysfs_path, CH201_DEFAULT_FW, FIRMWARE_PATH) != 0) {
    printf("CH201 firmware fail\n");
    return -EINVAL;
  }

  index = 0;
  counter = freq * dur;
  int sample = 255;
  if (init_algo(sample, iq_buffer)) {
    printf("algo init error\n");
    exit(0);
  }
  for (i = 0; i < 6; i++) {
    snprintf(file_name, 100, "%s/in_positionrelative%d_raw", sysfs_path, i + 18);
    fp = fopen(file_name, "wt");
    if (fp == NULL) {
      printf("error opening %s\n", file_name);
      exit(0);
    }
    fprintf(fp, "%d", sample);
    fclose(fp);
  }
  check_sensor_connection();
  last_timestamp = 0;
  timestamp = 0;
  scan_bytes = 0;
  num_sensors = 0;
  for (i = 0; i < 6; i++) {
    scan_bytes += sensor_connected[i] * 32;
    num_sensors += sensor_connected[i];
  }
  index = 0;
  for (i = 0; i < 6; i++) {
    if (sensor_connected[i] == 1) {
      sensor_connection[index] = i;
      index++;
    }
  }
  log_fp = fopen(log_file, "wt");
  if (log_fp == NULL) {
    printf("error opening log file %s\n", log_file);
    exit(0);
  }
  print_header(sample, freq, log_fp);
  scan_bytes += 32;
  setCounter(1);
  setFreq(5);
}

std::vector<int> Chirp::getDataOnce()
{
  char * buffer;
  int total_bytes;
  struct pollfd pfds[1];
  long long timestamp, last_timestamp;
  int ready;
  int fp_writes;
  int nfds;
  int target_bytes;
  int bytes;
  int index = 0;
  int j;
  buffer = (char *)orig_buffer;
  total_bytes = 0;
  std::vector<int> res;
  res.emplace_back(num_sensors);
  switch_streaming(1);
  pfds[0].fd = open(dev_path, O_RDONLY);
  pfds[0].events = (POLLIN | POLLRDNORM);
  last_timestamp = 0;
  ready = 1;
  fp_writes = 1;
  int cnt = 1;
  while (ready == 1) {
    char * tmp;
    short value;
    char * ptr;
    int dev_num;
    pfds[0].revents = 0;
    nfds = 1;
    ready = poll(pfds, nfds, 5000);
    if (ready == -1)
      printf("poll error\n");

    if (pfds[0].revents & (POLLIN | POLLRDNORM)) {
      target_bytes = scan_bytes;
      bytes = read(pfds[0].fd, buffer, target_bytes);
      total_bytes += bytes;
      tmp = (char *)&timestamp;
      memcpy(tmp, &buffer[scan_bytes - 8], 8);
      target_bytes = scan_bytes - 32;
      int i = 0;
      if (last_timestamp == 0)
        last_timestamp = timestamp;
      if (last_timestamp != timestamp) {
        index -= 7;
        fp_writes++;
        log_data(num_sensors, 225 /*sample*/, log_fp, last_timestamp);
        index = 0;
        last_timestamp = timestamp;
      }
      for (j = 0; j < num_sensors; j++) {
        for (i = 0; i < 7; i++) {
          value = buffer[i * 4 + 1 + j * 28];
          value <<= 8;
          value += buffer[i * 4 + j * 28];
          I[j][i + index] = value;
          value = buffer[i * 4 + 3 + j * 28];
          value <<= 8;
          value += buffer[i * 4 + 2 + j * 28];
          Q[j][i + index] = value;
        }
      }
      
      index += 7;
      ptr = buffer + 28 * num_sensors;

     
      for (j = 0; j < num_sensors; j++) {
        distance[j] = ptr[1 + j * 2];
        distance[j] <<= 8;
        distance[j] += ptr[j * 2];
        res.emplace_back(distance[j]);
      }
      ptr += 2 * num_sensors;
      for (j = 0; j < num_sensors; j++) {
        amplitude[j] = ptr[1 + j * 2];
        amplitude[j] <<= 8;
        amplitude[j] += ptr[j * 2];
      }
      ptr += 2 * num_sensors;
      for (j = 0; j < num_sensors; j++)
        mode[j] = ptr[j];
    }
    setCounter(0);
  }
  return res;
}

void Chirp::stopData()
{
  setCounter(0);
  switch_streaming(0);
}

Chirp::Chirp()
{
  init();
}

int main()
{
  std::cout << "Started" << std::endl;
  auto chirp = Chirp();
  auto res = chirp.getDataOnce();
  int ns = res[0];
  for (size_t i = 1; i < res.size(); ++i) {
    int sensorIndex = (i - 1) % ns + 1;
    std::cout << "Sensor " << sensorIndex << ": " << res[i] << std::endl;
  }

  chirp.stopData();

  return 0;
}
