#include "vips.h"
#include <stdio.h>
#include<stdlib.h>
#include <storage.h>
#include <time.h>
#include <stdlib.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <dlog.h>
#include <assert.h>
#include <time.h>
#include <feedback.h>
#include <efl_util.h>
#include <sound_manager.h>
#include <bluetooth.h>
#include <sensor.h> //Library for the sensor usage
#include <math.h>

//interval in millisecond for accelerometer
#define af_interval 5// 2-->500HZ, 3-->333HZ, 4-->250HZ, 5-->200HZ 10-->100HZ
//interval in millisecond for gyroscope
#define gf_interval 5

#define ICON_DIR "/opt/usr/apps/org.nussmc.vips/res/images"

char buf[PATH_MAX];
int32_t *pSampleData;
char* source_state;

float hop_start = 0;
float hop_end = 100;

//ll to float conversion factor
#define LLF_CONVERSION 10000000,0

static float lastCheckConnectionTime = 0;

////////////////////////////for BLE

#define LE_INITIAL_BUF_SIZE 2

char *response = "0";
char *tick_response[] = { "1", "2", "3", "4", "5" };
char *lastResponse = "0";

//client handle
bt_gatt_client_h client = NULL;

//server handle
static bt_gatt_server_h gattServer = NULL;

//service handle
static bt_gatt_h gattSvc = NULL;
const char *service_uuid = "180A";
bt_gatt_service_type_e type = BT_GATT_SERVICE_TYPE_PRIMARY;

//Characteristic handle
bt_gatt_h gattChara = NULL;
const char *charaUuid = "2A24";
const char *charaValue = "4d6163426f6f6b416972372c32";

//descriptor handle
static bt_gatt_h gattDescriptor = NULL;
const char *DescUuid = "2902";
const char *DescValue = "50";

//Latency measure
static float send_time;
static float receive_time;

char *upGestureDetector = "";
static float recordAccX[20];
static float recordGyroX[20];

static float recordAccY[20];
static float recordGyroY[20];

static float recordAccZ[20];
static float recordGyroZ[20];
static char* rpi_address = "";

struct Sample accelerometer_buf[20];
int currentAccPos = 0;
struct Sample gyroscope_buf[20];
int currentGyroPos = 0;

int gesture_count = 0;
bool isSuspended = false;
float suspend_time = 200;
float start_time = 0;

struct _sensor_info {
	sensor_h sensor; /* Sensor handle */
	sensor_listener_h sensor_listener; /* Sensor listener */
};
typedef struct _sensor_info sensorinfo_s;

static sensorinfo_s sensor_info;
static sensorinfo_s sensor_info_gyro;

static bt_advertiser_h advertiser = NULL;
//	set the LE state change callback
bt_adapter_le_advertising_state_changed_cb cb;
const char *time_svc_uuid_16 = "1805";
char service_data[] = { 0x02, 0x01, 0x01 };

static char *lastWrite = NULL;

static char *offline_text_0 = "Please connect to";
static char *offline_text_1 = "the Hub by restarting";
static char *offline_text_2 = "the Hub";

static char *online_text_0 = "Connected,";
static char *online_text_1 = "time to make some";
static char *online_text_2 = "Noise!";

static char * connection_status = "offline";

struct output lastOutput = { 0, 0 };
/**
 * this function returns the cutted current time in millisecond in float type
 */
static float get_current_millis() {
	float k;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	unsigned long long current_time_ms = spec.tv_sec * 1000LL
			+ spec.tv_nsec / 1000000LL;
	k = current_time_ms % LLF_CONVERSION;
	return k;
}
float calculateSD(float data[]) {

	int size = sizeof(data);
	long sum = 0.0, mean, standardDeviation = 0.0;

	int i;

	for (i = 0; i < size; ++i) {
		sum += data[i];
	}

	mean = sum / size;

	for (i = 0; i < size; ++i)
		standardDeviation += pow(data[i] - mean, 2);

	return sqrt(standardDeviation / size);
}

void addAccelerometerSampleToBuffer(struct Sample newSample) {
	if (currentAccPos >= 20) {
		struct Sample temp_buf[20];
		for (int i = 0; i < 20; i++) {
			temp_buf[i] = accelerometer_buf[i];
		}
		for (int i = 0; i < 19; i++) {
			accelerometer_buf[i] = temp_buf[i + 1];
		}
		accelerometer_buf[19] = newSample;
	} else if (currentAccPos == 0) {
		for (int i = 0; i < 20; i++) {
			accelerometer_buf[i].x = 0;
			accelerometer_buf[i].y = 0;
			accelerometer_buf[i].z = 0;
		}
	} else {
		accelerometer_buf[currentAccPos].x = newSample.x;
		accelerometer_buf[currentAccPos].y = newSample.y;
		accelerometer_buf[currentAccPos].z = newSample.z;
	}
	currentAccPos++;

}
void addGyroscopeSampleToBuffer(struct Sample newSample) {
	if (currentGyroPos >= 20) {
		struct Sample temp_buf[20];
		for (int i = 0; i < 20; i++) {
			temp_buf[i] = gyroscope_buf[i];
		}
		for (int i = 0; i < 19; i++) {
			gyroscope_buf[i] = temp_buf[i + 1];
		}
		gyroscope_buf[19] = newSample;
	} else if (currentGyroPos == 0) {
		for (int i = 0; i < 20; i++) {
			gyroscope_buf[i].x = 0;
			gyroscope_buf[i].y = 0;
			gyroscope_buf[i].z = 0;
		}
	} else {
		gyroscope_buf[currentGyroPos].x = newSample.x;
		gyroscope_buf[currentGyroPos].y = newSample.y;
		gyroscope_buf[currentGyroPos].z = newSample.z;
	}
	currentGyroPos++;
}

float *maxAndMinAndSum(float data[]) {
	int size = 20;

	float min = 0;
	float max = 0;

	float sum = 0;

	for (int i = 0; i < size; i++) {
		sum = sum + data[i];
		if (i == 0) {
			min = data[0];
			max = data[0];
		} else {
			if (min != fminf(data[i], min)) {
				min = data[i];
			}
			if (max != fmaxf(data[i], max)) {
				max = data[i];
			}
		}
	}
	float ret[3];
	ret[0] = max;
	ret[1] = min;
	ret[2] = sum;
	return ret;
}

static float Logistic(float x) {
	return (1 / (1 + exp(-x)));
}
static float *Softmax(float x0, float x1, float x2, float x3, float x4,
		float x5, float x6, float x7, float x8, float x9, float x10, float x11,
		float x12, float x13) {
	float inputs[] = { x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12,
			x13 };
	float softmax[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float sum = 0;

	for (int i = 0; i < 14; i++) {
		sum += exp(inputs[i]);
	}

	for (int i = 0; i < 14; i++) {
		softmax[i] = exp(inputs[i]) / sum;
	}
	return softmax;
}
//Traing model in code
static char* expression(void *data, float variable_1, float variable_2,
		float variable_3, float variable_4, float variable_5, float variable_6,
		float variable_7, float variable_8, float variable_9, float variable_10,
		float variable_11, float variable_12, float variable_13,
		float variable_14, float variable_15, float variable_16,
		float variable_17, float variable_18, float variable_19,
		float variable_20, float variable_21, float variable_22,
		float variable_23, float variable_24) {
	appdata_s *ad = data;
	char buf[PATH_MAX];

	float scaled_variable_1 = (variable_1 - 4.44357) / 7.42772;
	float scaled_variable_2 = (variable_2 - 1.574) / 6.50892;
	float scaled_variable_3 = (variable_3 - 13.7528) / 4.14454;
	float scaled_variable_4 = (variable_4 - 52.4426) / 52.5657;
	float scaled_variable_5 = (variable_5 - 49.4163) / 67.2678;
	float scaled_variable_6 = (variable_6 - 60.2393) / 128.759;
	float scaled_variable_7 = (variable_7 + 3.60775) / 7.29239;
	float scaled_variable_8 = (variable_8 + 5.36542) / 6.73846;
	float scaled_variable_9 = (variable_9 - 8.79931) / 3.72308;
	float scaled_variable_10 = (variable_10 + 42.9972) / 49.77;
	float scaled_variable_11 = (variable_11 + 51.4896) / 75.0965;
	float scaled_variable_12 = (variable_12 + 57.9253) / 119.342;
	float scaled_variable_13 = (variable_13 - 8.19986) / 149.535;
	float scaled_variable_14 = (variable_14 + 34.3344) / 131.039;
	float scaled_variable_15 = (variable_15 - 229.69) / 76.7141;
	float scaled_variable_16 = (variable_16 - 22.6874) / 704.622;
	float scaled_variable_17 = (variable_17 + 64.8686) / 1114;
	float scaled_variable_18 = (variable_18 - 3.1065) / 2397.29;
	float scaled_variable_19 = variable_19 / 1.14301;
	float scaled_variable_20 = (variable_20 - 1.04772) / 1.12272;
	float scaled_variable_21 = (variable_21 - 0.790871) / 0.893885;
	float scaled_variable_22 = variable_22 / 17.5914;
	float scaled_variable_23 = variable_23 / 17.9742;
	float scaled_variable_24 = (variable_24 - 18.6732) / 19.2369;

	float y_1_1 = Logistic(
			0.222087 - 2.0742 * scaled_variable_1 + 1.6888 * scaled_variable_2
					+ 2.92041 * scaled_variable_3 - 0.982234 * scaled_variable_4
					+ 3.11609 * scaled_variable_5 + 3.47787 * scaled_variable_6
					+ 0.636268 * scaled_variable_7 - 2.97415 * scaled_variable_8
					+ 0.599789 * scaled_variable_9
					- 0.447892 * scaled_variable_10
					+ 0.97525 * scaled_variable_11
					- 1.95557 * scaled_variable_12
					+ 4.71596 * scaled_variable_13
					+ 2.00837 * scaled_variable_14
					+ 2.38064 * scaled_variable_15
					+ 0.0909202 * scaled_variable_16
					- 3.06462 * scaled_variable_17
					+ 0.256928 * scaled_variable_18
					- 1.01312 * scaled_variable_19
					+ 0.0263702 * scaled_variable_20
					+ 0.137403 * scaled_variable_21
					+ 0.0694326 * scaled_variable_22
					+ 1.18604 * scaled_variable_23
					+ 0.400576 * scaled_variable_24);
	float y_1_2 = Logistic(
			-3.08323 + 4.73328 * scaled_variable_1
					+ 0.178124 * scaled_variable_2 - 3.03808 * scaled_variable_3
					- 0.460915 * scaled_variable_4
					- 0.628708 * scaled_variable_5 - 1.1211 * scaled_variable_6
					+ 0.621832 * scaled_variable_7 + 2.37792 * scaled_variable_8
					- 0.0146665 * scaled_variable_9
					- 0.20538 * scaled_variable_10
					- 5.47224 * scaled_variable_11
					+ 0.866518 * scaled_variable_12
					+ 4.48826 * scaled_variable_13
					+ 0.86196 * scaled_variable_14
					- 4.55489 * scaled_variable_15
					+ 0.806068 * scaled_variable_16
					- 0.624694 * scaled_variable_17
					+ 2.03943 * scaled_variable_18
					+ 0.796782 * scaled_variable_19
					+ 0.500147 * scaled_variable_20
					- 0.467777 * scaled_variable_21
					+ 0.394663 * scaled_variable_22
					+ 2.52779 * scaled_variable_23
					- 2.34684 * scaled_variable_24);
	float y_1_3 = Logistic(
			-3.01638 - 0.73635 * scaled_variable_1 - 3.19755 * scaled_variable_2
					+ 3.07284 * scaled_variable_3 + 1.72924 * scaled_variable_4
					- 0.0577498 * scaled_variable_5
					- 1.63622 * scaled_variable_6 - 3.31771 * scaled_variable_7
					+ 2.65699 * scaled_variable_8 + 1.07467 * scaled_variable_9
					- 0.924247 * scaled_variable_10
					- 4.02852 * scaled_variable_11
					- 0.120932 * scaled_variable_12
					- 0.81634 * scaled_variable_13
					+ 0.455927 * scaled_variable_14
					- 0.684022 * scaled_variable_15
					- 2.54619 * scaled_variable_16
					- 0.605234 * scaled_variable_17
					- 0.916299 * scaled_variable_18
					+ 0.338701 * scaled_variable_19
					+ 0.988952 * scaled_variable_20
					- 0.732314 * scaled_variable_21
					- 1.33948 * scaled_variable_22
					+ 2.77691 * scaled_variable_23
					- 1.07909 * scaled_variable_24);
	float y_1_4 = Logistic(
			1.98285 - 0.222461 * scaled_variable_1 + 3.7426 * scaled_variable_2
					+ 0.197274 * scaled_variable_3
					+ 0.415513 * scaled_variable_4 - 4.63573 * scaled_variable_5
					+ 2.79129 * scaled_variable_6 + 4.11055 * scaled_variable_7
					+ 1.67112 * scaled_variable_8 - 3.09763 * scaled_variable_9
					+ 1.04394 * scaled_variable_10
					- 7.48129 * scaled_variable_11
					+ 4.89379 * scaled_variable_12
					- 4.68947 * scaled_variable_13
					- 2.02343 * scaled_variable_14
					- 1.49951 * scaled_variable_15
					- 1.13526 * scaled_variable_16
					- 3.39209 * scaled_variable_17
					+ 1.40129 * scaled_variable_18
					+ 0.446235 * scaled_variable_19
					- 1.10117 * scaled_variable_20
					+ 0.358631 * scaled_variable_21
					- 1.26391 * scaled_variable_22
					+ 0.382607 * scaled_variable_23
					+ 2.93938 * scaled_variable_24);
	float y_1_5 = Logistic(
			-0.518736 - 2.34031 * scaled_variable_1
					- 0.506546 * scaled_variable_2 - 4.24759 * scaled_variable_3
					+ 0.528312 * scaled_variable_4 + 3.95051 * scaled_variable_5
					+ 1.92157 * scaled_variable_6 - 2.67904 * scaled_variable_7
					+ 1.3787 * scaled_variable_8 + 2.78677 * scaled_variable_9
					- 2.12845 * scaled_variable_10
					+ 5.79772 * scaled_variable_11
					+ 5.84512 * scaled_variable_12
					+ 1.93741 * scaled_variable_13 - 1.8697 * scaled_variable_14
					+ 0.68067 * scaled_variable_15
					+ 0.930661 * scaled_variable_16
					+ 3.17497 * scaled_variable_17
					+ 1.89596 * scaled_variable_18
					- 1.85733 * scaled_variable_19
					- 4.72877 * scaled_variable_20
					+ 0.36538 * scaled_variable_21
					+ 1.47655 * scaled_variable_22
					- 2.34288 * scaled_variable_23
					+ 1.74566 * scaled_variable_24);
	float y_1_6 = Logistic(
			-1.78749 - 2.61696 * scaled_variable_1 - 1.24237 * scaled_variable_2
					+ 4.93702 * scaled_variable_3 - 1.45568 * scaled_variable_4
					+ 0.635477 * scaled_variable_5 - 2.1074 * scaled_variable_6
					- 1.39884 * scaled_variable_7 + 0.59044 * scaled_variable_8
					+ 0.575174 * scaled_variable_9
					+ 1.07237 * scaled_variable_10
					+ 0.0991586 * scaled_variable_11
					+ 2.23753 * scaled_variable_12
					- 3.23744 * scaled_variable_13
					- 1.07829 * scaled_variable_14 + 4.5359 * scaled_variable_15
					+ 0.535224 * scaled_variable_16
					- 0.953146 * scaled_variable_17
					- 0.908539 * scaled_variable_18
					+ 0.419963 * scaled_variable_19
					- 0.257779 * scaled_variable_20
					+ 1.2603 * scaled_variable_21 - 2.18138 * scaled_variable_22
					- 1.59185 * scaled_variable_23
					- 3.90929 * scaled_variable_24);
	float y_1_7 = Logistic(
			-0.160982 + 5.16012 * scaled_variable_1
					- 3.06222 * scaled_variable_2 - 4.63777 * scaled_variable_3
					+ 0.94515 * scaled_variable_4
					+ 0.0694836 * scaled_variable_5
					- 0.604845 * scaled_variable_6 - 5.17515 * scaled_variable_7
					- 0.426673 * scaled_variable_8
					+ 0.967055 * scaled_variable_9
					+ 3.67924 * scaled_variable_10
					+ 2.77413 * scaled_variable_11
					- 7.26354 * scaled_variable_12
					- 2.78942 * scaled_variable_13
					+ 0.234382 * scaled_variable_14
					- 2.90787 * scaled_variable_15
					+ 0.649799 * scaled_variable_16
					- 2.55437 * scaled_variable_17
					- 2.91144 * scaled_variable_18
					+ 0.754254 * scaled_variable_19
					+ 2.47609 * scaled_variable_20
					+ 0.0996498 * scaled_variable_21
					- 0.683339 * scaled_variable_22
					+ 0.521782 * scaled_variable_23
					- 1.79071 * scaled_variable_24);
	float y_1_8 = Logistic(
			1.47859 - 6.56247 * scaled_variable_1 - 0.564416 * scaled_variable_2
					- 1.18662 * scaled_variable_3 + 1.52118 * scaled_variable_4
					- 3.24069 * scaled_variable_5 - 1.70944 * scaled_variable_6
					- 2.3609 * scaled_variable_7 - 1.26838 * scaled_variable_8
					- 0.311241 * scaled_variable_9
					+ 0.00115299 * scaled_variable_10
					- 1.38178 * scaled_variable_11
					+ 0.415684 * scaled_variable_12
					- 3.35173 * scaled_variable_13
					- 0.881467 * scaled_variable_14
					- 1.12595 * scaled_variable_15
					- 0.286534 * scaled_variable_16
					+ 1.7052 * scaled_variable_17
					+ 0.040945 * scaled_variable_18
					- 0.549157 * scaled_variable_19
					- 1.01798 * scaled_variable_20
					- 0.207082 * scaled_variable_21
					- 0.277727 * scaled_variable_22
					- 1.49586 * scaled_variable_23
					- 1.36929 * scaled_variable_24);
	float y_1_9 = Logistic(
			-1.93245 + 5.21733 * scaled_variable_1
					+ 0.195366 * scaled_variable_2
					+ 0.407802 * scaled_variable_3
					- 0.0503092 * scaled_variable_4
					+ 4.62798 * scaled_variable_5 + 3.34439 * scaled_variable_6
					- 5.01545 * scaled_variable_7 - 2.919 * scaled_variable_8
					+ 5.18901 * scaled_variable_9 + 1.75371 * scaled_variable_10
					+ 4.72472 * scaled_variable_11
					- 1.08917 * scaled_variable_12
					+ 0.486976 * scaled_variable_13
					- 4.32044 * scaled_variable_14 - 2.1025 * scaled_variable_15
					- 3.62776 * scaled_variable_16
					- 4.12175 * scaled_variable_17
					- 0.436417 * scaled_variable_18
					+ 3.0494 * scaled_variable_19
					- 0.0742354 * scaled_variable_20
					- 0.708905 * scaled_variable_21
					+ 1.2688 * scaled_variable_22
					- 0.582327 * scaled_variable_23
					+ 3.55654 * scaled_variable_24);

	float non_probabilistic_variable_25 = Logistic(
			-2.75554 - 8.20916 * y_1_1 + 14.4578 * y_1_2 - 8.83539 * y_1_3
					- 9.09127 * y_1_4 - 5.68543 * y_1_5 - 4.95698 * y_1_6
					- 10.2787 * y_1_7 + 10.8485 * y_1_8 - 5.3022 * y_1_9);
	float non_probabilistic_variable_26 = Logistic(
			-15.0593 - 6.98879 * y_1_1 - 6.9385 * y_1_2 - 13.0662 * y_1_3
					+ 8.59811 * y_1_4 + 7.88239 * y_1_5 + 5.3944 * y_1_6
					+ 9.69 * y_1_7 - 1.0117 * y_1_8 - 1.71163 * y_1_9);
	float non_probabilistic_variable_27 = Logistic(
			-3.25382 - 10.374 * y_1_1 - 7.49961 * y_1_2 - 0.45778 * y_1_3
					+ 9.783 * y_1_4 - 6.52136 * y_1_5 + 4.56078 * y_1_6
					+ 4.49589 * y_1_7 - 9.53601 * y_1_8 + 1.67271 * y_1_9);
	float non_probabilistic_variable_28 = Logistic(
			-5.97108 + 3.59715 * y_1_1 + 3.38646 * y_1_2 - 1.58226 * y_1_3
					- 8.84914 * y_1_4 + 2.4599 * y_1_5 - 6.91453 * y_1_6
					- 7.69226 * y_1_7 - 4.0404 * y_1_8 + 1.99444 * y_1_9);
	float non_probabilistic_variable_29 = Logistic(
			-4.99944 - 9.27849 * y_1_1 - 2.56634 * y_1_2 + 5.70902 * y_1_3
					- 4.29829 * y_1_4 + 7.36848 * y_1_5 - 5.27376 * y_1_6
					+ 4.09612 * y_1_7 - 8.52374 * y_1_8 + 2.32156 * y_1_9);
	float non_probabilistic_variable_30 = Logistic(
			-7.49626 - 1.00933 * y_1_1 + 2.21916 * y_1_2 + 10.557 * y_1_3
					+ 0.447209 * y_1_4 - 7.19128 * y_1_5 - 10.9998 * y_1_6
					+ 2.60512 * y_1_7 - 1.96308 * y_1_8 - 2.40328 * y_1_9);
	float non_probabilistic_variable_31 = Logistic(
			-0.158353 - 9.6291 * y_1_1 - 4.72336 * y_1_2 - 5.43226 * y_1_3
					- 5.01717 * y_1_4 - 7.03416 * y_1_5 - 2.97407 * y_1_6
					+ 0.833101 * y_1_7 + 3.1853 * y_1_8 + 2.43552 * y_1_9);
	float non_probabilistic_variable_32 = Logistic(
			-5.95004 + 9.32853 * y_1_1 - 4.4747 * y_1_2 - 3.48932 * y_1_3
					- 5.04803 * y_1_4 - 5.81369 * y_1_5 - 5.23065 * y_1_6
					+ 4.62038 * y_1_7 - 3.91041 * y_1_8 - 1.61246 * y_1_9);
	float non_probabilistic_variable_33 = Logistic(
			-4.23315 + 1.93077 * y_1_1 - 10.1472 * y_1_2 - 2.07241 * y_1_3
					+ 5.39941 * y_1_4 + 3.70951 * y_1_5 - 8.62422 * y_1_6
					- 3.58146 * y_1_7 + 0.171549 * y_1_8 - 1.45552 * y_1_9);
	float non_probabilistic_variable_34 = Logistic(
			-3.85692 + 4.15469 * y_1_1 + 2.06141 * y_1_2 - 8.14449 * y_1_3
					+ 4.26265 * y_1_4 - 9.04846 * y_1_5 - 4.70944 * y_1_6
					- 7.23961 * y_1_7 - 3.17997 * y_1_8 - 0.458751 * y_1_9);
	float non_probabilistic_variable_35 = Logistic(
			-11.6866 - 0.885041 * y_1_1 - 3.44905 * y_1_2 + 6.31082 * y_1_3
					- 1.9955 * y_1_4 - 1.2568 * y_1_5 + 4.53827 * y_1_6
					- 9.68371 * y_1_7 - 2.61097 * y_1_8 + 10.9349 * y_1_9);
	float non_probabilistic_variable_36 = Logistic(
			-5.02655 - 5.21262 * y_1_1 + 3.87302 * y_1_2 + 4.37755 * y_1_3
					+ 1.75915 * y_1_4 - 2.89541 * y_1_5 + 6.18876 * y_1_6
					- 3.88309 * y_1_7 - 10.728 * y_1_8 - 9.7661 * y_1_9);
	float non_probabilistic_variable_37 = Logistic(
			-7.8414 + 2.15406 * y_1_1 - 1.76372 * y_1_2 + 4.04725 * y_1_3
					- 4.44912 * y_1_4 - 5.12996 * y_1_5 + 2.4727 * y_1_6
					- 9.82024 * y_1_7 + 9.94306 * y_1_8 - 11.1294 * y_1_9);
	float non_probabilistic_variable_38 = Logistic(
			-1.53583 + 1.2369 * y_1_1 - 3.55257 * y_1_2 - 5.84919 * y_1_3
					- 4.69287 * y_1_4 + 1.87907 * y_1_5 + 5.72664 * y_1_6
					- 3.92164 * y_1_7 - 6.6347 * y_1_8 - 6.27139 * y_1_9);
	float softmax[14];
	float *pointer;
	int i;

	pointer = Softmax(non_probabilistic_variable_25,
			non_probabilistic_variable_26, non_probabilistic_variable_27,
			non_probabilistic_variable_28, non_probabilistic_variable_29,
			non_probabilistic_variable_30, non_probabilistic_variable_31,
			non_probabilistic_variable_32, non_probabilistic_variable_33,
			non_probabilistic_variable_34, non_probabilistic_variable_35,
			non_probabilistic_variable_36, non_probabilistic_variable_37,
			non_probabilistic_variable_38);

	float variable_25 = *(pointer);
	float variable_26 = *(pointer + 1);
	float variable_27 = *(pointer + 2);
	float variable_28 = *(pointer + 3);
	float variable_29 = *(pointer + 4);
	float variable_30 = *(pointer + 5);
	float variable_31 = *(pointer + 6);
	float variable_32 = *(pointer + 7);
	float variable_33 = *(pointer + 8);
	float variable_34 = *(pointer + 9);
	float variable_35 = *(pointer + 10);
	float variable_36 = *(pointer + 11);
	float variable_37 = *(pointer + 12);
	float variable_38 = *(pointer + 13);

	float max = 0;
	float rets[] = { variable_25, variable_26, variable_27, variable_28,
			variable_29, variable_30, variable_31, variable_32, variable_33,
			variable_34, variable_35, variable_36, variable_37, variable_38 };

	for (int i = 0; i < 14; i++) {
		if (i == 0) {
			max = rets[0];
		} else {
			if (max != fmaxf(rets[i], max)) {
				max = rets[i];

				sprintf(buf, "%.5f, %d", max, i);
//				dlog_print(DLOG_INFO, LOG_TAG, buf);
			}
		}
	}
	sprintf(buf,
			"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f, MAX:%.5f",
			variable_25, variable_26, variable_27, variable_28, variable_29,
			variable_30, variable_31, variable_32, variable_33, variable_34,
			variable_35, variable_36, variable_37, variable_38, max);
//	dlog_print(DLOG_INFO, LOG_TAG, buf);

	if (max == variable_35 || max == variable_36 || max == variable_37
			|| max == variable_38) {
//		if (max == variable_35 || max == variable_36) {
//		dlog_print(DLOG_INFO, LOG_TAG, "Drum bass");
		return "1";
	} else {
//		dlog_print(DLOG_INFO, LOG_TAG, "Null_gesture");
		return "0";
	}

}

/*********************************bluetooth low energy**************************************/
void init_bt() {

	bt_error_e ret;

	ret = bt_initialize();
	if (ret != BT_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG, "[bt_initialize] failed.");

		return;
	}
}

void onConnectToControl(char *hub_address) {
	dlog_print(DLOG_INFO, LOG_TAG, "onConnectToControl");
	dlog_print(DLOG_INFO, LOG_TAG, hub_address);
	changeCharaValue(hub_address);
}
//three difference LE state change callback
static void __bt_adapter_le_advertising_state_changed_cb(int result,
		bt_advertiser_h advertiser, bt_adapter_le_advertising_state_e adv_state,
		void *user_data) {
	dlog_print(DLOG_INFO, LOG_TAG, "Result: %d", result);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertiser: %p", advertiser);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertising %s [%d]",
			adv_state == BT_ADAPTER_LE_ADVERTISING_STARTED ?
					"started" : "stopped", adv_state);
}

static void __bt_adapter_le_advertising_state_changed_cb_2(int result,
		bt_advertiser_h advertiser, bt_adapter_le_advertising_state_e adv_state,
		void *user_data) {
	dlog_print(DLOG_INFO, LOG_TAG, "Result: %d", result);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertiser: %p", advertiser);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertising %s [%d]",
			adv_state == BT_ADAPTER_LE_ADVERTISING_STARTED ?
					"started" : "stopped", adv_state);
}

static void __bt_adapter_le_advertising_state_changed_cb_3(int result,
		bt_advertiser_h advertiser, bt_adapter_le_advertising_state_e adv_state,
		void *user_data) {
	dlog_print(DLOG_INFO, LOG_TAG, "Result: %d", result);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertiser: %p", advertiser);
	dlog_print(DLOG_INFO, LOG_TAG, "Advertising %s [%d]",
			adv_state == BT_ADAPTER_LE_ADVERTISING_STARTED ?
					"started" : "stopped", adv_state);
}
void __bt_gatt_connection_state_changed_cb(int result, bool connected,
		const char *remote_address, void *user_data) {
	appdata_s *ad = (appdata_s*) user_data;
	char buf[PATH_MAX];
	if (connected) {
		connection_status = "online";
		elm_image_file_set(ad->ic, ICON_DIR"/drum.png", NULL);
		elm_object_text_set(ad->label0, online_text_0);
		elm_object_text_set(ad->label1, online_text_1);
		elm_object_text_set(ad->label2, online_text_2);
		sprintf(buf, "Connected %s", remote_address);
//		elm_object_text_set(ad->label1, buf);
		dlog_print(DLOG_INFO, LOG_TAG, buf);
	} else {
		connection_status = "offline";
		elm_image_file_set(ad->ic, ICON_DIR"/warning.png", NULL);
		elm_object_text_set(ad->label0, offline_text_0);
		elm_object_text_set(ad->label1, offline_text_1);
		elm_object_text_set(ad->label2, offline_text_2);
		dlog_print(DLOG_INFO, LOG_TAG, "LE disconnected");
	}
}

void ServerNotificationSentCB(int result, const char *remote_address,
		bt_gatt_server_h server, bt_gatt_h characteristic, bool completed,
		void *user_data) {
	dlog_print(DLOG_INFO, LOG_TAG, "Notification to the device[%s] result[%d]",
			remote_address, result);
}
void __bt_gatt_server_read_value_requested_cb(const char *remote_address,
		int request_id, bt_gatt_server_h server, bt_gatt_h gatt_handle,
		int offset, void *user_data) {
	int ret;
	int *value;
	int *len;
	ret = bt_gatt_get_value(gatt_handle, &value, &len);
	bt_gatt_server_send_response(request_id, BT_GATT_REQUEST_TYPE_READ, offset,
			BT_ERROR_NONE, value, len);
}

bool isLastResponseInFiveSec() {
	float currentTime = get_current_millis();
	if (currentTime - lastCheckConnectionTime > 5000) {
		return false;
	} else {
		return true;
	}
}
void __bt_gatt_server_write_value_requested_cb(const char *remote_address,
		int request_id, bt_gatt_server_h server, bt_gatt_h gatt_handle,
		bool response_needed, int offset, const char *value, int len,
		void *user_data) {
	receive_time = get_current_millis();
	appdata_s *ad = (appdata_s*) user_data;
	char buf[PATH_MAX];
	int ret;
	char str[len + 1];
	str[len] = '\0';
	//str = (char *) malloc(len);
	strncpy(str, value, len);
	dlog_print(DLOG_INFO, LOG_TAG, "__bt_gatt_server_write_value_requested_cb");
	dlog_print(DLOG_INFO, LOG_TAG, "write the value: %s", str);

	if (strcmp(str, "connected") == 0) {
		connection_status = "online";
		elm_image_file_set(ad->ic, ICON_DIR"/drum.png", NULL);
		elm_object_text_set(ad->label0, online_text_0);
		elm_object_text_set(ad->label1, online_text_1);
		elm_object_text_set(ad->label2, online_text_2);
		rpi_address = remote_address;
		sprintf(buf, "%s", remote_address);
//		elm_object_text_set(ad->label1, buf);
		dlog_print(DLOG_INFO, LOG_TAG, buf);
		lastCheckConnectionTime = get_current_millis();
	} else {
//		onConnectToControl(rpi_address);
	}

	char *response_value = value;

	bt_gatt_server_send_response(request_id, BT_GATT_REQUEST_TYPE_WRITE, offset,
			BT_ERROR_NONE, response_value, len);

	if (lastWrite == NULL) {
		lastWrite = "0";
	}

	if (strcmp(str, "1") == 0) {
		if (strcmp(str, lastWrite) == 0) {
		} else {
//			changeAlgoTo("algo1", ad);
		}
		sprintf(buf, "Algo: algo1", len, str);
	} else if (strcmp(str, "2") == 0) {
		if (strcmp(str, lastWrite) == 0) {
		} else {
//			changeAlgoTo("algo2", ad);
		}
		sprintf(buf, "Algo: algo2", len, str);
	} else {
		sprintf(buf, "Algo: error", len, str);
	}
	lastWrite = str;
//	elm_object_text_set(ad->label0, buf);
}

void create_advertise() {
	dlog_print(DLOG_INFO, LOG_TAG, "Start advertise");

	static bt_advertiser_h advertiser_list[3] = { NULL, };
	static int advertiser_index = 0;

	int mode = BT_ADAPTER_LE_ADVERTISING_MODE_LOW_LATENCY;

	int manufacturer_id = 117;
	char *manufacture = NULL;
	char manufacture_0[] = { 0x0, 0x0, 0x0, 0x0 };
	char manufacture_1[] = { 0x01, 0x01, 0x01, 0x01 };
	char manufacture_2[] = { 0x02, 0x02, 0x02, 0x02 };
	char manufacture_3[] = { 0x03, 0x03, 0x03, 0x03 };
	const char *battery_svc_uuid_16 = "180f";
	const char *heart_rate_svc_uuid_16 = "180d";
	const char *immediate_alert_svc_uuid_16 = "1802";
	const char *ancs_uuid_128 = "7905F431-B5CE-4E99-A40F-4B1E122D00D0";
	int appearance = 192; /* 192 is a generic watch */

//	set the LE state change callback
	bt_adapter_le_advertising_state_changed_cb cb;

	if (advertiser_index == 0)
		cb = __bt_adapter_le_advertising_state_changed_cb;
	else if (advertiser_index == 1)
		cb = __bt_adapter_le_advertising_state_changed_cb_2;
	else
		cb = __bt_adapter_le_advertising_state_changed_cb_3;

	advertiser = advertiser_list[advertiser_index];
//	advertiser_index++;
//	advertiser_index %= 3;

	int ret;
	if (advertiser == NULL) {
		ret = bt_adapter_le_create_advertiser(&advertiser);
		dlog_print(DLOG_INFO, LOG_TAG, "created le advertiser(%d)", ret);
		advertiser_list[advertiser_index] = advertiser;
	} else {
		ret = bt_adapter_le_clear_advertising_data(advertiser,
				BT_ADAPTER_LE_PACKET_ADVERTISING);
		if (ret != BT_ERROR_NONE)
			dlog_print(DLOG_INFO, LOG_TAG, "clear advertising data [0x%04x]",
					ret);
		ret = bt_adapter_le_clear_advertising_data(advertiser,
				BT_ADAPTER_LE_PACKET_SCAN_RESPONSE);
		if (ret != BT_ERROR_NONE)
			dlog_print(DLOG_INFO, LOG_TAG, "clear scan response data [0x%04x]",
					ret);
	}

	ret = bt_adapter_le_set_advertising_mode(advertiser, mode);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add scan response data [0x%04x]", ret);

	ret = bt_adapter_le_add_advertising_service_uuid(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, time_svc_uuid_16);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add service_uuid [0x%04x]", ret);

	ret = bt_adapter_le_add_advertising_service_uuid(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, battery_svc_uuid_16);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add service_uuid [0x%04x]", ret);

	ret = bt_adapter_le_add_advertising_service_solicitation_uuid(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, heart_rate_svc_uuid_16);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add service_solicitation_uuid [0x%04x]",
				ret);

	ret = bt_adapter_le_set_advertising_appearance(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, appearance);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add appearance data [0x%04x]", ret);

	ret = bt_adapter_le_set_advertising_tx_power_level(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, true);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add tx_power_level [0x%04x]", ret);

	manufacture = manufacture_3;

	/* Default scan response data */
	ret = bt_adapter_le_add_advertising_service_data(advertiser,
			BT_ADAPTER_LE_PACKET_SCAN_RESPONSE, time_svc_uuid_16, service_data,
			sizeof(service_data));
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add service_data [0x%04x]", ret);

	ret = bt_adapter_le_set_advertising_device_name(advertiser,
			BT_ADAPTER_LE_PACKET_SCAN_RESPONSE, true);
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "set device name [0x%04x]", ret);

	ret = bt_adapter_le_add_advertising_manufacturer_data(advertiser,
			BT_ADAPTER_LE_PACKET_SCAN_RESPONSE, manufacturer_id, manufacture,
			sizeof(manufacture_0));
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "add manufacturer data [0x%04x]", ret);

//	advertising
	ret = bt_adapter_le_start_advertising_new(advertiser, cb, NULL);
	if (ret != BT_ERROR_NONE) {
		char* errMsg;
		errMsg = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "Advertising failed. err = %s", errMsg);
	}
}

void changeAdvertisingServiceData(char *ads) {

	if (strcmp(ads, "00") == 0) {
		service_data[0] = 0x00;
	} else if (strcmp(ads, "01") == 0) {
		service_data[0] = 0x01;
	} else if (strcmp(ads, "02") == 0) {
		service_data[0] = 0x02;
	} else if (strcmp(ads, "03") == 0) {
		service_data[0] = 0x03;
	} else if (strcmp(ads, "04") == 0) {
		service_data[0] = 0x04;
	} else if (strcmp(ads, "05") == 0) {
		service_data[0] = 0x05;
	} else {
		service_data[0] = 0x99;
		dlog_print(DLOG_ERROR, LOG_TAG, "changeAdvertisingServiceData failed");
	}
	dlog_print(DLOG_INFO, LOG_TAG, "change Value");
	create_advertise();
}

void createService(appdata_s *ad) {
	int ret = bt_gatt_set_connection_state_changed_cb(
			__bt_gatt_connection_state_changed_cb, ad);

	if (ret != BT_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG,
				"bt_gatt_set_connection_state_changed_cb failed. err = %s",
				get_error_message(ret));
	}

	dlog_print(DLOG_INFO, LOG_TAG, "Create Service");
	ret = bt_gatt_server_initialize();
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "Init GATT server failed. err = %s",
				err);
	}

	if (!gattServer) {
//		gattServer not exist, create server
		ret = bt_gatt_server_create(&gattServer);
		if (ret != BT_ERROR_NONE) {
			char* err;
			err = get_error_message(ret);
			dlog_print(DLOG_ERROR, LOG_TAG,
					"Create GATT server failed. err = %s", err);
		} else {
			dlog_print(DLOG_INFO, LOG_TAG, "Create GATT server Succeed");
		}
	}

	ret = bt_gatt_service_create(service_uuid, type, &gattSvc);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "Create GATT service failed. err = %s",
				err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "Create GATT service Succeed");
	}
	ret = bt_gatt_characteristic_create(charaUuid,
			BT_GATT_PERMISSION_READ | BT_GATT_PERMISSION_WRITE,
			BT_GATT_PROPERTY_INDICATE | BT_GATT_PROPERTY_READ
					| BT_GATT_PROPERTY_WRITE | BT_GATT_PROPERTY_NOTIFY,
			charaValue,
			LE_INITIAL_BUF_SIZE, &gattChara);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG,
				"create characteristic  failed. err = %s", err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "create characteristic Succeed");
	}

	ret = bt_gatt_server_set_read_value_requested_cb(gattChara,
			__bt_gatt_server_read_value_requested_cb, NULL);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "create read request failed. err = %s",
				err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "create read request Succeed");
	}
	ret = bt_gatt_server_set_write_value_requested_cb(gattChara,
			__bt_gatt_server_write_value_requested_cb, ad);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "create write request failed. err = %s",
				err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "create write request Succeed");
	}

	ret = bt_gatt_descriptor_create(DescUuid,
			BT_GATT_PERMISSION_READ | BT_GATT_PERMISSION_WRITE, DescValue,
			LE_INITIAL_BUF_SIZE, &gattDescriptor);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "create descriptor failed. err = %s",
				err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "create descriptor server Succeed");
	}

	ret = bt_gatt_characteristic_add_descriptor(gattChara, gattDescriptor);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG,
				"add descriptor to characteristic failed. err = %s", err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG,
				"add descriptor to characteristic Succeed");
	}
	ret = bt_gatt_service_add_characteristic(gattSvc, gattChara);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG,
				"add characteristic to service failed. err = %s", err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "add characteristic to service Succeed");
	}

	ret = bt_gatt_server_register_service(gattServer, gattSvc);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG,
				"register GATT service failed. err = %s", err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "register GATT service Succeed");
	}

	ret = bt_gatt_server_start();
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG, "start GATT server failed. err = %s",
				err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "start GATT server Succeed");
	}
}

void changeCharaValue(char *gesture) {
	int ret;
	if (lastResponse == NULL || lastResponse == "05") {
		response = ("01");
	} else {
		int a;
		/* for loop execution */
		for (a = 0; a < 4; a = a + 1) {
			if (tick_response[a] == lastResponse) {
				response = tick_response[a + 1];
			}
		}
	}
	lastResponse = response;

//	changeAdvertisingServiceData(response);
	char c[10];
	sprintf(c, "%.f", get_current_millis());
//	response = c;
//	strcat(response, c);

	response = gesture;

	ret = bt_gatt_set_value(gattChara, response, strlen(response));
	if (ret == BT_ERROR_NONE)

		send_time = get_current_millis();
	ret = bt_gatt_server_notify_characteristic_changed_value(gattChara,
			ServerNotificationSentCB, NULL, NULL);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
//		dlog_print(DLOG_ERROR, LOG_TAG,
//				"send notification callback failed. err = %s", err);
	} else {
//		dlog_print(DLOG_INFO, LOG_TAG, "send notification callback Succeed");
	}
}
/*****************************************************************************************/
static void win_delete_request_cb(void *data, Evas_Object *obj,
		void *event_info) {
	ui_app_exit();
}

static void win_back_cb(void *data, Evas_Object *obj, void *event_info) {
	appdata_s *ad = data;
	/* Let window go to hide state. */
	elm_win_lower(ad->win);
}

/**
 * Add a UI component
 */
static void my_box_pack(Evas_Object *box, Evas_Object *child, double h_weight,
		double v_weight, double h_align, double v_align) {
	/* Tell the child packed into the box to be able to expand */
//evas_object_size_hint_weight_set(child, h_weight, v_weight);
	/* Fill the expanded area (above) as opposed to centering in it */
	evas_object_size_hint_align_set(child, h_align, v_align);
	/* Set the child as the box content and show it */
	evas_object_show(child);
	elm_object_content_set(box, child);
	/* Put the child into the box */
	elm_box_pack_end(box, child);
	/* Show the box */
	evas_object_show(box);
}

static float get_absolute_max(float value1, float value2) {
	float v1 = value1 > 0.f ? value1 : -value1;
	float v2 = value2 > 0.f ? value2 : -value2;
	float result = v1 > v2 ? v1 : v2;
	return result;
}

/*******************************ACCELEROMETER AND PREDICTION***********************************/
/*********************************************************************************************/
void send_viberation_feedback() {
	int ret;
	ret = feedback_initialize();
	if (ret != 0) {
		dlog_print(DLOG_INFO, LOG_TAG, "feedback_initialize fail %s",
				get_error_message(ret));
	}
	ret = feedback_play_type(FEEDBACK_TYPE_VIBRATION, FEEDBACK_PATTERN_POWERON);
	if (ret != 0) {
		dlog_print(DLOG_INFO, LOG_TAG, "feedback_play_type fail %s",
				get_error_message(ret));
	}
}

static void _new_accelerometer_value(sensor_h sensor,
		sensor_event_s *sensor_data, void *user_data) {
	appdata_s *ad = (appdata_s*) user_data;
	char buf[PATH_MAX];
	if (get_current_millis() < start_time + suspend_time) {
		isSuspended = true;
	} else {
		isSuspended = false;
	}
//	Check if the device is still on line
	if (!isLastResponseInFiveSec()) {
		if (strcmp(connection_status, "offline") != 0) {
			elm_image_file_set(ad->ic, ICON_DIR"/warning.png", NULL);
			elm_object_text_set(ad->label0, offline_text_0);
			elm_object_text_set(ad->label1, offline_text_1);
			elm_object_text_set(ad->label2, offline_text_2);
			connection_status = "offline";
		}
	}

	if (sensor_data->value_count < 3)
		return;

	if (!isSuspended) {
		//	record the data
		struct Sample AccSample = { sensor_data->values[0],
				sensor_data->values[1], sensor_data->values[2] };

		addAccelerometerSampleToBuffer(AccSample);

		//		Feature Extraction
		if (currentAccPos >= 20) {
			for (int i = 0; i < 20; i++) {
				recordAccX[i] = accelerometer_buf[i].x;
				recordAccY[i] = accelerometer_buf[i].y;
				recordAccZ[i] = accelerometer_buf[i].z;
				recordGyroX[i] = gyroscope_buf[i].x;
				recordGyroY[i] = gyroscope_buf[i].y;
				recordGyroZ[i] = gyroscope_buf[i].z;
			}

//		Sum of all 6 array (for arrays that have same size, sum shows the same feature as mean with less computation)
			//			Max and Min of all 6 arrays
			float maxminsumAccX[3];
			float maxminsumAccY[3];
			float maxminsumAccZ[3];
			float maxminsumGyroX[3];
			float maxminsumGyroY[3];
			float maxminsumGyroZ[3];

			float *pointer;
			int i;

			pointer = maxAndMinAndSum(recordAccX);
			for (i = 0; i < 3; i++) {
				maxminsumAccX[i] = *(pointer + i);
			}
			pointer = maxAndMinAndSum(recordAccY);
			for (i = 0; i < 3; i++) {
				maxminsumAccY[i] = *(pointer + i);
			}
			pointer = maxAndMinAndSum(recordAccZ);
			for (i = 0; i < 3; i++) {
				maxminsumAccZ[i] = *(pointer + i);
			}
			pointer = maxAndMinAndSum(recordGyroX);
			for (i = 0; i < 3; i++) {
				maxminsumGyroX[i] = *(pointer + i);
			}
			pointer = maxAndMinAndSum(recordGyroY);
			for (i = 0; i < 3; i++) {
				maxminsumGyroY[i] = *(pointer + i);
			}
			pointer = maxAndMinAndSum(recordGyroZ);
			for (i = 0; i < 3; i++) {
				maxminsumGyroZ[i] = *(pointer + i);
			}

			//			Standard Deviation for all 6 arrays
			float SDAccX = calculateSD(recordAccX);
			float SDAccY = calculateSD(recordAccY);
			float SDAccZ = calculateSD(recordAccZ);
			float SDGyroX = calculateSD(recordGyroX);
			float SDGyroY = calculateSD(recordGyroY);
			float SDGyroZ = calculateSD(recordGyroZ);

			char* gesture = expression(ad, maxminsumAccX[0], maxminsumAccY[0],
					maxminsumAccZ[0], maxminsumGyroX[0], maxminsumGyroY[0],
					maxminsumGyroZ[0], maxminsumAccX[1], maxminsumAccY[1],
					maxminsumAccZ[1], maxminsumGyroX[1], maxminsumGyroY[1],
					maxminsumGyroZ[1], maxminsumAccX[2], maxminsumAccY[2],
					maxminsumAccZ[2], maxminsumGyroX[2], maxminsumGyroY[2],
					maxminsumGyroZ[2], SDAccX, SDAccY, SDAccZ, SDGyroX, SDGyroY,
					SDGyroZ);

			if (strcmp(gesture, "0") == 0) {
				gesture_count = 0;
			} else {
				gesture_count++;
			}
			if (gesture_count > 8) {
				float euclidean_distance = maxminsumAccX[0] * maxminsumAccX[0]
						+ maxminsumAccY[0] * maxminsumAccY[0]
						+ maxminsumAccZ[0] * maxminsumAccZ[0];
				if (euclidean_distance > 500) {
					if (lastOutput.maxZ != maxminsumAccZ[0]) {
						start_time = get_current_millis();
//						sprintf(buf,
//								"Is it HAPPENING: %.5f: [%.5f,%.5f,%.5f] (%.5f)___lastmaxX=%.5f , lastmaxZ=%.5f",
//								euclidean_distance, maxminsumAccX[0],
//								maxminsumAccY[0], maxminsumAccZ[0], start_time,
//								lastOutput.maxX, lastOutput.maxZ);
						sprintf(buf, "");
						for (int i = 0; i < 20; i++) {
							sprintf(buf, "%s ; %.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
									buf, accelerometer_buf[i].x,
									accelerometer_buf[i].y,
									accelerometer_buf[i].z, gyroscope_buf[i].x,
									gyroscope_buf[i].y, gyroscope_buf[i].z);
						}
						dlog_print(DLOG_INFO, LOG_TAG, buf);
//
						sprintf(buf,
								"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
								maxminsumAccX[0], maxminsumAccY[0],
								maxminsumAccZ[0], maxminsumGyroX[0],
								maxminsumGyroY[0], maxminsumGyroZ[0],
								maxminsumAccX[1], maxminsumAccY[1],
								maxminsumAccZ[1], maxminsumGyroX[1],
								maxminsumGyroY[1], maxminsumGyroZ[1],
								maxminsumAccX[2], maxminsumAccY[2],
								maxminsumAccZ[2], maxminsumGyroX[2],
								maxminsumGyroY[2], maxminsumGyroZ[2], SDAccX,
								SDAccY, SDAccZ, SDGyroX, SDGyroY, SDGyroZ);
						dlog_print(DLOG_INFO, LOG_TAG, buf);
						if (euclidean_distance < 1500) {
							changeCharaValue("0");
						} else if (euclidean_distance >= 150
								&& euclidean_distance < 3000) {
							changeCharaValue("1");
						} else {
							changeCharaValue("2");
						}
						start_time = get_current_millis();
						send_viberation_feedback();

						lastOutput.maxX = maxminsumAccX[0];
						lastOutput.maxZ = maxminsumAccZ[0];

						for (int i = 0; i < 3; i++) {
							maxminsumAccX[i] = 0;
							maxminsumAccY[i] = 0;
							maxminsumAccZ[i] = 0;
							maxminsumGyroX[i] = 0;
							maxminsumGyroY[i] = 0;
							maxminsumGyroZ[i] = 0;
						}
						for (int i = 0; i < 20; i++) {
							recordAccX[i] = 0;
							recordAccY[i] = 0;
							recordAccZ[i] = 0;
							recordGyroX[i] = 0;
							recordGyroY[i] = 0;
							recordGyroZ[i] = 0;
						}
					}
				}
				gesture_count = 0;
			}

		}
	}
}
static void _new_gyroscope_value(sensor_h sensor, sensor_event_s *sensor_data,
		void *user_data) {
	char buf[PATH_MAX];
	appdata_s *ad = (appdata_s*) user_data;

	if (sensor_data->value_count < 3)
		return;

	//	record the data
	struct Sample GyroSample = { sensor_data->values[0], sensor_data->values[1],
			sensor_data->values[2] };

	addGyroscopeSampleToBuffer(GyroSample);
}

static void start_accelerator_sensor(appdata_s *ad, int af) {
	sensor_error_e err = SENSOR_ERROR_NONE;
	sensor_get_default_sensor(SENSOR_ACCELEROMETER, &sensor_info.sensor);
	err = sensor_create_listener(sensor_info.sensor,
			&sensor_info.sensor_listener);
	sensor_listener_set_event_cb(sensor_info.sensor_listener, af,
			_new_accelerometer_value, ad);
	sensor_listener_start(sensor_info.sensor_listener);
}

static void start_gyroscope_sensor(appdata_s *ad, int gf) {
	sensor_error_e err = SENSOR_ERROR_NONE;
	sensor_get_default_sensor(SENSOR_GYROSCOPE, &sensor_info_gyro.sensor);
	err = sensor_create_listener(sensor_info_gyro.sensor,
			&sensor_info_gyro.sensor_listener);
	sensor_listener_set_event_cb(sensor_info_gyro.sensor_listener, gf,
			_new_gyroscope_value, ad);
	sensor_listener_start(sensor_info_gyro.sensor_listener);
}
static void stop_accelerator_sensor(appdata_s *ad) {
	sensor_listener_stop(sensor_info.sensor_listener);
}

static void stop_gyroscope_sensor(appdata_s *ad) {
	sensor_listener_stop(sensor_info_gyro.sensor_listener);
}
void start_recording(appdata_s *ad) {
	char buf[PATH_MAX];
	start_accelerator_sensor(ad, af_interval);
	start_gyroscope_sensor(ad, gf_interval);
}
void stop_recording(appdata_s *ad) {

	stop_accelerator_sensor(ad);
	stop_gyroscope_sensor(ad);
}
/* Button click event function */
void exit_tizen(void *data, Evas_Object *obj, void *event_info) {
	appdata_s *ad = (appdata_s*) data;
	int ret;
	/* Deregister callbacks */
	bt_adapter_unset_state_changed_cb();
	bt_adapter_unset_device_discovery_state_changed_cb();
	bt_device_unset_service_searched_cb();
	bt_socket_unset_data_received_cb();
	bt_socket_unset_connection_state_changed_cb();

	/* Release resources */

	/* Deinitialize Bluetooth */
	ret = bt_deinitialize();
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_ERROR, LOG_TAG, "[bt_deinitialize] failed.");
	stop_accelerator_sensor(ad);
	ui_app_exit();
}
static void create_base_gui(appdata_s *ad) {
	/* Window */
	/* Create and initialize elm_win.
	 elm_win is mandatory to manipulate window. */
	ad->win = elm_win_util_standard_add(PACKAGE, PACKAGE);
	elm_win_autodel_set(ad->win, EINA_TRUE);

	if (elm_win_wm_rotation_supported_get(ad->win)) {
		int rots[4] = { 0, 90, 180, 270 };
		elm_win_wm_rotation_available_rotations_set(ad->win,
				(const int *) (&rots), 4);
	}

	evas_object_smart_callback_add(ad->win, "delete,request",
			win_delete_request_cb,
			NULL);
	eext_object_event_callback_add(ad->win, EEXT_CALLBACK_BACK, win_back_cb,
			ad);

	/* Conformant */
	/* Create and initialize elm_conformant.
	 elm_conformant is mandatory for base gui to have proper size
	 when indicator or virtual keypad is visible. */
	ad->conform = elm_conformant_add(ad->win);
	elm_win_indicator_mode_set(ad->win, ELM_WIN_INDICATOR_SHOW);
	elm_win_indicator_opacity_set(ad->win, ELM_WIN_INDICATOR_OPAQUE);
	evas_object_size_hint_weight_set(ad->conform, EVAS_HINT_EXPAND,
	EVAS_HINT_EXPAND);
	elm_win_resize_object_add(ad->win, ad->conform);
	evas_object_show(ad->conform);

	/* Box can contain other elements in a vertical line (by default) */
	Evas_Object *box = elm_box_add(ad->win);
	evas_object_size_hint_weight_set(box, EVAS_HINT_EXPAND,
	EVAS_HINT_EXPAND);
	evas_object_size_hint_align_set(box, EVAS_HINT_EXPAND,
	EVAS_HINT_EXPAND);
	elm_object_content_set(ad->conform, box);
	evas_object_show(box);

	ad->ic = elm_icon_add(ad->conform);
	elm_image_file_set(ad->ic, ICON_DIR"/warning.png", NULL);
	evas_object_size_hint_min_set(ad->ic, ELM_SCALE_SIZE(50),
			ELM_SCALE_SIZE(50));
	evas_object_show(ad->ic);
	elm_box_pack_end(box, ad->ic);

	ad->label0 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label0, offline_text_0);
	my_box_pack(box, ad->label0, 1.5, 1.0, 0.5, -1.0);

	ad->label1 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label1, offline_text_1);
	my_box_pack(box, ad->label1, 1.0, 1.0, 0.5, -1.0);

	ad->label2 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label2, offline_text_2);
	my_box_pack(box, ad->label2, 1.0, 1.0, 0.5, -1.0);

//	/* Button */
//	Evas_Object *btn2 = elm_button_add(ad->conform);
//	elm_object_text_set(btn2, "Exit");
//	evas_object_smart_callback_add(btn2, "clicked", exit_tizen, ad);
//	my_box_pack(box, btn2, 1.0, 1.0, -1.0, -1.0);

	/* Show the window after the base GUI is set up */
	evas_object_show(ad->win);

	/* Show window after base gui is set up */
	evas_object_show(ad->win);
}

static bool app_create(void *data) {
	/* Hook to take necessary actions before main event loop starts
	 Initialize UI resources and application's data
	 If this function returns true, the main loop of application starts
	 If this function returns false, the application is terminated */
	appdata_s *ad = data;

	init_bt();
	create_base_gui(ad);
	createService(ad);
	create_advertise();

	start_recording(ad);

	return true;
}

static void app_control(app_control_h app_control, void *data) {
	/* Handle the launch request. */
}

static void app_pause(void *data) {
	/* Take necessary actions when application becomes invisible. */
	appdata_s *ad = data;
	stop_recording(ad);
}

static void app_resume(void *data) {
	/* Take necessary actions when application becomes visible. */
	appdata_s *ad = data;
	start_recording(ad);

	int ret = efl_util_set_window_screen_mode(ad->win,
			EFL_UTIL_SCREEN_MODE_ALWAYS_ON);
	if (ret != EFL_UTIL_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG,
				"efl_util_set_window_screen_mode fail err = %s",
				get_error_message(ret));
	} else {
	}
}

static void app_terminate(void *data) {
	/* Release all resources. */
	appdata_s *ad = data;
	int ret;

	/* Deregister callbacks */
	bt_adapter_unset_state_changed_cb();
	bt_adapter_unset_device_discovery_state_changed_cb();
	bt_device_unset_service_searched_cb();
	bt_socket_unset_data_received_cb();
	bt_socket_unset_connection_state_changed_cb();

	/* Release resources */

	/* Deinitialize Bluetooth */
	ret = bt_deinitialize();
	if (ret != BT_ERROR_NONE)
		dlog_print(DLOG_ERROR, LOG_TAG, "[bt_deinitialize] failed.");
	stop_accelerator_sensor(ad);
}

static void ui_app_lang_changed(app_event_info_h event_info, void *user_data) {
	/*APP_EVENT_LANGUAGE_CHANGED*/
	char *locale = NULL;
	system_settings_get_value_string(SYSTEM_SETTINGS_KEY_LOCALE_LANGUAGE,
			&locale);
	elm_language_set(locale);
	free(locale);
	return;
}

static void ui_app_orient_changed(app_event_info_h event_info, void *user_data) {
	/*APP_EVENT_DEVICE_ORIENTATION_CHANGED*/
	return;
}

static void ui_app_region_changed(app_event_info_h event_info, void *user_data) {
	/*APP_EVENT_REGION_FORMAT_CHANGED*/
}

static void ui_app_low_battery(app_event_info_h event_info, void *user_data) {
	/*APP_EVENT_LOW_BATTERY*/
}

static void ui_app_low_memory(app_event_info_h event_info, void *user_data) {
	/*APP_EVENT_LOW_MEMORY*/
}

int main(int argc, char *argv[]) {
	appdata_s ad = { 0, };
	int ret = 0;

	ui_app_lifecycle_callback_s event_callback = { 0, };
	app_event_handler_h handlers[5] = { NULL, };

	event_callback.create = app_create;
	event_callback.terminate = app_terminate;
	event_callback.pause = app_pause;
	event_callback.resume = app_resume;
	event_callback.app_control = app_control;

	ui_app_add_event_handler(&handlers[APP_EVENT_LOW_BATTERY],
			APP_EVENT_LOW_BATTERY, ui_app_low_battery, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_LOW_MEMORY],
			APP_EVENT_LOW_MEMORY, ui_app_low_memory, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_DEVICE_ORIENTATION_CHANGED],
			APP_EVENT_DEVICE_ORIENTATION_CHANGED, ui_app_orient_changed, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_LANGUAGE_CHANGED],
			APP_EVENT_LANGUAGE_CHANGED, ui_app_lang_changed, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_REGION_FORMAT_CHANGED],
			APP_EVENT_REGION_FORMAT_CHANGED, ui_app_region_changed, &ad);

	ret = ui_app_main(argc, argv, &event_callback, &ad);
	if (ret != APP_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG, "app_main() is failed. err = %d", ret);
	}

	return ret;
}
