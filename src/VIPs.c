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
#include <gesture_recognition.h>

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

float max_gyro_x = 0;
float max_gyro_y = 0;
float max_gyro_z = 0;

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

static int sound_mode_index = 0;
static char* sound_mode[] = { "0", "1" };
static int sound_mode_size = 2;
float last_tilt = 0;

gesture_h gesture_handle;

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
	float softmax[14];
	float *pointer;
	int i;

	float scaled_variable_1 = 2 * (variable_1 + 21.2) / (67.3 + 21.2) - 1;
	float scaled_variable_3 = 2 * (variable_3 + 16.6) / (29.1 + 16.6) - 1;
	float scaled_variable_5 = 2 * (variable_5 + 255.4) / (675.4 + 255.4) - 1;
	float scaled_variable_6 = 2 * (variable_6 + 269.3) / (622.7 + 269.3) - 1;
	float scaled_variable_7 = 2 * (variable_7 + 83.5) / (24.4 + 83.5) - 1;
	float scaled_variable_11 = 2 * (variable_11 + 786.7) / (272.2 + 786.7) - 1;
	float scaled_variable_12 = 2 * (variable_12 + 578.9) / (239.3 + 578.9) - 1;
	float scaled_variable_13 = 2 * (variable_13 + 658.6) / (652.3 + 658.6) - 1;
	float scaled_variable_15 = 2 * (variable_15 + 536.8) / (510.3 + 536.8) - 1;
	float scaled_variable_19 = 2 * (variable_19 - 0) / (25.8 - 0) - 1;
	float scaled_variable_20 = 2 * (variable_20 - 0) / (17.2 - 0) - 1;
	float scaled_variable_21 = 2 * (variable_21 - 0) / (14.6 - 0) - 1;
	float scaled_variable_23 = 2 * (variable_23 - 0) / (346.6 - 0) - 1;
	float scaled_variable_24 = 2 * (variable_24 - 0) / (245.9 - 0) - 1;
	float y_1_1 = Logistic(
			3.84693 + 0.864156 * scaled_variable_1
					+ 0.0126722 * scaled_variable_3
					+ 2.22452 * scaled_variable_5 + 4.99815 * scaled_variable_6
					- 3.8777 * scaled_variable_7 + 2.53394 * scaled_variable_11
					- 0.56755 * scaled_variable_12 + 6.7284 * scaled_variable_13
					+ 0.0243029 * scaled_variable_15
					- 2.55245 * scaled_variable_19
					- 0.122452 * scaled_variable_20
					+ 1.97159 * scaled_variable_21
					+ 2.35748 * scaled_variable_23
					+ 0.25102 * scaled_variable_24);
	float y_1_2 = Logistic(
			0.997523 - 1.25573 * scaled_variable_1 + 1.46679 * scaled_variable_3
					+ 2.04299 * scaled_variable_5 + 3.13009 * scaled_variable_6
					- 5.64739 * scaled_variable_7
					- 0.163151 * scaled_variable_11
					+ 0.1696 * scaled_variable_12 - 4.81621 * scaled_variable_13
					+ 1.25942 * scaled_variable_15
					- 0.646767 * scaled_variable_19
					+ 0.0837131 * scaled_variable_20
					+ 0.456602 * scaled_variable_21
					+ 0.0542992 * scaled_variable_23
					- 0.65525 * scaled_variable_24);
	float y_1_3 = Logistic(
			-0.275492 + 0.404695 * scaled_variable_1
					- 1.18241 * scaled_variable_3 + 5.22998 * scaled_variable_5
					- 7.09059 * scaled_variable_6 - 3.66479 * scaled_variable_7
					- 3.33964 * scaled_variable_11
					+ 5.85238 * scaled_variable_12
					+ 2.86495 * scaled_variable_13
					+ 3.89582 * scaled_variable_15
					- 2.76323 * scaled_variable_19
					- 2.45958 * scaled_variable_20
					+ 2.67848 * scaled_variable_21
					- 0.153801 * scaled_variable_23
					+ 1.69358 * scaled_variable_24);
	float non_probabilistic_variable_25 = Logistic(
			0.10008 - 11.0926 * y_1_1 - 12.0078 * y_1_2 + 2.59008 * y_1_3);
	float non_probabilistic_variable_26 = Logistic(
			0.421917 - 11.4324 * y_1_1 - 9.15654 * y_1_2 + 1.03497 * y_1_3);
	float non_probabilistic_variable_27 = Logistic(
			-27.8116 - 6.18631 * y_1_1 + 30.6264 * y_1_2 - 0.264297 * y_1_3);
	float non_probabilistic_variable_28 = Logistic(
			-11.2204 + 12.0866 * y_1_1 - 22.0974 * y_1_2 + 0.530628 * y_1_3);
	float non_probabilistic_variable_29 = Logistic(
			-4.50881 + 2.76866 * y_1_1 + 0.389287 * y_1_2 - 0.0636301 * y_1_3);
	float non_probabilistic_variable_30 = Logistic(
			2.48191 - 28.6614 * y_1_1 + 1.14825 * y_1_2 - 6.98153 * y_1_3);
	float non_probabilistic_variable_31 = Logistic(
			1.45718 - 2.79353 * y_1_1 - 4.62369 * y_1_2 - 13.5486 * y_1_3);
	float non_probabilistic_variable_32 = Logistic(
			0.925857 + 0.10625 * y_1_1 - 31.5662 * y_1_2 - 10.3392 * y_1_3);
	float non_probabilistic_variable_33 = Logistic(
			-6.23481 + 2.58449 * y_1_1 + 7.94667 * y_1_2 - 25.2398 * y_1_3);
	float non_probabilistic_variable_34 = Logistic(
			-14.8453 + 19.4954 * y_1_1 - 6.39813 * y_1_2 - 20.7147 * y_1_3);
	float non_probabilistic_variable_35 = Logistic(
			-24.6592 + 3.64919 * y_1_1 + 8.69399 * y_1_2 + 16.9344 * y_1_3);
	float non_probabilistic_variable_36 = Logistic(
			-20.0204 - 5.88453 * y_1_1 - 29.1113 * y_1_2 + 28.4171 * y_1_3);
	float non_probabilistic_variable_37 = Logistic(
			-9.96975 - 11.4451 * y_1_1 + 0.907963 * y_1_2 + 13.1933 * y_1_3);
	float non_probabilistic_variable_38 = Logistic(
			-10.8856 + 2.51423 * y_1_1 - 2.12679 * y_1_2 + 9.08299 * y_1_3);

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

	if (max == variable_35 || max == variable_36) {
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

void change_sound_mode(void *user_data) {
	appdata_s *ad = (appdata_s*) user_data;
	char buf[PATH_MAX];
	int sound_size = sound_mode_size;
	if (sound_mode_index < (sound_size - 1)) {
		sound_mode_index = sound_mode_index + 1;
	} else {
		sound_mode_index = 0;
	}
	sprintf(buf, "Sound_mode: %s",sound_mode[sound_mode_index]);
	elm_object_text_set(ad->label3, buf);
}
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
void erase(void *user_data) {
	appdata_s *ad = (appdata_s*) user_data;

	max_gyro_x = 0;
	max_gyro_y = 0;
	max_gyro_z = 0;

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

//			if (maxminsumGyroX[0] > max_gyro_x || maxminsumGyroY[0] > max_gyro_y
//					|| maxminsumGyroZ[0] > max_gyro_z) {
//				if (maxminsumGyroX[0] > max_gyro_x) {
//					max_gyro_x = maxminsumGyroX[0];
//				}
//				if (maxminsumGyroY[0] > max_gyro_y) {
//					max_gyro_y = maxminsumGyroY[0];
//				}
//				if (maxminsumGyroZ[0] > max_gyro_z) {
//					max_gyro_z = maxminsumGyroZ[0];
//				}
//				sprintf(buf, "Max_Gyro_xyz: %.f   %.f   %.f", max_gyro_x,
//						max_gyro_y, max_gyro_z);
//				dlog_print(DLOG_INFO, LOG_TAG, buf);
//			}

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

				if (maxminsumGyroX[0] > 250 && maxminsumGyroY[0] > 80
						&& maxminsumGyroZ[0] > 80) {
					float current_time = get_current_millis();
					if (current_time - last_tilt > 300) {
//						sprintf(buf, "maxminsumGyroX: %.f   %.f   %.f",
//								maxminsumGyroX[0], maxminsumGyroY[0],
//								maxminsumGyroZ[0]);
						dlog_print(DLOG_INFO, LOG_TAG, buf);
						last_tilt = get_current_millis();
						change_sound_mode(ad);
						send_viberation_feedback();
					}
				}
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

						sprintf(buf, "");
						if (euclidean_distance < 1500) {
							sprintf(buf, "0%s", sound_mode[sound_mode_index]);
						} else if (euclidean_distance >= 150
								&& euclidean_distance < 3000) {
							sprintf(buf, "1%s", sound_mode[sound_mode_index]);
						} else {
							sprintf(buf, "2%s", sound_mode[sound_mode_index]);
						}
						changeCharaValue(buf);

						start_time = get_current_millis();
						send_viberation_feedback();
						dlog_print(DLOG_INFO, LOG_TAG, "Drum: %s", buf);

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

	ad->label3 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label3, "Sound_mode: 0");
	my_box_pack(box, ad->label3, 1.0, 1.0, 0.5, -1.0);

//	/* Button */
//	Evas_Object *btn2 = elm_button_add(ad->conform);
//	elm_object_text_set(btn2, "erase");
//	evas_object_smart_callback_add(btn2, "clicked", erase, ad);
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

//	start_gesture_recognition(ad);

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
//	start_gesture_recognition(ad);
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
	gesture_stop_recognition(gesture_handle);
	gesture_release(gesture_handle);
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
