#include "vips.h"
#include <stdio.h>
#include <storage.h>
#include <time.h>
#include <stdlib.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <dlog.h>
#include <assert.h>
#include <time.h>
#include <sound_manager.h>
#include <bluetooth.h>
#include <sensor.h> //Library for the sensor usage
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"
#include <math.h>

//interval in millisecond for accelerometer
#define af_interval 5// 2-->500HZ, 3-->333HZ, 4-->250HZ, 5-->200HZ 10-->100HZ
//interval in millisecond for gyroscope
#define gf_interval 5

//movements predicted counter
static int movment_counter = 0;
//prediction factor (if it is low it will be very early)
static float prediciton_factor = 7;
//min duration for a movement ms
static float release_time = 150;
//last move time stamp ms
static float last_timestamp = 0;

//opt
static float last_x = 0;
static float last_z = 0;
//counter of recorded value accelerometer
static int counter_a = 0;
static int pressed = 0;
static int record = 0;
//d_ax = a(x0+T)-a(x0) where T is the sampling period so,a(x0+T)=a(x1)
static float d_ax = 0;
static float d_gz = 0; //
int flag_lag = 0;

// Variables for audio-playback
ALsizei size, freq;
ALenum format;
ALvoid *data;
ALboolean loop = AL_FALSE;
drwav* pWav;
ALCdevice *device;
ALCenum error;
ALCcontext *context;
ALfloat listenerOri[] = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };
ALuint source;
ALuint buffer;
//static char* filename="/opt/usr/media/Music/sample.wav";
//static char* filename = "/opt/usr/media/Music/Ludwig-Snare-C.wav";
static char* filename = "/opt/usr/media/Music/tick.wav";
static int *channels;
int32_t *pSampleData;
char* source_state;

float hop_start = 0;
float hop_end = 100;

//ll to float conversion factor
#define LLF_CONVERSION 10000000,0

////////////////////////////for BLE

#define LE_INITIAL_BUF_SIZE 2

const char *bt_server_address = "B8:27:EB:6A:72:33";
const char *mac_address = "7C:04:D0:C5:A3:61";

const char *service_uuid = "180A";
bt_gatt_service_type_e type = BT_GATT_SERVICE_TYPE_PRIMARY;

char *response = "0";
char *tick_response[] = { "1", "2", "3", "4", "5" };
char *lastResponse = "0";

//client handle
bt_gatt_client_h client = NULL;

//server handle
static bt_gatt_server_h gattServer = NULL;

//service handle
static bt_gatt_h gattSvc = NULL;

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

const char *statusIdle = "idle";
const char *statusListen = "listen";
const char *statusSuspend = "suspend";
static char *sensorStatus;
static long startingPoint = 0;
const long listeningTime = 100;
const long suspendingTime = 200;

static long recordAccCount = 0;
static long recordGyroCount = 0;

static float recordAccX[20];
static float recordGyroX[20];

static float recordAccY[20];
static float recordGyroY[20];

static float recordAccZ[20];
static float recordGyroZ[20];

//typedef struct appdata {
//	Evas_Object *win;
//	Evas_Object *conform;
//	Evas_Object *label;
//	Evas_Object *label0; /* write request value */
//	Evas_Object *label1; /* Current Movement counter */
//	Evas_Object *label2; /* Maximum acceleration value */
//} appdata_s;

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
	long sum = 0.0, mean, standardDeviation = 0.0;

	int i;

	for (i = 0; i < 10; ++i) {
		sum += data[i];
	}

	mean = sum / 10;

	for (i = 0; i < 10; ++i)
		standardDeviation += pow(data[i] - mean, 2);

	return sqrt(standardDeviation / 10);
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
static float *Softmax(float x1, float x2, float x3) {
	float inputs[] = { x1, x2, x3 };
	float softmax[] = { 0, 0, 0 };
	float sum = 0;

	for (int i = 0; i < 3; i++) {
		sum += exp(inputs[i]);
	}

	for (int i = 0; i < 3; i++) {
		softmax[i] = exp(inputs[i]) / sum;
	}
	return softmax;
}

//Trained model in code
static void expression(void *data, float variable_1, float variable_2,
		float variable_3, float variable_4, float variable_5, float variable_6,
		float variable_7, float variable_8, float variable_9, float variable_10,
		float variable_11, float variable_12, float variable_13,
		float variable_14, float variable_15, float variable_16,
		float variable_17, float variable_18, float variable_19,
		float variable_20, float variable_21, float variable_22,
		float variable_23, float variable_24) {
	appdata_s *ad = data;
	char buf[PATH_MAX];
	float scaled_variable_1 = (variable_1 - 12.6703) / 5.01258;
	float scaled_variable_2 = (variable_2 - 3.93215) / 8.60969;
	float scaled_variable_3 = (variable_3 - 19.4126) / 6.76107;
	float scaled_variable_4 = (variable_4 - 92.8492) / 52.795;
	float scaled_variable_5 = (variable_5 + 18.6896) / 80.342;
	float scaled_variable_6 = (variable_6 - 21.0222) / 75.9633;
	float scaled_variable_7 = (variable_7 + 1.07827) / 3.64859;
	float scaled_variable_8 = (variable_8 + 9.78093) / 6.50965;
	float scaled_variable_9 = (variable_9 - 0.370732) / 11.6911;
	float scaled_variable_10 = (variable_10 + 94.02) / 83.373;
	float scaled_variable_11 = (variable_11 + 315.636) / 108.635;
	float scaled_variable_12 = (variable_12 + 108.239) / 110.417;
	float scaled_variable_13 = (variable_13 - 109.367) / 51.1144;
	float scaled_variable_14 = (variable_14 + 44.2812) / 162.789;
	float scaled_variable_15 = (variable_15 - 183.716) / 190.547;
	float scaled_variable_16 = (variable_16 - 24.6541) / 1123.16;
	float scaled_variable_17 = (variable_17 + 3853.52) / 1811.63;
	float scaled_variable_18 = (variable_18 + 1034.69) / 1866.33;
	float scaled_variable_19 = (variable_19 - 3.05721) / 2.11778;
	float scaled_variable_20 = (variable_20 - 2.38248) / 1.74392;
	float scaled_variable_21 = (variable_21 - 3.05765) / 1.96718;
	float scaled_variable_22 = (variable_22 - 33.5419) / 20.9815;
	float scaled_variable_23 = (variable_23 - 49.6667) / 31.0831;
	float scaled_variable_24 = (variable_24 - 20.9051) / 16.0562;

	float y_1_1 = Logistic(
			-1.18521 + 0.0701464 * scaled_variable_1
					+ 2.56027 * scaled_variable_2 + 0.6252 * scaled_variable_3
					- 1.00917 * scaled_variable_4 - 0.618316 * scaled_variable_5
					- 1.51263 * scaled_variable_6 + 0.537774 * scaled_variable_7
					- 0.0619793 * scaled_variable_8
					- 0.178235 * scaled_variable_9
					+ 0.0738867 * scaled_variable_10
					- 1.41077 * scaled_variable_11
					- 3.79314 * scaled_variable_12 - 1.0086 * scaled_variable_13
					+ 1.51671 * scaled_variable_14
					+ 0.628457 * scaled_variable_15
					- 0.086686 * scaled_variable_16
					- 0.203457 * scaled_variable_17
					- 2.71377 * scaled_variable_18
					- 0.508664 * scaled_variable_19
					+ 1.56511 * scaled_variable_20
					- 0.214423 * scaled_variable_21
					- 0.234193 * scaled_variable_22
					- 0.130027 * scaled_variable_23
					+ 1.1992 * scaled_variable_24);
	float y_1_2 = Logistic(
			0.74521 - 0.117051 * scaled_variable_1 - 2.76736 * scaled_variable_2
					- 0.931835 * scaled_variable_3 + 1.52009 * scaled_variable_4
					+ 0.881154 * scaled_variable_5 + 1.92887 * scaled_variable_6
					- 0.640522 * scaled_variable_7 + 0.48555 * scaled_variable_8
					- 0.652122 * scaled_variable_9
					- 0.341161 * scaled_variable_10
					+ 0.929843 * scaled_variable_11
					+ 3.3963 * scaled_variable_12 + 1.18705 * scaled_variable_13
					- 2.29742 * scaled_variable_14
					+ 0.153115 * scaled_variable_15
					- 0.58605 * scaled_variable_16
					+ 0.832215 * scaled_variable_17
					+ 3.12628 * scaled_variable_18
					+ 0.611118 * scaled_variable_19
					- 1.60204 * scaled_variable_20
					- 0.270977 * scaled_variable_21
					+ 0.657833 * scaled_variable_22
					- 0.0960526 * scaled_variable_23
					- 1.41523 * scaled_variable_24);
	float y_1_3 = Logistic(
			3.29255 - 0.593192 * scaled_variable_1 + 1.00577 * scaled_variable_2
					- 0.406019 * scaled_variable_3
					- 0.927232 * scaled_variable_4 - 1.8474 * scaled_variable_5
					+ 0.499465 * scaled_variable_6
					- 0.428867 * scaled_variable_7 - 1.86737 * scaled_variable_8
					- 0.363737 * scaled_variable_9
					- 1.29025 * scaled_variable_10
					- 2.41469 * scaled_variable_11
					- 0.985284 * scaled_variable_12
					- 1.90178 * scaled_variable_13
					- 0.202464 * scaled_variable_14
					+ 0.0164118 * scaled_variable_15
					- 1.36293 * scaled_variable_16
					- 2.14009 * scaled_variable_17
					- 0.219123 * scaled_variable_18
					- 0.834949 * scaled_variable_19
					+ 1.39851 * scaled_variable_20
					+ 0.708755 * scaled_variable_21
					+ 0.149786 * scaled_variable_22
					+ 1.04075 * scaled_variable_23
					+ 1.33764 * scaled_variable_24);

	float non_probabilistic_variable_25 = Logistic(
			0.14876 - 2.11316 * y_1_1 + 3.59279 * y_1_2 - 10.2615 * y_1_3);
	float non_probabilistic_variable_26 = Logistic(
			-8.42531 - 9.62206 * y_1_1 + 2.70845 * y_1_2 + 11.6224 * y_1_3);
	float non_probabilistic_variable_27 = Logistic(
			2.04104 + 8.62543 * y_1_1 - 5.90975 * y_1_2 - 3.37696 * y_1_3);
	float softmax[3];
	float *pointer;
	int i;

	pointer = Softmax(non_probabilistic_variable_25,
			non_probabilistic_variable_26, non_probabilistic_variable_27);

	float variable_25 = *(pointer);
	float variable_26 = *(pointer + 1);
	float variable_27 = *(pointer + 2);

	char *gesture;
	float max = fmaxf(fmaxf(variable_25, variable_26), variable_27);

	if (max == variable_25) {
		gesture = "0";
	} else if (max == variable_26) {
		gesture = "1";
	} else if (max == variable_27) {
		gesture = "2";
	}

	changeCharaValue(gesture);
	sprintf(buf, "*s", gesture);
	dlog_print(DLOG_INFO, LOG_TAG, buf);
}

/************************************************************************************/
//------------------------ SOUND REPRODUCTION FUNCTIONS -----------------------------
static int read_wave() {
	pWav = drwav_open_file(filename);
	dlog_print(DLOG_DEFAULT, "WMS", "Not a relevant event");

	if (pWav == NULL) {
		return -1;
	}

	pSampleData = (int32_t*) malloc(
			(size_t) pWav->totalSampleCount * sizeof(int32_t));
	drwav_read_s32(pWav, pWav->totalSampleCount, pSampleData);

	// At this point pSampleData contains every decoded sample as signed 32-bit PCM.

	drwav_close(pWav);
	return 0;
}

/**
 * load wave
 */
static void init_base_player(void *data, Evas_Object *obj, void *event_info) {
	if (read_wave() == 0) {
		char name[PATH_MAX];
		sprintf(name, "c:%d :%d %d", (pWav->channels), pWav->sampleRate,
				pWav->bitsPerSample);
		elm_object_text_set(obj, name);
	} else {
		elm_object_text_set(obj, "ko");
	}
}

/**
 * INIT OPEN AL - init buffers and sources
 */
static int init_audio_reproduction() {
	//dev opening
	device = alcOpenDevice(NULL);
	if (!device) {
		return -1;
	}
	// Create context
	context = alcCreateContext(device, NULL);
	if (context == NULL) {
		alcCloseDevice(device);
		return -2;
	}

	// Set active context
	if (!alcMakeContextCurrent(context)) {
		alcDestroyContext(context);
		alcCloseDevice(device);
		return -3;
	}

	// Request a source name
	alGenSources((ALuint) 1, &source);

	// Set the default volume
	alSourcef(source, AL_GAIN, 1);

	// Set the default position of the sound
	alSource3f(source, AL_POSITION, 0, 0, 0);

	// Request a buffer name
	alGenBuffers(1, &buffer);

	ALuint frequency = 44100; //pWav->sampleRate
	ALenum format = AL_FORMAT_STEREO16;
	int datasize = 50000; //data

	// Specify sample data using alBufferData
	int c = 1;
	alBufferData(buffer, format, pSampleData, datasize, 22005);
	return 1;
}
static void play_sound() {
	// Function: _on_click1()
	// Source specifies the current buffer object
	alSourcei(source, AL_BUFFER, buffer);
	// Change the state to play
	alSourcePlay(source);
}

/*******************************************************************************************/

/*********************************bluetooth low energy**************************************/
void init_bt() {

	bt_error_e ret;

	ret = bt_initialize();
	if (ret != BT_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG, "[bt_initialize] failed.");

		return;
	}
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
void changeRead() {
	if (lastResponse == NULL || lastResponse == "05") {
		response = "01";
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
	ret = ret = bt_gatt_get_value(gatt_handle, &value, &len);
	bt_gatt_server_send_response(request_id, BT_GATT_REQUEST_TYPE_READ, offset,
			BT_ERROR_NONE, value, len);
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

	dlog_print(DLOG_INFO, LOG_TAG, "write the value: %s", str);

//	ret = bt_gatt_set_value(gatt_handle, value, len);
//	if (ret == BT_ERROR_NONE)
//		dlog_print(DLOG_INFO, LOG_TAG, "write Success");

//	ret = bt_gatt_server_notify_characteristic_changed_value(gattChara,
//			ServerNotificationSentCB, NULL, NULL);
//	if (ret != BT_ERROR_NONE) {
//		char* err;
//		err = get_error_message(ret);
//		dlog_print(DLOG_ERROR, LOG_TAG,
//				"send notification callback failed. err = %s", err);
//	} else {
//		dlog_print(DLOG_INFO, LOG_TAG, "send notification callback Succeed");
//	}
	char *response_value = value;

	bt_gatt_server_send_response(request_id, BT_GATT_REQUEST_TYPE_WRITE, offset,
			BT_ERROR_NONE, response_value, len);
//	dlog_print(DLOG_INFO, LOG_TAG, value);
//	float interval = receive_time - send_time;
//	dlog_print(DLOG_INFO, LOG_TAG, "Send-Receive Interval:  %f ", interval);
//	sprintf(buf, "R: %5.0f", interval);
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
	elm_object_text_set(ad->label0, buf);
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
	dlog_print(DLOG_INFO, LOG_TAG, "Create Service");
	int ret = bt_gatt_server_initialize();
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
	dlog_print(DLOG_INFO, LOG_TAG, "Change the value");
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
	dlog_print(DLOG_INFO, LOG_TAG, "Change the value %s", response);

//	changeAdvertisingServiceData(response);
	char c[10];
	sprintf(c, "%.f", get_current_millis());
	dlog_print(DLOG_INFO, LOG_TAG, c);
//	response = c;
//	strcat(response, c);

	response = gesture;

	ret = bt_gatt_set_value(gattChara, response, strlen(response));
	if (ret == BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "Success");

	send_time = get_current_millis();
	ret = bt_gatt_server_notify_characteristic_changed_value(gattChara,
			ServerNotificationSentCB, NULL, NULL);
	if (ret != BT_ERROR_NONE) {
		char* err;
		err = get_error_message(ret);
		dlog_print(DLOG_ERROR, LOG_TAG,
				"send notification callback failed. err = %s", err);
	} else {
		dlog_print(DLOG_INFO, LOG_TAG, "send notification callback Succeed");
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

/**
 * the callback from accelerometer
 *
 */
//static void _new_sensor_value(sensor_h sensor, sensor_event_s *sensor_data,
//		void *user_data) {
//	char buf[PATH_MAX];
//	appdata_s *ad = (appdata_s*) user_data;
//	if (flag_lag == 0) {
//		hop_start = get_current_millis();
//		flag_lag = 1;
//	} else {
//		flag_lag = 0;
//		hop_end = get_current_millis();
//		sprintf(buf, "lag :%.0f ", (hop_end - hop_start));
//		elm_object_text_set(ad->label2, buf);
//	}
//	float current_x = sensor_data->values[0];
////int j=0;
//	if (sensor_data->value_count < 3)
//		return;
//
//	sprintf(buf, "X : %0.1f / Y : %0.1f / Z : %0.1f", sensor_data->values[0],
//			sensor_data->values[1], sensor_data->values[2]);
////set the label text
//	elm_object_text_set(ad->label1, buf);
////update the derivative
//	d_ax = current_x - last_x;
////slope prediction
//	if (d_ax > prediciton_factor
//			&& ((last_timestamp + release_time) < get_current_millis())) {
//		last_timestamp = get_current_millis();
//		play_sound();
//
////		send_time = get_current_millis();
//		changeCharaValue();
//		movment_counter++;
//	} else {
//		sprintf(buf, "X :%d ", movment_counter);
//		elm_object_text_set(ad->label1, buf);
//	}
//	last_x = current_x;
//}
static void _new_accelerometer_value(sensor_h sensor,
		sensor_event_s *sensor_data, void *user_data) {
	char buf[PATH_MAX];
	appdata_s *ad = (appdata_s*) user_data;
	if (sensor_data->value_count < 3)
		return;

	char* sensorName = "accelerometer";

	float x = (sensor_data->values[0]);
	float y = (sensor_data->values[1]);
	float z = (sensor_data->values[2]);

//	get the square of Euclidean Distance of X,Y, and Z
	long sed = x * x + y * y + z * z;

	if (strcmp(sensorStatus, statusIdle) == 0) {
		if (sed > 250) {
			startingPoint = (long) get_current_millis();
			sensorStatus = statusListen;

			recordAccCount = 0;
			recordGyroCount = 0;
		}
	} else if (strcmp(sensorStatus, statusListen) == 0) {
		if (get_current_millis() - startingPoint >= listeningTime) {
			sensorStatus = statusSuspend;

//			Data segmentation of XYZ data

//			Sum of all 6 array (for arrays that have same size, sum shows the same feature as mean with less computation)
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

			expression(ad, maxminsumAccX[0], maxminsumAccY[0], maxminsumAccZ[0],
					maxminsumGyroX[0], maxminsumGyroY[0], maxminsumGyroZ[0],
					maxminsumAccX[1], maxminsumAccY[1], maxminsumAccZ[1],
					maxminsumGyroX[1], maxminsumGyroY[1], maxminsumGyroZ[1],
					maxminsumAccX[2], maxminsumAccY[2], maxminsumAccZ[2],
					maxminsumGyroX[2], maxminsumGyroY[2], maxminsumGyroZ[2],
					SDAccX, SDAccY, SDAccZ, SDGyroX, SDGyroY, SDGyroZ);
		}
		if (recordAccCount < 20) {
			recordAccX[recordAccCount] = x;
			recordAccY[recordAccCount] = y;
			recordAccZ[recordAccCount] = z;
			recordAccCount++;
		}

	} else if (strcmp(sensorStatus, statusSuspend) == 0) {
		if (get_current_millis() - startingPoint
				>= listeningTime + suspendingTime) {
			sensorStatus = statusIdle;
		}
	}
}
static void _new_gyroscope_value(sensor_h sensor, sensor_event_s *sensor_data,
		void *user_data) {
	char buf[PATH_MAX];
	appdata_s *ad = (appdata_s*) user_data;
	if (sensor_data->value_count < 3)
		return;

	char* sensorName = "gyroscope";
	long x = (long) (sensor_data->values[0]);
	long y = (long) (sensor_data->values[1]);
	long z = (long) (sensor_data->values[2]);

	//	get the square of Euclidean Distance of X,Y, and Z
	long sed = x * x + y * y + z * z;

	//Record data is in listen status
	if (strcmp(sensorStatus, statusListen) == 0) {
		if (recordGyroCount < 20) {
			recordGyroX[recordGyroCount] = x;
			recordGyroY[recordGyroCount] = y;
			recordGyroZ[recordGyroCount] = z;
			recordGyroCount++;
		}
	}
}

static void start_accelerator_sensor(appdata_s *ad, int af) {
	sensor_error_e err = SENSOR_ERROR_NONE;
	sensor_get_default_sensor(SENSOR_ACCELEROMETER, &sensor_info.sensor);
	err = sensor_create_listener(sensor_info.sensor,
			&sensor_info.sensor_listener);
	sensor_listener_set_event_cb(sensor_info.sensor_listener, af,
			_new_accelerometer_value, ad);
	sensor_listener_start(sensor_info.sensor_listener);

	sensorStatus = statusIdle;
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
//static void changeAlgoTo(char *algo, appdata_s *ad) {
//	if (strcmp(algo, "algo1") == 0) {
////		use the algo from Andrea
//		prediciton_factor = 7;
//		release_time = 150;
//		stop_gyroscope_sensor(ad);
//		start_accelerator_sensor(ad, af_interval);
//	} else if (strcmp(algo, "algo2") == 0) {
////		use the algo from dipster
//		prediciton_factor = 35;
//		release_time = 170;
//		stop_accelerator_sensor(ad);
//		start_gyroscope_sensor(ad, gf_interval);
//	}
//}
/* Button click event function */
static void btn_clicked_init_max_acc_value(void *data, Evas_Object *obj,
		void *event_info) {
	init_base_player(data, obj, event_info);
	init_audio_reproduction();
	appdata_s *ad = (appdata_s*) data;
	char buf[PATH_MAX];
	if (pressed == 0) {
		pressed = 1;
		//update button label
		sprintf(buf, "-");
		elm_object_text_set(ad->label2, buf);
		elm_object_text_set(obj, "Re-init");
		//init the audio
	} else {
		pressed = 0;
		//update button label
		elm_object_text_set(obj, "PLAY");
	}
}
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

	/* First label (for the sensor support) */
	ad->label0 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label0, "-");
	my_box_pack(box, ad->label0, 1.5, 1.0, 0.5, -1.0);

	/* Second label (for the current Movemenr counter) */
	ad->label1 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label1, "-");
	my_box_pack(box, ad->label1, 1.0, 1.0, 0.5, -1.0);

	/* Button */
	Evas_Object *btn = elm_button_add(ad->conform);
	elm_object_text_set(btn, "START");
	evas_object_smart_callback_add(btn, "clicked",
			btn_clicked_init_max_acc_value, ad);
	my_box_pack(box, btn, 1.0, 1.0, -1.0, -1.0);

	/* Button */
	Evas_Object *btn2 = elm_button_add(ad->conform);
	elm_object_text_set(btn2, "Exit");
	evas_object_smart_callback_add(btn2, "clicked", exit_tizen, ad);
	my_box_pack(box, btn2, 1.0, 1.0, -1.0, -1.0);

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
