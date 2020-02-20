#include "vips.h"
#include <stdio.h>
#include <storage.h>
#include <time.h>
#include <stdlib.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <dlog.h>
#include <assert.h>
#include <sound_manager.h>
#include <bluetooth.h>
#include <sensor.h> //Library for the sensor usage
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

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
//counter of recorded value accelerometer
static int counter_a = 0;
static int pressed = 0;
static int record = 0;
//d_ax = a(x0+T)-a(x0) where T is the sampling period so,a(x0+T)=a(x1)
static float d_ax = 0;
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

char *response = "00";
char *tick_response[] = { "01", "02", "03", "04", "05" };
char *lastResponse = NULL;

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

typedef struct appdata {
	Evas_Object *win;
	Evas_Object *conform;
	Evas_Object *label;
	Evas_Object *label0; /* Whether the accelerator sensor is supported */
	Evas_Object *label1; /* Current acceleration value */
	Evas_Object *label2; /* Maximum acceleration value */
} appdata_s;

struct _sensor_info {
	sensor_h sensor; /* Sensor handle */
	sensor_listener_h sensor_listener; /* Sensor listener */
};
typedef struct _sensor_info sensorinfo_s;

static sensorinfo_s sensor_info;
static sensorinfo_s sensor_info_gyro;

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
	int ret;
	dlog_print(DLOG_INFO, LOG_TAG, "write the value");

	ret = bt_gatt_set_value(gatt_handle, value, len);
	if (ret == BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "write Success");

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
	char *response_value = value;
	bt_gatt_server_send_response(request_id, BT_GATT_REQUEST_TYPE_WRITE, offset,
			BT_ERROR_NONE, response_value, len);
}

void create_advertise() {
	dlog_print(DLOG_INFO, LOG_TAG, "Start advertise");

	static bt_advertiser_h advertiser = NULL;
	static bt_advertiser_h advertiser_list[3] = { NULL, };
	static int advertiser_index = 0;

	int manufacturer_id = 117;
	char *manufacture = NULL;
	char manufacture_0[] = { 0x0, 0x0, 0x0, 0x0 };
	char manufacture_1[] = { 0x01, 0x01, 0x01, 0x01 };
	char manufacture_2[] = { 0x02, 0x02, 0x02, 0x02 };
	char manufacture_3[] = { 0x03, 0x03, 0x03, 0x03 };
	char service_data[] = { 0x01, 0x02, 0x02 };
	const char *time_svc_uuid_16 = "1805";
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

	ret = bt_adapter_le_add_advertising_service_solicitation_uuid(advertiser,
			BT_ADAPTER_LE_PACKET_ADVERTISING, immediate_alert_svc_uuid_16);
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

void createService() {
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
			__bt_gatt_server_write_value_requested_cb, NULL);
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

void changeCharaValue() {
	int ret;
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
	dlog_print(DLOG_INFO, LOG_TAG, "Change the value");

	ret = bt_gatt_set_value(gattChara, response, 2);
	if (ret == BT_ERROR_NONE)
		dlog_print(DLOG_INFO, LOG_TAG, "Success");

	ret = bt_gatt_server_notify_characteristic_changed_value(gattChara,
			ServerNotificationSentCB, NULL, "01");
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
 * Shows if the sensor is supported and display the result in label0
 */
static void show_is_supported(appdata_s *ad) {
	char buf[PATH_MAX];
	bool is_supported = false;
	sensor_is_supported(SENSOR_ACCELEROMETER, &is_supported);
//sensor_is_supported(SENSOR_GYROSCOPE, &is_supported)
	sprintf(buf, "Acceleration sensor",
			is_supported ? "support" : "not support");
	elm_object_text_set(ad->label0, buf);
}

/**
 * the callback from accelerometer
 *
 */
static void _new_sensor_value(sensor_h sensor, sensor_event_s *sensor_data,
		void *user_data) {
	char buf[PATH_MAX];
	appdata_s *ad = (appdata_s*) user_data;
	if (flag_lag == 0) {
		hop_start = get_current_millis();
		flag_lag = 1;
	} else {
		flag_lag = 0;
		hop_end = get_current_millis();
		sprintf(buf, "lag :%.0f ", (hop_end - hop_start));
		elm_object_text_set(ad->label2, buf);
	}
	float current_x = sensor_data->values[0];
//int j=0;
	if (sensor_data->value_count < 3)
		return;

	sprintf(buf, "X : %0.1f / Y : %0.1f / Z : %0.1f", sensor_data->values[0],
			sensor_data->values[1], sensor_data->values[2]);
//set the label text
	elm_object_text_set(ad->label1, buf);
//update the derivative
	d_ax = current_x - last_x;
//slope prediction
	if (d_ax > prediciton_factor
			&& ((last_timestamp + release_time) < get_current_millis())) {
		last_timestamp = get_current_millis();
		//play
		/********************/
		//play_sound(ad,0);
		/********************/
		play_sound();

//		changeRead();
		changeCharaValue();
		movment_counter++;
		// hop_end=get_current_millis();
		// sprintf(buf, "lag :%.1f ",(hop_end-hop_start));
		// elm_object_text_set(ad->label2,buf);
	} else {
		sprintf(buf, "X :%d ", movment_counter);
		elm_object_text_set(ad->label1, buf);
	}
	last_x = current_x;
}

static void start_accelerator_sensor(appdata_s *ad, int af) {
	sensor_error_e err = SENSOR_ERROR_NONE;
	sensor_get_default_sensor(SENSOR_ACCELEROMETER, &sensor_info.sensor);
	err = sensor_create_listener(sensor_info.sensor,
			&sensor_info.sensor_listener);
	sensor_listener_set_event_cb(sensor_info.sensor_listener, af,
			_new_sensor_value, ad);
	sensor_listener_start(sensor_info.sensor_listener);
}

static void stop_accelerator_sensor(appdata_s *ad) {
	sensor_listener_stop(sensor_info.sensor_listener);
}

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
	evas_object_size_hint_weight_set(box, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);
	evas_object_size_hint_align_set(box, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);
	elm_object_content_set(ad->conform, box);
	evas_object_show(box);

	/* First label (for the sensor support) */
	ad->label0 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label0, "-");
	my_box_pack(box, ad->label0, 1.5, 0.0, -1.0, -1.0);

	/* Second label (for the current acceleration value) */
	ad->label1 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label1, "-");
	my_box_pack(box, ad->label1, 1.0, 1.0, -1.0, -1.0);

	/* Button */
	Evas_Object *btn = elm_button_add(ad->conform);
	elm_object_text_set(btn, "START");
	evas_object_smart_callback_add(btn, "clicked",
			btn_clicked_init_max_acc_value, ad);
	my_box_pack(box, btn, 1.0, 0.0, -1.0, -1.0);

	/* Third label (for the maximum value) */
	ad->label2 = elm_label_add(ad->conform);
	elm_object_text_set(ad->label2, "-");
	my_box_pack(box, ad->label2, 1.0, 1.0, 0.5, -1.0);

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
	createService();
	create_advertise();

	return true;
}

static void app_control(app_control_h app_control, void *data) {
	/* Handle the launch request. */
}

static void app_pause(void *data) {
	/* Take necessary actions when application becomes invisible. */
	appdata_s *ad = data;
	stop_accelerator_sensor(ad);
}

static void app_resume(void *data) {
	/* Take necessary actions when application becomes visible. */
	appdata_s *ad = data;
	start_accelerator_sensor(ad, af_interval);
}

static void app_terminate(void *data) {
	/* Release all resources. */
	appdata_s *ad = data;
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
