#ifndef __vips_H__
#define __vips_H__

#include <app.h>
#include <Elementary.h>
#include <system_settings.h>
#include <efl_extension.h>
#include <dlog.h>

typedef struct appdata {
	Evas_Object *win;
	Evas_Object *conform;
	Evas_Object *label;
	Evas_Object *label0; /* write request value */
	Evas_Object *label1; /* Current Movement counter */
	Evas_Object *label2; /* Maximum acceleration value */

	Evas_Object *ic;
} appdata_s;

struct Sample {
	float x;
	float y;
	float z;
};

struct output{
	float maxX;
	float maxZ;
};
static void changeAlgoTo(char *algo, appdata_s *ad);


void changeCharaValue(char *gesture);
#ifdef  LOG_TAG
#undef  LOG_TAG
#endif
#define LOG_TAG "vips"

#if !defined(PACKAGE)
#define PACKAGE "org.example.vips"

#endif

#endif /* __vips_H__ */
