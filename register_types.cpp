#include "core/class_db.h"
#include "register_types.h"
#include "gdlm_sensor.h"

void register_gdleapmotionV2_types() {
	ClassDB::register_class<GDLMSensor>();
}

void unregister_gdleapmotionV2_types() {
	/* this space deliberately left blank */
}
