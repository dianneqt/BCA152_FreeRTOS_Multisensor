#include "system_state.h"

// The one and only event group. Created in rtosObjectsInit().
EventGroupHandle_t systemEvents = NULL;