#include <parkassist.h>

// utility functions

int64_t esp_millis() {
    return ( (int64_t)(esp_timer_get_time() / 1000));
}

void getPreferences();

void setPreferences();

