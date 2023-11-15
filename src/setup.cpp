#include "setup.h"
#include "ei_at_handlers.h"
#include "ei_flash_nano_ble33.h"
#include "ei_device_nano_ble33.h"
#include "ei_microphone.h"
#include "ei_inertialsensor.h"
#include "ei_inertialsensor_rev2.h"
#include "ei_environmentsensor.h"
#include "ei_environmental_rev2.h"
#include "ei_interactionsensor.h"
#include "ei_camera.h"
#include "ei_run_impulse.h"
#include "firmware-sdk/ei_device_info_lib.h"

static ATServer *at;

mbed::DigitalOut led(LED1);

void print_memory_info()
{
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) ei_malloc(cnt * sizeof(mbed_stats_stack_t));

    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        ei_printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
    }
    free(stats);

    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    ei_printf("Heap size: %lu / %lu bytes\r\n", heap_stats.current_size, heap_stats.reserved_size);
}

void ei_main_init(void)
{
    EiDeviceNanoBle33 *dev = static_cast<EiDeviceNanoBle33*>(EiDeviceInfo::get_device());
    ei_printf("Hello from Edge Impulse on Arduino Nano 33 BLE Sense\r\n"
              "Compiled on %s %s\r\n",
              __DATE__,
              __TIME__);

    // we cannot flash anymore after hitting a hard fault, so let's wait 10 seconds
    // so we can attach to the serial port
    // for (size_t ix = 0; ix < 10; ix++) {
    //     ei_printf("Waiting to start: %lu\n", ix);
    //     led = !led;
    //     wait_ms(1000);
    // }

    if (ei_inertial_init() == false) {        
        ei_inertial_rev2_init();
    }
    
    if (ei_environment_init() == false) {
        ei_environment_rev2_init();
    }
    
    ei_interaction_init();
    ei_camera_init();

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
}

void ei_main()
{
    /* handle command comming from uart */
    char data = Serial.read();

    while (data != 0xFF) {
        at->handle(data);
        data = Serial.read();
    }
}
