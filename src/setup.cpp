#include "setup.h"
#include "ei_device_nano_ble33.h"
#include "nano_fs_commands.h"
#include "at_cmd_repl_mbed.h"
#include "ei_microphone.h"
#include "ei_inertialsensor.h"
#include "ei_environmentsensor.h"
#include "ei_interactionsensor.h"
#include "ei_camera.h"

#include "ei_sampler.h"
#include "ei_run_impulse.h"


EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo*>(&EiDevice);
EventQueue main_application_queue;
static unsigned char repl_stack[8 * 1024];
static AtCmdRepl repl(&main_application_queue, ei_get_serial(), sizeof(repl_stack), repl_stack);

mbed::DigitalOut led(LED1);

void print_memory_info() {
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) malloc(cnt * sizeof(mbed_stats_stack_t));

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


#ifdef __MBED__
static bool ei_nano_fs_read_buffer(size_t begin, size_t length, mbed::Callback<void(uint8_t*, size_t)> data_fn) {
#else
static bool ei_nano_fs_read_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t)) {
#endif

    size_t pos = begin;
    size_t bytes_left = length;

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            return true;
        }

        int r = ei_nano_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            return false;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    return true;

}


void ei_main() {
    ei_printf("Edge Impulse standalone inferencing (Mbed)\n");

    // we cannot flash anymore after hitting a hard fault, so let's wait 10 seconds
    // so we can attach to the serial port
    // for (size_t ix = 0; ix < 10; ix++) {
    //     ei_printf("Waiting to start: %lu\n", ix);
    //     led = !led;
    //     wait_ms(1000);
    // }

    ei_inertial_init();
    ei_environment_init();
    ei_interaction_init();
    ei_camera_init();
    ei_nano_fs_init();
    

    // intialize configuration
    ei_config_ctx_t config_ctx = { 0 };
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function();
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();

    config_ctx.load_config = &ei_nano_fs_load_config;
    config_ctx.save_config = &ei_nano_fs_save_config;
    config_ctx.list_files = NULL;
    config_ctx.read_buffer = &ei_nano_fs_read_buffer;
    config_ctx.take_snapshot = &ei_camera_take_snapshot_encode_and_output;
    config_ctx.start_snapshot_stream = &ei_camera_start_snapshot_stream_encode_and_output;

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    }
    else {
        ei_printf("Loaded configuration\n");
    }

    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_normal);
    ei_at_cmd_register("RUNIMPULSEDEBUG=", "Run the impulse with extra (base64) debug output (USEMAXRATE?(y/n))", run_nn_debug);
    ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse continuously", run_nn_continuous_normal);

    repl.start_repl();
    main_application_queue.dispatch_forever();
}
