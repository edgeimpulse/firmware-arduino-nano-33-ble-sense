/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EDGE_IMPULSE_AT_COMMANDS_H_
#define _EDGE_IMPULSE_AT_COMMANDS_H_

#include "mbed.h"

// maximum number of commands
#ifndef EI_AT_MAX_CMDS
#define EI_AT_MAX_CMDS      32
#endif // EI_AT_MAX_CMDS

typedef struct {
    const char *cmd;
    const char *description;
    uint8_t arg_count;
    void *fn;
} ei_at_cmd_t;

// store all of the registered commands here
static ei_at_cmd_t ei_at_cmds[EI_AT_MAX_CMDS];
// next index where we'll insert data
static size_t ei_at_cmds_ix = 0;


/**
 * Add an AT command with zero arguments
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)()) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 0;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Add an AT command with one argument
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)(char*)) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 1;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Add an AT command with two arguments
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)(char*, char*)) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 2;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Add an AT command with three arguments
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)(char*, char*, char*)) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 3;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Add an AT command with four arguments
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)(char*, char*, char*, char*)) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 4;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Add an AT command with five arguments
 * @param cmd The command without AT+ in front of it (e.g. WIFI=)
 * @param description Human-friendly description (used to print the help file)
 * @param fn Function to be called when this AT command gets invoked
 */
bool ei_at_cmd_register(const char *cmd, const char *description, void (*fn)(char*, char*, char*, char*, char*)) {
    if (ei_at_cmds_ix + 1 == EI_AT_MAX_CMDS) return false;
    if (fn == NULL) return false;

    memset(&ei_at_cmds[ei_at_cmds_ix], 0, sizeof(ei_at_cmd_t));

    ei_at_cmds[ei_at_cmds_ix].cmd = cmd;
    ei_at_cmds[ei_at_cmds_ix].description = description;
    ei_at_cmds[ei_at_cmds_ix].arg_count = 5;
    ei_at_cmds[ei_at_cmds_ix].fn = (void*)fn;

    ei_at_cmds_ix++;
    return true;
}

/**
 * Print an overview of all registered AT commands
 */
void ei_at_cmd_print_info() {
    for (size_t ix = 0; ix < ei_at_cmds_ix; ix++) {
        ei_printf("AT+%s - %s", ei_at_cmds[ix].cmd, ei_at_cmds[ix].description);
        if (ei_at_cmds[ix].arg_count == 1) {
            ei_printf(" (1 parameter)");
        }
        else if (ei_at_cmds[ix].arg_count > 1) {
            ei_printf(" (%u parameters)", ei_at_cmds[ix].arg_count);
        }
        ei_printf("\n");
    }
}

/**
 * Execute an AT command
 * @param fn function pointer
 * @param arg_count arguments counter
 * @param args Arguments as string
 */
static void ei_at_cmd_exec(ei_at_cmd_t cmd, char *args[10]) {
    // now we need to cast the function back...
    switch (cmd.arg_count) {
        case 0: {
            void (*fn)() = (void (*)())cmd.fn;
            fn();
            break;
        }
        case 1: {
            void (*fn)(char*) = (void (*)(char*))cmd.fn;
            fn(args[1]);
            break;
        }
        case 2: {
            void (*fn)(char*,char*) = (void (*)(char*,char*))cmd.fn;
            fn(args[1], args[2]);
            break;
        }
        case 3: {
            void (*fn)(char*,char*,char*) = (void (*)(char*,char*,char*))cmd.fn;
            fn(args[1], args[2], args[3]);
            break;
        }
        case 4: {
            void (*fn)(char*,char*,char*,char*) = (void (*)(char*,char*,char*,char*))cmd.fn;
            fn(args[1], args[2], args[3], args[4]);
            break;
        }
        case 5: {
            void (*fn)(char*,char*,char*,char*,char*) = (void (*)(char*,char*,char*,char*,char*))cmd.fn;
            fn(args[1], args[2], args[3], args[4], args[5]);
            break;
        }
        default: {
            ei_printf("Unexpected arg count (%u)\n", cmd.arg_count);
            break;
        }
    }
}

/**
 * AT Command handler, invoke this function when a new line was passed in over the AT interface
 * This function dynamically allocates memory!
 * @param cmd_in Line to be interpreted
 * @returns true if handled, false if not handled
 */
bool ei_at_cmd_handle(const char *cmd_in) {
    // copy buffer
    char *cmd = (char*)calloc(strlen(cmd_in) + 1, 1);
    //ei_printf("cmd in: %s %d pointer: %X\r\n", cmd_in, strlen(cmd_in), cmd);
    memcpy(cmd, cmd_in, strlen(cmd_in));
//ei_printf("cmd handle: %s %d\r\n", cmd, strlen(cmd));
    // strip \r\n or spaces at the end...
    for (int ix = strlen(cmd) - 1; ix >= 0; ix--) {
        if (cmd[ix] == '\r' || cmd[ix] == '\n' || cmd[ix] == ' ') {
            cmd[ix] = '\0';
        }
        else {
            break;
        }
    }

    // grab the number of AT arguments
    uint8_t arg_count = 0;
    char *args[10]; // max 10 arguments, needs to be free'd at the end!
    size_t last_begin_ix = 0;
    char *cmd_ptr = cmd + 3; // this brings us right after the AT+
    bool first_argument = true;
    bool handled = false;

    // now do some command parsing...
    if (strlen(cmd) < 4 || cmd[0] != 'A' || cmd[1] != 'T' || cmd[2] != '+') {
        ei_printf("Not a valid AT command (%s)\n", cmd);
        goto clear_up;
    }

    // now loop through all the arguments until we see , or =
    for (size_t ix = 0; ix <= strlen(cmd_ptr); ix++) {
        bool valid_arg = false;

        // example: AT+INFO? or AT+WIFI=
        if (first_argument && (cmd_ptr[ix] == '=' || cmd_ptr[ix] == '?')) {
            valid_arg = true;
        }

        // after the first, just split by comma
        if (!first_argument && cmd_ptr[ix] == ',') {
            valid_arg = true;
        }

        // end of string, to handle the last argument
        if (ix == strlen(cmd_ptr)) {
            valid_arg = true;
        }

        if (valid_arg) {
            // if we're in the first argument and not at the end of the string I'd like to read the separation character
            int extra_byte = first_argument && ix != strlen(cmd_ptr) ? 1 : 0;
            // " around a value? remove them
            if (cmd_ptr[ix - 1] == '"' && cmd_ptr[last_begin_ix] == '"' && ix - last_begin_ix >= 2) {
                last_begin_ix += 1;
                extra_byte -= 1;
            }
            char *copy = (char*)calloc(ix - last_begin_ix + 1 + extra_byte, 1);
            if (!copy) {
                goto clear_up;
            }
            memcpy(copy, cmd_ptr + last_begin_ix, ix - last_begin_ix + extra_byte);
            args[arg_count] = copy;
            arg_count++;
            last_begin_ix = ix + 1;

            // last argument
            if (ix == strlen(cmd_ptr) && strcmp(copy, "") == 0) {
                arg_count--;
                free(copy);
            }

            // first entry needs to have = or ?
            if (first_argument) {
                first_argument = false;
            }
        }

        if (arg_count > sizeof(args) / sizeof(args[0])) {
            break;
        }
    }

    // alright, arg_count is always > 0 because the strlen check on cmd earlier, but double-check
    // just in case someone changes that code
    if (arg_count == 0) {
        ei_printf("Not a valid AT command (%s)\n", cmd);
        goto clear_up;
    }

    // so now we can see which command we should invoke
    for (size_t ix = 0; ix < ei_at_cmds_ix; ix++) {
        // matches?
        if (strcmp(ei_at_cmds[ix].cmd, args[0]) != 0) {
            continue;
        }

        if (ei_at_cmds[ix].arg_count != arg_count - 1) {
            continue;
        }

        ei_at_cmd_exec(ei_at_cmds[ix], args);

        handled = true;
        break;
    }

    // handled = handle_at_command(args, arg_count);

    if (!handled) {
        ei_printf("No handler for AT command (%s)\n", cmd);
        goto clear_up;
    }

    // @todo: when we terminate this thread, then this cleanup code is not run and we leak memory...
    // this is a problem with the current architecture, perhaps should move the
    // threading back to this file...
    // (or just switch to statically allocated memory)
clear_up:
    // free data
    for (uint8_t ix = 0; ix < arg_count; ix++) {
        free(args[ix]);
    }
    free(cmd);

    return true;
}

#endif // _EDGE_IMPULSE_AT_COMMANDS_H_
