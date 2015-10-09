/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include "nsh.h"
#include "nsh_console.h"
#include "nuttx/arch.h"
#include "nuttx/irq.h"
#include <nuttx/hires_tmr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The returned value should be zero for success or TRUE or non zero for
 * failure or FALSE.
 */

typedef int (*exec_t)(void);

struct print_arg_s {
    FAR struct nsh_vtbl_s *vtbl;
    bool csv;
    uint32_t total_time;
};

struct perf_track_settings {
    bool start;
    bool stop;
    bool comma_seperated_value;
    bool set_load;
    unsigned char load_percent;
    bool stop_load;
    bool timer_test;
};

static bool g_active_cpu_load = false;
static unsigned char target_load_value;

#define DELTA_TIME(start_time, curr_time)   \
    ((curr_time > start_time)?(curr_time - start_time):((0xffffffff - start_usec) + curr_time))

#define IDLE_LOAD(load_percent)   \
        ((load_percent > 100)?(0):(100 - load_percent))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_USEC_MEASURE_PERF)

/************************************************************************
 * Name: print_irq_data
 *
 * Description:
 *   Function pointer is passed to irq_perf_foreach such that every
 *   interrupt is enumerated and named
 *
 *   Function must match the "irq_foreach_t" type
 *
 * Inputs:
 *   irq - from tsb_irq_foreach
 *   *name - string for irq
 *   time - time logged for irq
 *   arg - pointer to print_arg_s
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void print_irq_data(int irq, FAR const char *name, uint32_t time, FAR void *arg)
{
    struct print_arg_s *print_arg;

    /* print data as comma separated values */
    print_arg = (struct print_arg_s*)arg;

    if(time > 0) {
        if(print_arg->csv) {
            nsh_output(print_arg->vtbl, "%d,%s,%d\r\n",
                irq,
                name,
                time);
        }
        else {
            /* check for timer rollover */
            if(time == 0xffffffff) {
                /* Add a half a percent for rounding
                */
               nsh_output(print_arg->vtbl, "%3d | %20s | %25s |\r\n",
                   irq,
                   name,
                   "     Timer Rollover   ");
            }
            else {
               /* Add a half a percent for rounding
                */
               nsh_output(print_arg->vtbl, "%3d | %20s | %10d | %14d%% |\r\n",
                   irq,
                   name,
                   time,
                   ((time*100)+(print_arg->total_time/200))/print_arg->total_time);
            }
        }
    }
}

/************************************************************************
 * Name: print_sched_data
 *
 * Description:
 *   Function pointer is passed to sched_perf_foreach such that every
 *   tcb is enumerated and named
 *
 *   Function must match the "sched_perf_foreach_t" type
 *
 * Inputs:
 *   pid - pid from tcb
 *   *name - string for tcb
 *   time - time logged for tcb
 *   arg - pointer to print_arg_s
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void print_sched_data(pid_t pid, FAR const char *name, uint32_t time, FAR void *arg)
{
    struct print_arg_s *print_arg;

    /* print data as comma separated values */
    print_arg = (struct print_arg_s*)arg;

    if(time > 0) {
        if(print_arg->csv) {
            nsh_output(print_arg->vtbl, "%d,%s,%d\r\n",
                pid,
                name,
                time);
        }
        else {
            /* check for timer rollover */
            if(time == 0xffffffff) {
                /* Add a half a percent for rounding
                */
               nsh_output(print_arg->vtbl, "%3d | %20s | %25s |\r\n",
                   pid,
                   name,
                   "     Timer Rollover   ");
            }
            else {
               /* Add a half a percent for rounding
                */
               nsh_output(print_arg->vtbl, "%3d | %20s | %10d | %14d%% |\r\n",
                   pid,
                   name,
                   time,
                   ((time*100)+(print_arg->total_time/200))/print_arg->total_time);
            }
        }
    }
}


/************************************************************************
 * Name: parse_perf_track_args
 *
 * Description:
 *   parse the arguments to the "pt" command line
 *
 * Inputs:
 *   vtbl - print re-director
 *   argc - number of arguments
 *   argv - sting pointer to each argument
 *   settings - returned data of the parsed arguments
 *
 * Return Value:
 *   OK - found a valid argument
 *   ERROR - no valid argument found
 *
 ************************************************************************/
int parse_perf_track_args(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv, struct perf_track_settings *settings)
{
    bool found_one_valid = false;
    int curr_arg;
    int ret_value = ERROR;

    memset(settings, 0, sizeof(struct perf_track_settings));

    if(argc < 2)
    {
        nsh_output(vtbl, "Performance Tracking, Invalid Arguments");
    }
    else {

        curr_arg = 1;

        while(curr_arg < argc)
        {
            if (strcmp(argv[curr_arg], "-start") == 0) {
                settings->start = true;
                found_one_valid = true;
            }
            else if (strcmp(argv[curr_arg], "-stop") == 0) {
                settings->stop = true;
                found_one_valid = true;
            }
            else if (strcmp(argv[curr_arg], "-setl") == 0) {
                if (sscanf(argv[curr_arg +1], "%u", &settings->load_percent) != 1) {
                    nsh_output(vtbl, "for command '-setl' no valid Load Number was found\n");
                }
                else if(settings->load_percent > 100) {
                    nsh_output(vtbl, "for command '-setl' no valid Load Number was found\n");
                }
                else {
                    nsh_output(vtbl, "Setting CPU load value to %d\n",settings->load_percent);
                    curr_arg++;
                    settings->set_load = true;
                    found_one_valid = true;
                }
            }
            else if (strcmp(argv[curr_arg], "-stopl") == 0) {
                settings->stop_load = true;
                found_one_valid = true;
            }
            else if (strcmp(argv[curr_arg], "-csv") == 0) {
                settings->comma_seperated_value = true;
            }
            else if (strcmp(argv[curr_arg], "-t") == 0) {
                settings->timer_test = true;
                found_one_valid = true;
            }
            curr_arg++;
        }

        if(found_one_valid) {
            //start take precedence over stop
            if(settings->start && settings->stop) {
                settings->stop = false;
            }

            //set load take precedence over stop load
            if(settings->set_load && settings->stop_load) {
                settings->stop_load = false;
            }

            ret_value = OK;
        }
        else {
            nsh_output(vtbl, "Performance Tracking, No valid arguments found\n");
        }
    }

    return ret_value;
}

/************************************************************************
 * Name: print_perf_tracking
 *
 * Description:
 *   Print the statistics on performance tracking
 *
 * Inputs:
 *   vtbl - print re-director
 *   comma_seperated_value - if true output data in comma separated format
 *     otherwise output as human readable format
 *
 * Return Value:
 *   0 - always
 *
 ************************************************************************/
void print_perf_tracking (FAR struct nsh_vtbl_s *vtbl, bool comma_seperated_value) {
    uint32_t total_perf_time;
    struct print_arg_s print_arg;

    total_perf_time = get_total_perf_time();

    print_arg.vtbl = vtbl;
    print_arg.total_time = total_perf_time;
    print_arg.csv = comma_seperated_value;

    if(!print_arg.csv) {
        nsh_output(vtbl, "Stop Performance Tracking\r\n");
    }

    /* Output interrupt timing */
    if(print_arg.csv) {
        nsh_output(vtbl, "TotalPerfTime,%d\r\n", total_perf_time);

        nsh_output(vtbl, "Irq,IrqName,TimeSpent\r\n");

        /* use callback function to print irq times */
        irq_perf_foreach(print_irq_data, (void*)&print_arg);
    }
    else
    {
        nsh_output(vtbl, "Total Perf Time Measurement: %d uSec\r\n", total_perf_time);

        nsh_output(vtbl, "Interrupt Timing:\r\n");
        nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
              "Irq","Irq Name","Time Spent","Percent of Time");
        nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
                        "---","--------------------","----------","---------------");
        /* use callback function to print irq times */
        irq_perf_foreach(print_irq_data, (void*)&print_arg);

        nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
                    "---","--------------------","----------","---------------");

        /* Output tcb timing */
        if(print_arg.csv) {
            nsh_output(vtbl, "Pid,Name,TimeSpent\r\n");

            /* use callback function to print tcb times */
            sched_perf_foreach(print_sched_data, (void*)&print_arg);
        }
        else
        {
            nsh_output(vtbl, "TCB Timing:\r\n");
            nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
                  "Pid","Name","Time Spent","Percent of Time");
            nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
                            "---","--------------------","----------","---------------");
            /* use callback function to print tcb times */
            sched_perf_foreach(print_sched_data, (void*)&print_arg);

            nsh_output(vtbl, "%3s | %20s | %10s | %15s |\r\n",
                        "---","--------------------","----------","---------------");
        }
    }
}

/************************************************************************
 * Name: load_cpu_task
 *
 * Description:
 *   Separate Nuttx task to load CPU performance
 *
 * Inputs:
 *   argc - not used
 *   argv - not used
 *
 * Return Value:
 *   none
 *
 ************************************************************************/
static int load_cpu_task(int argc, char **argv)
{
    volatile int i;
    uint32_t start_usec = 0;
    uint32_t sleep_usec = 0;
    uint32_t active_usec = 0;
    uint32_t delta_usec = 0;
    unsigned char current_load_value = 0;

    printf("Starting CPU Load Task\n");

    while(g_active_cpu_load) {

        /* change do not directly use target load because it
         * can change at any time
         */
        if( current_load_value != target_load_value ) {

            current_load_value = target_load_value;

            /* ensure value is a percentage */
            if(target_load_value > 100) {
                target_load_value = 100;
            }

            printf("Target CPU Load is: %d\n",current_load_value);
        }

        /* Let the CPU idle the inverse of the load value*/
        start_usec = hrt_getusec();
        if(IDLE_LOAD(current_load_value) > 0) {
            usleep(IDLE_LOAD(current_load_value));
        }

        sleep_usec = DELTA_TIME(start_usec,hrt_getusec());

        /* scale the CPU active time to match the actual time slept */
        active_usec = (100 * sleep_usec)/IDLE_LOAD(current_load_value);

        /* This loop burns up CPU time
         * poll timer till load value is met
         */
        delta_usec = 0;
        while(delta_usec < active_usec) {
            delta_usec = DELTA_TIME(start_usec,hrt_getusec());
        }
    }

  printf("Exiting CPU Load Task \n",i);

  return 0;
}

/************************************************************************
 * Name: start_load_task
 *
 * Description:
 *   Spawn a child task to create a dummy CPU load
 *
 * Inputs:
 *   vtbl - print re-director
 *
 * Return Value:
 *   none
 *
 ************************************************************************/
void start_load_task(FAR struct nsh_vtbl_s *vtbl, unsigned char load_percent)
{
    pid_t child_pid;

    if(!g_active_cpu_load) {
        g_active_cpu_load = true;

        target_load_value = load_percent;
        nsh_output(vtbl, "Setup target CPU load for: %d%%\n",target_load_value);

        /* Don't put the stack lower than 1024 toshiba processor
         * or you will get exception on freeing the stack during cleanup */
        child_pid = task_create("Load CPU Task", 50, 1024, load_cpu_task, NULL);
        if (child_pid < 0) {
            nsh_output(vtbl, "Load Task: task_create failed: %d\n", errno);
            g_active_cpu_load = false;
        }
        else {
            nsh_output(vtbl, "Created Load Task (pid=%d)\n", child_pid);
        }
    }
    else
    {
        if(load_percent != target_load_value) {
            nsh_output(vtbl, "Changing target CPU load to: %d%%)\n", load_percent);
            target_load_value = load_percent;
        }
        else {
            nsh_output(vtbl, "No change to target CPU load: %d%%)\n", load_percent);
        }
    }
  }

/************************************************************************
 * Name: perf_timer_test
 *
 * Description:
 *   Print performance clock times.
 *   Its intended that serial input will be time stamped on the development
 *   PC.
 *
 *   Comparing the performance values with the PC clock values gives and idea
 *   how accurate the performance clock is.
 *
 * Inputs:
 *   vtbl - print re-director
 *
 * Return Value:
 *   none
 *
 ************************************************************************/
void perf_timer_test(FAR struct nsh_vtbl_s *vtbl)
{
    uint32_t seconds_to_count;
    int i;
    struct timespec ts;
    uint32_t usec = 0;

    seconds_to_count = 1000;
    nsh_output(vtbl, "Count for %d seconds.\r\n\n", seconds_to_count);


    start_perf_track();

    for(i = 0; i < seconds_to_count; i++) {
      usec = hrt_getusec();
      hrt_gettimespec(&ts);

      if(i == 0) {
          continue;
      }

      nsh_output(vtbl, "%d \r\n", i);
      nsh_output(vtbl, "usec %d\r\n", usec);
      nsh_output(vtbl, "sec %d, nsec %d\r\n", ts.tv_sec, ts.tv_nsec);

      up_udelay(1000000);
    }

    stop_perf_track();
}

/************************************************************************
 * Name: cmd_perf_track
 *
 * Description:
 *   Main interface called from Nuttx shell.
 *
 * Inputs:
 *   vtbl - print re-director
 *   argc - argument count
 *   argv - arguments
 *
 * Return Value:
 *   OK always
 *
 ************************************************************************/
int cmd_perf_track(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
    struct perf_track_settings settings;

    if(parse_perf_track_args(vtbl, argc, argv, &settings) == OK) {
        if(settings.start) {
            nsh_output(vtbl, "Start Performance Tracking\r\n");

            start_perf_track();
        }

        if(settings.stop) {
            stop_perf_track();

            print_perf_tracking(vtbl, settings.comma_seperated_value);
        }

        if(settings.set_load) {
            nsh_output(vtbl, "Start Test CPU load:\r\n");

            start_load_task(vtbl, settings.load_percent);
        }

        if(settings.stop_load) {
            nsh_output(vtbl, "Stop Test CPU load:\r\n");

            g_active_cpu_load = false;
        }

        if(settings.timer_test) {
            nsh_output(vtbl, "Simple Time Test:\r\n");

            perf_timer_test(vtbl);
        }
    }

  return OK;
}

#endif
