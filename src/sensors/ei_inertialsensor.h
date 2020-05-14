
#ifndef _EI_INERTIALSENSOR_H
#define _EI_INERTIALSENSOR_H

/* Include ----------------------------------------------------------------- */
#include "ei_sampler.h"

/** Number of axis used and sample data format */
typedef float sample_format_t;
#define N_AXIS_SAMPLED			3
#define SIZEOF_N_AXIS_SAMPLED	(sizeof(sample_format_t) * N_AXIS_SAMPLED)


/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_init(void);
bool ei_inertial_sample_start(sampler_callback callback, float sample_interval_ms);
bool ei_inertial_setup_data_sampling(void);

#endif