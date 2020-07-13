#ifndef LORA_H
#define LORA_H

#include "config/ttn.h"

void lora_job_send_status(byte[PAYLOAD_SIZE]);
void lora_job_init(void);

#endif