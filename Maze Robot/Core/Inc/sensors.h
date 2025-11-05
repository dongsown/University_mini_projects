#ifndef SENSORS_H_
#define SENSORS_H_

#include "main.h"
#include <stdbool.h>

#define NUM_SENSORS 3

extern uint16_t sensor_distances[NUM_SENSORS];


void sensors_init(void);
void sensors_read_all(void);
uint16_t get_front_distance(void);
uint16_t get_left_distance(void);
uint16_t get_right_distance(void);


#endif /* SENSORS_H_ */