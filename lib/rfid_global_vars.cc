#include "rfid_global_vars.h"
#include <cstdio>
#include <stdlib.h>


reader_state * global_reader_state;

void init_global_reader_state(){
  
 
  if(!global_reader_state){
    global_reader_state = (reader_state *)malloc(sizeof(reader_state));
    //global_reader_state->tag_bit_vector = (char*) malloc(512 * sizeof(char));
    global_reader_state->tag_bit_vector = (char*) malloc(max_tag_response * sizeof(char));
    global_reader_state->command_gate_status = GATE_RESET;
    global_reader_state->num_bits_to_decode = 17;
    global_reader_state->num_bits_decoded = 0;
    global_reader_state->num_samples_per_pulse = NUM_SAMPLES_PER_PULSE;
    global_reader_state->num_samples_to_ungate = 0;

    global_reader_state->center_edge_found = false;
    global_reader_state->center_tracking = false;

    global_reader_state->tag_num = 10;

    global_reader_state->num_timer_cycles = 0;
    global_reader_state->tag_sleep_cycle_table = (int*) malloc(global_reader_state->tag_num*sizeof(int));
    global_reader_state->tag_sleep_cycle_runtime = (int*) malloc(global_reader_state->tag_num*sizeof(int));
    global_reader_state->tag_session_id_list = (bool*) malloc(global_reader_state->tag_num*sizeof(bool));

    global_reader_state->tag_throughput = (long double*) malloc(global_reader_state->tag_num*sizeof(long double));

    global_reader_state->trigger_cmd = false;
    global_reader_state->reader_control_timer = false;

    global_reader_state->tag_id_training = 0;

    global_reader_state->tag_training_round = 500;

    global_reader_state->reader_running = true;
  }
 
}
