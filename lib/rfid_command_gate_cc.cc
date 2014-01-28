/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rfid_command_gate_cc.h>
#include <gr_io_signature.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <float.h>
#include <cstdio>
#include <signal.h>

#include "debug.h"


// This alarm system is necessary because of the gnuradio scheduler.
// If no samples are passed through to the reader bloc, it never
// gets scheduled. Also, it cannot act as an independent source, because
// it will get scheduled EVERY time if it is not producing output. 
bool trigger_cycle = false;
static itimerval timer;
bool init_signal = false;
int last_timer_cycle = 0;

void 
catch_trigger_alarm (int sig){

  global_reader_state->num_timer_cycles++;

  if(global_reader_state->reader_state == INVENTORY){
	for(int i=0; i<global_reader_state->tag_num; i++){
		if(global_reader_state->tag_session_id_list[i] == false && global_reader_state->tag_sleep_cycle_table[i]>0){
			if(global_reader_state->tag_sleep_cycle_runtime[i]>=1){
				global_reader_state->tag_sleep_cycle_runtime[i]--;
				if(global_reader_state->tag_sleep_cycle_runtime[i]==0){
					int command[] = {i};
			                gr_message_sptr tag_q_msg = gr_make_message(0,
                        	                             	 		     sizeof(int),
			        	                                             0,
                        				                             (1) * sizeof(int));
			                memcpy(tag_q_msg->msg(), &command, 1 * sizeof(int));

			                d_tag_q_out->insert_tail(tag_q_msg);
					global_reader_state->tag_session_id_list[i] = true;
				}
			}
		}
	}
  }

  if(global_reader_state->trigger_cmd == true){
	if(global_reader_state->cur_cycle < global_reader_state->num_cycles){
		int command[] = {TIMER_FIRED};
	  	gr_message_sptr ctrl_msg = gr_make_message(0,
        	                                     sizeof(int),
                	                             0,
                        	                     (1) * sizeof(int));
	  	memcpy(ctrl_msg->msg(), &command, 1 * sizeof(int));
        	d_ctrl_out->insert_tail(ctrl_msg);
		//printf("Timer fired starting cycle - out loop. Finished %d cycle.\n", global_reader_state->cur_cycle);
	}
	else{
		global_reader_state->reader_running = false;
	}
	if(global_reader_state->cur_cycle == global_reader_state->num_cycles + 1){
      		printf("Last Cycle Started\n");
    	}
  }

  if(!TIMED_CYCLE_MODE){
    timeval t = {0,0};
    timer.it_interval = t;
    timer.it_value = t;
    signal(sig, catch_trigger_alarm);
    setitimer(ITIMER_REAL, &timer, NULL);
  }

}



rfid_command_gate_cc_sptr
rfid_make_command_gate_cc (int pw, int T1, int sample_rate)
{
  return rfid_command_gate_cc_sptr (new rfid_command_gate_cc (pw, T1, sample_rate));
}

rfid_command_gate_cc::rfid_command_gate_cc(int pw, int T1, int sample_rate)
  : gr_block("rfid_command_gate_cc",
	     gr_make_io_signature (1, 1, sizeof(gr_complex)),
	     gr_make_io_signature (1, 1, sizeof(gr_complex))),
    d_pw(pw),
    d_T1(T1),
    d_sample_rate(sample_rate)
  
{

  d_pw_num_samples = (int)((d_pw / (float) 1000000) * d_sample_rate);

  d_T1_num_samples =  (int)((d_T1 / (float)1000000) * d_sample_rate);
  global_reader_state->T1_value = d_T1;
  global_reader_state->us_per_rcv = (float)1000000 / d_sample_rate;
  printf("us Per Sample: %f Num samples per pulse:%d T1:%d\n", global_reader_state->us_per_rcv, d_pw_num_samples, d_T1_num_samples);
  init_global_reader_state();

  //Setup structure to hold samples. Used to track avg signal amplitude.
  d_window_length = (int)((1 / (1000000 / (float)d_sample_rate)) * AVG_WIN);
  d_window_samples = (float *)malloc(d_window_length * sizeof(float));
  for (int i = 0; i < d_window_length; i++){
    d_window_samples[i] = 0;
  }
  d_window_index = 0;
  d_avg_amp = 0;
 
  //Set up timer
  //timeval t = {READER_CYCLE_TIMER_RATE / 1000, (READER_CYCLE_TIMER_RATE % 1000) * 1000};
  timeval t = {READER_CYCLE_TIMER_RATE / 1000000, READER_CYCLE_TIMER_RATE};
  timer.it_interval = t;
  timer.it_value = t;

  signal(SIGALRM, catch_trigger_alarm);
  setitimer(ITIMER_REAL, &timer, NULL);

  buf = (float *) malloc(102400 * sizeof(float));
  d_num_pulses = 0;

  sleep_cycle = (int *) malloc(2000*sizeof(int));
  for(int i=0; i<2000; i++){
	sleep_cycle[i] = 0;
  }
  sleep_cycle_index = 0;

}

rfid_command_gate_cc::~rfid_command_gate_cc()
{}

inline bool 
rfid_command_gate_cc::is_positive_edge(float sample){
  return sample > d_thresh;
  
}
inline bool 
rfid_command_gate_cc::is_negative_edge(float sample){
  return sample < d_thresh;
  
}

int rfid_command_gate_cc::general_work(int noutput_items,
					gr_vector_int &ninput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items)
{
  const gr_complex * in = (const gr_complex *)input_items[0];
  gr_complex * out = (gr_complex * )output_items[0];
  int consumed = 0;
  int written = 0;

  float flt_value = 0;
  for(int i = 0; i < std::min(noutput_items , ninput_items[0]); i++){

    if(global_reader_state->command_gate_status == GATE_RESET){

      global_reader_state->command_gate_status = GATE_CLOSED;
      d_sample_count = 0;

      d_num_pulses = 0;
      
    }

    //Track average amplitude
    d_avg_amp = ((d_avg_amp * (d_window_length - 1)) + 
		 (d_avg_amp - d_window_samples[d_window_index]) + 
		 std::abs(in[i])) / d_window_length;       //Calculate avg by factoring out oldest value, adding newest
    d_window_samples[d_window_index] = std::abs(in[i]);    //Replace oldest value
    d_window_index = (d_window_index + 1) % d_window_length; //Increment point to oldest value

    d_thresh = d_avg_amp * THRESH_FRACTION;  //Threshold for detecting negative/positive edges

    consumed++;
    flt_value = std::abs(in[i]);
   
    if(global_reader_state->command_gate_status == GATE_CLOSED){

      global_reader_state->trigger_cmd = true;

      d_sample_count++;

      if (is_negative_edge(std::abs(in[i])) && !neg_edge_found){
	d_sample_count = 0;
	neg_edge_found = true;
      }
      
      if(neg_edge_found){
	if(is_positive_edge(std::abs(in[i]))){
	  if(d_sample_count > d_pw_num_samples / 2 && d_sample_count<100){
	    d_num_pulses++;
	    d_sample_count++;
	    neg_edge_found = false;
	  }
	  else{
	    //Too short to be a reader pulse
	    d_num_pulses = 0;
	  }
	  d_sample_count = 0;
	}
      }
    
      /*if(d_sample_count == (d_T1_num_samples / 4) * 3){
	//Calculate noise power
	int start_index = d_window_index - (d_T1_num_samples / 4) - 1; //Look back in buffer. 
	float * buffer = (float *)malloc((d_T1_num_samples / 4) * sizeof(float));
	if (start_index < 0){
	  start_index = d_window_length + start_index;  //Counting in from the end.
	}
	for(int j = 0; j < d_T1_num_samples / 4; j++){

	  buffer[j] = std::abs(d_window_samples[(j + start_index) % d_window_length]);
	  
	}
	calc_signal_stats(buffer, d_T1_num_samples / 4, &global_reader_state->max_pwr, &global_reader_state->min_pwr, &global_reader_state->avg_pwr, &global_reader_state->std_dev_noise);
	
      }*/

      //if(d_sample_count > d_T1_num_samples){
	//if(is_positive_edge(std::abs(in[i])) && d_num_pulses >= 26){ // > 5 avoids triggering on power down and TRCal
	if(is_positive_edge(std::abs(in[i])) && d_num_pulses >= 16 && d_avg_amp>2*pow(10,4)){
	  	global_reader_state->command_gate_status = GATE_OPEN;
	 	global_reader_state->decoder_status = DECODER_SEEK_PREAMBLE;
		d_num_pulses = 0;
	  	d_sample_count = 0;
		d_pass_count = 0;
		index=0;
		init_signal = true;
		global_reader_state->trigger_cmd = false;
	}
	//d_num_pulses = 0;
	//d_sample_count = 0;
      //}
      
      
    }
    if (global_reader_state->command_gate_status == GATE_OPEN){

      //Calculate signal power
      /*if(d_sample_count == d_T1_num_samples * 3){ //Should be well into signal. 

	int start_index = d_window_index - (d_T1_num_samples / 4) - 1; //Look back in buffer. 
	float * buffer = (float *)malloc((d_T1_num_samples / 4) * sizeof(float));
	if (start_index < 0){
	  start_index = d_window_length + start_index;  //Counting in from the end.
	}
	for(int j = 0; j < d_T1_num_samples / 4; j++){

	  buffer[j] = std::abs(d_window_samples[(j + start_index) % d_window_length]);
	  
	}
	calc_signal_stats(buffer, d_T1_num_samples / 4, &global_reader_state->max_pwr, &global_reader_state->min_pwr, &global_reader_state->avg_pwr, &global_reader_state->std_dev_signal);
      }*/

	d_pass_count++;
	if(d_pass_count>100){
		out[written++] = in[i];
		buf[index] = std::abs(in[i]);
		index++;
		//if(d_sample_count++ > global_reader_state->num_samples_to_ungate){
		/*if(d_pass_count > 50000){
                	global_reader_state->command_gate_status = GATE_RESET;
			global_reader_state->trigger_cmd = true;
		}*/
		if(index>=300){
			float avg = 0;
			float var = 0;
			for(int j=0; j<index; j++){
				avg += std::abs(buf[j]);
			}
			avg = avg/index;
			for(int j=0; j<index; j++){
				var += pow(std::abs(buf[j])-avg,2);
			}
			var = var/index;
			var = sqrt(var);
			//printf("var/avg:%f\n", var/avg);
			if(var/avg<0.005){
				//if(!global_reader_state->reader_control_timer){
					global_reader_state->command_gate_status = GATE_RESET;
					global_reader_state->trigger_cmd = true;
                			//d_pass_count = 0;
				//}
			}
			else{
				global_reader_state->trigger_cmd = false;
				if(init_signal && global_reader_state->reader_state == SLEEP_CYCLE_PROBLE){
					sleep_cycle[sleep_cycle_index] = global_reader_state->num_timer_cycles - last_timer_cycle + 1;
					sleep_cycle_index++;
					last_timer_cycle = global_reader_state->num_timer_cycles;
					if((global_reader_state->cur_cycle%1000==0) && (global_reader_state->tag_sleep_cycle_table[global_reader_state->tag_session_id] == -1)){
						int tmp = 0;
                                                for(int i=0; i<sleep_cycle_index; i++){
                                                        tmp += sleep_cycle[i];
                                                }
                                                tmp = (int) tmp/sleep_cycle_index;
						global_reader_state->tag_sleep_cycle_table[global_reader_state->tag_session_id] = (int) tmp;
                                                sleep_cycle_index = 0;
					}
					else if(sleep_cycle_index >= 10){
						int tmp = 0;
                                                for(int i=0; i<sleep_cycle_index; i++){
                                                        tmp += sleep_cycle[i];
                                                }
                                                tmp = (int) tmp/sleep_cycle_index;
                                                global_reader_state->tag_sleep_cycle_table[global_reader_state->tag_session_id] = (int) tmp;
                                                sleep_cycle_index = 0;
					}
					init_signal = false;
				}
			}
			index=0;
		}
	}
    }  

  }

  /*if(trigger_cycle){
    //printf("d_pass_count:%d\n", d_pass_count);
    //printf("Trigger cycle:%d\n", global_reader_state->cur_cycle);
    trigger_cycle = false;
    
    if(global_reader_state->cur_cycle < global_reader_state->num_cycles){
      gr_message_sptr ctrl_msg = gr_make_message(0,
					     sizeof(int),
					     0,
					     (1) * sizeof(int));
      int command[] = {TIMER_FIRED};
      memcpy(ctrl_msg->msg(), &command, 1 * sizeof(int));
      d_ctrl_out->insert_tail(ctrl_msg);
      //printf("Timer fired starting cycle. Finished %d cycle.\n", global_reader_state->cur_cycle);
      
    }
    if(global_reader_state->cur_cycle == global_reader_state->num_cycles + 1){
      printf("Last Cycle Started\n");
    }
  }*/

  consume_each(consumed);

  return written;
}

void 
rfid_command_gate_cc::calc_signal_stats(float * buffer, int len, double * max, double * min, double * avg, double * std_dev)
{

  *max = DBL_MIN;
  *min = DBL_MAX;
  double tmp_avg = 0;
  double tmp_std_dev = 0;

  for (int i = 0; i < len; i++){
    tmp_avg += buffer[i];
    if(buffer[i] > * max){
      *max = buffer[i];
    }
    if(buffer[i] < * min){
      *min = buffer[i];
    }
  }
  tmp_avg = tmp_avg / len;
  //Calculate STD_DEV
  for (int i = 0; i < len; i++){

    tmp_std_dev += std::pow((buffer[i] - tmp_avg)  ,2);
  }
  

  tmp_std_dev = tmp_std_dev / len;
  tmp_std_dev = sqrt(tmp_std_dev);
 
  
  *avg = tmp_avg;
  *std_dev = tmp_std_dev;


  
}


void
rfid_command_gate_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++){
    ninput_items_required[i] = noutput_items + history();
  }   
}

