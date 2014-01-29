/* See license.txt for license information. */

//******************************************************************************
//  Moo 1.1
//  Device: MSP430-2618
//
//  Version HW1.1_SW1.1
//
//  Pinout:
//             P1.1 = Data out signal
//             P1.2 = Data in signal
//             P1.3 = Data in enable
//
//             P2.0 = Supervisor In
//             P4.0 = Supervisor In
//
//
//	This is a partial implementation of the RFID EPCGlobal C1G2 protocol
//  for the Moo 1.1 hardware platform.
//
//	What's missing:
//        - SECURED and KILLED states.
//        - No support for WRITE, KILL, LOCK, ACCESS, BLOCKWRITE and BLOCKERASE
//          commands.
//        - Commands with EBVs always assume that the field is 8 bits long.
//        - SELECTS don't support truncation.
//        - READs ignore membank, wordptr, and wordcount fields. (What READs do
//          return is dependent on what application you have configured in step
//          1.)
//        - I sometimes get erroneous-looking session values in QUERYREP
//          commands. For the time being, I just parse the command as if the
//          session value is the same as my previous_session value.
//        - QUERYs use a pretty aggressive slotting algorithm to preserve power.
//          See the comments in that function for details.
//        - Session timeouts work differently than what's in table 6.15 of the
//          spec.  Here's what we do:
//            SL comes up as not asserted, and persists as long as it's in ram
//            retention mode or better.
//            All inventory flags come up as 'A'. S0's inventory flag persists
//            as long as it's in ram retention mode or better. S1's inventory
//            flag persists as long as it's in an inventory round and in ram
//            retention mode or better; otherwise it is reset to 'A' with every
//            reset at the top of the while loop. S2's and S3's inventory flag
//            is reset to 'A' with every reset at the top of the while loop.
//******************************************************************************

#include "moo.h"
#include "stdlib.h"

/*******************************************************************************
 ****************  Edit mymoo.h to configure this Moo  *************************
 ******************************************************************************/

#define CMD_BUFFER_SIZE 32 // max of 32 bytes rec. from reader

#define NUM_QUERY_BITS 24

volatile unsigned char cmd[CMD_BUFFER_SIZE+1]; // stored command from reader

volatile unsigned char* destorig = &cmd[0]; // pointer to beginning of cmd

// #pragma data_alignment=2 is important in sendResponse() when the words are
// copied into arrays.  Sometimes the compiler puts reply[0] on an odd address,
// which cannot be copied as a word and thus screws everything up.
#pragma data_alignment=2

// compiler uses working register 4 as a global variable
// Pointer to &cmd[bits]
volatile __no_init __regvar unsigned char* dest @ 4;

// compiler uses working register 5 as a global variable
// count of bits received from reader
volatile __no_init __regvar unsigned short bits @ 5;
unsigned short TRcal=0;

unsigned char delimiterNotFound = 0;
unsigned char TRext = 0;
unsigned short inInventoryRound = 0;

int i;

int counter = 0x00;

unsigned int tx_len = 0;
unsigned int sleep_time = 1;

unsigned int total_sleep_time = 1;
unsigned int total_sleep_time_pre = 1;
unsigned int total_tx_len = 0;
unsigned int total_tx_len_pre = 0;

unsigned int sleep_time_list[5] = {0,0,0,0,0};
unsigned int throughput_list[5] = {0,0,0,0,0};

char delimiter_found = 0x00;

#define MAX_TX_LEN 1000

// 232-1 261-2 241-3 252-4 260-5 263-6 258-7 251-8 248-9 230-10
// 249-11 254-12 239-13 238-14 259-15 265-16 244-17 223-18 242-19 208-20

#define id 0x10

volatile unsigned char datalog[]= {0x5C, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id,
id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id, id};

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;
  
  P1SEL = 0;
  P2SEL = 0;
  P8SEL = 0xC0;

  P1IE = 0;
  P1IFG = 0;
  P2IFG = 0;

  DRIVE_ALL_PINS
    
  if(!is_power_good())
    sleep();
  
  RECEIVE_CLOCK;
  TACTL = 0;
  asm("MOV #0000h, R9");
  
  setup_to_receive();  
  while(1){
    if (TAR > 0x256 || delimiterNotFound)
    {
      if(!is_power_good()) {
        sleep();
      }
      setup_to_receive();
    }
    
    if ( (bits == 14)  && ( ( cmd[0] & 0xFF ) == 0x10 ) )
    {
      TACCTL1 &= ~CCIE;
      TAR = 0;
      
      //voltage_down();
      tx_len = 50;
      
      for(i=0;i<10;i++){
        asm("NOP");
      }

      sendToReader(&datalog[0], tx_len);

      //P4OUT |= LED_POWER;
      //for(i=0; i<10000; i++){ asm("NOP");}
      //P4OUT &= ~LED_POWER;
      //for(i=0; i<10000; i++){ asm("NOP");}

      setup_to_receive();
    }
  }
  
  while(1)
  {
    P1OUT |= DEBUG_1_7;
    P1OUT &= ~DEBUG_1_7;
  
    sleep_time = 10;
    sleep_time_list[0] = sleep_time;
    throughput_list[0] = cal_throughput(sleep_time);
    
    sleep_time = sleep_time<<1;
    sleep_time_list[1] = sleep_time;
    throughput_list[1] = cal_throughput(sleep_time);
    
    sleep_time = sleep_time<<1;
    sleep_time_list[2] = sleep_time;
    throughput_list[2] = cal_throughput(sleep_time);
        
    while(1){
      if((throughput_list[2]<=throughput_list[1]) && (throughput_list[1]<=throughput_list[0])){
        
        P2OUT |= DEBUG_2_1;
        P2OUT &= ~DEBUG_2_1;
        
        sleep_time_list[0] = sleep_time_list[1];
        sleep_time_list[1] = sleep_time_list[2];
        throughput_list[0] = throughput_list[1];
        throughput_list[1] = throughput_list[2];

        if(sleep_time>16000){
          break;
        }
        
        sleep_time = sleep_time<<1;
        sleep_time_list[2] = sleep_time;
        throughput_list[2] = cal_throughput(sleep_time);
      }
      else if((throughput_list[2]>=throughput_list[1]) && (throughput_list[1]<=throughput_list[0])){
        sleep_time_list[4] = sleep_time_list[2];
        throughput_list[4] = throughput_list[2];
        
        while(1){
          sleep_time = (sleep_time_list[4]+sleep_time_list[0])>>1;
          sleep_time_list[2] = sleep_time;
          throughput_list[2] = cal_throughput(sleep_time);
          
          sleep_time = (sleep_time_list[2]+sleep_time_list[0])>>1;
          sleep_time_list[1] = sleep_time;
          throughput_list[1] = cal_throughput(sleep_time);
          
          sleep_time = (sleep_time_list[2]+sleep_time_list[4])>>1;
          sleep_time_list[3] = sleep_time;
          throughput_list[3] = cal_throughput(sleep_time);
          
          if(throughput_list[3]<=throughput_list[1]){
            sleep_time_list[0] = sleep_time_list[1];
            throughput_list[0] = throughput_list[1];
          }
          else{
            sleep_time_list[4] = sleep_time_list[3];
            throughput_list[4] = throughput_list[3];
          }
          
          if(sleep_time_list[4]-sleep_time_list[0]<=30){
            break;
          }
        }
        
        sleep_time = sleep_time_list[4];
        
        break;
      }
      else if((throughput_list[2]>=throughput_list[1]) && (throughput_list[1]>=throughput_list[0])){
        
        sleep_time_list[2] = sleep_time_list[1];
        sleep_time_list[1] = sleep_time_list[0];
        throughput_list[2] = throughput_list[1];
        throughput_list[1] = throughput_list[0];

        if(sleep_time==0){
          break;
        }
        
        sleep_time = sleep_time>>1;
        sleep_time_list[0] = sleep_time;
        throughput_list[0] = cal_throughput(sleep_time);
      }
    }
  
    P1OUT |= DEBUG_1_7;
    P1OUT &= ~DEBUG_1_7;    
    
    //sleep_time = 300;

    for(i=0;i<1000;i++)
    {     
      if(!is_power_good()){
        sleep();
      }
      if(sleep_time>=1){
        tx_sleep(sleep_time);
      }
  
      setup_to_receive();

      while(1){
        if (TAR > 0x256 || delimiterNotFound)
        {
          if(!is_power_good()) {
            sleep();
          }
          setup_to_receive();
        }
        
        if ( (bits == 14)  && ( ( cmd[0] & 0xFF ) == id) )
        {
          P2OUT |= DEBUG_2_2;
          P2OUT &= ~DEBUG_2_2;
          
          TACCTL1 &= ~CCIE;
          TAR = 0;
          
          voltage_down();
          tx_len = MAX_TX_LEN;
          
          for(i=0;i<10;i++){
            asm("NOP");
          }
    
          sendToReader(&datalog[0], tx_len);
          setup_to_receive();      
          break;
        }
      }
    }
  }

}

unsigned int cal_throughput(unsigned int sleep_time){
  unsigned int total_sleep_time = 0;
  unsigned int total_tx_len = 0;
  unsigned int counter = 0;
  unsigned int tx_len = 0;
  unsigned int sleep_time_current = sleep_time;
  unsigned int throughput = 0;
  
  for(i=0; i<1; i++){
    
    set_timer();
    
    if(!is_power_good()){
      sleep();
    }
    
    total_sleep_time = total_sleep_time + TBR;
    if(sleep_time>=1){
      tx_sleep(sleep_time_current);
    }
    
    total_sleep_time = total_sleep_time + sleep_time;
    set_timer();
    
    TACCTL1 &= ~CCIE;
    TAR = 0;    
    tx_len = MAX_TX_LEN;
    
    counter = 0;
    voltage_down();
    sendToReader(&datalog[0], tx_len);
    tx_len=tx_len-counter;
    
    total_sleep_time = total_sleep_time + TBR;
    total_tx_len = total_tx_len + tx_len;
  }
  
  throughput = total_sleep_time/total_tx_len;
  
  return throughput;
}

//************************** SETUP TO RECEIVE  *********************************
// note: port interrupt can also reset, but it doesn't call this function
//       because function call causes PUSH instructions prior to bit read
//       at beginning of interrupt, which screws up timing.  so, remember
//       to change things in both places.
static inline void setup_to_receive()
{
  _BIC_SR(GIE); // temporarily disable GIE so we can sleep and enable interrupts
                // at the same time

  P1OUT |= RX_EN_PIN;

  delimiterNotFound = 0;
  // setup port interrupt on pin 1.2
  P1SEL &= ~BIT2;  //Disable TimerA2, so port interrupt can be used
  // Setup timer. It has to setup because there is no setup time after done with
  // port1 interrupt.
  TACTL = 0;
  TAR = 0;
  TACCR0 = 0xFFFF;    // Set up TimerA0 register as Max
  TACCTL0 = 0;
  TACCTL1 = SCS + CAP;   //Synchronize capture source and capture mode
  TACTL = TASSEL1 + MC1 + TAIE;  // SMCLK and continuous mode and Timer_A
                                 // interrupt enabled.

  // initialize bits
  bits = 0;
  // initialize dest
  dest = destorig;  // = &cmd[0]
  // clear R6 bits of word counter from prior communications to prevent dest++
  // on 1st port interrupt
  asm("CLR R6");

  P1IE = 0;
  P1IES &= ~RX_PIN; // Make positive edge for port interrupt to detect start of
                    // delimiter
  P1IFG = 0;  // Clear interrupt flag

  delimiter_found=0x00;
  P1IE  |= RX_PIN; // Enable Port1 interrupt
  _BIS_SR(LPM4_bits | GIE);
  return;
}

inline void sleep()
{    
  P1OUT &= ~RX_EN_PIN;
  // enable port interrupt for voltage supervisor
  P2IES = 0;
  P2IFG = 0;
  P2IE |= VOLTAGE_SV_PIN;
  P1IE = 0;
  P1IFG = 0;
  TACTL = 0;

  _BIC_SR(GIE); // temporarily disable GIE so we can sleep and enable interrupts
                // at the same time
  P2IE |= VOLTAGE_SV_PIN; // Enable Port 2 interrupt

  if (is_power_good())
    P2IFG = VOLTAGE_SV_PIN;

  _BIS_SR(LPM3_bits | GIE);

  return;
}

inline void voltage_down()
{
  P2IES = 1;
  P2IFG = 0;
  P2IE |= VOLTAGE_SV_PIN;
  TACTL = 0;
  _BIC_SR(GIE);
  P2IE |= VOLTAGE_SV_PIN;
  _BIS_SR(GIE);
  
  /*P2IES = 1;
  P2IFG = 0;
  P2IE |= VOLTAGE_SV_PIN;
  TACTL = 0;
  _BIS_SR(GIE);*/
  
  return;
}

inline void set_timer(){
  TBCTL |= TBCLR;
  TBR = 0;
  TBCTL = TBSSEL_1 + MC_2;
}

inline void stop_timer(){    
  TBCTL = 0;
  TBCCTL0 = 0;
  TBCCTL1 = 0;
}

inline void tx_sleep(unsigned int gap)
{  
  P1OUT &= ~RX_EN_PIN;
  
  TBCTL |= TBCLR;
  TBR = 0;
  TBCCR0 = gap;
  TBCTL = TBSSEL_1 + MC_1 + TBIE;
  _BIS_SR(LPM3_bits + GIE);
  return;
}

unsigned short is_power_good()
{
  return P2IN & VOLTAGE_SV_PIN;
}

//*************************************************************************
//************************ PORT 2 INTERRUPT *******************************

// Pin Setup :
// Description : Port 2 interrupt wakes on power good signal from supervisor.

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)   // (5-6 cycles) to enter interrupt
{  
  if(P2IES==1){  
    TACCTL0 = 0;  // DON'T NEED THIS NOP
    asm("MOV R5, counter\n");
    asm("MOV #0001, R5\n");
    P2IFG = 0x00;
    P2IE = 0;
    TACTL = 0;
    TACCTL0 = 0;
    TACCTL1 = 0;
    TAR = 0;
  }
  else{
    P2IFG = 0x00;
    P2IE = 0;
    P1IFG = 0;
    P1IE = 0;
    TACTL = 0;
    TACCTL0 = 0;
    TACCTL1 = 0;
    TAR = 0;
    LPM3_EXIT;
  }
}

#if USE_2618
#pragma vector=TIMERA0_VECTOR
#else
#pragma vector=TIMERA0_VECTOR
#endif
__interrupt void TimerA0_ISR(void)   // (5-6 cycles) to enter interrupt
{
  TACTL = 0;    // have to manually clear interrupt flag
  TACCTL0 = 0;  // have to manually clear interrupt flag
  TACCTL1 = 0;  // have to manually clear interrupt flag
  LPM4_EXIT;
}

#pragma vector=TIMERB1_VECTOR
__interrupt void TimerB1_ISR(void)   // (5-6 cycles) to enter interrupt
{
  TBCTL = 0;
  TBCCTL0 = 0;
  TBCCTL1 = 0;
  LPM3_EXIT;
}

//*************************************************************************
//************************ PORT 1 INTERRUPT *******************************

// warning   :  Whenever the clock frequency changes, the value of TAR should be
//              changed in aesterick lines
// Pin Setup :  P1.2
// Description : Port 1 interrupt is used as finding delimeter.

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)   // (5-6 cycles) to enter interrupt
{


#if USE_2618
  asm("MOV TAR, R7");  // move TAR to R7(count) register (3 CYCLES)
#else
  asm("MOV TAR, R7");  // move TAR to R7(count) register (3 CYCLES)
#endif
  P1IFG = 0x00;       // 4 cycles
  TAR = 0;            // 4 cycles
  LPM4_EXIT;

  asm("CMP #0000h, R5\n");          // if (bits == 0) (1 cycle)
  asm("JEQ bit_Is_Zero_In_Port_Int\n");                // 2 cycles
  // bits != 0:
  asm("MOV #0000h, R5\n");          // bits = 0  (1 cycles)

  asm("CMP #0010h, R7\n");          // finding delimeter (12.5us, 2 cycles)
                                    // 2d -> 14
  asm("JNC delimiter_Value_Is_wrong\n");            //(2 cycles)
  asm("CMP #0040h, R7");            // finding delimeter (12.5us, 2 cycles)
                                    // 43H
  asm("JC  delimiter_Value_Is_wrong\n");
  asm("CLR P1IE");
#if USE_2618
  asm("BIS #8010h, TACCTL1\n");     // (5 cycles)   TACCTL1 |= CM1 + CCIE
#else
  asm("BIS #8010h, TACCTL1\n");     // (5 cycles)   TACCTL1 |= CM1 + CCIE
#endif
  asm("MOV #0004h, P1SEL\n");       // enable TimerA1    (4 cycles)
  asm("RETI\n");

  asm("delimiter_Value_Is_wrong:\n");
  asm("BIC #0004h, P1IES\n");
  asm("MOV #0000h, R5\n");          // bits = 0  (1 cycles)
  delimiterNotFound = 1;
  asm("RETI");

  asm("bit_Is_Zero_In_Port_Int:\n");                 // bits == 0
#if USE_2618
  asm("MOV #0000h, TAR\n");     // reset timer (4 cycles)
#else
  asm("MOV #0000h, TAR\n");     // reset timer (4 cycles)
#endif
  asm("BIS #0004h, P1IES\n");   // 4 cycles  change port interrupt edge to neg
  asm("INC R5\n");            // 1 cycle
  asm("RETI\n");

}

//*************************************************************************
//************************ Timer INTERRUPT *******************************

// Pin Setup :  P1.2
// Description :

#if USE_2618
#pragma vector=TIMERA1_VECTOR
#else
#pragma vector=TIMERA1_VECTOR
#endif
__interrupt void TimerA1_ISR(void)   // (6 cycles) to enter interrupt
{

    asm("MOV 0174h, R7");  // move TACCR1 to R7(count) register (3 CYCLES)
    TAR = 0;               // reset timer (4 cycles)
    TACCTL1 &= ~CCIFG;      // must manually clear interrupt flag (4 cycles)

    //<------up to here 26 cycles + 6 cyles of Interrupt == 32 cycles -------->
    asm("CMP #0003h, R5\n");      // if (bits >= 3).  it will do store bits
    asm("JGE bit_Is_Over_Three\n");
    // bit is not 3
    asm("CMP #0002h, R5\n");   // if ( bits == 2)
    asm("JEQ bit_Is_Two\n");         // if (bits == 2).

    // <----------------- bit is not 2 ------------------------------->
    asm("CMP #0001h, R5\n");      // if (bits == 1) -- measure RTcal value.
    asm("JEQ bit_Is_One\n");          // bits == 1

    // <-------------------- this is bit == 0 case --------------------->
    asm("bit_Is_Zero_In_Timer_Int:");
    asm("CLR R6\n");
    asm("INC R5\n");        // bits++
    asm("RETI");
    // <------------------- end of bit 0  --------------------------->

    // <-------------------- this is bit == 1 case --------------------->
    asm("bit_Is_One:\n");         // bits == 1.  calculate RTcal value
    asm("MOV R7, R9\n");       // 1 cycle
    asm("RRA R7\n");    // R7(count) is divided by 2.   1 cycle
    asm("MOV #0FFFFh, R8\n");   // R8(pivot) is set to max value    1 cycle
    asm("SUB R7, R8\n");        // R8(pivot) = R8(pivot) -R7(count/2) make new
                                // R8(pivot) value     1 cycle
    asm("INC R5\n");        // bits++
    asm("CLR R6\n");
    asm("RETI\n");
    // <------------------ end of bit 1 ------------------------------>

    // <-------------------- this is bit == 2 case --------------------->
    asm("bit_Is_Two:\n");
    asm("CMP R9, R7\n");    // if (count > (R9)(180)) this is hardcoded number,
                            // so have  to change to proper value
    asm("JGE this_Is_TRcal\n");
    // this is data
    asm("this_Is_Data_Bit:\n");
    asm("ADD R8, R7\n");   // count = count + pivot
    // store bit by shifting carry flag into cmd[bits]=(dest*) and increment
    // dest*  (5 cycles)
    asm("ADDC.b @R4+,-1(R4)\n"); // roll left (emulated by adding to itself ==
                                 // multiply by 2 + carry)
    // R6 lets us know when we have 8 bits, at which point we INC dest* (1
    // cycle)
    asm("INC R6\n");
    asm("CMP #0008,R6\n\n");   // undo increment of dest* (R4) until we have 8
                               // bits
    asm("JGE out_p\n");
    asm("DEC R4\n");
    asm("out_p:\n");           // decrement R4 if we haven't gotten 16 bits yet
                               // (3 or 4 cycles)
    asm("BIC #0008h,R6\n");   // when R6=8, this will set R6=0   (1 cycle)
    asm("INC R5\n");
    asm("RETI");
    // <------------------ end of bit 2 ------------------------------>

    asm("this_Is_TRcal:\n");
    asm("MOV R7, R5\n");    // bits = count. use bits(R5) to assign new value of
                            // TRcal
    TRcal = bits;       // assign new value     (4 cycles)
    asm("MOV #0003h, R5\n");      // bits = 3..assign 3 to bits, so it will keep
                                  // track of current bits    (2 cycles)
    asm("CLR R6\n"); // (1 cycle)
    asm("RETI");

   // <------------- this is bits >= 3 case ----------------------->
    asm("bit_Is_Over_Three:\n");     // bits >= 3 , so store bits
    asm("ADD R8, R7\n");    // R7(count) = R8(pivot) + R7(count),
    // store bit by shifting carry flag into cmd[bits]=(dest*) and increment
    // dest* (5 cycles)
    asm("ADDC.b @R4+,-1(R4)\n"); // roll left (emulated by adding to itself ==
                                 // multiply by 2 + carry)
    // R6 lets us know when we have 8 bits, at which point we INC dest* (1
    // cycle)
    asm("INC R6\n");
    asm("CMP #0008,R6\n");   // undo increment of dest* (R4) until we have 8
                             // bits
    asm("JGE out_p1\n");
    asm("DEC R4\n");
    asm("out_p1:\n");           // decrement R4 if we haven't gotten 16 bits yet
                                // (3 or 4 cycles)
    asm("BIC #0008h,R6\n");   // when R6=8, this will set R6=0   (1 cycle)
    asm("INC R5\n");              // bits++
    asm("RETI\n");
    // <------------------ end of bit is over 3 ------------------------------>
}


//
//
// experimental M4 code
//
//

/******************************************************************************
*   Pin Set up
*   P1.1 - communication output
*******************************************************************************/
void sendToReader(volatile unsigned char *data, unsigned int numOfBits)
{  
  SEND_CLOCK;

  TACTL &= ~TAIE;
  TAR = 0;
  // assign data address to dest
  dest = data;
  // Setup timer
  P1SEL |= TX_PIN; //  select TIMER_A0

  BCSCTL2 |= DIVM_1;

/*******************************************************************************
*   The starting of the transmitting code. Transmitting code must send 4 or 16
*   of M/LF, then send 010111 preamble before sending data package. TRext
*   determines how many M/LFs are sent.
*
*   Used Register
*   R4 = CMD address, R5 = bits, R6 = counting 16 bits, R7 = 1 Word data, R9 =
*   temp value for loop R10 = temp value for the loop, R13 = 16 bits compare,
*   R14 = timer_value for 11, R15 = timer_value for 5
*******************************************************************************/


  //<-------- The below code will initiate some set up ---------------------->//
    //asm("MOV #05h, R14");
    //asm("MOV #02h, R15");
    bits = TRext;           // 5 cycles
    asm("NOP");             // 1 cycles
    asm("CMP #0001h, R5");  // 1 cycles
    asm("JEQ TRextIs_1");   // 2 cycles
    asm("MOV #0004h, R9");  // 1 cycles
    asm("JMP otherSetup");  // 2 cycles

    // initialize loop for 16 M/LF
    asm("TRextIs_1:");
    asm("MOV #000fh, R9");    // 2 cycles    *** this will chagne to right value
    asm("NOP");

    //
    asm("otherSetup:");
    bits = numOfBits;         // (2 cycles).  This value will be adjusted. if
                              // numOfBit is constant, it takes 1 cycles
    asm("NOP");               // (1 cycles), zhangh 0316

    asm("MOV #0bh, R14");     // (2 cycles) R14 is used as timer value 11, it
                              // will be 2 us in 3 MHz
    asm("MOV #05h, R15");     // (2 cycles) R15 is used as tiemr value 5, it
                              // will be 1 us in 3 MHz
    asm("MOV @R4+, R7");      // (2 cycles) Assign data to R7
    asm("MOV #0010h, R13");   // (2 cycles) Assign decimal 16 to R13, so it will
                              // reduce the 1 cycle from below code
    asm("MOV R13, R6");       // (1 cycle)
    asm("SWPB R7");           // (1 cycle)    Swap Hi-byte and Low byte
    
    TACTL |= TACLR;   //reset timer A
    TACTL = TASSEL1 + MC0;     // up mode

    TACCR0 = 5;  // this is 1 us period( 3 is 430x12x1)

    TAR = 0;
    TACCTL0 = OUTMOD2; // RESET MODE

    asm("NOP");
    asm("NOP");
    asm("NOP");
/***********************************************************************
*   The main loop code for transmitting data in 3 MHz.  This will transmit data
*   in real time.
*   R5(bits) and R6(word count) must be 1 bigger than desired value.
*   Ex) if you want to send 16 bits, you have to store 17 to R5.
************************************************************************/

    // this is starting of loop
    asm("LOOPAGAIN:");
    asm("DEC R5");                              // 1 cycle
    asm("JEQ Three_Cycle_Loop_End");            // 2 cycle
    //<--------------loop condition ------------
//    asm("NOP");                                 // 1 cycle
    asm("RLC R7");                              // 1 cycle
    asm("JNC bit_is_zero");	                // 2 cycles  ..6

    // bit is 1
    asm("bit_is_one:");
//    asm("NOP");                               // 1 cycle
#if USE_2618
    asm("MOV R14, TACCR0");                   // 3 cycles   ..9
#else
    asm("MOV R14, TACCR0");                   // 3 cycles   ..9
#endif                // 4 cycles   ..11

    asm("DEC R6");                              // 1 cycle  ..10
    asm("JNZ bit_Count_Is_Not_16");              // 2 cycle    .. 12
    // This code will assign new data from reply and then swap bytes.  After
    // that, update R6 with 16 bits
    //asm("MOV @R4+, R7");
#if USE_2618
    asm("MOV R15, TACCR0");                   // 3 cycles   .. 15
#else
    asm("MOV R15, TACCR0");                   // 3 cycles   .. 15
#endif

    asm("MOV R13, R6");                         // 1 cycle   .. 16
    asm("MOV @R4+, R7");                        // 2 cycles  .. 18

    asm("SWPB R7");                             // 1 cycle    .. 19
    //asm("MOV R13, R6");                         // 1 cycle
    // End of assigning data byte
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("JMP LOOPAGAIN");                       // 2 cycle    .. 24

    asm("seq_zero:");
#if USE_2618
    asm("MOV R15, TACCR0");         // 3 cycles       ..3
#else
    asm("MOV R15, TACCR0");         // 3 cycles       ..3
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");                     // 1 cycles .. 6


    // bit is 0, so it will check that next bit is 0 or not
    asm("bit_is_zero:");				// up to 6 cycles
    asm("DEC R6");                      // 1 cycle   .. 7
    asm("JNE bit_Count_Is_Not_16_From0");           // 2 cycles  .. 9
    // bit count is 16
    asm("DEC R5");                      // 1 cycle   .. 10
    asm("JEQ Thirteen_Cycle_Loop_End");     // 2 cycle   .. 12
    // This code will assign new data from reply and then swap bytes.  After
    // that, update R6 with 16 bits
    asm("MOV @R4+,R7");                 // 2 cycles     14
    asm("SWPB R7");                     // 1 cycle      15
    asm("MOV R13, R6");                 // 1 cycles     16
    // End of assigning new data byte
    asm("RLC R7");		        // 1 cycles     17
    asm("JC nextBitIs1");	        // 2 cycles  .. 19
    // bit is 0
#if USE_2618
    asm("MOV R14, TACCR0");             // 3 cycles  .. 22
#else
    asm("MOV R14, TACCR0");             // 3 cycles  .. 22
#endif
    // Next bit is 0 , it is 00 case
    asm("JMP seq_zero");                // 2 cycles .. 24

// <---------this code is 00 case with no 16 bits.
    asm("bit_Count_Is_Not_16_From0:");                  // up to 9 cycles
    asm("DEC R5");                          // 1 cycle      10
    asm("JEQ Thirteen_Cycle_Loop_End");     // 2 cycle    ..12
    asm("NOP");         	            // 1 cycles    ..13
    asm("NOP");                             // 1 cycles    ..14
    asm("NOP");                             // 1 cycles    ..15
    asm("RLC R7");	                    // 1 cycle     .. 16
    asm("JC nextBitIs1");	            // 2 cycles    ..18

#if USE_2618
    asm("MOV R14, TACCR0");               // 3 cycles   .. 21
#else
    asm("MOV R14, TACCR0");               // 3 cycles   .. 21
#endif
    asm("NOP");                         // 1 cycle   .. 22
    asm("JMP seq_zero");        // 2 cycles    .. 24

// whenever current bit is 0, then next bit is 1
    asm("nextBitIs1:");     // 18
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");       // 24

    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("JMP bit_is_one");  // end of bit 0 .. 6

    asm("bit_Count_Is_Not_16:");       // up to here 14
#if USE_2618
    asm("MOV R15, TACCR0");             // 3 cycles   .. 15
#else
    asm("MOV R15, TACCR0");             // 3 cycles   .. 15
#endif

    asm("NOP");                               // 1 cycle .. 16
    asm("NOP");                               // 1 cycle .. 17
    asm("NOP");                               // 1 cycle .. 18
    asm("NOP");                               // 1 cycle .. 19
    asm("NOP");                               // 1 cycle .. 20
    asm("NOP");                               // 1 cycle .. 21
    asm("NOP");                               // 1 cycle .. 22
    asm("JMP LOOPAGAIN");     // 2 cycle          .. 24

    // below code is the end of loop code
    asm("Three_Cycle_Loop_End:");
    asm("JMP lastBit");     // 2 cycles   .. 5

    asm("Thirteen_Cycle_Loop_End:");  // up to 12
    asm("NOP");   // 1
    asm("NOP");   // 2
    asm("NOP");   // 3
    asm("NOP");   // 4
    asm("NOP");   // 5
    asm("NOP");   // 6
    asm("NOP");   // 7
    asm("NOP");   // 8
    asm("NOP");   // 9
    asm("NOP");   // 10
    asm("NOP");   // 11
    asm("NOP");   // 12 ..24
    asm("NOP");   // 13
    asm("NOP");   // 14
    asm("JMP lastBit"); // 16
/***********************************************************************
*   End of main loop
************************************************************************/
// this is last data 1 bit which is dummy data
    asm("lastBit:");  // up to 4
    asm("NOP");       // 5
    asm("NOP");
#if USE_2618
    asm("MOV.B R14, TACCR0");// 3 cycles
#else
    asm("MOV.B R14, TACCR0");// 3 cycles
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
#if USE_2618
    asm("MOV.B R15, TACCR0");
#else
    asm("MOV.B R15, TACCR0");
#endif
    asm("NOP");
    asm("NOP");
    // experiment

    asm("NOP");

    //TACCR0 = 0;

    TACCTL0 = 0;  // DON'T NEED THIS NOP
    RECEIVE_CLOCK;

}