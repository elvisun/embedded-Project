/*  Definitions  */
#define  TASK_STK_SIZE		256         // Size of each task's stacks (# of bytes)
#define  PotMax				1031			// Maximum POT poistion reading
#define	PotMin				2057			// Minimum POT position reading
#define	IrVmax				1544			// Maximum IR Sesnor reading
#define	IrVmin				1993			// Minimum IR Sensor reading
#define  MAX_COUNT			100
#define	ON						1				// Both experimental, and development board work on an active low configuration
#define	OFF					0				// Both experimental, and development board work on an active low configuration
#define	N_TASKS				4				// Number of tasks
#define  DATAARRAYSIZE     90          // size for message Q


// Redefine uC/OS-II configuration constants as necessary
#define  OS_MAX_EVENTS		2           // Maximum number of events (semaphores, queues, mailboxes)
#define  OS_MAX_TASKS		11          // Maximum number of tasks system can create (less stat and idle tasks)
#define  OS_TASK_STAT_EN	1           // Enable statistics task creation
#define  OS_TICKS_PER_SEC	128         // Number of Ticks per second

/* Other Definitions */
#define POT_CHAN				0                    // channel 0 of ADC (ADC0)
#define IRS_CHAN				1							// channel 1 of ADC (ADC1)
#define MOT_CHAN				1                    // channel 1 of digital output (OUT1)
#define MAX_PWIDTH			1016                 // the maximum pulse width in TMRB clock cycles

//#define STDIO_DISABLE_FLOATS


/* Variable declarations */
char	TMRB_MSB;									// this 8-bit value will be written to TBM2R (bits 8 and 9 of TMRB2 match register)
char	TMRB_LSB;									// this 8-bit value will be written to TBL2R (bits 0 to 7 of TMRB2 match register)
int	PulseWidth;                         // Duty Cycle of the PWM signal
float	PotNorm;										// Scaled value of the POT reading

int tubeHeight;
int ballXPos, ballYPos;

char TMRB_10_count;                       // This variable is incrimented at the begining of every PWM cycle. When it is equal to ten, it is
                                          // reset to zero, and duty cycle values are calculated from updated ADC input.
                                          // The value is updated in the Timer B ISR to either 1 during the duty cycle or 0 for remainder.
#use "ucos2.lib"

UBYTE			TaskData[N_TASKS];      		// Parameters to pass to each task
OS_EVENT		*ADCSem;								// Semaphore to access ADC
OS_EVENT    *loggingSem;
OS_EVENT    *messageQ;

void  InitializeTimers();						// Setup Timer A and B interrupts
void  CalculateDutyCycle();               // Update the duty cycle
void  Tmr_B_ISR();                        // Timer B interrupt service routine
void  ShowStat();                         // Display update
void  TaskInput(void *data);					// Function prototypes of the task
void	TaskControl (void *data);				// Function prototypes of the task
void  TaskStart(void *data);              // Function prototype of startup task
void  DispStr(int x, int y, char *s);
void  TaskLogging (void *data) ;          // Function to print out the position
void *dataArray[DATAARRAYSIZE];           // Q pointer storage array

void main (void)
{

   brdInit();                             // Initialize MCU baord
   OSInit();                              // Initialize uC/OS-II
   ADCSem = OSSemCreate(1);               // Semaphores for ADC inputs
   loggingSem = OSSemCreate(0);           // 0 since using as flag
   messageQ = OSQCreate(&dataArray[0],  DATAARRAYSIZE);
   OSTaskCreate(TaskStart, (void *)0, TASK_STK_SIZE, 10);
   OSStart();                             // Start multitasking
}

void TaskStart (void *data)
{
   OSStatInit();
   OSTaskCreate(TaskInput, (void *)&TaskData[1], TASK_STK_SIZE, 11);
   OSTaskCreate(TaskControl, (void *)&TaskData[2], TASK_STK_SIZE, 5);
   OSTaskCreate(TaskLogging, (void *)&TaskData[3], TASK_STK_SIZE, 12);
   InitializeTimers();							// Setup timer A and B interrupts

   tubeHeight = 30;

   DispStr(0, 10, "Analog Input Reading: xxxx  ");
   DispStr(0, 13, "#Tasks          : xxxxx  CPU Usage: xxxxx   %");
   DispStr(0, 14, "#Task switch/sec: xxxxx");
   DispStr(0, 15, "<-PRESS 'Q' TO QUIT->");

   for (;;) {
         ShowStat();
         OSTimeDly(OS_TICKS_PER_SEC);     // Wait one second
    }
}



nodebug void TaskInput (void *date)
{
	auto	UBYTE err;
   char	display[64];
   int	PotRead;

   for(;;) {
      OSSemPend(ADCSem,0,&err);
   	PotRead = anaIn(POT_CHAN);					// Read POT output voltage
      OSSemPost(ADCSem);

   	sprintf(display, "%d", PotRead);			// Write POT voltage on STDIO
   	DispStr(22, 10, display);


   	if (PotRead >= PotMax && PotRead <= PotMin)
   		PotNorm = ((float)PotRead-(float)PotMax)/((float)PotMin-(float)PotMax);
   	else if (PotRead < PotMax)
   		PotNorm = 0;
   	else if (PotRead > PotMin)
   		PotNorm = 1;

      OSTimeDly(OS_TICKS_PER_SEC);
   }
}


nodebug void TaskLogging (void *data){
   float p, i, d, ir, pv, es;
   int i, initialRow;    // counter variable for loop
   initialRow = 30;
   for(;;){
      
      // p, i, d, ir, pot value, error signal         6 values
      p = OSQPend(messageQ, 0, &err);
      i = OSQPend(messageQ, 0, &err);
      d = OSQPend(messageQ, 0, &err);
      ir = OSQPend(messageQ, 0, &err);
      pv = OSQPend(messageQ, 0, &err);
      es = OSQPend(messageQ, 0, &err);

      sprintf(display, "%f", p);
      DispStr(0, initialRow + i , display);
      sprintf(display, "%f", i);
      DispStr(10, initialRow + i , display);
      sprintf(display, "%f", d);
      DispStr(20, initialRow + i , display);

      sprintf(display, "%f", ir);
      DispStr(30, initialRow + i , display);
      sprintf(display, "%f", pv);
      DispStr(40, initialRow + i , display);
      sprintf(display, "%f", es);
      DispStr(50, initialRow + i , display);
      i++;

      //free running loop, runs whenever CPU is free
    }
}


nodebug void TaskControl (void *data)
{
   auto UBYTE err;
   int IrSen counter;
   float IrNorm, ErrSig, MIN_PW, KP;
   float dataStorage[90]; 
   counter = 0;
   MIN_PW = 0.6;
   KP = 0.7;
   for (;;) {

      OSSemPend(ADCSem,0,&err);
      IrSen = anaIn(IRS_CHAN);
      OSSemPost(ADCSem);
      if (IrSen >= IrVmax && IrSen <= IrVmin)
   		IrNorm = ((float)IrSen-(float)IrVmax)/((float)IrVmin-(float)IrVmax);
   	else if (IrSen < IrVmax)
   		IrNorm = 0;
   	else if (IrSen > IrVmin)
   		IrNorm = 1;
      ErrSig = PotNorm-IrNorm;
      PulseWidth = (int)((MIN_PW - KP*ErrSig)*MAX_PWIDTH);
      if (PulseWidth > MAX_PWIDTH){
      	PulseWidth = MAX_PWIDTH;}
      else if (PulseWidth < 0){
      	PulseWidth = 0;}
      TMRB_LSB = (char)PulseWidth;
   	TMRB_MSB = (char)(PulseWidth >> 2);


      //change ball y position to graph 
      ballYPos = (int)((IrNorm * tubeHeight));

      // p, i, d, ir, pot value, error signal         6 values
      dataStorage[counter] = kp;
      dataStorage[counter+1] = ki;
      dataStorage[counter+2] = kd;
      dataStorage[counter+3] = IrSen;
      dataStorage[counter+4] = PotNorm;
      dataStorage[counter+5] = ErrSig;
      
      OSQPost(messageQ, *dataStorage[counter]);        // are we passing by value???
      OSQPost(messageQ, *dataStorage[counter+1]);
      OSQPost(messageQ, *dataStorage[counter+2]);
      OSQPost(messageQ, *dataStorage[counter+3]);
      OSQPost(messageQ, *dataStorage[counter+4]);
      OSQPost(messageQ, *dataStorage[counter+5]);

      counter += 6;
      if (counter > 89){
         counter = 0;
      }

      OSTimeDly(1);									// Delay the task
   }
}


nodebug root interrupt void Tmr_B_ISR()
{
   char TMRB_status;

   TMRB_status = RdPortI(TBCSR);					// Read-out and clear interrupt status flags
   if(TMRB_status & 0x02){							// A new PWM cycle, if Match 1 reg is triggered
      digOut(MOT_CHAN, ON);						// Set PWM output high
      WrPortI(TBM1R, NULL, 0x00);				// set up Match 1 reg to interrupt at the begining of the next cycle
      WrPortI(TBL1R, NULL, 0x00);				// set up Match 1 reg to interrupt at the begining of the next cycle
   }
   if(TMRB_status & 0x04){					// If Match 2 reg is triggered, output will be low for the rest of the cycle
     digOut(MOT_CHAN, OFF);						// Set PWM output low */                                       // Drop output flag to 0
      WrPortI(TBM2R, NULL, TMRB_MSB);			// set up Match 2 reg to corespond with duty cycle
      WrPortI(TBL2R, NULL, TMRB_LSB);			// set up Match 2 reg to corespond with duty cycle
   }
   OSIntExit();
}


/*
*********************************************************************************************************
*                                      HELPER FUNCTIONS
*********************************************************************************************************
*/

void InitializeTimers()
{
   TMRB_MSB = 0x40;									// Initialize TMRB2 match register to coincide with 50% duty cycle
   TMRB_LSB = 0xFF;									// Initialize TMRB2 match register to coincide with 50% duty cycle
   TMRB_10_count = 0;								// Initialize the Timer B interrupt counter (PWM cycle counter) to zero

   /* Setup Timer A */
   WrPortI(TAT1R, &TAT1RShadow, 0xFF);			// set TMRA1 to count down from 255
   WrPortI(TACR, &TACRShadow, 0x00);			// Disable TMRA interrupts (TMRA used only to clock TMRB)
   WrPortI(TACSR, &TACSRShadow, 0x01);			// Enable main clock (PCLK/2) for TMRA1

   /* set up Timer B for generating PWM */
   SetVectIntern(0x0b, Tmr_B_ISR);				// set up timer B interrupt vector
   WrPortI(TBCR, &TBCRShadow, 0x05);			// clock timer B with TMRA1 to priority 1 interrupt
   WrPortI(TBM1R, NULL, 0x00);					// set up match register 1 to 0x00
   WrPortI(TBL1R, NULL, 0x00);					// set up match register 1 to 0x00
   WrPortI(TBM2R, NULL, TMRB_MSB);				// set up match register 2 to to give an initial 50% duty cycle
   WrPortI(TBL2R, NULL, TMRB_LSB);				// set up match register 2 to to give an initial 50% duty cycle
   WrPortI(TBCSR, &TBCSRShadow, 0x07);			// enable Timer B to interrupt on B1 and B2 match
}

void ShowStat()
{
   static char Key, s[50];

   sprintf(s, "%5d", OSTaskCtr);					// Display #tasks running
   DispStr(18, 13, s);
   sprintf(s, "%5d", OSCPUUsage);				// Display CPU usage in %
   DispStr(36, 13, s);
   sprintf(s, "%5d", OSCtxSwCtr);				// Display avg #context switches per 5 seconds
   DispStr(18, 14, s);
   OSCtxSwCtr = 0;

   if (kbhit()) {										// See if key has been pressed
      Key = getchar();
      if (Key == 0x71 || Key == 0x51)			// Yes, see if it's the q or Q key
         exit(0);
   }
}

void DispStr (int x, int y, char *s)
{
   x += 0x20;
   y += 0x20;
   printf ("\x1B=%c%c%s", x, y, s);
}


void DisplayTower ()
{
    auto int i;
    const int XX, YY;
    ballXPos = 72;
    XX = 70;
    YY = 5;
    DispStr  (XX,YY,"=======");
    for(i=0; i < tubeHeight; i++)
        DispStr (XX,YY+1+i,"| |");
    DispStr (XX,YY+1+i,"=======");
    DispStr(ballXPos, YY + 1 + i - ballYPos, "O");
}

// =====
// |   |
// |   |
// |   |
// |   |
// |   |
// | 0 |
// |   |
// |   |
// |   |
// |   |
// =====