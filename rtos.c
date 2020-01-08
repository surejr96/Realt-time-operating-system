//RTOS Framework - Spring 2019
//Project done by Surej Rajkumar

int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if(prische==0)
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        else if(prische==1)
        {
            if(tcb[task].skipc == 0)
                        {
                            tcb[task].skipc = tcb[task].currentPriority+8;
                            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
                        }
                        else if(tcb[task].skipc != 0)
                        {
                            tcb[task].skipc -=1;
                            ok = 0;
                        }
        }
    }
    return task;
}
8:21 PM

int r;

                  putsUart0("TASK NAME");

                  putsUart0("\t");

                  putsUart0("PID NAME");

                  putsUart0("\r\n");

                  for(r=0;r<10;r++)

                    {



                        putsUart0(tcb[r].name);

                        if(strcmp(&tcb[r].name,"idle")==0||strcmp(&tcb[r].name,"oneshot")==0 || strcmp(&tcb[r].name,"uncoop")==0 || strcmp(&tcb[r].name,"shell")==0)

                        putsUart0("\t\t");

                        else putsUart0("\t");

                        uint16_t pid_name = tcb[r].pid;

                        ltoa(pid_name,buffer);

                        putsUart0(buffer);

                        putsUart0("\r\n");

                    }
10:50 PM

strcpy(tcb[i].name,name);
11:30 PM



{

        uint8_t i;

        uint8_t j;

        for (i = 0; i < taskCount; i++)
        {
            if (tcb[i].pid == step7semaphore)
            {
                tcb[i].state = STATE_INVALID;
                tcb[i].pid = 0;
                taskCount--;
                step7semaphore = tcb[i].semaphore;
                tcb[i].pid = 0;
                step7semaphore->count++;
                for (j = 0; j <= 5; j++)
                {
                    if (step7semaphore->processQueue[j] == (uint32_t) tcb[task_num].pid)
                        for (j = 0; j <= step7semaphore->queueSize; j++)
                        {
                            step7semaphore->processQueue[j] =
                                    step7semaphore->processQueue[j + 1];
                        }

                }
                step7semaphore->queueSize--;
            }
        }
    }
10:19 AM
// RTOS Framework - Spring 2019
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED
#define PB1          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //PA2
#define PB2          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) //PA3
#define PB3          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) //PA4
#define PB4          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) //PA5
#define PB5          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //PA6
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    char s_name[80];
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

//global variables
uint32_t main_spval;
uint32_t reg0;
uint32_t reg1;
uint32_t reg2;
char str[81];
uint8_t pos[80];
char type[80];
uint8_t argc=0;
uint8_t arg0value[80];
uint8_t arg1value[80];
uint8_t arg2value[80];
uint8_t ps_bit = 1;
uint8_t pe_bit = 0;
uint8_t task_num;


struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
struct semaphore *step7semaphore;


// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t skipcount;
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
    // REQUIRED: initialize systick for 1ms system timer
    NVIC_ST_RELOAD_R |= 0x00009C3F;
    NVIC_ST_CURRENT_R |= 0x00000000;
    NVIC_ST_CTRL_R |= 0x00000007;
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if(ps_bit==0)
        {
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        else if(ps_bit==1)
        {
            if(tcb[task].skipcount == 0)
            {
                tcb[task].skipcount = tcb[task].currentPriority+8;
                ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);

            }
            else if(tcb[task].skipcount != 0)
            {
                tcb[task].skipcount -=1;
                ok = 0;
            }
        }
    }
    return task;
}


void rtosStart()
{
    pe_bit = 1;
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    taskCurrent = rtosScheduler();
    // Add code to initialize the SP with tcb[task_current].sp;
    main_spval = getsp();
    setsp(tcb[taskCurrent].sp);
    tcb[taskCurrent].state = STATE_READY;
    fn = (_fn) tcb[taskCurrent].pid;
    (*fn)();//calling function pointers
}

bool createThread(_fn fn, char name[], int priority)
{



    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name

    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            strcpy(tcb[i].name,name);
            tcb[i].currentPriority = priority;
            tcb[i].skipcount = tcb[i].priority + 8;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC #104");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name

    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            //strcpy(tcb[i].name,name);
            tcb[i].currentPriority = priority;
            tcb[i].skipcount = tcb[i].priority + 8;
            // increment task count
            taskCount++;
            ok = true;
        }
    }


}

struct semaphore* createSemaphore(uint8_t count, char name[])
{

    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        strcpy(pSemaphore->s_name,name);
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #100");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #101");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    //step7semaphore = pSemaphore;
    __asm(" SVC #102");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    //step7semaphore = pSemaphore;
    __asm(" SVC #103");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t j=0;

    for(j=0;j<=8;j++)
    {
        if(tcb[j].ticks != 0)
        {
            if(tcb[j].state == STATE_DELAYED)
            {
                tcb[j].ticks--;
            }
        }
        if(tcb[j].ticks == 0)
        {
            if(tcb[j].state == STATE_DELAYED)
            {
                tcb[j].state = STATE_READY;
            }
        }
    }

    if(pe_bit==1)
    {
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm(" PUSH {R4,R5,R6,R7}");
    tcb[taskCurrent].sp = getsp();
    setsp(main_spval);
    taskCurrent = rtosScheduler();
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setsp(tcb[taskCurrent].sp);
        __asm(" POP {R4,R5,R6,R7}");
    }
    else
    {
        //seed stack
        stack[taskCurrent][255] = 0x41000000;
        stack[taskCurrent][254] = (uint32_t) tcb[taskCurrent].pid;
        stack[taskCurrent][253] = (uint32_t) tcb[taskCurrent].pid;
        stack[taskCurrent][252] = 0x12;
        stack[taskCurrent][251] = 0x3;
        stack[taskCurrent][250] = 0x2;
        stack[taskCurrent][249] = 0x1;
        stack[taskCurrent][248] = 0x0;
        stack[taskCurrent][247] = 0xFFFFFFF9;
        setsp(tcb[taskCurrent].sp-36);
        tcb[taskCurrent].state = STATE_READY;

    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr()
{
    step7semaphore  = getreg0();
    reg0 = getreg0();
    reg1 = getreg1();
    reg2 = getreg2();



    uint8_t svcalled = getsvc();
    switch(svcalled)
    {
    case 100:
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    case 101:
        tcb[taskCurrent].ticks = reg0;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    case 102:

        if(step7semaphore->count == 0)
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            uint8_t k = 0;
            for(k=0;k<MAX_QUEUE_SIZE;k++)
            {
                if(step7semaphore->processQueue[k] == 0)
                {
                    step7semaphore->processQueue[k] = taskCurrent;
                    step7semaphore->queueSize++;
                    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
                }
            }
        }
        else
        {
            tcb[taskCurrent].semaphore = step7semaphore;
            step7semaphore->count--;
        }
        break;

    case 103:

        //step7semaphore->count++;
        //if(step7semaphore->count != 0)
    {
        if(step7semaphore->queueSize>0)
        {
            tcb[step7semaphore->processQueue[0]].state = STATE_READY;
            //step7semaphore->count--;
            tcb[step7semaphore->processQueue[0]].semaphore = step7semaphore;
            step7semaphore->processQueue[0] = 0;
        }
    }
    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    break;

    case 104:

    {

        uint8_t i;

        uint8_t j;

        for (i = 0; i < taskCount; i++)
        {
            if (tcb[i].pid == step7semaphore)
            {
                tcb[i].state = STATE_INVALID;
                tcb[i].pid = 0;
                taskCount--;
                step7semaphore = tcb[i].semaphore;
                tcb[i].pid = 0;
                step7semaphore->count++;
                for (j = 0; j <= 5; j++)
                {
                    if (step7semaphore->processQueue[j] == (uint32_t) tcb[task_num].pid)
                        for (j = 0; j <= step7semaphore->queueSize; j++)
                        {
                            step7semaphore->processQueue[j] =
                                    step7semaphore->processQueue[j + 1];
                        }

                }
                step7semaphore->queueSize--;
            }
        }
    }

        break;



    }
}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
    //           5 pushbuttons, and uart
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    //Leds
    GPIO_PORTF_DIR_R = 0x04;
    GPIO_PORTE_DIR_R = 0x1E;
    GPIO_PORTF_DEN_R = 0x04;
    GPIO_PORTE_DEN_R = 0x1E;

    //Push buttons
    GPIO_PORTA_DEN_R = 0x7C;
    GPIO_PORTA_PUR_R = 0x7C;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    if((PB1 == 0)&&(PB2 == 1)&&(PB3 == 1)&&(PB4 == 1)&&(PB5 == 1))
        return 1;
    if((PB1 == 1)&&(PB2 == 0)&&(PB3 == 1)&&(PB4 == 1)&&(PB5 == 1))
        return 2;
    if((PB1 == 1)&&(PB2 == 1)&&(PB3 == 0)&&(PB4 == 1)&&(PB5 == 1))
        return 4;
    if((PB1 == 1)&&(PB2 == 1)&&(PB3 == 1)&&(PB4 == 0)&&(PB5 == 1))
        return 8;
    if((PB1 == 1)&&(PB2 == 1)&&(PB3 == 1)&&(PB4 == 1)&&(PB5 == 0))
        return 16;
    return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//timer
/*void wtimer()
{
  time_counter++;
  counter++;
  if(counter==0xFFFFFFFF)
  {
      counter=0;
}

}*/

//sets the stack pointer to the desired value
void setsp(uint32_t value)
{
    __asm(" MOV SP, R0");
    __asm(" SUB SP, #8");
}

//gets the value of the stack pointer
uint32_t getsp()
{
    __asm(" MOV R0, SP");
    //return tcb[taskCurrent].sp;
}


uint8_t getsvc()
{
    __asm(" MOV R0,SP");
    __asm(" ADD R0,#40");
    __asm(" LDR R0,[R0]");
    __asm(" SUB R0,#2");
    __asm(" LDR R0,[R0]");

}


uint32_t getreg0()
{
}

uint32_t getreg1()
{
    __asm(" MOV R1, R0");

}

uint32_t getreg2()
{
    __asm(" MOV R2, R1");

}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
    yield();
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        yield();
    return UART0_DR_R & 0xFF;

}
//Blocking function that returns a string

void getstring()
{
    uint8_t max_chars=80;
    uint8_t count;
    char c=0;
    count=0;
    while(c!=13)
    {
        c=getcUart0();
        if(c==8)
        {
            if(count>0)
            {
                count--;
            }
        }
        else if(c==13 || count==max_chars)
        {
            str[count]=0;
            c=13;
        }
        else if(c>=32)
        {
            str[count]=tolower(c);
            count++;
        }
    }

    putsUart0("\r\n");
    //yield();
}

//Function that returns number of arguments, position and type

void parse_str()
{
    uint8_t count=0;
    uint8_t j=0;
    uint8_t i=1;
    if(((str[0]>=48)&&(str[0]<=57))||((str[0]>=65)&&(str[0]<=90))||((str[0]>=97)&&(str[0]<=122)))
    {
        count++;
        argc=count;
        pos[j]=0;
        j++;
        if((str[0]>=48)&&(str[0]<=57))
        {
            type[j-1]=110; //ascii value for 'n'
        }
        else if(((str[0]>=65)&&(str[0]<=90))||((str[0]>=97)&&(str[0]<=122)))
        {
            type[j-1]=97; //ascii value for 'a'

        }
    }
    for(i=1; i<=strlen(str);i++)
    {
        if((((str[i]>=48)&&(str[i]<=57))||((str[i]>=65)&&(str[i]<=90))||((str[i]>=97)&&(str[i]<=122))))
        {
            if(!(((str[i-1]>=48)&&(str[i-1]<=57))||((str[i-1]>=65)&&(str[i-1]<=90))||((str[i-1]>=97)&&(str[i-1]<=122))))
            {
                count++;
                argc=count;
                pos[j]=i;
                j++;
                if((str[i]>=48)&&(str[i]<=57))
                {
                    type[j-1]=110; //ascii value for 'n'
                }
                else if(((str[i]>=65)&&(str[i]<=90))||((str[i]>=97)&&(str[i]<=122)))
                {
                    type[j-1]=97; //ascii value for 'a'
                }
            }

        }


    }

}

//Function that stores values of the two arguments after the set command

void getvalue()
{
    uint8_t value0=pos[0];
    uint8_t value1=pos[1];
    uint8_t value2=pos[2];
    uint8_t p=0;
    uint8_t k=0;
    uint8_t j=0;

    for(j=0;j<sizeof(arg0value);j++)
    {
        arg0value[j]= 0;
    }
    for(j=0;j<sizeof(arg1value);j++)
    {
        arg1value[j]= 0;
    }
    for(j=0;j<sizeof(arg0value);j++)
    {
        arg1value[j]= 0;
    }

    while((((str[value0]>=48)&&(str[value0]<=57))||((str[value0]>=65)&&(str[value0]<=90))||((str[value0]>=97)&&(str[value0]<=122))))
    {
        arg0value[p]=str[value0];
        p++;
        value0++;
    }
    while((((str[value1]>=48)&&(str[value1]<=57))||((str[value1]>=65)&&(str[value1]<=90))||((str[value1]>=97)&&(str[value1]<=122))))
    {
        arg1value[k]=str[value1];
        k++;
        value1++;
    }
    while((((str[value2]>=48)&&(str[value2]<=57))||((str[value2]>=65)&&(str[value2]<=90))||((str[value2]>=97)&&(str[value2]<=122))))
    {
        arg2value[j]=str[value2];
        j++;
        value2++;
    }
}

void reset()
{
    NVIC_APINT_R = 0x05FA0004;
    return 0;
}

//Function that returns true if the command is set and there are aleast two arguments after that



// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

//added to check step5
//void idle2()
//{
//    while(true)
//    {
//        YELLOW_LED = 1;
//        waitMicrosecond(100000);
//        YELLOW_LED = 0;
//        yield();
//    }
//}


void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{


    while (true)
    {
        putsUart0("\n");
        putsUart0("\r *********");
        putsUart0("\r\n 6314 RTOS \r\n Command list: ");
        putsUart0("\r\n pi [priority inheritance]");
        putsUart0("\r\n psched on/off [schedule priority of round robin]");
        putsUart0("\r\n preempt on/off [preempt on or no]");
        putsUart0("\r\n reboot [system reset]");
        putsUart0("\r\n pidof [name of the process]");
        putsUart0("\r\n kill [kill thread]");
        putsUart0("\r\n ps[process status]");
        putsUart0("\r\n ipcs [inter process communication] \r\n ");
        putsUart0("\r\n");

        // REQUIRED: add processing for the shell commands through the UART here
        getstring();
        parse_str();
        getvalue();
        char arg0char[80];
        char arg1char[80];
        char arg2char[80];
        uint8_t j;
        for(j=0;j<sizeof(arg0char);j++)
        {
            arg0char[j]= 0;
        }
        for(j=0;j<sizeof(arg1char);j++)
        {
            arg1char[j]= 0;
        }
        for(j=0;j<sizeof(arg0char);j++)
        {
            arg1char[j]= 0;
        }

        //uint8_t j;
        for(j=0;j<sizeof(arg0value);j++)
        {
            arg0char[j]= arg0value[j];
        }
        for(j=0;j<sizeof(arg0value);j++)
        {
            arg1char[j]= arg1value[j];
        }
        for(j=0;j<sizeof(arg0value);j++)
        {
            arg2char[j]= arg2value[j];
        }

        if(strcmp("pi",arg0char)==0)
        {
            putsUart0("\r\n pi mode \n");
        }
        else if(strcmp("psched",arg0char)==0)
        {

            if(strcmp("off",arg1char)==0)
            {
                putsUart0("\r\n priority scheduling off \n");
                ps_bit = 0;
            }
            if(strcmp("on",arg1char)==0)
            {
                putsUart0("\r\n priority scheduling on \n");
                ps_bit = 1;
            }

        }
        else if(strcmp("preempt",arg0char)==0)
        {
            if(strcmp("off",arg1char)==0)
            {
                putsUart0("\r\n preemption off \n");
                pe_bit = 0;
            }
            if(strcmp("on",arg1char)==0)
            {
                putsUart0("\r\n preemption on \n");
                pe_bit = 1;
            }

        }
        else if(strcmp("reboot",arg0char)==0)
        {
            reset();
        }
        else if(strcmp("pidof",arg0char)==0)
        {

            if(strcmp("idle",arg1value)==0)
            {
                putsUart0("pid of idle : ");
                char process_id[80];
                uint32_t id = tcb[0].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("lengthyfn",arg1value)==0)
            {
                putsUart0("pid of lengthyfn : ");
                char process_id[80];
                uint32_t id = tcb[1].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("flash4hz",arg1value)==0)
            {
                putsUart0("pid of flash4hz : ");
                char process_id[80];
                uint32_t id = tcb[2].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("oneshot",arg1value)==0)
            {
                putsUart0("pid of oneshot : ");
                char process_id[80];
                uint32_t id = tcb[3].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("readkeys",arg1value)==0)
            {
                putsUart0("pid of readkeys : ");
                char process_id[80];
                uint32_t id = tcb[4].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("debounce",arg1value)==0)
            {
                putsUart0("pid of debounce : ");
                char process_id[80];
                uint32_t id = tcb[5].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("important",arg1value)==0)
            {
                putsUart0("pid of important : ");
                char process_id[80];
                uint32_t id = tcb[6].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("uncoop",arg1value)==0)
            {
                putsUart0("pid of uncoop : ");
                char process_id[80];
                uint32_t id = tcb[7].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            if(strcmp("shell",arg1value)==0)
            {
                putsUart0("pid of shell : ");
                char process_id[80];
                uint32_t id = tcb[8].pid;
                ltoa(id,process_id,10);
                //putsUart0("\r\n");
                putsUart0(process_id);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }



        }
        else if(strcmp("kill",arg0char)==0)
        {
            if(strcmp("idle",arg1char)==0)
            {
                task_num = 0;
                destroyThread(idle);
            }
             if(strcmp("lengthyfn",arg1char)==0)
             {
                 task_num = 1;
                 destroyThread(lengthyFn);
             }
             if(strcmp("flash4hz",arg1char)==0)
             {
                 task_num = 2;
                 destroyThread(flash4Hz);
             }
             if(strcmp("oneshot",arg1char)==0)
             {
                 task_num = 3;
                 destroyThread(oneshot);
             }
             if(strcmp("readKeys",arg1char)==0)
             {
                 task_num = 4;
                 destroyThread(oneshot);
             }
             if(strcmp("debounce",arg1char)==0)
             {
                 task_num = 5;
                 destroyThread(oneshot);
             }
             if(strcmp("important",arg1char)==0)
             {
                 task_num = 6;
                 destroyThread(oneshot);
             }
             if(strcmp("uncoop",arg1char)==0)
             {
                 task_num = 7;
                 destroyThread(oneshot);
             }
        }
        else if(strcmp("ps",arg0char)==0)
        {
            uint8_t n=0;
            putsUart0("process name");
            putsUart0("\t\t");
            putsUart0("pid");
            putsUart0("\t\t");
            putsUart0("priority");
            putsUart0("\r\n");

            for(n=0;n<MAX_TASKS-1;n++)
            {
                putsUart0(tcb[n].name);
                putsUart0("\t\t");
                char process_id[80];
                char prio[80];
                int16_t id = tcb[n].pid;
                ltoa(id,process_id,10);
                putsUart0(process_id);
                putsUart0("\t\t");
                int16_t priority_no = tcb[n].priority;
                ltoa(priority_no,prio,10);
                putsUart0(prio);
                putsUart0("\r\n");

            }

        }
        else if(strcmp("ipcs",arg0char)==0)
        {
            putsUart0("semaphore name");
            putsUart0("\t\t");
            putsUart0("count");
            putsUart0("\t\t");
            putsUart0("queue size");
            putsUart0("\t\t");
            putsUart0("process queue");
            putsUart0("\r\n");

            //char semaphorename[80];
            uint16_t countsema;
            uint16_t queuesema;
            uint32_t processqueue;
            uint16_t queuesize;
            uint8_t j;
            uint8_t i;
            for(i=0;i<MAX_SEMAPHORES-1;i++)F
            {
                putsUart0(semaphores[i].s_name);
                putsUart0("\t\t");
                countsema = semaphores[i].count;
                char countsema_b[80];
                ltoa(countsema,countsema_b,10);
                putsUart0(countsema_b);
                putsUart0("\t\t");
                queuesize = semaphores[i].queueSize;
                char queuesize_b[80];
                ltoa(queuesize,queuesize_b);
                putsUart0(queuesize_b);
                putsUart0("\t\t");

                for(j=0;j<MAX_QUEUE_SIZE-1;j++)
                {
                    processqueue = semaphores[i].processQueue[j];
                    char processqueue_b[80];
                    ltoa(processqueue,processqueue_b);
                    putsUart0(processqueue_b);
                    putsUart0(" ");

                }
                putsUart0("\n\r");
            }

        }
    }
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1,"keyPressed");
    keyReleased = createSemaphore(0,"keyReleased");
    flashReq = createSemaphore(5,"flashReq");
    resource = createSemaphore(1,"resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);
    //added to check step 5
    //ok =  createThread(idle2, "Idle2", 7);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
