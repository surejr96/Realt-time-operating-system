// RTOS Framework - Spring 2019
// J Losh
// Student Name: Surej Rajkumar
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
#include <string.h>
#include <stdlib.h>
#include <hw_nvic.h>
#include <hw_types.h>
//#include<stdio.h>
#include<ctype.h>
#include "tm4c123gh6pm.h"
#include <hw_nvic.h>
//#include <hw_types.h>
#include <math.h>
// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))//PE1
#define GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))//PE2
#define YELLOW_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))//PE3
#define ORANGE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))//PE4
#define PB0       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //PA2 PB0
#define PB1       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) //PA3 PB1
#define PB2       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) //PA4 PB2
#define PB3       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) //PA5 PB3
#define PB4       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //PA6 PB4
uint32_t varstckptr;
uint8_t snum;
uint8_t prische = 0;
uint32_t reg1;
uint32_t reg0;
uint32_t reg2;
uint32_t S0;
uint32_t pi;
char str[80];
bool pre=false;
int16_t nampid;
uint8_t buttons;

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
    char name[32];
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;
struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
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
int b[3];
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
    uint16_t   skipc;
} tcb[MAX_TASKS];
char alpnam[32];
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
    NVIC_ST_RELOAD_R = 0x000093CF;
    NVIC_ST_CURRENT_R = 0x00000000;
    NVIC_ST_CTRL_R=0x00000007;
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
        if(prische==0)
        {
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        else if(prische==1)
        {
//            if(tcb[task].skipc == 0)
//            {
               tcb[task].skipc = tcb[task].currentPriority+8;
               ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
//            }
//            else if(tcb[task].skipc != 0)
//                        {
//                            tcb[task].skipc -=1;
//                            ok = 0;
                       }
  //      }
    }
    return task;
}
void rtosStart()
{ pre=true;
// REQUIRED: add code to call the first task to be run
_fn fn;
taskCurrent = rtosScheduler();
varstckptr= getsp();
setsp(tcb[taskCurrent].sp);
tcb[taskCurrent].state=STATE_READY;
fn = (_fn) tcb[taskCurrent].pid;
(*fn)();
// Add code to initialize the SP with tcb[task_current].sp;
}
bool createThread(_fn fn, char name[], int priority)
{
    //  char *name=getR1();
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
            tcb[i].currentPriority = priority;
            tcb[i].skipc = tcb[i].priority + 8;
            strcpy(tcb[i].name,name);
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
    __asm ( " SVC #104 ");
}
// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}
struct semaphore* createSemaphore(uint8_t count,char nprint[32])
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        strcpy( pSemaphore->name,nprint);
    }
    return pSemaphore;
}
// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #100");
}
int getR0 ()
{
}
int getR1 ()
{
    __asm(" MOV R1,R0 ");
}
int getR2 ()
{
    __asm(" MOV R2,R1 ");
}
int getsvc()
{
    __asm(" MOV R0,SP ");
    __asm(" ADD R0,#40 ");
    __asm(" LDR R0,[R0] ");
    __asm(" SUB R0,#2 " );
    __asm(" LDR R0,[R0] ");
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
    __asm(" SVC #102");
}
// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #103");
}
// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t x = 0;
    for(x=0;x<=8;x++)
    {
        if(tcb[x].ticks!=0 && tcb[x].state == STATE_DELAYED)
        {
            tcb[x].ticks = tcb[x].ticks-1;
        }
        if(tcb[x].ticks==0 && tcb[x].state == STATE_DELAYED)
        {
            tcb[x].state = STATE_READY;
        }
    }
    if(pre)
    {
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }
}
// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm(" PUSH{R4-R7} ");
    tcb[taskCurrent].sp=getsp();
    setsp(varstckptr);
    taskCurrent = rtosScheduler();
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setsp(tcb[taskCurrent].sp);
        __asm(" POP{R4-R7} ");
    }
    else
    {
        stack[taskCurrent][255] = 0x41000000; //same
        stack[taskCurrent][254] = tcb[taskCurrent].pid;
        stack[taskCurrent][253] = tcb[taskCurrent].pid;
        stack[taskCurrent][252] = 0x12;
        stack[taskCurrent][251] = 0x3;
        stack[taskCurrent][250] = 0x2;
        stack[taskCurrent][249] = 0x1;
        stack[taskCurrent][248] = 0x0;
        stack[taskCurrent][247] = 0xFFFFFFF9; //same
        setsp(tcb[taskCurrent].sp-36);
        tcb[taskCurrent].state = STATE_READY;
    }
}
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    reg0= getR0();
    reg1= getR1();
    reg2= getR2();
    snum = getsvc();
    struct semaphore *S0 = getR0();
    S0=reg0;
    switch (snum)
    {
    case 100: NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    break;
    case 101: tcb[taskCurrent].ticks = reg1;
    tcb[taskCurrent].state = STATE_DELAYED;
    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    break;
    case 102:
        if (S0->count > 0)
        {
            tcb[taskCurrent].semaphore = S0;
            S0->count--;
        }
        else
        {
            S0->processQueue[S0->queueSize]= taskCurrent;
            S0->queueSize++;
            tcb[taskCurrent].state = STATE_BLOCKED;
        }
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;
    case 103:
        S0->count+=1;
        if(S0->queueSize>0)
        {
            tcb[S0->processQueue[0]].state = STATE_READY;
            tcb[S0->processQueue[0]].semaphore = S0;
            S0->processQueue[0] = 0;
            S0->queueSize-=1;
            uint16_t j=1;
            for(j=1;j<MAX_QUEUE_SIZE;j++)
                S0->processQueue[j-1]=S0->processQueue[j];
        }
        S0->count-=1;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    case 104:
    {
        uint8_t d;
        for (d = 0; d < 9; d++)
        { nampid = tcb[d].pid;
        if (tcb[d].pid == S0)
        {
            tcb[d].state = STATE_INVALID;
            tcb[d].pid = 0;
            taskCount--;
        }
        }
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
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
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;
    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;
    // Configure BLUE LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x04;  // bits 2 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04;  // enable LEDs and pushbuttons
    // CONFIGURE r,g,y,O LED PE
    GPIO_PORTE_DIR_R = 0x1E;  // bits 1,2,3 AND 4 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R = 0x1E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x1E;  // enable
    // CONFIGURE PA PUSH BUTTONS
    GPIO_PORTA_DIR_R = 0x83;  // bits 2 - 6 are INputs, other pins are OUTputs
    // GPIO_PORTA_DR2R_R =0x7C; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0x7C;  // enable
    GPIO_PORTA_PUR_R  = 0x7C;    // enable internal pull-up for push button

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
    // Blocking function that writes a string when the UART buffer is not full
}
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
    yield();
}
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
// TO CLEAR THE STRING
void bufclr()
{
    int o;
    for (o=0;o<=80;o++)
    {
        str[o]='\0';
    }
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
int32_t getsp()
{
    __asm(" MOV R0,SP");
}
int32_t setsp()
{
    __asm(" MOV SP,R0");
}
uint8_t readPbs()
{ uint8_t a;

if(PB0==0)
{
    a=     pow(2,0);
    return a;
}
else if(PB1==0)
{
    a=     pow(2,1);
    return a;
}
else if(PB2==0)
{
    a=     pow(2,2);
    return a;
}
else if(PB3==0)
{
    a=     pow(2,3);
    return a;
}
else if(PB4==0)
{
    a=     pow(2,4);
    return a;
}
else
    return 0;
}
//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
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
void reset()
{
    putsUart0("reset");
    putsUart0("\n");
    putsUart0("\r");
    ResetISR();
   // HWREG(NVIC_APINT)= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ;
    return 0;
}
void shell()
{
    bufclr();
    while (true)
    {
        putsUart0("\n RTOS \r\n");
        char ch,ch1;
        int i=0;
        putsUart0("priority - on/off (pi) \r\n");
        putsUart0("sched - round robin(on) or priority(off) \r\n");
        putsUart0("Preempt - on/off (preempt) \r\n");
        putsUart0("reboot \r\n");
        putsUart0("pid of process name (pid) \r\n");
        putsUart0("Kill Pid  (killpid) \r\n");
        putsUart0("process status (prsts) \r\n");
        putsUart0("ipcs  \r\n");
        putsUart0("Enter String \r\n");
        while(i<80)
        {
            ch = getcUart0();
            if(ch==0x08) //BP
            {
                if (i>0)
                {
                    --i;
                }
            }
            else if  (ch==0x0D) //enter
            {
                str[i]=0;
                //i=0;
                break;
                //putsUart0("\n");
            }
            else if(ch>20)
            {
                ch1=tolower(ch);
                str[i]=ch1;
                i=i+1;
            }
        }
        putsUart0("\n");
        putsUart0(str);
        int j,k,flag,count;
        flag=0;
        k=strlen(str);
        putsUart0("\n");
        putsUart0("\r");
        char a[3];
        int p=0;
        count=0;
        for (j=0;j<k;j++)
        {
            if (isspace (str[j]))
            {
                flag=0;
                str[j]='\0';
                j--;
            }
            else if (isalpha(str[j]) && (flag==0))
            {
                //a=a+1;
                a[p]='a'; // a or n
                flag=1;
                b[p]= j;
                p++;
                count++;
            }
            else if (isdigit(str[j]) && (flag==0))
            {
                flag=1;
                b[p]= j; // pos vect
                a[p]='n'; // a or n
                p++;
                count++;
            }
            else if (ispunct(str[j]))
            {
                str[j]='\0';
                flag=0;
                b[p]= j;
                a [p]=str[j];
                p++;
                j--;
            }
        }
        putsUart0("\n");

        if (strcmp ("reset", &str[b[0]]) == 0)
        {
            reset();
        }
        else if (strcmp ("preempt", &str[b[0]]) == 0 && (strcmp ("on", &str[b[1]]) == 0))
        {
            putsUart0("preemption on");//PD2
            pre = true;
        }
        else if (strcmp ("preempt", &str[b[0]]) == 0 && (strcmp ("off", &str[b[1]]) == 0))
        {
            putsUart0("preemption off");//PD2
            pre = false;
        }
        else if (strcmp ("pi", &str[b[0]]) == 0 && (strcmp ("on", &str[b[1]]) == 0))
        {
            putsUart0("pri on");//PD2

        }
        else if (strcmp ("pi", &str[b[0]]) == 0 && (strcmp ("off", &str[b[1]]) == 0))
        {
            putsUart0("pri off");//PD2

        }
        else if (strcmp ("sched", &str[b[0]]) == 0 && (strcmp ("on", &str[b[1]]) == 0))
        {
            putsUart0("priority on");//PD2
            prische=1;
        }
        else if (strcmp ("sched", &str[b[0]]) == 0 && (strcmp ("off", &str[b[1]]) == 0))
        {
            putsUart0("rr");//PD2
            prische=0;
        }
        else if (strcmp ("pid", &str[b[0]]) == 0)
        {
            putsUart0(" pid number ");
            if (strcmp ("lenghty", &str[b[1]]) == 0)
            {
                nampid = tcb[1].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("idle", &str[b[1]]) == 0)
            {
                nampid = tcb[0].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("flash4hz", &str[b[1]]) == 0)
            {
                nampid = tcb[2].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("oneshot", &str[b[1]]) == 0)
            {
                nampid = tcb[3].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("readkeys", &str[b[1]]) == 0)
            {
                nampid = tcb[4].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("debounce", &str[b[1]]) == 0)
            {
                nampid = tcb[5].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("important", &str[b[1]]) == 0)
            {
                nampid = tcb[6].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("uncoop", &str[b[1]]) == 0)
            {
                nampid = tcb[7].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
            else if (strcmp ("shell", &str[b[1]]) == 0)
            {
                nampid = tcb[8].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
            }
        }
        else if (strcmp ("prsts", &str[b[0]]) == 0)
        {
            int d;
            putsUart0("PID Num");
            putsUart0("\t");
            putsUart0("\t");
            putsUart0("Priority");
            putsUart0("\t");
            putsUart0("Task");
            putsUart0("\r\n");
            for(d=0;d<9;d++)
            {
                nampid = tcb[d].pid;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
                putsUart0("\t");
                putsUart0("\t");
                nampid = tcb[d].priority;
                ltoa(nampid,alpnam);
                putsUart0(alpnam);
                putsUart0("\t");
                putsUart0("\t");
                putsUart0(tcb[d].name);
                putsUart0("\t");
                // nampid=0;
                putsUart0("\r\n");
            }
        }
        else if (strcmp ("killpid", &str[b[0]]) == 0)
        {
            putsUart0(" killing ");
            //  uint8_t a;
            if (strcmp ("lengthy", &str[b[1]]) == 0)
            {
                putsUart0(" lengthy ");
                destroyThread(lengthyFn);
            }
            else  if (strcmp ("idle", &str[b[1]]) == 0)
            {
                putsUart0(" idle ");
                destroyThread(idle);
            }
            else  if (strcmp ("flash4hz", &str[b[1]]) == 0)
            {
                putsUart0(" flash4hz ");
                destroyThread(flash4Hz);
            }
            else  if (strcmp ("oneshot", &str[b[1]]) == 0)
            {
                putsUart0(" oneshot ");
                destroyThread(oneshot);
            }
            else  if (strcmp ("readkeys", &str[b[1]]) == 0)
            {
                putsUart0(" readkeys ");
                destroyThread(readKeys);
                //   a=4;
            }
            else  if (strcmp ("debounce", &str[b[1]]) == 0)
            {
                putsUart0(" debounce ");
                destroyThread(debounce);
                //   a=5;
            }
            else  if (strcmp ("important", &str[b[1]]) == 0)
            {
                putsUart0(" important ");
                destroyThread(important);
                //    a=6;
            }
            else  if (strcmp ("uncoop", &str[b[1]]) == 0)
            {
                putsUart0(" uncoop ");
                destroyThread(uncooperative);
                //  a=7;
            }
            else  if (strcmp ("shell", &str[b[1]]) == 0)
            {
                putsUart0(" shell ");
                destroyThread(shell);
                //  a=8;
            }
        }
        else if (strcmp ("ipcs", &str[b[0]]) == 0)
        {
            putsUart0("process queue");
            putsUart0("\t\t");
            putsUart0("semaphore name");
            putsUart0("\t\t");
            putsUart0("count");
            putsUart0("\t\t");
            putsUart0("queue size");
            putsUart0("\r\n");
            uint16_t ipcs;
            uint16_t ipcsque;
            uint32_t ipcspro;
            uint16_t ipcssize;
            uint8_t ipcs2;
            uint8_t ipcs1;
            for(ipcs1=0;ipcs1<5-1;ipcs1++)
            {
                for(ipcs2=0;ipcs2<5-1;ipcs2++)
                {
                    ipcspro = semaphores[ipcs1].processQueue[ipcs2];
                    char ipcspro_b[80];
                    ltoa(ipcspro,ipcspro_b);
                    putsUart0(ipcspro_b);
                    putsUart0(" ");
                }
                putsUart0("\t\t");
                putsUart0(semaphores[ipcs1].name);
                putsUart0("\t\t");
                ipcs = semaphores[ipcs1].count;
                char ipcs_b[80];
                ltoa(ipcs,ipcs_b);
                putsUart0(ipcs_b);
                putsUart0("\t\t");
                ipcssize = semaphores[ipcs1].queueSize;
                char ipcssize_a[80];
                ltoa(ipcssize,ipcssize_a);
                putsUart0(ipcssize_a);
                putsUart0("\t\t");
                putsUart0("\n\r");
            }
        }
        else
        {
            putsUart0(" Inavalid");
        }
    }
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
int main(void)
{ // Initialize hardware
    initHw();
    bool ok;
    /*   while (1)
    {
        waitPbPress();
    }*/
    rtosInit();
    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);
    // Initialize semaphores
    keyPressed = createSemaphore(1,"KeyPressed");
    keyReleased = createSemaphore(0,"keyReleased");
    flashReq = createSemaphore(5,"flashReq");
    resource = createSemaphore(1,"resource");
    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);
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

