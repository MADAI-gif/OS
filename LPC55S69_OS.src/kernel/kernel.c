#include <stdlib.h>
#include "kernel.h"
#include "list.h"

#ifndef NULL
#define NULL 0
#endif


/*****************************************************************************
 * Global variables
 *****************************************************************************/

static uint32_t id=1;
Task * tsk_running = NULL;	/* pointer to ready task list : the first
                                     node is the running task descriptor */
Task * tsk_prev = NULL;
Task * tsk_sleeping = NULL;	/* pointer to sleeping task list */

/*****************************************************************************
 * SVC dispatch
 *****************************************************************************/
/* sys_add
 *   test function
 */
int sys_add(int a, int b)
{
    return a+b;
}

/* syscall_dispatch
 *   dispatch syscalls
 *   n      : syscall number
 *   args[] : array of the parameters (4 max)
 */
int32_t svc_dispatch(uint32_t n, uint32_t args[])
{
    int32_t result = -1;
    void *buffer;

    switch(n) {
        case 0:  // Test function
            result = sys_add((int)args[0], (int)args[1]);
            break;
        case 1:  // os_alloc (malloc)
            buffer = malloc((size_t)args[0]);
            result = (buffer == NULL) ? -1 : (int32_t)buffer;
            break;
        case 2:  // os_free
            free((void *)args[0]);
            result = 0;
            break;
        case 3:  // os_start
            result = sys_os_start();
            break;
        case 4:  // task_new
            result = sys_task_new((TaskCode)args[0], args[1]);
            break;
        case 5:  // task_id
            result = sys_task_id();
            break;
        case 6:  // task_wait
            result = sys_task_wait(args[0]);
            break;
        case 7:  // task_kill
            result = sys_task_kill();
            break;
        case 8:  // sem_new
            result = (int32_t)sys_sem_new(args[0]);
            break;
        case 9:  // sem_p
            result = sys_sem_p((Semaphore*)args[0]);
            break;
        case 10: // sem_v
            result = sys_sem_v((Semaphore*)args[0]);
            break;
        default:
            result = -1;
            break;
    }
    return result;
}

void sys_switch_ctx()
{
	SCB->ICSR |= 1<<28; // set PendSV to pending
}
/*****************************************************************************
 * Round robin algorithm
 *****************************************************************************/
#define SYS_TICK  10 // system tick in ms pour 1s = 689

uint32_t sys_tick_cnt=0;

/* tick_cb
 *   system tick callback: task switching, ...
 */
void sys_tick_cb()
{
//	Task* prev = tsk_running;
	tsk_running->status = TASK_SLEEPING;
	tsk_prev = tsk_running;
	tsk_running = tsk_prev->next;
	tsk_running->status = TASK_RUNNING;
	sys_switch_ctx();


	//delay management

	int size = list_size(tsk_sleeping);
	for(int i = 0; i < size; ++i)
	{
		tsk_sleeping->delay -= SYS_TICK;
		if (tsk_sleeping->delay <= 0)//if timeout
		{
			tsk_sleeping->delay = 0;//reset delay

			Task* tskDelay;
			//get next sleeping task and store current to tskDelay
			tsk_sleeping = list_remove_head(tsk_sleeping,&tskDelay);
			tskDelay->status = TASK_READY;//set current task to ready
			tsk_running = list_insert_tail(tsk_running,tskDelay);//add it to running tasks
		}
		else//no timeout, just skip it
			tsk_sleeping = tsk_sleeping->next;

		sys_switch_ctx();
	}
}

void SysTick_Handler(void)
{
	sys_tick_cnt++;

	if (sys_tick_cnt == SYS_TICK) {
		sys_tick_cnt = 0;
		sys_tick_cb();
	}
}

/*****************************************************************************
 * General OS handling functions
 *****************************************************************************/

/* sys_os_start
 *   start the first created task
 */
int32_t sys_os_start()
{
	tsk_running->status = TASK_RUNNING;
	sys_switch_ctx();
    // Reset BASEPRI
    __set_BASEPRI(0);

	// Set systick reload value to generate 1ms interrupt
    SysTick_Config(SystemCoreClock / 1000U);
    return 0;
}

/*****************************************************************************
 * Task handling functions
 *****************************************************************************/
void task_kill();

/* sys_task_new
 *   create a new task :
 *   func      : task code to be run
 *   stacksize : task stack size
 *
 *   Stack frame:
 *      |    xPSR    |
 *      |     PC     |
 *      |     LR     |
 *      |     R12    |    ^
 *      |     R3     |    ^
 *      |     R2     |    | @++
 *      |     R1     |
 *      |     R0     |
 *      +------------+
 *      |     R11    |
 *      |     R10    |
 *      |     R9     |
 *      |     R8     |
 *      |     R7     |
 *      |     R6     |
 *      |     R5     |
 *      |     R4     |
 *      +------------+
 *      | EXC_RETURN |
 *      |   CONTROL  | <- sp
 *      +------------+
 */
int32_t sys_task_new(TaskCode func, uint32_t stacksize)
{
	// get a stack with size multiple of 8 bytes
	uint32_t size = stacksize>96 ? 8*(((stacksize-1)/8)+1) : 96;
	
	Task *new_tsk = (Task *)malloc(sizeof(Task) + size);
	if (new_tsk == NULL)
	    return -1;

    // Initialiser le descripteur de tâche
    new_tsk->id = id++;
    new_tsk->status = TASK_READY;
    new_tsk->delay = 0;

    // Initialiser la pile
    //new_task->splim = (uint32_t *)((uint8_t *)new_task + sizeof(Task));
    new_tsk->splim = (uint32_t*)(new_tsk+1);
    new_tsk->sp = new_tsk->splim + (size / sizeof(uint32_t));

    // R0-R12
    // CTRL
    // EXC_RET
    // LR (return addr)
    // PC (task func addr)
    // xPSR

    new_tsk->sp -= 18; // Reserve context space

    new_tsk->sp[0] = (0x0) | (0x1 << 0); // CTRL = unprivileged , voir page 8 doc 0
    new_tsk->sp[1] = 0xFFFFFFFD; // EXC_RET = thread, psp , voir page 14 doc 1
    new_tsk->sp[15] = (uint32_t)task_kill; // LR = return address
    new_tsk->sp[16] = (uint32_t)func; // PC = task function address
    new_tsk->sp[17] = 1 << 24; // xPSR = 1 << 24 = 0x01000000 , voir page 7 doc 1

    tsk_running = list_insert_tail(tsk_running, new_tsk);
    if(tsk_running == NULL)
    {
    	return -1;
    }
    //list_display(tsk_running);
    return new_tsk->id;
}

/* sys_task_kill
 *   kill oneself
 */
int32_t sys_task_kill()
{
    Task *tskToKill;

    tsk_running = list_remove_head(tsk_running, &tskToKill);

    // Check if there was a task to kill
    if (tskToKill == NULL)
        return -1;
    free(tskToKill);

    // Check if the task list is now empty
    if (tsk_running != NULL)
        tsk_running->status = TASK_RUNNING;

    // Switch context
    sys_switch_ctx();
	return 0;
}

/* sys_task_id
 *   returns id of task
 */
int32_t sys_task_id()
{
	if(tsk_running != NULL)
		return tsk_running->id;
    return -1;
}


/* sys_task_yield
 *   run scheduler to switch to another task
 */
int32_t sys_task_yield()
{

    return -1;
}

/* task_wait
 *   suspend the current task until timeout
 */
int32_t sys_task_wait(uint32_t ms)
{
	/* A COMPLETER */
	tsk_running = list_remove_head(tsk_running, &tsk_prev);
	tsk_sleeping = list_insert_tail(tsk_sleeping, tsk_prev);
	tsk_prev->delay = ms;
	tsk_prev->status = TASK_WAITING;
	tsk_running->status = TASK_RUNNING;
	sys_switch_ctx();
    return -1;
}


/*****************************************************************************
 * Semaphore handling functions
 *****************************************************************************/

/* sys_sem_new
 *   create a semaphore
 *   init    : initial value
 */
Semaphore * sys_sem_new(int32_t init)
{
	/* A COMPLETER */
	Semaphore *new_sem = (Semaphore *)malloc(sizeof(Semaphore));
		if (new_sem == NULL)
		    return -1;
	new_sem->count = init;
	new_sem->waiting = NULL;
    return new_sem;
}

/* sys_sem_p
 *   take a token
 */
int32_t sys_sem_p(Semaphore * sem)
{
	/* A COMPLETER */
	--sem->count;
	if(sem->count < 0)
	{
		//transférer de la liste tsk_running à la liste waiting
		Task *task;
		tsk_running = list_remove_head(tsk_running, &task);
		sem->waiting = list_insert_tail(sem->waiting, task);
		sem->waiting->status = TASK_WAITING;
		tsk_running->status = TASK_RUNNING;
		//Débloquer la tâche consiste en l’opération dans l’autre sens
		tsk_prev = task;
		//réaliser la commutation de contexte associée
		sys_switch_ctx();
	}
	return sem->count;
}

/* sys_sem_v
 *   release a token
 */
int32_t sys_sem_v(Semaphore * sem)
{
	++sem->count;
	if(sem->waiting != NULL)
	{
		Task* task;
		sem->waiting = list_remove_head(sem->waiting, &task);
		tsk_prev = tsk_running;

		tsk_running = list_insert_head(tsk_running, task);
		task->status = TASK_RUNNING;
		tsk_prev->status = TASK_READY;

		sys_switch_ctx();
	}
	return sem->count;
}
