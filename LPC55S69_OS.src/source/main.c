#include <stdio.h>
#include "board.h"
#include <string.h>

#include "oslib.h"

#define MAIN_EX12

/*********************************************************************/
#ifdef MAIN_TEST
int res;

int main()
{
	res = test_add(8,7);
	void* buffer = os_alloc(4);
    if (buffer == NULL)
    {
		printf("Échec de l'allocation mémoire.\n");
		return -1;
    }
   	return 0;
}
#endif
/*********************************************************************/
#ifdef MAIN_EX1
void code1()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
}

void code2()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);

        for (int k=0;k<50000;k++);
    }
}

void code3()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);

        for (int k=0;k<50000;k++);
    }
}


int main()
{
    task_new(code1, 512);   // tache 1
    task_new(code2, 512);   // tache 2
    task_new(code3, 512);   // tache 3
    int id_run = task_id();
    printf("tache %d\r\n",(int)id_run);
    os_start();

    return 0;
}
#endif
/*********************************************************************/
#ifdef MAIN_EX2

void code()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
}

int main()
{
    task_new(code, 512);   // tache 1
    task_new(code, 512);   // tache 2
    task_new(code, 512);   // tache 3

    os_start();

    return 0;
}
#endif
/*********************************************************************/
#ifdef MAIN_EX3

Semaphore* sem=NULL;
int cpt=0;
int count;

int sw_user()
{
    static uint16_t btn1_state=0xFFFF;
    btn1_state = (btn1_state<<1) | (GPIO_PinRead(BOARD_SW3_GPIO,BOARD_SW3_GPIO_PORT,BOARD_SW3_GPIO_PIN) & 1) | 0xE000;
    if (btn1_state==0xF000) return 1;
    return 0;
}

void tache1()
{
    while (1) {
    			//printf("Token count before sem_p = %d\r\n", sem->count);
    	        sem_p(sem);
    	        //printf("Token count after sem_p = %d\r\n", sem->count);
    	        printf("Tache 1 : cpt = %d\r\n", ++cpt);
    }
}

void tache2()
{
    while (1) {
    	if (sw_user()) {
    		sem_v(sem);
    	}
    	volatile int k;
    	for (k=0;k<1000;k++) ;
    }
}

int main()
{
	gpio_pin_config_t sw3cfg = { kGPIO_DigitalInput, 0};
	GPIO_PortInit(BOARD_SW3_GPIO,BOARD_SW3_GPIO_PORT);
	GPIO_PinInit(BOARD_SW3_GPIO,BOARD_SW3_GPIO_PORT,BOARD_SW3_GPIO_PIN,&sw3cfg);

    sem = sem_new(0);

    task_new(tache1, 512);
    task_new(tache2, 256);

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX4

Semaphore * mutex=NULL;

void code()
{
    uint32_t id=task_id();

    while(1) {

        sem_p(mutex);
        printf("tache %d\r\n", (int)id);
        sem_v(mutex);

        for (int k=0;k<50000;k++);
    }
}

void dummy()
{
    while (1);
}

int main()
{
    mutex = sem_new(1);

    task_new(code, 512);   // task 1
    task_new(code, 512);   // task 2
    task_new(code, 512);   // task 3
    task_new(dummy, 128);   // task 4

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX5
static void leds_init()
{
	gpio_pin_config_t ledcfg = { kGPIO_DigitalOutput, 1};
	GPIO_PortInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT);
	GPIO_PinInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN,&ledcfg);
}

static void leds(uint32_t val)
{
	// bit 0 of val controls LED RED
	GPIO_PinWrite(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN, (~(val>>0))&1);
	//  bit 1 of val controls LED GREEN
	GPIO_PinWrite(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN, (~(val>>1))&1);
	//  bit 2 of val controls LED BLUE
	GPIO_PinWrite(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN, (~(val>>2))&1);
}

void tache1()
{
	uint32_t state=0;

    while (1) {
    	state=!state;
        leds(state<<2);			// toggle led
        task_wait(200);
    }
}

void tache2()
{
    while (1) {
    }
}


int main()
{
	leds_init();

	task_new(tache1, 256);
    task_new(tache2, 128);

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX6
Semaphore * mutex=NULL;

static void leds_init()
{
	gpio_pin_config_t ledcfg = { kGPIO_DigitalOutput, 1};
	GPIO_PortInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT);
	GPIO_PinInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN,&ledcfg);
}

static void leds(uint32_t val)
{
	// bit 0 of val controls LED RED
	GPIO_PinWrite(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN, (~(val>>0))&1);
	//  bit 1 of val controls LED GREEN
	GPIO_PinWrite(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN, (~(val>>1))&1);
	//  bit 2 of val controls LED BLUE
	GPIO_PinWrite(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN, (~(val>>2))&1);
}

void tache1()
{
    uint32_t id=task_id();

    while (1) {
        sem_p(mutex);
        printf("- tache %d\r\n", (int)id);
        sem_v(mutex);
        leds(0);			// toggle blue led
        task_wait(1500);
        leds(1<<2);			// toggle blue led
        task_wait(1500);
    }
}

void tache2()
{
    uint32_t id=task_id();

    while (1) {
        sem_p(mutex);
        printf("tache %d\r\n", (int)id);
        sem_v(mutex);
        task_wait(500);
    }
}

void tache3()
{
    while(1) { }
}


int main()
{
	leds_init();

    mutex=sem_new(1);

    task_new(tache1, 512);
    task_new(tache2, 512);
    task_new(tache3, 128);

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX7
void code1()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
}

void code2()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
}

void code3()
{
    uint32_t id=task_id();

    while(1) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
}

void code4()
{
    int   i;
    uint32_t id=task_id();

    for (i=0;i<10;i++) {
		printf("tache %d\r\n",(int)id);
        for (int k=0;k<50000;k++);
    }
    printf("end of task %d\r\n",id);
}

int main()
{
    task_new(code1, 512);   // task 1
    task_new(code2, 512);   // task 2
    task_new(code3, 512);   // task 3
    task_new(code4, 512);   // task 4

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX8
char buffer[32];
int fd;

void tache1()
{
	int n, i;

	fd=open("/dev/test", O_READ);
	if (fd!=-1) {
		// lseek(fd,5);
		n=read(fd, buffer, 32);

		for(i=0;i<n;i++) putchar((int)buffer[i]);

		close(fd);
	}

	while (1) { }
}

void idle()
{
    while (1) { }
}

int main()
{
    task_new(tache1,512);
    task_new(idle,0);

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX9

Semaphore * sem;

void tache1()
{
    uint8_t data=1;

    int fd=open("/dev/leds",O_WRONLY);

    while (1) {
        write(fd,&data,1);
        data=data<<1;
        if (data==0x8) data=1;
        task_wait(500);
    }
}

void idle()
{
    while(1) {}
}


int main()
{
    task_new(tache1,256);
    task_new(idle,0);

    os_start();

    return 0;
}

#endif
/*********************************************************************/
#ifdef MAIN_EX10

void tache1()
{
    uint8_t data=1;

    int lfd=open("/dev/leds",O_WRONLY);

    int bfd=open("/dev/swuser", O_RDONLY);

    while (1) {
        int pushed;
        read(bfd,(void*)&pushed,4);    // bloque la tache jusqu'Ã  l'appui sur swcenter
        write(lfd,&data,1);
        data=data<<1;
        if (data==0x8) data=1;
    }
}


void idle()
{
    while(1) {}
}


int main()
{
    task_new(tache1,256);
    task_new(idle,0);

    os_start();

    return 0;
}
#endif
/*********************************************************************/
#ifdef MAIN_EX11

void tache1()
{
    int serial = open("/dev/serial", O_RDWR);
    if (serial < 0) {
        perror("Failed to open /dev/serial");
        return;
    }

    char buff[255];
    char* strDebug = "Ceci est un message\n";
    write(serial, strDebug, strlen(strDebug));

    while (1)
    {
        if (read(serial, buff, 1) > 0)
        {
            write(serial, buff, 1);
        }
    }
    //close(serial);
}

void idle()
{
    while (1){}
}

int main()
{
    task_new(tache1, 1024);
    task_new(idle, 0);
    os_start();
    return 0;
}

#endif
/*********************************************************************/


#include "termio.h"

#ifdef MAIN_EX12

void tache1()
{

	term_init(80,24);
	char string_char[10];
    char c;
    term_printf("hello from termio.c ");

    while (1)
    {
    	readline("<>",string_char,9);
    	term_printf("%s",string_char);

    }

    //close(serial);
}

void idle()
{
    while (1){}
}

int main()
{
    task_new(tache1, 1024);
    task_new(idle, 0);
    os_start();
    return 0;
}

#endif
/*********************************************************************/

/*********************************************************************/
#ifdef MAIN_EX13

void tache1()
{
	int serial = open("/dev/serial",O_WRONLY);
	int afd=open("/dev/accel", O_RDONLY);


    char buff[255];
    char strDebug[] = "Accelerometer Data:\n";
    write(serial, strDebug, strlen(strDebug));

    int32_t accData[3];

    while (1)
    {
        status_t res = read(afd,&accData,sizeof(accData));
        if (res == sizeof(accData)) {
            snprintf(buff, sizeof(buff), "X: %d, Y: %d, Z: %d\r\n", accData[0], accData[1], accData[2]);
            write(serial, buff, strlen(buff));
        } else {
            //snprintf(buff, sizeof(buff), "Failed to read accelerometer data\n");
            //write(serial, buff, strlen(buff));
        }
        task_wait(500);
    }

    //close(serial);
}

void idle()
{
    while (1){}
}

int main()
{
    task_new(tache1, 1024);  // Create task to read accelerometer and send data via UART
    task_new(idle, 0);  // Idle task
    os_start();  // Start the OS and execute tasks
    return 0;
}

#endif



#ifdef MAIN_EX14

extern bool is_sd_card_present;

void tache1()

{

  while (!is_sd_card_present) {
    printf("Waiting for SD card...\n");

    task_wait(100);

  }

  int sd_card_fd = open("/dev/sd", O_RDWR);

  if (sd_card_fd < 0) {

    perror("Failed to open /dev/sd");

    return;

  }

  char write_buffer[] = "This is a test data written to SD card.";
  char read_buffer[255];

  int bytes_written, bytes_read;

  bytes_written = write(sd_card_fd, write_buffer, strlen(write_buffer));
  bytes_read = read(sd_card_fd, read_buffer, sizeof(read_buffer));
  close(sd_card_fd);

}

void idle()

{

  while (1) {

  }

}

int main()

{

  // Initialize pins and peripherals BOARD_
  // Start tasks
  task_new(tache1, 2048);
  task_new(idle, 0);
  // Start the OS scheduler
  os_start();
  return 0;

}

#endif