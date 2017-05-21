
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <arch/byteorder.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>
#include <nuttx/time.h>
#include <nuttx/wqueue.h>
#include <arch/board/mods.h>
#include <stm32_exti.h>

#include "hdk.h"

#define MODIST_RAW_NAME           "MODIST"
#define MODIST_RAW_DRIVER_VERSION 0x01
#define MODIST_RAW_MAX_LATENCY    2000

//Command IDs
#define MODIST_RAW_COMMAND_INVALID 0x00
#define MODIST_RAW_COMMAND_INFO    0x01
#define MODIST_RAW_COMMAND_SONAR   0x02

static struct work_s data_report_work;

struct modist_raw_info {
  struct device *gDevice;     //device handler
  uint16_t interval;          //reporting interval
  raw_send_callback gCallback;
  uint8_t client_verified;
};

struct modist_raw_msg {
    uint8_t     cmd_id;
    uint8_t     size;
    uint8_t     payload[];
} __packed;

struct timespec last_clock;
struct device *clock_dev;

//Send Data
static int modist_raw_send(struct device *dev, uint32_t len, uint8_t data[])
{
  printf("Sending\n");
  struct modist_raw_info *info = NULL;

  if(!dev || !device_get_private(dev)) return -ENODEV;
  info = device_get_private(dev);

  if( info->gCallback )
  {
    info->gCallback( info->gDevice, len, data);
  }

  printf("Sent\n");
  return 0;
};

static void sonar_response(void* args)
{
  printf("it came back crawling 2: +backing +crawlous\n");
  struct timespec new_clock;
  clock_gettime(CLOCK_REALTIME, &new_clock);
  if( clock_dev )
  {
    uint32_t secs = timespec_to_usec( &new_clock ) - timespec_to_usec( &last_clock );
    printf("%d\n", secs);
    printf("it hit");
    uint8_t data[4] = {0};
    int i;
    for (i = 0; i<4;++i) data[i] = ((uint8_t*)&secs)[3-i];
    printf( "%d", data );
    modist_raw_send( clock_dev, sizeof( data ), data );
    //clock_dev = NULL;
  }
  gpio_direction_out( CALC_GPIO_NUM('E',8), 1 );
}

int sonar_interrupt( int dis, void* ard )
{
  if(!work_available(&data_report_work))
    work_cancel(LPWORK, &data_report_work );
  work_queue(LPWORK, &data_report_work, sonar_response, NULL, 0);
  return 0;
};

//probe
static int modist_raw_probe(struct device *dev)
{
  struct modist_raw_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->client_verified = 0;
    info->gDevice = dev;
    device_set_private(dev, info);

    gpio_direction_out( CALC_GPIO_NUM('A',1), 0 );
    gpio_direction_in( CALC_GPIO_NUM('A',2) );
    //gpio_irqattach( CALC_GPIO_NUM('A',2), sonar_interrupt );
    stm32_gpiosetevent( CALC_GPIO_NUM('A',2), true, false, false, sonar_interrupt );
    gpio_direction_out( CALC_GPIO_NUM('E',8), 0 );
    clock_dev = NULL;

    gpio_set_value( CALC_GPIO_NUM('A',1), 0 );
    gpio_set_value( CALC_GPIO_NUM('A',1), 1 );
    usleep(10);
    gpio_set_value( CALC_GPIO_NUM('A',1), 0 );

    gpio_set_value( CALC_GPIO_NUM('E',8), 1 );

    printf("Probe complete\n");

    return 0;
};

static void modist_raw_remove(struct device *dev)
{
    printf("Entering remove\n");
    struct modist_raw_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    gpio_deactivate( CALC_GPIO_NUM('A',1) );
    gpio_deactivate( CALC_GPIO_NUM('A',2) );
    gpio_deactivate( CALC_GPIO_NUM('E',8) );
    free(info);
    device_set_private(dev, NULL);
}

//Receive data
static int modist_raw_recv( struct device *dev, uint32_t len, uint8_t data[])
{
  struct modist_raw_msg *smsg;

  smsg = (struct modist_raw_msg *)&data[0];

   if ((len == 0) || (len < (sizeof(struct modist_raw_msg))))
       return -EINVAL;

   printf("recv\n");
   gpio_set_value( CALC_GPIO_NUM('E',8), 0 );
   switch (smsg->cmd_id) {
     case MODIST_RAW_COMMAND_INFO:
      printf("MODIST INFO COMMAND\n");
      modist_raw_send( dev, smsg->size, smsg->payload );
      break;
     case MODIST_RAW_COMMAND_SONAR:
      printf("MODIST SONAR COMMAND\n");

      gpio_set_value( CALC_GPIO_NUM('A',1), 0 );
      gpio_set_value( CALC_GPIO_NUM('A',1), 1 );
      usleep(10);
      gpio_set_value( CALC_GPIO_NUM('A',1), 0 );
      clock_gettime(CLOCK_REALTIME, &last_clock);
      clock_dev = dev;
      //modist_raw_send( dev, smsg->size, smsg->payload );
   }

   return 0;
};


//Register the callback function
static int modist_raw_register_callback(struct device *dev, raw_send_callback callback)
{
  printf("Registering Callback\n");
  struct modist_raw_info *info = NULL;

  if(!dev || !device_get_private(dev)) return -ENODEV;
  info = device_get_private(dev);

  info->gCallback = callback;
  return 0;
};

//Unregister the callback function
static int modist_raw_unregister_callback(struct device *dev)
{
  printf("Unregistering CAllback\n");
  struct modist_raw_info *info = NULL;

  if(!dev || !device_get_private(dev)) return -ENODEV;

  info = device_get_private(dev);

  info->gCallback = NULL;
  return 0;
};


static struct device_raw_type_ops modist_raw_type_ops =
{
  .recv = modist_raw_recv,
  .register_callback = modist_raw_register_callback,
  .unregister_callback = modist_raw_unregister_callback,
};

static struct device_driver_ops modist_driver_ops =
{
  .probe = modist_raw_probe,
  .remove = modist_raw_remove,
  .type_ops = &modist_raw_type_ops,
};

struct device_driver mods_raw_modist_driver =
{
  .type = DEVICE_TYPE_RAW_HW,
  .name = "mods_raw_modist",
  .desc = "Modist Raw Interface",
  .ops = &modist_driver_ops,
};
