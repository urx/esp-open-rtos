#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <string.h>
#include <semphr.h>
#include <esp/spi.h>
#include <mcp2515/mcp2515.h>
#include <mcp2515/mcp2515_defs.h>

#include <ssid_config.h>
#include "lwip/api.h"

#include <mcp2515/mcp2515_regs.h>
uint8_t mcp2515_read_reg(uint8_t const reg);

#define SECOND (1000 / portTICK_PERIOD_MS)
#define CS_PIN   15
#define INT_PIN  5
#define SPI_BUS  1

uint8_t need_to_change_speed = 0;
uint8_t connected = 0;

typedef struct mcp2515_stats {
  uint32_t merf;
  uint32_t errif;
  uint32_t read_error;
  uint32_t pkts;
  uint32_t rx1ovr;
  uint32_t rx0ovr;
  uint32_t txbo;
  uint32_t txep;
  uint32_t rxep;
  uint32_t txwar;
  uint32_t rxwar;
  uint32_t ewarn;
} mcp2515_stats_t;

QueueHandle_t xQueue_events;
SemaphoreHandle_t inTransaction_semaphore;

mcp2515_stats_t stats = {
  .merf = 0,
  .errif = 0,
  .read_error = 0,
  .pkts = 0,
  .rx1ovr = 0,
  .rx0ovr = 0,
  .txbo = 0,
  .txep = 0,
  .rxep = 0,
  .rxwar = 0,
  .ewarn = 0,
};

static void can_speed_print_verbose(mcp2515_can_speed_t speed)
{
  switch (speed) {
    case CAN_5KBPS:
    printf("5KBPS");
    break;

    case CAN_10KBPS:
    printf("10KBPS");
    break;

    case CAN_20KBPS:
    printf("20KBPS");
    break;

    case CAN_31K25BPS:
    printf("31K25BPS");
    break;

    case CAN_33KBPS:
    printf("33KBPS");
    break;

    case CAN_40KBPS:
    printf("40KBPS");
    break;

    case CAN_50KBPS:
    printf("50KBPS");
    break;

    case CAN_80KBPS:
    printf("80KBPS");
    break;

    case CAN_100KBPS:
    printf("100KBPS");
    break;

    case CAN_125KBPS:
    printf("125KBPS");
    break;

    case CAN_200KBPS:
    printf("200KBPS");
    break;

    case CAN_250KBPS:
    printf("250KBPS");
    break;

    case CAN_500KBPS:
    printf("500KBPS");
    break;

    case CAN_1000KBPS:
    printf("1000KBPS");
    break;

    default:
    printf("unknown");
    break;
  }
}

/*
static void can_speed_brutforce_task(void *pvParameters)
{
  printf("%s: Started can speed changer task\n", __FUNCTION__);

  mcp2515_can_speed_t speed = CAN_5KBPS;
  while (1) {
    vTaskDelay(2*SECOND);

    if (xSemaphoreTake(inTransaction_semaphore, SECOND) != pdTRUE) continue;

    if (!need_to_change_speed) {
      if (xSemaphoreGive(inTransaction_semaphore) != pdTRUE) {
        printf("%s: error on xSemaphoreGive() call\n", __FUNCTION__);
        goto mcp2515_error;
      }
      continue; // no need to change speed
    }

    mcp2515_int_enable(0xFF, 0); // turn off a

    memset(&stats, 0, sizeof(stats));
    need_to_change_speed = 0;
    if (mcp2515_request_operation_mode(CONFIGURATION_MODE) < 0) {
      printf("%s: error while change mcp2515 operation mode to <configuration mode>\n",
              __FUNCTION__);
      goto mcp2515_error;
    }

    printf("Setting CAN speed to: ");
    can_speed_print_verbose(speed);
    printf("\n");
    if (mcp2515_set_can_speed(speed)) {
      printf("%s: error while settings mcp2515 CAN speed\n", __FUNCTION__);
      goto mcp2515_error;
    }

    if (mcp2515_request_operation_mode(LISTEN_ONLY_MODE) < 0) {
      printf("%s: error while change mcp2515 operation mode to <listen_only mode>\n",
              __FUNCTION__);
      goto mcp2515_error;
    }

    mcp2515_clear_can_irq_flags(0xFF); //FIXME: need proper mask here;
    mcp2515_clear_eflg(0xFF);
    mcp2515_int_enable(RX0IE | RX1IE, 1);

    if (++speed == CAN_SPEED_MAX) speed = CAN_5KBPS;

    if (xSemaphoreGive(inTransaction_semaphore) != pdTRUE) {
      printf("%s: error on xSemaphoreGive() call\n", __FUNCTION__);
      goto mcp2515_error;
    }
    
  }

  return;

mcp2515_error:
  while(1) {
    vTaskDelay(5*SECOND);
  }
}
*/


static void udp_server(void *pvParameters)
{
  int err = ERR_OK;
  printf("%s: Started can message dumper task\n", __FUNCTION__);

  while(!connected) {
    vTaskDelay(SECOND);
    printf("WIFI is not connected yet\n");
  }
  //ip_addr_t dhcp_ip;
  //IP4_ADDR(&dhcp_ip, 172,16,5,2);
  //dhcpserver_start(&dhcp_ip, 4);

  struct netconn *conn = netconn_new(NETCONN_TCP);
  if (!conn) {
      printf("Error while creating struct netconn\n");
      goto error;
  }

  err = netconn_bind(conn, IP_ANY_TYPE, 6666);
  if (err != ERR_OK) {
    printf("Unable to bind, no data will be sent (%s)\n", lwip_strerr(err));
    goto error;
  }

  printf("UDP Server is waiting for connection\n");
  netconn_listen(conn);

  while (1) { 
    struct netconn *client = NULL;
    err = netconn_accept(conn, &client);
    if (err != ERR_OK) {
      netconn_delete(client);
      continue;
    }

    printf("Got client!\n");
    ip_addr_t client_addr;
    uint16_t port_ignore;
    netconn_peer(client, &client_addr, &port_ignore);
    while (1) {
      printf("%d %d %d %d\n", stats.pkts, stats.read_error, stats.errif, stats.merf);
      printf("%d %d %d %d %d %d %d %d\n", stats.rx1ovr, stats.rx0ovr, stats.txbo, stats.txep, stats.rxep, stats.txwar, stats.rxwar, stats.ewarn);
      while(uxQueueMessagesWaiting(xQueue_events)) {
	printf("Start sending frames to client\n");
	can_frame_t frame;
	if (xQueueReceive(xQueue_events, &frame, SECOND) == pdTRUE) {
	  netconn_write(client, &frame, sizeof(frame), NETCONN_COPY);
	} else {
	  printf("No data yet\n");
	}
	vTaskDelay(SECOND/10);
      }
    }
  }

error:
  netconn_delete(conn);

  while(1) {
    printf("%s: error_loop\n", __FUNCTION__);
    vTaskDelay(SECOND);
  }
}

void gpio_int_handler(uint8_t gpio_num)
{
  can_frame_t frm;
  uint8_t err = 0;
  uint8_t can_irq_flags = 0;
  uint8_t eflg = 0;
  printf("%s\n", __FUNCTION__);
  //if (xSemaphoreTakeFromISR(inTransaction_semaphore, NULL) != pdTRUE) {
  //    /* we are got intterrupt in the middle of some configuration process. drop it */
  //    return;
 // }

  printf("%s 2\n", __FUNCTION__);
  need_to_change_speed = 1;
  can_irq_flags = mcp2515_read_can_irq_flags();
  eflg = mcp2515_read_eflg();

  if (can_irq_flags & (CANINTF_RX0IF | CANINTF_RX1IF)){

    err = mcp2515_read_can_frame(&frm);
    if (err) {
      stats.read_error ++;
      goto bailout;
    }

    need_to_change_speed = 0;
    
    if (xQueueIsQueueFullFromISR(xQueue_events)) {
      /* nothing to do, we have no space to store received frame */
      goto bailout;
    }

    if (xQueueSend(xQueue_events, (void *)(&frm), 0) == pdFALSE) {
      printf("Queue overflow\n");
    }
    stats.pkts ++;
  }  else if (can_irq_flags & CANINTF_ERRIF) {
    /* pars eflg register here */

    if ( eflg & EFLG_RX1OVR) stats.rx1ovr ++;
    if ( eflg & EFLG_RX0OVR) stats.rx0ovr ++;
    if ( eflg & EFLG_TXBO) stats.txbo ++;
    if ( eflg & EFLG_TXEP) stats.txep ++;
    if ( eflg & EFLG_RXEP) stats.rxep ++;
    if ( eflg & EFLG_TXWAR) stats.txwar ++;
    if ( eflg & EFLG_RXWAR) stats.rxwar ++;
    if ( eflg & EFLG_EWARN) stats.ewarn ++;
    stats.errif ++;
  } else if (can_irq_flags & CANINTF_MERRF) {
    stats.merf ++;
  }

bailout:
  mcp2515_clear_eflg(eflg);
  mcp2515_clear_can_irq_flags(can_irq_flags); // mark irqs as processed

  xSemaphoreGiveFromISR(inTransaction_semaphore, NULL);
  return;
}


static void wifi_ap_task(void *pvParameters)
{
  uint8_t status = 0;
  uint8_t retries = 30;
#if 0
  struct sdk_softap_config ap_config = {
    .ssid = AP_SSID,
    .ssid_hidden = 0,
    .channel = 3,
    .ssid_len = strlen(AP_SSID),
    .authmode = AUTH_WPA_WPA2_PSK,
    .password = AP_PSK,
    .max_connection = 3,
    .beacon_interval = 100,
  };

  printf("Setting up SOFTAP:...");
  sdk_wifi_set_opmode(SOFTAP_MODE);

  struct ip_info ap_ip;
  IP4_ADDR(&ap_ip.ip, 172, 16, 5, 1);
  IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
  IP4_ADDR(&ap_ip.netmask, 255, 255, 0, 0);
  sdk_wifi_set_ip_info(1, &ap_ip);

  sdk_wifi_softap_set_config(&ap_config);
  sdk_wifi_status_led_install(2, PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
  printf("done\n");
#endif

  struct sdk_station_config config = {
    .ssid = WIFI_SSID,
    .password = WIFI_PASS,
  };

  printf("Current mode: %d\n", sdk_wifi_get_opmode());
  printf("Configuring WiFI as STATION...");
  sdk_wifi_set_opmode(STATION_MODE);
  sdk_wifi_station_set_config(&config);
  sdk_wifi_station_connect();

  while(1) {
    while (status != STATION_GOT_IP && retries) {
      status = sdk_wifi_station_get_connect_status();
      printf("status: %d\n", status);
      if(status == STATION_WRONG_PASSWORD) {
	printf("WiFi: wrong password\n");
	break;
      } else if(status == STATION_NO_AP_FOUND) {
	printf("WiFi: AP not found\n");
	break;
      } else if(status == STATION_CONNECT_FAIL) {
	printf("WiFi: connection failed\n");
	break;
      }
      vTaskDelay(SECOND);
      retries--;
    }
    if (status == STATION_GOT_IP) {
      printf("WiFi: connected\n");
      vTaskDelay(SECOND*10);
      connected = 1;
    }
  }
}

void user_init(void)
{
  // Setup HW
  uart_set_baud(0, 115200);
  vTaskDelay(SECOND*2);
  printf("SDK version:%s, free_heap: %u\n", sdk_system_get_sdk_version(), xPortGetFreeHeapSize());

  xTaskCreate(wifi_ap_task, "WiFi task", 512, NULL, 2, NULL);

  printf("Creating xQueue for receiving can messages...");
  xQueue_events = xQueueCreate(20, sizeof(can_frame_t)); // queue for 20 frames
  printf("done\n");

  vSemaphoreCreateBinary(inTransaction_semaphore);

  printf("Initialize SPI...");
  spi_init(1, SPI_MODE0, SPI_FREQ_DIV_125K, 1, SPI_LITTLE_ENDIAN, true);
  gpio_enable(CS_PIN, GPIO_OUTPUT);
  gpio_enable(INT_PIN, GPIO_INPUT);

  printf("done\n");

  printf("Initialize MCP2515...");
  mcp2515_device_t dev = {
    .spi_bus_no = SPI_BUS,
    .cs_pin   = CS_PIN,
  };

  mcp2515_setup_device(&dev);
  if (mcp2515_init(CAN_500KBPS)) {
    printf("MCP2515 init failed, IC unaccessible.\n");
    goto error;
  }

  if (mcp2515_request_operation_mode(LISTEN_ONLY_MODE) < 0) {
    printf("Unable to set MCP2515 to desired mode: %d.\n", LISTEN_ONLY_MODE);
    goto error;
  }
  printf("done\n");

  xTaskCreate(udp_server, "udp server", 512, NULL, 2, NULL);
  //xTaskCreate(can_speed_brutforce_task, "brutforce_task", 512, NULL, 2, NULL);

  gpio_set_interrupt(INT_PIN, GPIO_INTTYPE_EDGE_NEG, gpio_int_handler);
  
  return;

error:
  while(1) {
    printf("%s: some error occured :(\n", __FUNCTION__);
    vTaskDelay(1*SECOND);
  }
}
