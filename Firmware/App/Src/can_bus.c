#include "can_bus.h"
#include "robstride.h"
#include <string.h>

CanBus g_can[2];

/* ------------------------------------------------------------ DWT timing */
void app_dwt_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;   /* unlock DWT on Cortex-M7 */
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t app_micros(void)
{
	return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

void app_delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (SystemCoreClock / 1000000u);

	while ((DWT->CYCCNT - start) < ticks) {
	}
}

/* ------------------------------------------------------------ init */
void can_bus_init_all(void)
{
	memset(g_can, 0, sizeof(g_can));

	spi_dev_init(&g_can[0].spi, CAN1_SPI_HANDLE, CAN1_CS_GPIO_Port, CAN1_CS_Pin);
	spi_dev_init(&g_can[1].spi, CAN2_SPI_HANDLE, CAN2_CS_GPIO_Port, CAN2_CS_Pin);

	g_can[0].idx = 0;
	g_can[1].idx = 1;
	g_can[0].init_err = mcp2515_init(&g_can[0].mcp, &g_can[0].spi, CAN1_INT_GPIO_Port, CAN1_INT_Pin);
	g_can[1].init_err = mcp2515_init(&g_can[1].mcp, &g_can[1].spi, CAN2_INT_GPIO_Port, CAN2_INT_Pin);
}

/* ------------------------------------------------------------ TX */
int can_bus_send(uint8_t bus, const CanFrame *f)
{
	CanBus *b = &g_can[bus];

	if (!b->mcp.ok) {
		return -3;
	}
	/* The MCP2515 has only 2 RX buffers and each command makes a motor reply.
	 * Drain RX before every send so replies to earlier frames don't overflow. */
	can_bus_poll(bus);

	uint32_t t0 = app_micros();
	for (;;) {
		int r = mcp2515_send(&b->mcp, f);

		if (r != -1) {
			return r;           /* 0 ok, -2 SPI error */
		}
		if (app_micros() - t0 > 300) {
			return -1;          /* 3 buffers still full after 300 us */
		}
		can_bus_poll(bus);      /* keep draining RX while we wait */
	}
}

static int wait_tx_idle(uint8_t bus, uint32_t timeout_us)
{
	uint32_t t0 = app_micros();

	for (;;) {
		can_bus_poll(bus);   /* always drain RX, even if TX is already idle */
		if (mcp2515_tx_idle(&g_can[bus].mcp)) {
			return 0;
		}
		if (app_micros() - t0 > timeout_us) {
			return -1;
		}
	}
}

int can_bus_send_ordered(uint8_t bus, const CanFrame *f)
{
	if (!g_can[bus].mcp.ok) {
		return -3;
	}
	wait_tx_idle(bus, 2000);
	int r = mcp2515_send(&g_can[bus].mcp, f);
	wait_tx_idle(bus, 2000);
	return r;
}

/* ------------------------------------------------------------ RX */
void can_bus_poll(uint8_t bus)
{
	CanBus *b = &g_can[bus];

	if (!b->mcp.ok) {
		return;
	}
	for (int n = 0; n < 8 && mcp2515_int_active(&b->mcp); n++) {
		CanFrame f;
		int r = mcp2515_recv(&b->mcp, &f);

		if (r == 1) {
			b->last_rx_ms = HAL_GetTick();
			if (f.ext) {
				rs_on_frame(bus, &f);
			}
		} else {
			/* INT low but no frame -> error interrupt */
			mcp2515_service_errors(&b->mcp);
			break;
		}
	}
}

void can_bus_health_update(void)
{
	for (int i = 0; i < 2; i++) {
		if (g_can[i].mcp.ok) {
			mcp2515_read_counters(&g_can[i].mcp);
		}
	}
}

/* ------------------------------------------------------------ identification */
void can_bus_mark_seen(uint8_t bus, uint8_t motor_id)
{
	CanBus *b = &g_can[bus];

	if (b->scanning && motor_id < 128) {
		b->scan_bitmap[motor_id >> 3] |= (uint8_t)(1u << (motor_id & 7));
	}
}

int can_bus_scan(uint8_t bus, uint8_t *ids, uint8_t max_ids)
{
	CanBus *b = &g_can[bus];

	if (!b->mcp.ok) {
		return -1;
	}
	memset(b->scan_bitmap, 0, sizeof(b->scan_bitmap));
	b->scanning = 1;

	uint32_t fail_before = b->mcp.st.tx_fail;
	for (uint8_t id = 1; id < 128; id++) {
		CanFrame f;

		rs_make_frame(&f, RS_T_GET_ID, rs_host_id, id);
		can_bus_send(bus, &f);
		app_delay_us(250);
		can_bus_poll(bus);
	}
	uint32_t t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 50) {
		can_bus_poll(bus);
	}
	b->scanning = 0;

	/* every frame failed (nobody ACKed) -> wiring / bitrate / crystal problem */
	if (b->mcp.st.tx_fail - fail_before > 100) {
		return -1;
	}
	int n = 0;
	for (uint8_t id = 1; id < 128 && n < max_ids; id++) {
		if (b->scan_bitmap[id >> 3] & (1u << (id & 7))) {
			ids[n++] = id;
		}
	}
	return n;
}
