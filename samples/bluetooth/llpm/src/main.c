/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/console/console.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/random/random.h>
#include <zephyr/kernel.h>
#include <bluetooth/services/latency.h>
#include <bluetooth/services/latency_client.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/hci_vs_sdc.h>

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define INTERVAL_MIN      0x6    /* 6 units,  7.5 ms */
#define INTERVAL_MIN_US  7500    /* 7.5 ms */
#define INTERVAL_LLPM  0x0D01    /* Proprietary  1 ms */
#define INTERVAL_LLPM_US 1000


static K_SEM_DEFINE(m_test_ready_sem, 0, 1);
static K_MUTEX_DEFINE(m_dm_mutex);
static struct bt_conn *m_central_conn;
static struct bt_conn *m_peripheral_conn;
static struct bt_latency m_latency;
static struct bt_latency_client m_latency_client_central = {0};
static struct bt_latency_client m_latency_client_peripheral = {0};
static struct bt_le_ext_adv *m_ext_adv;
static struct bt_le_conn_param *m_conn_param =
	BT_LE_CONN_PARAM(INTERVAL_LLPM, INTERVAL_LLPM, 0, 100);
static struct bt_conn_info m_conn_info = {0};

/* Track test readiness for each connection */
static bool m_central_test_ready;
static bool m_peripheral_test_ready;

/* Queue for pending discovery starts */
static struct {
	struct bt_conn *conn;
	struct bt_latency_client *latency_client;
	bool pending;
} m_pending_discovery = {0};

/* Work item to start pending discovery (deferred from callback) */
static void pending_discovery_work_handler(struct k_work *work);
static K_WORK_DEFINE(m_pending_discovery_work, pending_discovery_work_handler);

/* Dedicated work queue for advertiser switching to avoid blocking system work queue */
static K_KERNEL_STACK_DEFINE(adv_switch_workq_stack, 2048);
static struct k_work_q adv_switch_workq;

static void adv_switch_handler(struct k_work *work)
{
	int err;
	printk("Switching to connectable advertising\n");
	
	/* Switch to extended connectable advertising (no scan response) */
	const struct bt_le_adv_param conn_adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN,
		0x0020,
		0x0020,
		NULL
	);
	const struct bt_data ad_conn[] = {
		/* Put name and UUID in advertising data */
		{ .type = BT_DATA_NAME_COMPLETE, .data = DEVICE_NAME, .data_len = DEVICE_NAME_LEN },
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LATENCY_VAL),
	};

	err = bt_le_ext_adv_stop(m_ext_adv);
	if (err) {
		printk("Ext adv stop failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_update_param(m_ext_adv, &conn_adv_param);
	if (err) {
		printk("Ext adv update param failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(m_ext_adv, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
	if (err) {
		printk("Ext adv set data (conn) failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_start(m_ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Ext adv start (conn) failed (err %d)\n", err);
		return;
	}
}

static K_WORK_DELAYABLE_DEFINE(m_adv_switch_work, adv_switch_handler);

static void scanned_cb(struct bt_le_ext_adv *adv,
		       struct bt_le_ext_adv_scanned_info *info)
{
	static int count = 0;
	count++;
	if (count < 50) {
		return;
	}
	printk("Scanned %d times\n", count);
	/* Keep callback minimal - printk can block and slow down event processing */
	/* Cancel the delayed work and switch immediately */
	k_work_cancel_delayable(&m_adv_switch_work);
	/* Schedule the work to run immediately on dedicated work queue (no delay) */
	/* This avoids blocking the BT work queue that processes RX events */
	k_work_schedule_for_queue(&adv_switch_workq, &m_adv_switch_work, K_NO_WAIT);
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.scanned = scanned_cb,
};

static const struct bt_data m_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static const struct bt_data m_sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_FALSE_LATENCY_VAL),
};

static struct {
	uint32_t latency;
	uint32_t crc_mismatches;
} m_llpm_latency;

void scan_filter_match(struct bt_scan_device_info *device_info,
		       struct bt_scan_filter_match *filter_match,
		       bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %d\n",
	       addr, connectable);
}

void scan_filter_no_match(struct bt_scan_device_info *device_info,
			  bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	//printk("Filter does not match. Address: %s connectable: %d\n",
	//       addr, connectable);
}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
		scan_connecting_error, NULL);

static void scan_init(void)
{
	int err;
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = 0x0011,
		.window = 0x0010,
	};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = true,
		.scan_param = &scan_param,
		.conn_param = m_conn_param
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_LATENCY);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

/* Forward declarations */
static void discovery_complete(struct bt_gatt_dm *dm, void *context);
static void discovery_service_not_found(struct bt_conn *conn, void *context);
static void discovery_error(struct bt_conn *conn, int err, void *context);
static void pending_discovery_work_handler(struct k_work *work);

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	struct bt_latency_client *latency = context;
	struct bt_conn *conn;
	struct bt_conn_info conn_info;
	bool is_central = false;

	if (!dm || !latency) {
		return;
	}

	/* Get connection info BEFORE releasing dm */
	conn = bt_gatt_dm_conn_get(dm);
	if (!conn) {
		return;
	}

	if (bt_conn_get_info(conn, &conn_info) != 0) {
		return;
	}

	is_central = (conn_info.role == BT_CONN_ROLE_CENTRAL);

	/* Skip bt_gatt_dm_data_print to save stack - it's just for debugging */
	bt_latency_handles_assign(dm, latency);
	bt_gatt_dm_data_release(dm);

	/* Use the connection stored in latency client (set by handles_assign) */
	if (!latency->conn) {
		k_mutex_unlock(&m_dm_mutex);
		return;
	}

	/* Mark test ready for the appropriate connection */
	if (is_central) {
		m_central_test_ready = true;
	} else {
		m_peripheral_test_ready = true;
	}

	/* Signal that a connection is ready */
	k_sem_give(&m_test_ready_sem);

	/* Release mutex */
	k_mutex_unlock(&m_dm_mutex);

	/* Schedule pending discovery to run in a work queue (not in BT RX context) */
	if (m_pending_discovery.pending) {
		k_work_submit(&m_pending_discovery_work);
	}
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	struct bt_latency_client *latency = context;
	
	ARG_UNUSED(conn);
	
	if (latency) {
		/* Clear the latency client on error */
		memset(latency, 0, sizeof(*latency));
	}

	/* Release mutex */
	k_mutex_unlock(&m_dm_mutex);

	/* Schedule pending discovery to run in a work queue (not in BT RX context) */
	if (m_pending_discovery.pending) {
		k_work_submit(&m_pending_discovery_work);
	}
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	struct bt_latency_client *latency = context;
	
	ARG_UNUSED(conn);
	ARG_UNUSED(err);
	
	if (latency) {
		/* Clear the latency client on error */
		memset(latency, 0, sizeof(*latency));
	}

	/* Release mutex */
	k_mutex_unlock(&m_dm_mutex);

	/* Schedule pending discovery to run in a work queue (not in BT RX context) */
	if (m_pending_discovery.pending) {
		k_work_submit(&m_pending_discovery_work);
	}
}

static void pending_discovery_work_handler(struct k_work *work)
{
	struct bt_conn *pending_conn;
	struct bt_latency_client *pending_latency;
	int pending_err;

	ARG_UNUSED(work);

	if (!m_pending_discovery.pending || !m_pending_discovery.conn) {
		return;
	}

	pending_conn = m_pending_discovery.conn;
	pending_latency = m_pending_discovery.latency_client;
	
	m_pending_discovery.pending = false;
	m_pending_discovery.conn = NULL;
	m_pending_discovery.latency_client = NULL;

	printk("Starting queued discovery\n");
	if (k_mutex_lock(&m_dm_mutex, K_NO_WAIT) == 0) {
		pending_err = bt_gatt_dm_start(pending_conn, BT_UUID_LATENCY, 
					       &discovery_cb, pending_latency);
		if (pending_err) {
			printk("Queued discovery start failed (err %d)\n", pending_err);
			k_mutex_unlock(&m_dm_mutex);
			/* Release connection reference on error */
			bt_conn_unref(pending_conn);
		}
		/* On success, discovery_complete will handle connection reference */
	} else {
		/* Mutex locked, shouldn't happen but handle it */
		printk("Failed to lock mutex for queued discovery\n");
		bt_conn_unref(pending_conn);
	}
}

static void adv_start(void)
{
	int err;

	/* Extended non-connectable scannable advertising.
	 * Note: Extended advertising cannot be both connectable AND scannable,
	 * but scannable-only is supported.
	 * Note: Zephyr validation incorrectly rejects advertising data when scannable,
	 * so we must use empty advertising data and put everything in scan response.
	 * This will trigger AUX_ADV_IND and chained AUX packets on secondary channels.
	 * Device name and UUID are in scan response.
	 * BT_LE_ADV_OPT_NOTIFY_SCAN_REQ enables scan request notifications.
	 */
	const struct bt_le_adv_param long_adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_NOTIFY_SCAN_REQ,
		0x0030,
		0x0030,
		NULL);
	
	/* Advertising data: empty (Zephyr bug: rejects ad_len when scannable)
	 * All data must be in scan response due to validation bug in adv.c:1649
	 */
	const struct bt_data *ad_ext = NULL;
	size_t ad_len = 0;

	err = bt_le_ext_adv_create(&long_adv_param, &adv_cb, &m_ext_adv);
	if (err) {
		printk("Ext adv create failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(m_ext_adv, ad_ext, ad_len, m_sd, ARRAY_SIZE(m_sd));
	if (err) {
		printk("Ext adv set data (long) failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_start(m_ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Ext adv start (long) failed (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
	
	/* Schedule switch to connectable advertising after a delay as fallback.
	 * If a scan request is received, scanned_cb will switch immediately.
	 */
	//k_work_reschedule_for_queue(&adv_switch_workq, &m_adv_switch_work, K_SECONDS(5));
}

static void scan_start(void)
{
	int err;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info conn_info;
	struct bt_latency_client *latency_client;
	bool is_central;

	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		NVIC_SystemReset();
		return;
	}

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		printk("Getting conn info failed (err %d)\n", err);
		return;
	}

	is_central = (conn_info.role == BT_CONN_ROLE_CENTRAL);

	/* Store connection reference based on role */
	if (is_central) {
		if (m_central_conn) {
			printk("Warning: Central connection already exists, releasing old one\n");
			bt_conn_unref(m_central_conn);
		}
		/* Clear old latency client data */
		memset(&m_latency_client_central, 0, sizeof(m_latency_client_central));
		m_central_conn = bt_conn_ref(conn);
		latency_client = &m_latency_client_central;
		printk("Connected as CENTRAL\n");
	} else {
		if (m_peripheral_conn) {
			printk("Warning: Peripheral connection already exists, releasing old one\n");
			bt_conn_unref(m_peripheral_conn);
		}
		/* Clear old latency client data */
		memset(&m_latency_client_peripheral, 0, sizeof(m_latency_client_peripheral));
		m_peripheral_conn = bt_conn_ref(conn);
		latency_client = &m_latency_client_peripheral;
		printk("Connected as PERIPHERAL\n");
	}

	/* Do NOT stop scanning or advertising - we want both roles active */
	/* Start service discovery for this connection */
	/* Note: bt_gatt_dm only supports one discovery at a time, so we need to serialize */
	if (k_mutex_lock(&m_dm_mutex, K_NO_WAIT) != 0) {
		/* Another discovery is in progress, queue this one */
		printk("Discovery in progress, queuing discovery for %s\n",
		       is_central ? "central" : "peripheral");
		m_pending_discovery.conn = bt_conn_ref(conn);
		m_pending_discovery.latency_client = latency_client;
		m_pending_discovery.pending = true;
		return;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_LATENCY, &discovery_cb, latency_client);
	if (err) {
		printk("GATT discovery start failed (err %d)\n", err);
		k_mutex_unlock(&m_dm_mutex);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info conn_info;
	bool is_central;

	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	/* Determine which connection was disconnected */
	if (bt_conn_get_info(conn, &conn_info) == 0) {
		is_central = (conn_info.role == BT_CONN_ROLE_CENTRAL);
		
		if (is_central) {
			if (m_central_conn == conn) {
				bt_conn_unref(m_central_conn);
				m_central_conn = NULL;
				m_central_test_ready = false;
				memset(&m_latency_client_central, 0, sizeof(m_latency_client_central));
				printk("Central connection released\n");
			}
		} else {
			if (m_peripheral_conn == conn) {
				bt_conn_unref(m_peripheral_conn);
				m_peripheral_conn = NULL;
				m_peripheral_test_ready = false;
				memset(&m_latency_client_peripheral, 0, sizeof(m_latency_client_peripheral));
				printk("Peripheral connection released\n");
			}
		}

		/* Clean up pending discovery if it's for this connection */
		if (m_pending_discovery.conn == conn) {
			printk("Cleaning up pending discovery for disconnected connection\n");
			bt_conn_unref(m_pending_discovery.conn);
			m_pending_discovery.conn = NULL;
			m_pending_discovery.latency_client = NULL;
			m_pending_discovery.pending = false;
		}
	}

	/* Reset if both connections are lost */
	if (!m_central_conn && !m_peripheral_conn) {
		printk("All connections lost, resetting\n");
#if !defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
		__NVIC_SystemReset();
#endif
	}
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info conn_info;
	const char *role_str = "unknown";

	if (bt_conn_get_info(conn, &conn_info) == 0) {
		role_str = (conn_info.role == BT_CONN_ROLE_CENTRAL) ? "central" : "peripheral";
	}

	if (interval == INTERVAL_LLPM) {
		printk("[%s] Connection interval updated: LLPM (1 ms)\n", role_str);
	} else {
		__ASSERT_NO_MSG(interval == INTERVAL_MIN);
		printk("[%s] Connection interval updated: 7.5 ms\n", role_str);
	}
}

static int enable_llpm_mode(void)
{
	int err;
	sdc_hci_cmd_vs_llpm_mode_set_t cmd_enable;

	cmd_enable.enable = true;

	err = hci_vs_sdc_llpm_mode_set(&cmd_enable);
	if (err) {
		return err;
	}

	printk("LLPM mode enabled\n");
	return 0;
}

static int vs_change_connection_interval(struct bt_conn *conn, uint16_t interval_us)
{
	int err;
	uint16_t conn_handle;
	sdc_hci_cmd_vs_conn_update_t cmd_conn_update;

	if (!conn) {
		printk("Connection is NULL\n");
		return -EINVAL;
	}

	err = bt_hci_get_conn_handle(conn, &conn_handle);
	if (err) {
		printk("Failed obtaining conn_handle (err %d)\n", err);
		return err;
	}

	cmd_conn_update.conn_handle         = conn_handle;
	cmd_conn_update.conn_interval_us    = interval_us;
	cmd_conn_update.conn_latency        = 0;
	cmd_conn_update.supervision_timeout = 300;

	err = hci_vs_sdc_conn_update(&cmd_conn_update);
	if (err) {
		printk("Update connection parameters failed (err %d)\n", err);
		return err;
	}

	return 0;
}

static bool on_vs_evt(struct net_buf_simple *buf)
{
	uint8_t code;
	sdc_hci_subevent_vs_qos_conn_event_report_t *evt;

	code = net_buf_simple_pull_u8(buf);
	if (code != SDC_HCI_SUBEVENT_VS_QOS_CONN_EVENT_REPORT) {
		return false;
	}

	evt = (void *)buf->data;
	m_llpm_latency.crc_mismatches += evt->crc_error_count;

	return true;
}

static void latency_response_handler(const void *buf, uint16_t len)
{
	uint32_t latency_time;

	if (len == sizeof(latency_time)) {
		/* compute how long the time spent */
		latency_time = *((uint32_t *)buf);
		uint32_t cycles_spent = k_cycle_get_32() - latency_time;
		m_llpm_latency.latency =
			(uint32_t)k_cyc_to_ns_floor64(cycles_spent) / 2000;
	}
}

static const struct bt_latency_client_cb latency_client_cb = {
	.latency_response = latency_response_handler
};

static void test_run_connection(struct bt_conn *conn, struct bt_latency_client *latency_client, const char *role_str)
{
	int err;
	uint32_t num_packets = (sys_rand32_get() % 5U) + 1U; /* 1-5 packets */
	struct bt_conn *write_conn;
	uint16_t write_handle;
	
	if (!conn || !latency_client) {
		printk("[%s] Connection or client pointer is NULL\n", role_str);
		return;
	}

	if (!latency_client->conn) {
		printk("[%s] Latency client has no connection\n", role_str);
		return;
	}

	/* Verify the connection matches */
	if (latency_client->conn != conn) {
		printk("[%s] Connection mismatch: client->conn != conn\n", role_str);
		return;
	}

	if (latency_client->handle == 0) {
		printk("[%s] Latency client has invalid handle\n", role_str);
		return;
	}

	/* Save connection and handle to local variables to avoid race condition
	 * where disconnect callback zeros latency_client between check and use
	 */
	write_conn = latency_client->conn;
	write_handle = latency_client->handle;
	
	for (uint32_t i = 0; i < num_packets; i++) {
		/* Check connection state before each write to handle disconnection */
		struct bt_conn_info conn_info;
		if (bt_conn_get_info(write_conn, &conn_info) != 0) {
			printk("[%s] Failed to get connection info, stopping writes\n", role_str);
			return;
		}
		if (conn_info.state != BT_CONN_STATE_CONNECTED) {
			printk("[%s] Connection not connected (state %d), stopping writes\n", role_str, conn_info.state);
			return;
		}

		uint8_t payload[8];
		uint32_t now = k_cycle_get_32();
		memcpy(&payload[0], &now, sizeof(now));
		uint32_t rnd = sys_rand32_get();
		memcpy(&payload[4], &rnd, sizeof(rnd));

		err = bt_gatt_write_without_response(write_conn,
						     write_handle,
						     payload, sizeof(payload),
						     false);
		if (err) {
			printk("[%s] Write wo rsp failed (err %d)\n", role_str, err);
			return;
		}
	}
}

static void test_run(void)
{
	bool central_ready = m_central_test_ready;
	bool peripheral_ready = m_peripheral_test_ready;

	if (!central_ready && !peripheral_ready) {
		/* No connections ready */
		return;
	}

	/* Reset readiness flags */
	m_central_test_ready = false;
	m_peripheral_test_ready = false;

	/* Run test for both connections if they're ready */
	while (m_central_conn || m_peripheral_conn) {
		/* Test central connection */
		if (m_central_conn && m_latency_client_central.conn) {
			test_run_connection(m_central_conn, &m_latency_client_central, "central");
		}

		/* Test peripheral connection */
		if (m_peripheral_conn && m_latency_client_peripheral.conn) {
			test_run_connection(m_peripheral_conn, &m_latency_client_peripheral, "peripheral");
		}

		/* Sleep random 700us - 5000us */
		uint32_t sleep_us = 700U + (sys_rand32_get() % (10000U - 700U + 1U));
		k_usleep(sleep_us);
	}
}

#if defined(CONFIG_BT_SMP)
void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	printk("Security changed: level %i, err: %i %s\n", level, err, bt_security_err_to_str(err));

	if (err != 0) {
		printk("Failed to encrypt link\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_PAIRING_NOT_SUPPORTED);
		return;
	}
	/*Start service discovery when link is encrypted*/
	/* Discovery is now handled in connected() callback */
}
#endif /* CONFIG_BT_SMP */

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed,
#endif /* CONFIG_BT_SMP */
};

int main(void)
{
	int err;

#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	/* Poll if the DTR flag was set, optional */
	while (!dtr) {
		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		k_msleep(100);
	}
#endif

	//console_init();

	printk("Starting Bluetooth LLPM sample\n");

	/* Initialize dedicated work queue for advertiser switching */
	k_work_queue_init(&adv_switch_workq);
	k_work_queue_start(&adv_switch_workq, adv_switch_workq_stack,
			   K_KERNEL_STACK_SIZEOF(adv_switch_workq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&adv_switch_workq.thread, "adv_switch");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	err = bt_latency_init(&m_latency, NULL);
	if (err) {
		printk("Latency service initialization failed (err %d)\n", err);
		return 0;
	}

	err = bt_latency_client_init(&m_latency_client_central, &latency_client_cb);
	if (err) {
		printk("Latency client (central) initialization failed (err %d)\n", err);
		return 0;
	}

	err = bt_latency_client_init(&m_latency_client_peripheral, &latency_client_cb);
	if (err) {
		printk("Latency client (peripheral) initialization failed (err %d)\n", err);
		return 0;
	}

	if (enable_llpm_mode()) {
		printk("Enable LLPM mode failed.\n");
		return 0;
	}

	printk("Starting both scanning and advertising\n");

	printk("Central. Starting scanning\n");
	scan_init();
	scan_start();
	printk("Peripheral. Starting advertising\n");
	adv_start();

	for (;;) {
		k_sem_take(&m_test_ready_sem, K_FOREVER);
		test_run();
	}
}
