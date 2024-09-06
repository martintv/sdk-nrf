/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hogp.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(central_hids, LOG_LEVEL_INF);

#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

#define LINK_NUM	8
#define WAIT_TIMEOUT_MS 20000

#define CONN_AUTH_CALLBACKS 0

static struct bt_conn *default_conn;
static struct bt_conn *auth_conn;

enum {
	CONN_INFO_CONNECTED,
	CONN_INFO_DISCONNECTED,
	CONN_INFO_SECURITY_CHANGED,
	CONN_INFO_PAIRING_COMPLETED,
	CONN_INFO_DISCOVER_COMPLETED,
	CONN_INFO_HIDS_CLIENT_READY,

	/* Total number of flags - must be at the end of the enum */
	CONN_INFO_NUM_FLAGS,
};

static atomic_t scanned_count;
typedef struct {
	ATOMIC_DEFINE(flags, CONN_INFO_NUM_FLAGS);
	struct bt_conn *conn;
	struct bt_hogp hogp;
	uint32_t mouse_report_count;
	uint32_t mouse_report_first_ms;
	uint32_t mouse_report_last_ms;
} conn_info_t;
static conn_info_t conn_infos[LINK_NUM];

conn_info_t *get_conn_info(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(conn_infos); i++) {
		if (conn == conn_infos[i].conn) {
			return &conn_infos[i];
		}
	}

	return NULL;
}

static bool all_peers_connected(void)
{
	for (int i = 0; i < ARRAY_SIZE(conn_infos); i++) {
		if (conn_infos[i].conn == NULL) {
			return false;
		}
	}

	return true;
}

void wait_peer_for(struct bt_conn *conn, int flag)
{
	uint32_t duration = 0;
	uint32_t start_time = k_uptime_get_32();

	conn_info_t *conn_info = get_conn_info(conn);
	while (!atomic_test_bit(conn_info->flags, flag) && duration < WAIT_TIMEOUT_MS) {
		k_sleep(K_MSEC(5));
		duration = k_uptime_get_32() - start_time;
	}
}

static conn_info_t *assign_new_conn(struct bt_conn *conn)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (conn_infos[i].conn == NULL) {
			conn_infos[i].conn = conn;
			atomic_set_bit(conn_infos[i].flags, CONN_INFO_CONNECTED);
			return &conn_infos[i];
		}
	}

	return NULL;
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (!filter_match->uuid.match || (filter_match->uuid.count != 1)) {

		LOG_ERR("Invalid device connected");

		return;
	}

	const struct bt_uuid *uuid = filter_match->uuid.uuid[0];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched on UUID 0x%04x.\nAddress: %s connectable: %s",
		BT_UUID_16(uuid)->val, addr, connectable ? "yes" : "no");
	atomic_inc(&scanned_count);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_ERR("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	LOG_INF("Connecting...");
}

static void scan_filter_no_match(struct bt_scan_device_info *device_info, bool connectable)
{
	// LOG_WRN("Scan filter no match!");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match, scan_connecting_error,
		scan_connecting);

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_ERR("Failed to connect to %s, 0x%02x %s\n", addr, conn_err,
			bt_hci_err_to_str(conn_err));
		if (conn == default_conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			/* This demo doesn't require active scan */
			err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)\n", err);
			}
		}
		return;
	} else {
		assign_new_conn(conn);
	}
	LOG_INF("Connected: %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	conn_info_t *conn_info = get_conn_info(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	atomic_set_bit(conn_info->flags, CONN_INFO_DISCONNECTED);
	LOG_INF("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u\n", addr, level);
	} else {
		LOG_ERR("Security failed: %s level %u err %d %s\n", addr, level, err,
			bt_security_err_to_str(err));
	}
	conn_info_t *conn_info = get_conn_info(conn);
	atomic_set_bit(conn_info->flags, CONN_INFO_SECURITY_CHANGED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected, .disconnected = disconnected, .security_changed = security_changed};

static void scan_init(void)
{
	int err;

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1, .scan_param = NULL, .conn_param = BT_LE_CONN_PARAM_DEFAULT};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HIDS);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)\n", err);

		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)\n", err);
	}
}

static uint8_t hogp_notify_cb(struct bt_hogp *hogp, struct bt_hogp_rep_info *rep, uint8_t err,
			      const uint8_t *data)
{
	uint8_t size = bt_hogp_rep_size(rep);

	if (!data) {
		return BT_GATT_ITER_STOP;
	}
	LOG_DBG("Notification, id: %u, size: %u, data: 0x%x 0x%x 0x%x", bt_hogp_rep_id(rep), size,
		data[0], data[1], data[2]);

	struct bt_conn *conn = bt_hogp_conn(hogp);
	conn_info_t *conn_info = get_conn_info(conn);
	conn_info->mouse_report_count++;
	if (conn_info->mouse_report_first_ms == 0) {
		conn_info->mouse_report_first_ms = k_uptime_get_32();
	}
	conn_info->mouse_report_last_ms = k_uptime_get_32();

	return BT_GATT_ITER_CONTINUE;
}

static void hogp_ready_cb(struct bt_hogp *hogp)
{
	int err;
	struct bt_hogp_rep_info *rep = NULL;

	struct bt_conn *conn = bt_hogp_conn(hogp);
	conn_info_t *conn_info = get_conn_info(conn);
	atomic_set_bit(conn_info->flags, CONN_INFO_HIDS_CLIENT_READY);

	LOG_INF("HIDS is ready to work\n");
	while (NULL != (rep = bt_hogp_rep_next(hogp, rep))) {
		if (bt_hogp_rep_type(rep) == BT_HIDS_REPORT_TYPE_INPUT) {
			LOG_INF("Subscribe to report id: %u", bt_hogp_rep_id(rep));
			err = bt_hogp_rep_subscribe(hogp, rep, hogp_notify_cb);
			if (err) {
				LOG_INF("Subscribe error (%d)\n", err);
			}
		}
	}
}

static void hogp_prep_fail_cb(struct bt_hogp *hogp, int err)
{
	LOG_ERR("ERROR: HIDS client preparation failed!\n");
}

/* HIDS client initialization parameters */
static const struct bt_hogp_init_params hogp_init_params = {.ready_cb = hogp_ready_cb,
							    .prep_error_cb = hogp_prep_fail_cb};

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
	conn_info_t *conn_info = get_conn_info(conn);
	atomic_set_bit(conn_info->flags, CONN_INFO_DISCOVER_COMPLETED);

	LOG_INF("The discovery procedure succeeded");

	bt_gatt_dm_data_print(dm);

	bt_hogp_init(&conn_info->hogp, &hogp_init_params);
	err = bt_hogp_handles_assign(dm, &conn_info->hogp);
	if (err) {
		LOG_ERR("Could not init HIDS client object, error: %d\n", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data, error "
			"code: %d\n",
			err);
	}
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_ERR("The service could not be found during the discovery\n");
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("The discovery procedure failed with %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void gatt_discover(struct bt_conn *conn, void *data)
{
	int err;
	err = bt_gatt_dm_start(conn, BT_UUID_HIDS, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("Start discovery procedure fails: %d", err);
	}

	wait_peer_for(conn, CONN_INFO_DISCOVER_COMPLETED);
	wait_peer_for(conn, CONN_INFO_HIDS_CLIENT_READY);
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	uint32_t button = button_state & has_changed;

	if (auth_conn) {
		if (button & KEY_PAIRING_ACCEPT) {
			bt_conn_auth_passkey_confirm(auth_conn);
			LOG_INF("Numeric Match");
		}

		if (button & KEY_PAIRING_REJECT) {
			bt_conn_auth_cancel(auth_conn);
			LOG_INF("Numeric Reject");
		}

		return;
	}
}

#if CONN_AUTH_CALLBACKS
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	conn_info_t *conn_info = get_conn_info(conn);
	atomic_set_bit(conn_info->flags, CONN_INFO_PAIRING_COMPLETED);

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {.pairing_complete = pairing_complete,
							       .pairing_failed = pairing_failed};

int init(void)
{
	int err;

	LOG_INF("Starting Bluetooth Central HIDS example");

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("failed to register authorization callbacks.");
		return err;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization info callbacks.");
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	scan_init();

	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Failed to initialize buttons (err %d)", err);
		return err;
	}

	memset(conn_infos, 0, sizeof(conn_infos));

	return 0;
}

void connect_all(void)
{
	uint32_t duration = 0;
	uint32_t start_time = k_uptime_get_32();
	do {
		bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		k_msleep(5);
		duration = (k_uptime_get_32() - start_time);
	} while (!all_peers_connected() && duration < WAIT_TIMEOUT_MS);

	bt_scan_stop();
}

static void set_security(struct bt_conn *conn, void *data)
{
	int level = BT_SECURITY_L2;
	int err = bt_conn_set_security(conn, level);
	if (!err) {
		LOG_INF("Security level is set to : %u", level);
	} else {
		LOG_ERR("Failed to set security (%d).", err);
	}

	wait_peer_for(conn, CONN_INFO_SECURITY_CHANGED);
	wait_peer_for(conn, CONN_INFO_PAIRING_COMPLETED);
}

static void reset_counters(struct bt_conn *conn, void *data)
{
	conn_info_t *conn_info = get_conn_info(conn);
	conn_info->mouse_report_count = 0;
	conn_info->mouse_report_first_ms = 0;
	conn_info->mouse_report_last_ms = 0;
}

static void print_stats_conn(struct bt_conn *conn, void *data)
{
	conn_info_t *conn_info = get_conn_info(conn);
	const uint8_t conn_idx = bt_conn_index(conn);
	uint32_t avg_report_interval =
		(conn_info->mouse_report_last_ms - conn_info->mouse_report_first_ms) /
		(conn_info->mouse_report_count - 1);
	LOG_INF("Stats Conn: conn_idx:%02d, mouse_report_count:%02d, mouse_report_interval:%02d",
		conn_idx, conn_info->mouse_report_count, avg_report_interval);
}

void print_stats(void)
{
	uint16_t conn_count = 0;
	uint16_t paired_count = 0;
	uint16_t discovered_count = 0;
	uint16_t hids_client_ready_count = 0;

	for (int i = 0; i < ARRAY_SIZE(conn_infos); i++) {
		if (conn_infos[i].conn != NULL) {
			conn_count++;
			if (atomic_test_bit(conn_infos[i].flags, CONN_INFO_PAIRING_COMPLETED)) {
				paired_count++;
			}

			if (atomic_test_bit(conn_infos[i].flags, CONN_INFO_DISCOVER_COMPLETED)) {
				discovered_count++;
			}

			if (atomic_test_bit(conn_infos[i].flags, CONN_INFO_HIDS_CLIENT_READY)) {
				hids_client_ready_count++;
			}
		}
	}
	uint16_t scanned = atomic_get(&scanned_count);
	LOG_INF("Stats Summary: Scanned:%02d, Connected:%02d, Paired:%02d, Discovered:%02d, "
		"HIDS_Ready:%02d",
		scanned, conn_count, paired_count, discovered_count, hids_client_ready_count);
	bt_conn_foreach(BT_CONN_TYPE_LE, print_stats_conn, NULL);
}

int main(void)
{
	int err = init();
	if (err) {
		LOG_ERR("Application init failed:%d\n", err);
		return 0;
	}

	connect_all();

	bt_conn_foreach(BT_CONN_TYPE_LE, set_security, NULL);

	bt_conn_foreach(BT_CONN_TYPE_LE, gatt_discover, NULL);

	bt_conn_foreach(BT_CONN_TYPE_LE, reset_counters, NULL);

	while (1) {
		/* Wait enough to get mouse event reports.*/
		k_msleep(10000);
		print_stats();
	}

	return 0;
}
