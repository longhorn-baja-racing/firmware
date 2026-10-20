#include <string.h>

#include "lbr/transport_links.h"
#include "sdkconfig.h"

#if CONFIG_BT_NIMBLE_ENABLED

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "lbr/protocol.h"
#include "lbr/version.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/*
 * BLE walk-up diagnostics: a GATT server any generic BLE app (e.g. nRF
 * Connect) can read at the car without a cable. Read-only:
 *
 *   service  4c425200-...   LBR status
 *     char   4c425201-...   status text (same as USB STAT / WiFi /status)
 *     char   4c425202-...   versions: "fw=x.y.z protocol=n"
 *
 * Long values are served with ATT read-blob; the stack handles offsets.
 * BLE carries no telemetry stream, so send() is not used.
 */

static const char *TAG = "lbr_ble";

#define LBR_UUID128(last)                                                                          \
    BLE_UUID128_INIT(0x4c, 0x42, 0x52, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x80, 0x5f, \
                     0x9b, 0x34, (last))

static const ble_uuid128_t k_svc_uuid     = LBR_UUID128(0x00);
static const ble_uuid128_t k_status_uuid  = LBR_UUID128(0x01);
static const ble_uuid128_t k_version_uuid = LBR_UUID128(0x02);

static lbr_link_hooks_t s_hooks;
static lbr_transport_t  s_link;
static uint8_t          s_addr_type;
static uint32_t         s_reads;

static int access_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn;
    (void)attr;
    (void)arg;
    static char text[1024];
    size_t      n = 0;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (ble_uuid_cmp(ctxt->chr->uuid, &k_status_uuid.u) == 0) {
        n = s_hooks.status != NULL ? s_hooks.status(text, sizeof text) : 0;
    } else {
        n = (size_t)snprintf(text, sizeof text, "fw=%u.%u.%u protocol=%u", LBR_FW_VERSION_MAJOR,
                             LBR_FW_VERSION_MINOR, LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION);
    }
    s_reads++;
    return os_mbuf_append(ctxt->om, text, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def k_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &k_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {.uuid = &k_status_uuid.u, .access_cb = access_cb, .flags = BLE_GATT_CHR_F_READ},
                {.uuid = &k_version_uuid.u, .access_cb = access_cb, .flags = BLE_GATT_CHR_F_READ},
                {0},
            },
    },
    {0},
};

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    const char              *name   = ble_svc_gap_device_name();
    fields.flags                    = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                     = (uint8_t *)name;
    fields.name_len                 = (uint8_t)strlen(name);
    fields.name_is_complete         = 1;
    if (ble_gap_adv_set_fields(&fields) != 0) {
        ESP_LOGE(TAG, "adv fields rejected");
        return;
    }
    struct ble_gap_adv_params adv = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    if (ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL) != 0) {
        ESP_LOGE(TAG, "advertising failed");
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
        case BLE_GAP_EVENT_ADV_COMPLETE:
            advertise();
            break;
        default:
            break;
    }
    return 0;
}

static void on_sync(void)
{
    if (ble_hs_id_infer_auto(0, &s_addr_type) != 0) {
        ESP_LOGE(TAG, "no BLE address");
        return;
    }
    advertise();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int ble_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    (void)self;
    (void)pkt;
    (void)len;
    return -1; /* diagnostics only: no stream over BLE */
}

static void ble_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    (void)self;
    out->packets_sent = s_reads;
    out->connected    = ble_gap_conn_active();
}

lbr_transport_t *lbr_link_ble_start(const lbr_link_hooks_t *hooks)
{
    s_hooks = *hooks;
    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed");
        return NULL;
    }
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(k_services) != 0 || ble_gatts_add_svcs(k_services) != 0 ||
        ble_svc_gap_device_name_set(CONFIG_LBR_BLE_NAME) != 0) {
        ESP_LOGE(TAG, "GATT setup failed");
        return NULL;
    }
    nimble_port_freertos_init(host_task);
    s_link = (lbr_transport_t){
        .name      = "ble",
        .kind      = LBR_LINK_BLE,
        .send      = ble_send,
        .get_stats = ble_stats,
    };
    return &s_link;
}

#else /* !CONFIG_BT_NIMBLE_ENABLED */

lbr_transport_t *lbr_link_ble_start(const lbr_link_hooks_t *hooks)
{
    (void)hooks;
    return NULL; /* Bluetooth disabled in this build */
}

#endif
