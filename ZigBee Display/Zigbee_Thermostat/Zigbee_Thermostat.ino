#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"   // legacy: #include "SH1106.h"

#define MAX_SENSORS 2


#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))

// Display Initialisieren
SH1106Wire display(0x3c, SDA, SCL);     // ADDRESS, SDA, SCL


/* Default Coordinator config */
#define ESP_ZB_ZC_CONFIG()                                                                                        \
  {                                                                                                               \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR, .install_code_policy = INSTALLCODE_POLICY_ENABLE, .nwk_cfg = { \
      .zczr_cfg =                                                                                                 \
        {                                                                                                         \
          .max_children = MAX_CHILDREN,                                                                           \
        },                                                                                                        \
    }                                                                                                             \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

/* Temperature sensor device parameters */
typedef struct temp_sensor_device_params_s {
  esp_zb_ieee_addr_t ieee_addr;
  uint8_t endpoint;
  uint16_t short_addr;
} temp_sensor_device_params_t;

typedef struct zbstring_s {
  uint8_t len;
  char data[];
} ESP_ZB_PACKED_STRUCT zbstring_t;

static temp_sensor_device_params_t temp_sensor;

/* Zigbee configuration */
#define MAX_CHILDREN                10                                   /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE   false                                /* enable the install code policy for security */
#define HA_THERMOSTAT_ENDPOINT      1                                    /* esp light switch device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME \
  "\x0B"                  \
  "ESPRESSIF"
#define MODEL_IDENTIFIER "\x09" CONFIG_IDF_TARGET

void splashscreen(){

  for(int progress = 0; progress < 100; progress++){

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 12, "ZigBee Monitor");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 30, "Vers. 0.0");

  display.drawProgressBar(1, 45, 120, 10, progress);

  display.display();

  delay(16);

  }

}

static temp_sensor_device_params_t temp_sensors[MAX_SENSORS];
static int sensor_count = 0;

/********************* Zigbee functions **************************/
static float zb_s16_to_temperature(int16_t value) {
  return 1.0 * value / 100;
}


static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    /* Local binding succeeds */
    if (bind_req->req_dst_addr == esp_zb_get_short_address()) {
      log_i("Successfully bind the temperature sensor from address(0x%x) on endpoint(%d)", temp_sensor.short_addr, temp_sensor.endpoint);

      /* Read peer Manufacture Name & Model Identifier */
      esp_zb_zcl_read_attr_cmd_t read_req;
      read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
      read_req.zcl_basic_cmd.src_endpoint = HA_THERMOSTAT_ENDPOINT;
      read_req.zcl_basic_cmd.dst_endpoint = temp_sensor.endpoint;
      read_req.zcl_basic_cmd.dst_addr_u.addr_short = temp_sensor.short_addr;
      read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

      uint16_t attributes[] = {
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
      };
      read_req.attr_number = ARRAY_LENTH(attributes);
      read_req.attr_field = attributes;

      esp_zb_zcl_read_attr_cmd_req(&read_req);
    }
    if (bind_req->req_dst_addr == temp_sensor.short_addr) {
      log_i("The temperature sensor from address(0x%x) on endpoint(%d) successfully binds us", temp_sensor.short_addr, temp_sensor.endpoint);
    }
    free(bind_req);
  } else {
    /* Bind failed, maybe retry the binding ? */

    // esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
  }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS && sensor_count < MAX_SENSORS) {
    log_i("Found temperature sensor");

    /* Store the information of the remote device */
    temp_sensors[sensor_count].endpoint = endpoint;
    temp_sensors[sensor_count].short_addr = addr;
    esp_zb_ieee_address_by_short(temp_sensors[sensor_count].short_addr, temp_sensors[sensor_count].ieee_addr);

    log_d("Temperature sensor found: short address(0x%x), endpoint(%d)", temp_sensors[sensor_count].short_addr, temp_sensors[sensor_count].endpoint);
    
    /* Proceed with binding */
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
    bind_req->req_dst_addr = addr;
    memcpy(bind_req->src_address, temp_sensors[sensor_count].ieee_addr, sizeof(esp_zb_ieee_addr_t));
    bind_req->src_endp = endpoint;
    bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;

    /* Populate the destination info */
    bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_get_long_address(bind_req->dst_address_u.addr_long);
    bind_req->dst_endp = HA_THERMOSTAT_ENDPOINT;

    /* Request binding for the sensor */
    esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
    
    sensor_count++;
  }
}

static void find_temperature_sensor(esp_zb_zdo_match_desc_req_param_t *param, esp_zb_zdo_match_desc_callback_t user_cb, void *user_ctx) {
  uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT};
  param->profile_id = ESP_ZB_AF_HA_PROFILE_ID;
  param->num_in_clusters = 1;
  param->num_out_clusters = 0;
  param->cluster_list = cluster_list;
  esp_zb_zdo_match_cluster(param, user_cb, (void *)&temp_sensor);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      log_i("Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        log_i("Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          log_i("Start network formation");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
          log_i("Device rebooted");
          log_i("Openning network for joining for %d seconds", 180);
          esp_zb_bdb_open_network(180);
        }
      } else {
        log_e("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        log_i(
          "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address()
        );
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        log_i("Restart network formation (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        log_i("Network steering started");
      }
      break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
      dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
      log_i("New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
      esp_zb_zdo_match_desc_req_param_t cmd_req;
      cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
      cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
      find_temperature_sensor(&cmd_req, user_find_cb, NULL);
      break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
      if (err_status == ESP_OK) {
        if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
          log_i("Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
        } else {
          log_w("Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
        }
      }
      break;
    default: log_i("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

bool ankerpunkt = false;
uint16_t zaehlen = 0;
uint16_t zaehlen2 = 0;
int16_t anzeige_temperatur = 0;
int16_t anzeige_luftfeuchte = 0;
int16_t anzeige_hoehe = 0;
int16_t anzeige_luftdruck = 0;

static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute) {
  if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT && attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
    int16_t value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
    float temperature = zb_s16_to_temperature(value);

    // Ausgabe für alle Sensoren

    log_i("Sensor %d - Measured Value: %.2f", cluster_id, temperature);

    if ((value > 4000) && !ankerpunkt){

      ankerpunkt = true;

    }

    if (ankerpunkt){

      if((zaehlen % 2 == 0) && (zaehlen2 % 2 == 0)){

        anzeige_luftfeuchte = value;
        zaehlen++;

      }

      else if ((zaehlen % 2 != 0) && (zaehlen2 % 2 == 0)){

        anzeige_hoehe = value;
        zaehlen2++;

      }

      else if ((zaehlen % 2 != 0) && (zaehlen2 % 2 != 0)){

        anzeige_luftdruck = value;

        zaehlen++;

      }

      else if ((zaehlen % 2 == 0) && (zaehlen2 % 2 != 0)){

        anzeige_temperatur = value;

        zaehlen2++;

      }

      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 0, "Sensorwerte");
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 20, "Temperatur: ");
      display.drawString(80, 20, String(zb_s16_to_temperature(anzeige_temperatur)) + " °C");
      display.drawString(0, 30, "Luftfeuchtigkeit: ");
      display.drawString(80, 30, String(zb_s16_to_temperature(anzeige_luftfeuchte)) + " %");
      display.drawString(0, 40, "Höhe: ");
      display.drawString(80, 40, String(anzeige_hoehe) + "m");
      display.drawString(0, 50, "Luftdruck: ");
      display.drawString(80, 50, String(anzeige_luftdruck) + " hPa");
      display.display();
    }

    else{

      display.clear();
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_10);
      display.drawString(64, 32, "Waiting...");
      display.display();


    }

  }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message) {
  if (!message) {
    log_e("Empty message");
  }
  if (message->status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Received message: error status(%d)", message->status);
  }
  log_i(
    "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr, message->src_endpoint,
    message->dst_endpoint, message->cluster
  );
  esp_app_zb_attribute_handler(message->cluster, &message->attribute);
  return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message) {
  if (!message) {
    log_e("Empty message");
  }
  if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Received message: error status(%d)", message->info.status);
  }
  log_i(
    "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->info.src_address.u.short_addr,
    message->info.src_endpoint, message->info.dst_endpoint, message->info.cluster
  );

  esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
  while (variable) {
    log_i(
      "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status, message->info.cluster,
      variable->attribute.id, variable->attribute.data.type, variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0
    );
    if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
      esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
    }

    variable = variable->next;
  }

  return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message) {
  if (!message) {
    log_e("Empty message");
  }
  if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Received message: error status(%d)", message->info.status);
  }
  esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
  while (variable) {
    log_i(
      "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)", variable->status, message->info.cluster, variable->direction,
      variable->attribute_id
    );
    variable = variable->next;
  }

  return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:            ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message); break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:     ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message); break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID: ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message); break;
    default:                                       log_w("Receive Zigbee action(0x%x) callback", callback_id); break;
  }
  return ret;
}

static esp_zb_cluster_list_t *custom_thermostat_clusters_create(esp_zb_thermostat_cfg_t *thermostat) {
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(thermostat->basic_cfg));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)MANUFACTURER_NAME));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)MODEL_IDENTIFIER));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(thermostat->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)
  );
  ESP_ERROR_CHECK(
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE)
  );
  ESP_ERROR_CHECK(
    esp_zb_cluster_list_add_thermostat_cluster(cluster_list, esp_zb_thermostat_cluster_create(&(thermostat->thermostat_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)
  );
  /* Add temperature measurement cluster for attribute reporting */
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
  return cluster_list;
}

static esp_zb_ep_list_t *custom_thermostat_ep_create(uint8_t endpoint_id, esp_zb_thermostat_cfg_t *thermostat) {
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = endpoint_id, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID, .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(ep_list, custom_thermostat_clusters_create(thermostat), endpoint_config);
  return ep_list;
}

static void esp_zb_task(void *pvParameters) {
  /* Initialize Zigbee stack */
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
  esp_zb_init(&zb_nwk_cfg);
  /* Create customized thermostat endpoint */
  esp_zb_thermostat_cfg_t thermostat_cfg = ESP_ZB_DEFAULT_THERMOSTAT_CONFIG();
  esp_zb_ep_list_t *esp_zb_thermostat_ep = custom_thermostat_ep_create(HA_THERMOSTAT_ENDPOINT, &thermostat_cfg);
  /* Register the device */
  esp_zb_device_register(esp_zb_thermostat_ep);

  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}



/********************* Arduino functions **************************/
void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  display.init();
  display.flipScreenVertically();

  splashscreen();

  // Init Zigbee
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };

  ESP_ERROR_CHECK(esp_zb_platform_config(&config));


  // Start Zigbee task
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {

}
