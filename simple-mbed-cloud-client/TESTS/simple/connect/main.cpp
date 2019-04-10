#include "mbed.h"
#include "FATFileSystem.h"
#include "simple-mbed-cloud-client.h"

#include "utest/utest.h"
#include "unity/unity.h"
#include "greentea-client/test_env.h"

using namespace utest::v1;

// Default storage definition.
BlockDevice* bd = BlockDevice::get_default_instance();
FATFileSystem fs("sd", bd);

static const ConnectorClientEndpointInfo* endpointInfo;
void registered(const ConnectorClientEndpointInfo *endpoint) {
    printf("Connected to Mbed Cloud. Device ID: %s\n",
            endpoint->internal_endpoint_name.c_str());
    endpointInfo = endpoint;
}

void smcc_register(void) {

    int iteration = 0;
    int timeout = 0;
    char _key[20] = { };
    char _value[128] = { };

    greentea_send_kv("device_ready", true);
    greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));

    iteration = atoi(_value);

    // Connection definition.
#if MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == ETHERNET
    NetworkInterface *net = NetworkInterface::get_default_instance();
    nsapi_error_t status = net->connect();
#elif MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == WIFI
    WiFiInterface *net = WiFiInterface::get_default_instance();
    nsapi_error_t status = net->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
#elif MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == CELLULAR
    CellularBase *net = CellularBase::get_default_instance();
    net->set_sim_pin(MBED_CONF_NSAPI_DEFAULT_CELLULAR_SIM_PIN);
    net->set_credentials(MBED_CONF_NSAPI_DEFAULT_CELLULAR_APN, MBED_CONF_NSAPI_DEFAULT_CELLULAR_USERNAME, MBED_CONF_NSAPI_DEFAULT_CELLULAR_PASSWORD);
    nsapi_error_t status = net->connect();
#elif MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == MESH
    MeshInterface *net = MeshInterface::get_default_instance();
    nsapi_error_t status = net->connect();
#else
    #error "Default network interface not defined"
#endif

    // Must have IP address.
    TEST_ASSERT_NOT_EQUAL(net->get_ip_address(), NULL);
    if (net->get_ip_address() == NULL) {
        printf("[ERROR] No IP address obtained from network.\r\n");
        greentea_send_kv("fail_test", 0);
    }

    // Connection must be successful.
    TEST_ASSERT_EQUAL(status, 0);
    if (status == 0 && net->get_ip_address() != NULL) {
        printf("[INFO] Connected to network successfully. IP address: %s\n", net->get_ip_address());
    } else {
        printf("[ERROR] Failed to connect to network.\r\n");
        greentea_send_kv("fail_test", 0);
    }

    SimpleMbedCloudClient client(net, bd, &fs);

    if (iteration == 0) {
        printf("[INFO] Resetting storage to a clean state for test.\n");
        client.reformat_storage();
    }

    // SimpleMbedCloudClient initialization must be successful.
    int client_status = client.init();
    TEST_ASSERT_EQUAL(client_status, 0);
    if (client_status == 0) {
        printf("[INFO] Simple Mbed Cloud Client initialization successful. \r\n");
    } else {
        printf("[ERROR] Simple Mbed Cloud Client failed to initialize.\r\n");
        greentea_send_kv("fail_test", 0);
    }

    client.on_registered(&registered);
    client.register_and_connect();

    timeout = 5000;
    while (timeout && !client.is_client_registered()) {
        timeout--;
        wait_ms(1);
    }

    // Registration to Mbed Cloud must be successful.
    TEST_ASSERT_TRUE(client.is_client_registered());
    if (!client.is_client_registered()) {
        printf("[ERROR] Device failed to register.\r\n");
        greentea_send_kv("fail_test", 0);
    } else {
        printf("[INFO] Simple Mbed Cloud Client successfully registered to Mbed Cloud.\r\n");
    }

    // Allow 500ms for Mbed Cloud to update the device directory.
    timeout = 500;
    while (timeout) {
        timeout--;
        wait_ms(1);
    }

    if (iteration == 0) {
        // Start host tests with device id
        printf("[INFO] Starting Mbed Cloud verification using Python SDK...\r\n");
        greentea_send_kv("device_api_registration", endpointInfo->internal_endpoint_name.c_str());

        // Wait for Host Test and API response (blocking here)
        greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));

        // Ensure the state is 'registered' in the Device Directory
        TEST_ASSERT_EQUAL_STRING("registered", _value);
        if (strcmp(_value, "registered") != 0) {
            printf("[ERROR] Device could not be verified as registered in Device Directory.\r\n");
            greentea_send_kv("fail_test", 0);
        } else {
            printf("[INFO] Device is registered in the Device Directory.\r\n");
        }

    } else {
        printf("Verifying consistent endpoint...\r\n");
        greentea_send_kv("device_verification", endpointInfo->internal_endpoint_name.c_str());

        // Wait for Host Test to verify consistent device ID (blocking here)
        greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));
        TEST_ASSERT_EQUAL_STRING("True", _value);
        if (strcmp(_value, "True") != 0) {
            printf("[ERROR] Device ID is inconsistent. SOTP and Secure Storage was not preserved.\r\n");
            greentea_send_kv("fail_test", 0);
        } else {
            printf("[INFO] Device ID consistent, SOTP and Secure Storage is preserved correctly.\r\n");
        }
    }

    // Reset on first iteration of test.
    if (iteration == 0) {
        printf("[INFO] Resetting device.\r\n");
        greentea_send_kv("advance_test", 0);
        greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));
        if (strcmp(_key, "reset") == 0) {
            system_reset();
        }
    } else {
        greentea_send_kv("advance_test", 0);
    }
}

int main(void) {
    GREENTEA_SETUP(150, "sdk_host_tests");
    smcc_register();

    return 0;
}
