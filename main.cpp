// ----------------------------------------------------------------------------
// Copyright 2016-2018 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "mbed.h"
#include "simple-mbed-cloud-client.h"
#include "FATFileSystem.h"

// To enable the GROVE sensors, uncomment the next line.
//#define ENABLE_GROVE

// An event queue is a very useful structure to debounce information between contexts (e.g. ISR and normal threads)
// This is great because things such as network operations are illegal in ISR, so updating a resource in a button's fall() function is not allowed
EventQueue eventQueue;
Thread thread1;

// Default block device
BlockDevice* bd = BlockDevice::get_default_instance();
FATFileSystem fs("sd", bd);

// Default network interface object
NetworkInterface *net;
 
InterruptIn sw2(SW2);     // A button on the board.
DigitalOut led2(LED2);    // LED on the board.

// Declaring pointers for access to Mbed Cloud Client resources outside of main()
MbedCloudClientResource *button_res;
MbedCloudClientResource *pattern_res;
#ifdef ENABLE_GROVE
MbedCloudClientResource *temperature_res;
MbedCloudClientResource *illuminance_res;
MbedCloudClientResource *buzzer_res;
#endif // ENABLE_GROVE

static bool button_pressed = false;
static int button_count = 0;
      
void button_press() {
    button_pressed = true;
    ++button_count;
    button_res->set_value(button_count);
}

/**
 * PUT handler
 * @param resource The resource that triggered the callback
 * @param newValue Updated value for the resource
 */
void pattern_updated(MbedCloudClientResource *resource, m2m::String newValue) {
    printf("PUT received, new value: %s\n", newValue.c_str());
}

/**
 * POST handler
 * @param resource The resource that triggered the callback
 * @param buffer If a body was passed to the POST function, this contains the data.
 *               Note that the buffer is deallocated after leaving this function, so copy it if you need it longer.
 * @param size Size of the body
 */
void blink_callback(MbedCloudClientResource *resource, const uint8_t *buffer, uint16_t size) {
    printf("POST received. Going to blink LED pattern: %s\n", pattern_res->get_value().c_str());

    static DigitalOut augmentedLed(LED1); // LED that is used for blinking the pattern

/* Removed for ARMCC */
#if 0
    // Parse the pattern string, and toggle the LED in that pattern
    string s = std::string(pattern_res->get_value().c_str());
    size_t i = 0;
    size_t pos = s.find(':');
    while (pos != string::npos) {
        wait_ms(atoi(s.substr(i, pos - i).c_str()));
        augmentedLed = !augmentedLed;

        i = ++pos;
        pos = s.find(':', pos);

        if (pos == string::npos) {
            wait_ms(atoi(s.substr(i, s.length()).c_str()));
            augmentedLed = !augmentedLed;
        }
    }
#endif
}

/**
 * POST handler. Beeps buzzer for a certain time when any data comes.
 */
 #ifdef ENABLE_GROVE
void buzzer_callback(MbedCloudClientResource *resource, const uint8_t *buffer, uint16_t size) {
    const int BUZZER_ON = 1;            // Polarity of the digital pin to beep the buzzer.
    const int BUZZER_OFF = 0;           // Polarity of the digital pin to stop the buzzer.
    const int SOUND_DURATION = 500;     // Duration
    static DigitalOut buzzer(D3, BUZZER_OFF);  // Buzzer must be connected to D3.
    printf("POST received. Going to beep the buzzer for %d ms.\n", SOUND_DURATION);

    buzzer = BUZZER_ON;
    ThisThread::sleep_for(SOUND_DURATION);
    buzzer = BUZZER_OFF;
}
#endif // ENABLE_GROVE

/**
 * Notification callback handler
 * @param resource The resource that triggered the callback
 * @param status The delivery status of the notification
 */
void button_callback(MbedCloudClientResource *resource, const NoticationDeliveryStatus status) {
    printf("Button notification, status %s (%d)\n", MbedCloudClientResource::delivery_status_to_string(status), status);
}

/**
 * Registration callback handler
 * @param endpoint Information about the registered endpoint such as the name (so you can find it back in portal)
 */
void registered(const ConnectorClientEndpointInfo *endpoint) {
    printf("Connected to Mbed Cloud. Endpoint Name: %s\n", endpoint->internal_endpoint_name.c_str());
}

/**
 * Get the sensor values.
 */
#ifdef ENABLE_GROVE
void update_sensors() {
    // Get temperature sensor value.
    // See how to covert AD value to temperature:
    // http://wiki.seeedstudio.com/Grove-Temperature_Sensor_V1.2/
    // Conversion formula contains redundancy, but not corrected here.
    const int B = 4275;      // B value of the thermistor
    const int R0 = 100000;   // R0 = 100k
    static AnalogIn adc_temperature(A0);  // temperature sensor must be connected to A0
    float a = adc_temperature.read()*1023.0;
    float R = 1023.0/a-1.0;
    R = R0*R;

    float temperature = 1.0/(log(R/R0)/B+1/298.15) - 273.15;

    // Get ambient light luminosity.
    // http://wiki.seeedstudio.com/Grove-Light_Sensor/
    static AnalogIn adc_illuminance(A1);
    uint16_t illuminance = adc_illuminance.read_u16();

    // Set the obtained sensor values.
    temperature_res->set_value(temperature);
    illuminance_res->set_value(illuminance);

    printf("Sensor values: temperature %4.1f C, illuminance %d\n", temperature, illuminance);
}
#endif // ENABLE_GROVE


int main(void) {
    printf("Starting Simple Mbed Cloud Client example\n");
    printf("Connecting to the network using Ethernet...\n");

    // Setup the button 
    sw2.mode(PullUp);
    // If the SW2 button on the board is pushed at the init, format the storage.
    // Note that the polarity of SW2 is active low. i.e. 0 is pushed.
    if(sw2 == 0) {
        printf("SW2 is being pushed. Format storage...");
        if(fs.format(bd) != 0) {
            printf("Failed to format the storage.");
            return -1;
        } else {
            printf("The starage was formatted. Program stops here.\n");
            // Turn on the blue LED to inform the storage was formatted successfully.
            DigitalOut tmp(LED_BLUE, 1);
            return 0;
        }
    }

    // Connect to the internet (DHCP is expected to be on)
    net = NetworkInterface::get_default_instance();

    nsapi_error_t status = net->connect();

    if (status != NSAPI_ERROR_OK) {
        printf("Connecting to the network failed %d!\n", status);
        return -1;
    }

    printf("Connected to the network successfully. IP address: %s\n", net->get_ip_address());

    // SimpleMbedCloudClient handles registering over LwM2M to Mbed Cloud
    SimpleMbedCloudClient client(net, bd, &fs);
    int client_status = client.init();
    if (client_status != 0) {
        printf("Initializing Mbed Cloud Client failed (%d)\n", client_status);
        return -1;
    }

    // Creating resources, which can be written or read from the cloud
    button_res = client.create_resource("3200/0/5501", "button_count");
    button_res->set_value(0);
    button_res->methods(M2MMethod::GET);
    button_res->observable(true);
    button_res->attach_notification_callback(button_callback);

    pattern_res = client.create_resource("3201/0/5853", "blink_pattern");
    pattern_res->set_value("500:500:500:500:500:500:500:500");
    pattern_res->methods(M2MMethod::GET | M2MMethod::PUT);
    pattern_res->attach_put_callback(pattern_updated);

    MbedCloudClientResource *blink_res = client.create_resource("3201/0/5850", "blink_action");
    blink_res->methods(M2MMethod::POST);
    blink_res->attach_post_callback(blink_callback);

#ifdef ENABLE_GROVE
    temperature_res = client.create_resource("3303/0/5700", "temperature");
    temperature_res->set_value(0.0f);
    temperature_res->methods(M2MMethod::GET);
    temperature_res->observable(true);

    illuminance_res = client.create_resource("3301/0/5700", "illuminance");
    illuminance_res->set_value(0.0f);
    illuminance_res->methods(M2MMethod::GET);
    illuminance_res->observable(true);
    
    buzzer_res = client.create_resource("3201/1/5550", "buzzer");
    buzzer_res->methods(M2MMethod::POST);
    buzzer_res->attach_post_callback(buzzer_callback);
#endif // ENABLE_GROVE

    printf("Initialized Mbed Cloud Client. Registering...\n");

    // Callback that fires when registering is complete
    client.on_registered(&registered);

    // Register with Mbed Cloud
    client.register_and_connect();
    
    // The button fall handler is placed in the event queue so it will run in
    // thread context instead of ISR context, which allows safely updating the cloud resource         
    sw2.fall(eventQueue.event(&button_press));
    // The button connected to the GROVE shield acts the same as SW2.
    InterruptIn extBtn(D2);
    extBtn.fall(eventQueue.event(&button_press));
    button_count = 0;

#ifdef ENABLE_GROVE
    Ticker timer;
    timer.attach(eventQueue.event(update_sensors), 3.0);
#endif /* ENABLE_SENSORS */

    // Start the event queue in a separate thread so the main thread continues
    thread1.start(callback(&eventQueue, &EventQueue::dispatch_forever));

    while(1)
    {
        wait_ms(100);

        if (button_pressed) {
            button_pressed = false;
            printf("button clicked %d times\r\n", button_count);            
        }
        
    }
}
