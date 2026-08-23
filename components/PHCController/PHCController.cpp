#include <algorithm>
#include <string>
#include "esphome/core/log.h"
#include "PHCController.h"

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "esphome/components/uart/uart_component_esp32_arduino.h"
#elif USE_ESP8266
#include "esphome/components/uart/uart_component_esp8266.h"
#elif USE_ESP_IDF
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif

namespace esphome
{
    namespace phc_controller
    {

        static const char *TAG = "phc_controller";

        static const char *emd_function_name(uint8_t action)
        {
            switch (action)
            {
            case 0x02:
                return "ein>0 (Taster gedrueckt)";
            case 0x03:
                return "aus<0 (kurz losgelassen)";
            case 0x04:
                return "ein>1 (>1s gehalten)";
            case 0x05:
                return "aus>1 (losgelassen nach >1s)";
            case 0x06:
                return "ein>2 (>2s gehalten)";
            case 0x07:
                return "aus (losgelassen)";
            default:
                return "unbekannt";
            }
        }

        void PHCController::setup()
        {
            /*
            Since Arduino Core version 2.0.0+ the timing of acknowledgement messages is too large, the reason for this issue is related to the way in which the uart input buffer is pocessed
            See these related issues
            https://github.com/espressif/arduino-esp32/issues/6689
            https://github.com/espressif/arduino-esp32/issues/6921
            setRxFIFOFull(1) is used to force immediate parsing of incoming data which fixes the delay issue
            */
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#pragma message("Arduino Framework only supported until ESPHome 2025.9!")
            uart::ESP32ArduinoUARTComponent *uartComponent = static_cast<uart::ESP32ArduinoUARTComponent *>(this->parent_);
            uartComponent->get_hw_serial()->setRxTimeout(1);
            uartComponent->get_hw_serial()->setRxFIFOFull(1);
#elif USE_ESP_IDF
            uart::IDFUARTComponent *uartComponent = static_cast<uart::IDFUARTComponent *>(this->parent_);
            uart_port_t hw_serial_number = static_cast<uart_port_t>(uartComponent->get_hw_serial_number());
            uart_set_rx_timeout(hw_serial_number, 1);
            uart_set_rx_full_threshold(hw_serial_number, 1);
#endif

            if (flow_control_pin_ != NULL)
            {
                flow_control_pin_->setup();
                flow_control_pin_->digital_write(false);
            }
            high_freq_.start();

            // Delay setup from here, since bus might be busy from ESP init.
            delay(500);
            setup_known_modules();
            last_message_time_ = millis();
        }

        // Maximum number of bytes read from the UART per loop() call. This
        // bounds the time spent here so the shared ESPHome loop is never
        // starved even if the bus keeps firing continuously. This is a
        // safety cap, not a batching delay - frames are still processed
        // (and acknowledged) immediately as soon as they are complete, see
        // below.
        static const size_t MAX_BYTES_PER_LOOP = 64;

        void PHCController::loop()
        {
            size_t bytes_read = 0;

            // Read and process interleaved, one byte at a time. This keeps
            // acknowledgement latency as low as possible: a frame is handled
            // (and acknowledged) the moment its last byte arrives, not only
            // after a whole batch of bytes has been drained from the UART.
            // The PHC bus requires very fast acknowledgements (see protocol
            // docs) - modules that don't get acked in time keep
            // retransmitting, which then floods the bus further.
            while (bytes_read < MAX_BYTES_PER_LOOP && available())
            {
                rx_buffer_.push_back(read());
                bytes_read++;

                // Try to extract as many complete, valid frames as currently
                // possible. On a length or checksum mismatch we do NOT
                // discard the whole assumed frame - we only shift the buffer
                // forward by a single byte and retry. This allows the parser
                // to resynchronize after a single corrupt/spurious byte on
                // the bus (e.g. caused by reflections from a missing/faulty
                // termination resistor, or a partial write collision on a
                // half-duplex line) instead of losing an entire frame's
                // worth of following bytes.
                while (rx_buffer_.size() >= 2)
                {
                    uint8_t address = rx_buffer_[0];
                    uint8_t toggle_and_length = rx_buffer_[1];
                    bool toggle = toggle_and_length & 0x80;            // Mask the MRB (toggle bit)
                    uint8_t content_length = toggle_and_length & 0x7F; // Mask everything except for the MSB (message length)

                    // Assert message length is plausible
                    if (content_length > 3)
                    {
                        // Implausible length - this byte cannot be a valid
                        // frame start. Shift forward by one byte and retry.
                        skipped_bytes_.push_back(rx_buffer_[0]);
                        rx_buffer_.erase(rx_buffer_.begin());
                        continue;
                    }

                    size_t frame_length = static_cast<size_t>(content_length) + 4; // header(2) + content + checksum(2)

                    // Not enough data buffered yet for a full frame - wait
                    // for more bytes (next iteration of this loop, or the
                    // next loop() call).
                    if (rx_buffer_.size() < frame_length)
                        break;

                    // Read the checksum
                    uint16_t msg_checksum = (rx_buffer_[frame_length - 1] << 8) | rx_buffer_[frame_length - 2];

                    // Validate the checksum
                    uint16_t calculated_checksum = util::PHC_CRC(rx_buffer_.data(), content_length + 2); // crc for content and prefix

                    if (calculated_checksum != msg_checksum)
                    {
                        // On this bus a single 0x00 turnaround byte between
                        // frames is normal, not exceptional - this path is
                        // hit on almost every frame purely because of that.
                        // We don't log here directly; instead we collect the
                        // skipped bytes and only report them once we find
                        // the next valid frame, so we can tell a routine
                        // 1-byte gap apart from an actual desync further
                        // down (see below).
                        skipped_bytes_.push_back(rx_buffer_[0]);

                        // Shift forward by a single byte instead of
                        // discarding the whole assumed frame, then retry
                        // parsing from there.
                        rx_buffer_.erase(rx_buffer_.begin());

                        // Safety valve: if we never find a valid frame (e.g.
                        // sustained line noise), don't let this grow
                        // unbounded - report and reset periodically.
                        if (skipped_bytes_.size() >= 32)
                        {
                            std::string hex_dump;
                            char byte_str[4];
                            for (uint8_t b : skipped_bytes_)
                            {
                                snprintf(byte_str, sizeof(byte_str), "%02X ", b);
                                hex_dump += byte_str;
                            }
                            ESP_LOGW(TAG, "Unable to resync after %zu bytes, discarding: %s", skipped_bytes_.size(), hex_dump.c_str());
                            skipped_bytes_.clear();
                        }
                        continue;
                    }

                    // Valid frame found. If we had to skip more than the
                    // single byte that's normal on this bus between frames,
                    // that's worth surfacing - it points at real bus noise
                    // rather than the routine turnaround gap.
                    if (skipped_bytes_.size() > 1)
                    {
                        std::string hex_dump;
                        char byte_str[4];
                        for (uint8_t b : skipped_bytes_)
                        {
                            snprintf(byte_str, sizeof(byte_str), "%02X ", b);
                            hex_dump += byte_str;
                        }
                        ESP_LOGW(TAG, "Resynced after %zu unexpected bytes (bus noise?): %s", skipped_bytes_.size(), hex_dump.c_str());
                    }
                    skipped_bytes_.clear();

                    last_message_time_ = millis();

                    {
                        uint8_t log_device_id = address & 0x1F;
                        uint8_t log_device_class = address & 0xE0;
                        std::string content_hex;
                        char byte_str[4];
                        for (size_t i = 0; i < content_length; i++)
                        {
                            snprintf(byte_str, sizeof(byte_str), "%02X ", rx_buffer_[2 + i]);
                            content_hex += byte_str;
                        }
                        ESP_LOGD(TAG, "RX class=0x%02X DIP=%d toggle=%d content=[%s]", log_device_class, log_device_id, toggle, content_hex.c_str());
                    }

                    process_command(&address, toggle, rx_buffer_.data() + 2, &content_length);

                    // Valid frame consumed - remove exactly this frame from
                    // the buffer and continue with whatever follows it.
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_length);
                }
            }

            if (!states_synced_)
            {
                if (millis() - last_message_time_ > INITIAL_SYNC_DELAY * 1000)
                {
                    sync_states();
                    states_synced_ = true;
                }
            }
        }

        void PHCController::dump_config()
        {
            ESP_LOGCONFIG(TAG, "PHC Controller");
            check_uart_settings(19200, 2, uart::UART_CONFIG_PARITY_NONE, 8);
            if (flow_control_pin_ != NULL)
                LOG_PIN("flow_control_pin: ", flow_control_pin_);

            ESP_LOGCONFIG(TAG, "PHC - NR. of  AMD: %i", amds_.size());
            ESP_LOGCONFIG(TAG, "PHC - NR. of  JRM: %i", jrms_.size());
            ESP_LOGCONFIG(TAG, "PHC - NR. of  EMD: %i", emds_.size());
            ESP_LOGCONFIG(TAG, "PHC - NR. of  EMD-Lights: %i", emd_lights_.size());
        }

        void PHCController::process_command(uint8_t *device_class_id, bool toggle, uint8_t *message, uint8_t *length)
        {
            uint8_t device_id = *device_class_id & 0x1F; // DIP settings (5 LSB)
            uint8_t device_class = *device_class_id & 0xE0;
            // EMD
            if (device_class == EMD_MODULE_ADDRESS)
            {
                // Initial configuration request message
                if (message[0] == 0xFF)
                {
                    //  Configure EMD
                    delayMicroseconds(TIMING_DELAY);
                    send_emd_config(device_id);
                    return;
                }

                uint8_t channel = (message[0] & 0xF0) >> 4;
                // Handle acknowledgement (such as switch led state)
                if (message[0] == 0x00)
                {
                    bool handled = false;
                    uint8_t channels = message[1];
                    for (uint8_t i = 0; i < 8; i++)
                    {
                        if (emd_lights_.count(util::key(device_id, i)))
                        {
                            auto *emd_light = emd_lights_[util::key(device_id, i)];

                            // Mask the channel and publish states accordingly
                            bool state = channels & (0x1 << i);
                            emd_light->publish_state(state);

                            handled = true;
                        }
                    }

                    if (!handled)
                        ESP_LOGI(TAG, "No configuration found for Message from (EMD-Light) Module: [DIP: %i, channel: %i]", device_id, channel);
                }
                else
                {
                    uint8_t action = message[0] & 0x0F;
                    auto emd_it = emds_.find(util::key(device_id, channel));

                    // Send extra (speedy) acknowledgement, seems to help.
                    // This ALWAYS happens, regardless of dedup below - the
                    // bus protocol requires it every time, and skipping it
                    // would only cause the sender to retransmit more.
                    send_acknowledgement(*device_class_id, toggle);

                    if (emd_it != emds_.end())
                    {
                        auto *emd = emd_it->second;
                        bool new_state = emd->state; // no-op fallback: keep current on unrecognized action
                        if (action == 0x02)           // ON
                            new_state = true;
                        else if (action == 0x07 || action == 0x03 || action == 0x05) // OFF
                            new_state = false;

                        // Maintained contacts (window/door contacts, twilight
                        // sensors) keep resending the same event as long as
                        // the physical condition holds, which can mean
                        // hundreds of identical messages per minute. We
                        // still ack every single one (required above), but
                        // only log and publish_state when the state
                        // actually changes - this is what the person
                        // actually cares about seeing, and avoids flooding
                        // the log / re-publishing an unchanged state.
                        if (new_state != emd->state)
                        {
                            ESP_LOGD(TAG, "EMD [DIP %d, Kanal %d] \"%s\": %s", device_id, channel, emd->get_name().c_str(), emd_function_name(action));
                            emd->publish_state(new_state);
                        }
                        else
                        {
                            ESP_LOGV(TAG, "EMD [DIP %d, Kanal %d] \"%s\": %s (unveraendert, unterdrueckt)", device_id, channel, emd->get_name().c_str(), emd_function_name(action));
                        }
                        return;
                    }

                    ESP_LOGI(TAG, "No configuration found for Message from (EMD) Module: [DIP: %i, channel: %i]", device_id, channel);
                }
                return;
            }

            if (device_class == AMD_MODULE_ADDRESS || device_class == JRM_MODULE_ADDRESS)
            {
                // Initial configuration request message
                if (message[0] == 0xFF)
                {
                    delayMicroseconds(TIMING_DELAY);
                    send_amd_config(device_id);
                    return;
                }

                // Check for Acknowledgement
                if (message[0] == 0x00)
                {
                    ESP_LOGD(TAG, "AMD/JRM [DIP %d]: Status-Ack, Kanal-Bits=0x%02X", device_id, message[1]);

                    bool handled = false;
                    uint8_t channels = message[1];
                    for (uint8_t i = 0; i < 8; i++)
                    {
                        // Handle output switches
                        if (amds_.count(util::key(device_id, i)))
                        {
                            auto *amd = amds_[util::key(device_id, i)];

                            // Mask the channel and publish states accordingly
                            bool state = channels & (0x1 << i);
                            ESP_LOGD(TAG, "AMD [DIP %d, Kanal %d] \"%s\": -> %s", device_id, i, amd->get_name().c_str(), state ? "AN" : "AUS");
                            amd->publish_state(state);
                            handled = true;
                        }

                        // Handle output switches
                        if (jrms_.count(util::key(device_id, i)))
                        {
                            auto *jrm = jrms_[util::key(device_id, i)];
                            // For some reason the cover ack-message does not contain which covers are moving, so we are guessing that the channel has been processed
                            // This might lead to one cover not moving if 2 are manipulated at the same time
                            // Only accepting a single change will increase the chance of correct acknowledgement
                            if (jrm->current_operation != jrm->get_target_operation())
                            {
                                ESP_LOGD(TAG, "JRM [DIP %d, Kanal %d] \"%s\": Operation abgeschlossen", device_id, i, jrm->get_name().c_str());
                                jrm->current_operation = jrm->get_target_operation();
                                jrm->publish_state();
                                handled = true;
                                break;
                            }
                            handled = true;
                        }
                    }
                    if (!handled)
                        ESP_LOGI(TAG, "No configuration found for Message from (AMD/JRM) Module: [DIP: %i]", device_id);
                }
                return;
            }

            if (device_class == DIM_MODULE_ADDRESS)
            {
                // Initial configuration request message. Dimmer modules
                // apparently don't accept a plain acknowledgement here -
                // they keep re-requesting configuration forever until they
                // get a real config response, exactly like AMD/JRM modules.
                // OpenHAB's PHC binding confirms this: it uses the identical
                // config response for AM, JRM, and DIM modules, only the
                // address class differs - see send_amd_config above.
                if (message[0] == 0xFF)
                {
                    delayMicroseconds(TIMING_DELAY);
                    send_amd_config(device_id, DIM_MODULE_ADDRESS);
                    return;
                }

                // There is no dedicated DIM entity/component yet (no
                // brightness control from Home Assistant), so we can't
                // interpret or publish a meaningful state for other
                // messages from this module. We still acknowledge them so
                // the module doesn't retransmit unnecessarily.
                send_acknowledgement(*device_class_id, toggle);
                return;
            }

            // Send default acknowledgement
            send_acknowledgement(*device_class_id, toggle);
        }

        void inline PHCController::send_acknowledgement(uint8_t address, bool toggle)
        {
            uint8_t message[5] = {address, static_cast<uint8_t>((toggle ? 0x80 : 0x00) | 0x01), 0x00, 0x00, 0x00};
            uint16_t crc = util::PHC_CRC(message, 3);

            message[3] = static_cast<uint8_t>(crc & 0xFF);
            message[4] = static_cast<uint8_t>((crc & 0xFF00) >> 8);

            delayMicroseconds(TIMING_DELAY);
            write_array(message, 5, true);
        }

        void PHCController::send_amd_config(uint8_t id, uint8_t module_class)
        {
            ESP_LOGI(TAG, "Configuring Module (AMD/JRM/DIM, class 0x%02X): [DIP: %i]", module_class, id);

            uint8_t message[7] = {static_cast<uint8_t>(module_class | id), 0x03, 0xFE, 0x00, 0xFF, 0x00, 0x00};

            short crc = util::PHC_CRC(message, 5);
            message[5] = static_cast<uint8_t>(crc & 0xFF);
            message[6] = static_cast<uint8_t>((crc & 0xFF00) >> 8);

            write_array(message, 7, false);
        }

        void PHCController::send_emd_config(uint8_t id)
        {
            ESP_LOGI(TAG, "Configuring Module (EMD): [DIP: %i]", id);
            uint8_t message[56] = {0x00};
            message[0] = EMD_MODULE_ADDRESS | id;
            message[1] = 0x34; // 52 Bytes

            // source: https://github.com/openhab/openhab-addons/blob/da59cdd255a66275dd7ae11dd294fedca4942d30/bundles/org.openhab.binding.phc/src/main/java/org/openhab/binding/phc/internal/handler/PHCBridgeHandler.java
            int pos = 2;

            message[pos++] = 0xFE;
            message[pos++] = 0x00; // POR

            message[pos++] = 0x00;
            message[pos++] = 0x00;

            for (int i = 0; i < 16; i++)
            { // 16 inputs
                message[pos++] = ((i << 4) | 0x02);
                message[pos++] = ((i << 4) | 0x03);
                message[pos++] = ((i << 4) | 0x05);
            }

            short crc = util::PHC_CRC(message, 54);
            message[54] = static_cast<uint8_t>(crc & 0xFF);
            message[55] = static_cast<uint8_t>((crc & 0xFF00) >> 8);

            write_array(message, 56, false);
        }

        void PHCController::setup_known_modules()
        {
            std::vector<uint8_t> addresses;
            // Collect all EMD Adresses

            for (auto const &module : emds_)
            {
                if (std::find(addresses.begin(), addresses.end(), module.second->get_address()) == addresses.end())
                    addresses.push_back(module.first);
            }

            // Send all known EMD Configurations
            for (uint8_t address : addresses)
                send_emd_config(address);

            addresses.clear();

            // Collect all AMD/JRM Adresses (AMD_MODULE_ADDRESS and JRM_MODULE_ADDRESS are the same)
            for (auto const &module : amds_)
            {
                if (std::find(addresses.begin(), addresses.end(), module.second->get_address()) == addresses.end())
                    addresses.push_back(module.second->get_address());
            }
            for (auto const &module : jrms_)
            {
                if (std::find(addresses.begin(), addresses.end(), module.second->get_address()) == addresses.end())
                    addresses.push_back(module.first);
            }

            // Send all known AMD Configurations
            for (uint8_t address : addresses)
                send_amd_config(address);
        }

        void PHCController::sync_states()
        {
            for (auto const &emd_light : emd_lights_)
            {
                emd_light.second->sync_state();
                delay(40);
            }
            for (auto const &amd : amds_)
            {
                amd.second->sync_state();
                delay(40);
            }
            for (auto const &jrm : jrms_)
            {
                jrm.second->sync_state();
                delay(40);
            }
        }

        void PHCController::write_array(const uint8_t *data, size_t len, bool allow_weak_operation)
        {

            // skip writing if the bus is busy and rely on retransmits
            if (allow_weak_operation && available())
            {
                ESP_LOGW(TAG, "Skipped write (dest 0x%02X): bus busy, relying on retransmit", data[0]);
                return;
            }

            // Pull the write pin HIGH
            if (flow_control_pin_ != NULL)
            {
                flow_control_pin_->digital_write(true);
                delayMicroseconds(FLOW_PIN_PULL_HIGH_DELAY);
            }

            // Write data to the bus
            UARTDevice::write_array(data, len);

            // Flush everything out before pulling the flow control pin low
            UARTDevice::flush();

            // safety delay to prevent clashing with repsonses
            delay(1);

            // Pull the write pin LOW
            if (flow_control_pin_ != NULL)
            {
                delayMicroseconds(FLOW_PIN_PULL_LOW_DELAY);
                flow_control_pin_->digital_write(false);
            }
        }
    } // namespace phc_controller
} // namespace esphome
