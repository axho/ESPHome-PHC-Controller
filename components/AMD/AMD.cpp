#include "esphome/core/log.h"
#include "AMD.h"

namespace esphome
{
    namespace AMD_binary
    {

        static const char *TAG = "AMD";

        void AMD::setup()
        {
        }

        void AMD::loop()
        {
            if (get_state() != target_state)
            {
                // Wait before retransmitting
                if (millis() - last_request > RESEND_TIMEOUT)
                {
                    if (resend_counter < MAX_RESENDS)
                    {
                        // Try resending as long as possible
                        int resend = resend_counter;
                        toggle_map->flip_toggle(this);
                        write_state(target_state);
                        resend_counter = resend + 1;
                    }
                    else
                    {
                        // Give up. OpenHAB's PHC binding - which runs
                        // reliably against this same bus - does NOT write
                        // anything back to the module here; it only corrects
                        // its own toggle-bit bookkeeping and accepts defeat.
                        // We do the same: reset our bookkeeping directly
                        // (NOT via write_state(), which would also transmit
                        // on the bus - that was the bug in the previous
                        // version of this fix, which never terminated the
                        // retry loop and spammed this warning every single
                        // loop() call forever).
                        ESP_LOGW(TAG, "Device not responding! Is the device connected to the bus? (DIP: %i)", address);
                        target_state = get_state();
                        resend_counter = 0;
                        last_request = millis();
                        publish_state(get_state());
                    }
                }
            }
        }

        void AMD::write_state(bool state)
        {
            resend_counter = 0;
            target_state = state;
            toggle_map->flip_toggle(this);
            // We don't publish the state here, we wait for the answer and use the callback to make sure the output acutally switched

            // 3 MSBits determine the channel, lower 5 bits are for functionality
            uint8_t function = (channel << 5) | (state ? 0x02 : 0x03);

            uint8_t message[5] = {static_cast<uint8_t>(AMD_MODULE_ADDRESS | address), static_cast<uint8_t>((toggle_map->get_toggle(this) ? 0x80 : 0x00) | 0x01), function, 0x00, 0x00};
            short crc = util::PHC_CRC(message, 3);

            message[3] = static_cast<uint8_t>(crc & 0xFF);
            message[4] = static_cast<uint8_t>((crc & 0xFF00) >> 8);

            write_array(message, 5, true);
            last_request = millis();
        }

        void AMD::dump_config()
        {
            ESP_LOGCONFIG(TAG, "AMD DIP-ID: %u\t Channel: %u", address, channel);
        }

    } // namespace AMD_binary
} // namespace esphome
