#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

const uint PWM_PIN = 22;
const uint BUTTON_PIN = 21;
const uint LED_PIN = 25;

int main() {
    gpio_init(BUTTON_PIN);
    gpio_init(LED_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // Assuming active-low button

    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.7803);
    pwm_init(slice_num, &config, true);

    absolute_time_t start_time;
    bool stop = false;
    start_time = get_absolute_time();

    while (1) {
        pwm_clear_irq(pwm_gpio_to_slice_num(PWM_PIN));
 
        if (gpio_get(BUTTON_PIN)==0) {
            pwm_set_gpio_level(PWM_PIN, 0.65 * 65025);
        } else{
            pwm_set_gpio_level(PWM_PIN, 0.35 * 65025);
        }

        //  if (absolute_time_diff_us(start_time, get_absolute_time()) >= 2000000) {
        //     pwm_set_gpio_level(PIN, 0.5 * 65025);
        //     gpio_put(LED_PIN, 0);               // Turn off LED if needed
        //     stop = true;              // Reset timer flag
        // }


    }
}
