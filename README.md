# mext_nspwmout_stm32

PwmOutとほとんど同じインターフェース

`suspend`, `resume`はありません

パルス幅の設定には，`period_{ms, us}`, `pulsewidth_{ms, us}`に加え，`period_ns`, `pulsewidth_ns`が使えます

```cpp
#include "mbed.h"
#include "NsPwmOut.h"

NsPwmOut led1(LED1);

int main() {
    led1.period_us(50);
    
    unsigned char brightness;
    for (;;) {
        led1.write(brightness / 255.f);
        ++brightness;
        thread_sleep_for(10);
    }
}
```
