# mext_nspwmout_stm32

PwmOutとほとんど同じインターフェースです

- `suspend`, `resume`はありません

- パルス幅の設定には，`period_{ms, us}`, `pulsewidth_{ms, us}`に加え，`period_ns`, `pulsewidth_ns`が使えます

タイマーによってPwmOutを使うかNsPwmOutを使うかを揃える必要があります

```cpp
#include "mbed.h"
#include "NsPwmOut.h"

using mext::NsPwmOut;

NsPwmOut led1(LED1);

int main() {
	led1.period_us(50);
	
	unsigned char brightness = 0;
	for (;;) {
		led1.write(brightness / 255.f);
		++brightness;
		thread_sleep_for(10);
	}
}
```
