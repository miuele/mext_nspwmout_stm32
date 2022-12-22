#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "pwmout_device.h"

#include <stdint.h>

/*
 * Many parts of this code were taken from Zephyr PWM STM32 driver
 * https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/pwm/pwm_stm32.c (Apache 2.0)
 *
 * and also from mbed
 * https://github.com/ARMmbed/mbed-os/blob/master/targets/TARGET_STM/pwmout_api.c
 *
*/

namespace mhal {

struct nspwmout_t {
	PWMName pwm;
	PinName pin;
	uint32_t period_cycles;
	uint32_t pulse_cycles;
	uint8_t channel;
	uint8_t inverted;
};

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define NSPWM_TIMER_MAX_CH 6u
#else
#define NSPWM_TIMER_MAX_CH 4u
#endif

inline uint32_t ch2ll(uint32_t chan) {
	/** Channel to LL mapping. */
	static constexpr uint32_t ch2ll_map[NSPWM_TIMER_MAX_CH] = {
		LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
		LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if defined(LL_TIM_CHANNEL_CH6)
		LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
	};
	return ch2ll_map[chan - 1];
}


#if defined(LL_TIM_CHANNEL_CH4N)
#define NSPWM_TIMER_MAX_NCH 4u
#elif defined(LL_TIM_CHANNEL_CH1N)
#define NSPWM_TIMER_MAX_NCH 3u
#else
#define NSPWM_TIMER_MAX_NCH 0u
#endif

inline uint32_t ch2ll_n(uint32_t chan) {
	/** Some stm32 mcus have complementary channels : 3 or 4 */
	static constexpr uint32_t ch2ll_n_map[NSPWM_TIMER_MAX_NCH] = {
#if defined(LL_TIM_CHANNEL_CH1N)
		LL_TIM_CHANNEL_CH1N,
		LL_TIM_CHANNEL_CH2N,
		LL_TIM_CHANNEL_CH3N,
#if defined(LL_TIM_CHANNEL_CH4N)
		/** stm32g4x and stm32u5x have 4 complementary channels */
		LL_TIM_CHANNEL_CH4N,
#endif /* LL_TIM_CHANNEL_CH4N */
#endif /* LL_TIM_CHANNEL_CH1N */
	};
	return ch2ll_n_map[chan - 1];
}

inline void set_timer_compare(TIM_TypeDef *tim, uint32_t chan, uint32_t c) {
	/** Channel to compare set function mapping. */
	static void (*const set_timer_compare_map[NSPWM_TIMER_MAX_CH])(TIM_TypeDef *, uint32_t) = {
		LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
		LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
#if defined(LL_TIM_CHANNEL_CH6)
		LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
	};
	set_timer_compare_map[chan - 1](tim, c);
}

inline uint32_t nspwmout_get_tim_clk(nspwmout_t *obj) {
	/*	Parse the pwm / apb mapping table to find the right entry */
	const pwm_apb_map_t *map = pwm_apb_map_table;
	while (map->pwm != obj->pwm) {
		if (map->pwm == 0) {
			error("Unknown PWM instance");
		}
		++map;
	}

	uint32_t apb_presc = 0;
	uint32_t pclk = 0;
	if (map->pwmoutApb == PWMOUT_ON_APB1) {
		apb_presc = (uint32_t)LL_RCC_GetAPB1Prescaler();
		pclk = HAL_RCC_GetPCLK1Freq();
	} else if (map->pwmoutApb == PWMOUT_ON_APB2) {
		apb_presc = (uint32_t)LL_RCC_GetAPB2Prescaler();
		pclk = HAL_RCC_GetPCLK2Freq();
	} else {
		MBED_ASSERT(false);
	}

#if defined(DCKCFGR) && defined(RCC_DCKCFGR_TIMPRE)
	MBED_ASSERT((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == 0u);
#endif

	if (apb_presc == RCC_HCLK_DIV1) {
		return pclk;
	} else {
		return pclk * 2;
	}
}

inline void nspwmout_init(nspwmout_t *obj, PinName pin)
{
	obj->pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
	TIM_TypeDef *timer = (TIM_TypeDef *)obj->pwm;

	uint32_t function = (uint32_t)pinmap_find_function(pin, PinMap_PWM);

	MBED_ASSERT(function != (uint32_t)NC);

#if defined(TIM1_BASE)
	if (obj->pwm == PWM_1) {
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
#endif
#if defined(TIM2_BASE)
	if (obj->pwm == PWM_2) {
		__HAL_RCC_TIM2_CLK_ENABLE();
	}
#endif
#if defined(TIM3_BASE)
	if (obj->pwm == PWM_3) {
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
#endif
#if defined(TIM4_BASE)
	if (obj->pwm == PWM_4) {
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
#endif
#if defined(TIM5_BASE)
	if (obj->pwm == PWM_5) {
		__HAL_RCC_TIM5_CLK_ENABLE();
	}
#endif
#if defined(TIM8_BASE)
	if (obj->pwm == PWM_8) {
		__HAL_RCC_TIM8_CLK_ENABLE();
	}
#endif
#if defined(TIM9_BASE)
	if (obj->pwm == PWM_9) {
		__HAL_RCC_TIM9_CLK_ENABLE();
	}
#endif
#if defined(TIM10_BASE)
	if (obj->pwm == PWM_10) {
		__HAL_RCC_TIM10_CLK_ENABLE();
	}
#endif
#if defined(TIM11_BASE)
	if (obj->pwm == PWM_11) {
		__HAL_RCC_TIM11_CLK_ENABLE();
	}
#endif
#if defined(TIM12_BASE)
	if (obj->pwm == PWM_12) {
		__HAL_RCC_TIM12_CLK_ENABLE();
	}
#endif
#if defined(TIM13_BASE)
	if (obj->pwm == PWM_13) {
		__HAL_RCC_TIM13_CLK_ENABLE();
	}
#endif
#if defined(TIM14_BASE)
	if (obj->pwm == PWM_14) {
		__HAL_RCC_TIM14_CLK_ENABLE();
	}
#endif
#if defined(TIM15_BASE)
	if (obj->pwm == PWM_15) {
		__HAL_RCC_TIM15_CLK_ENABLE();
	}
#endif
#if defined(TIM16_BASE)
	if (obj->pwm == PWM_16) {
		__HAL_RCC_TIM16_CLK_ENABLE();
	}
#endif
#if defined(TIM17_BASE)
	if (obj->pwm == PWM_17) {
		__HAL_RCC_TIM17_CLK_ENABLE();
	}
#endif
#if defined(TIM18_BASE)
	if (obj->pwm == PWM_18) {
		__HAL_RCC_TIM18_CLK_ENABLE();
	}
#endif
#if defined(TIM19_BASE)
	if (obj->pwm == PWM_19) {
		__HAL_RCC_TIM19_CLK_ENABLE();
	}
#endif
#if defined(TIM20_BASE)
	if (obj->pwm == PWM_20) {
		__HAL_RCC_TIM20_CLK_ENABLE();
	}
#endif
#if defined(TIM21_BASE)
	if (obj->pwm == PWM_21) {
		__HAL_RCC_TIM21_CLK_ENABLE();
	}
#endif
#if defined(TIM22_BASE)
	if (obj->pwm == PWM_22) {
		__HAL_RCC_TIM22_CLK_ENABLE();
	}
#endif

	pin_function(pin, function);

	obj->channel = STM_PIN_CHANNEL(function);
	obj->inverted = STM_PIN_INVERTED(function);
	obj->pin = pin;
	obj->period_cycles = 0;
	obj->pulse_cycles = 0;

	LL_TIM_InitTypeDef init;
	LL_TIM_StructInit(&init);

	init.Prescaler = 0u;
	init.CounterMode = LL_TIM_COUNTERMODE_UP;
	init.Autoreload = 0u;
	init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

	if (LL_TIM_Init(timer, &init) != SUCCESS) {
		error("failed to initialize timer");
	}

	if (IS_TIM_BREAK_INSTANCE(timer)) {
		LL_TIM_EnableAllOutputs(timer);
	}

	LL_TIM_EnableCounter(timer);
}

inline void nspwmout_free(nspwmout_t *obj)
{
	pin_function(obj->pin, STM_PIN_DATA(STM_MODE_ANALOG, GPIO_NOPULL, 0));
}

namespace detail_nspwmout {

inline void set_cycles(TIM_TypeDef *timer, uint32_t channel, bool cmpl, uint32_t period_cycles, uint32_t pulse_cycles)
{
	if (channel < 1u || (!cmpl && channel > NSPWM_TIMER_MAX_CH) || (cmpl && channel > NSPWM_TIMER_MAX_NCH)) {
		error("Invalid channel (%d)", (int)channel);
	}

	if (!IS_TIM_32B_COUNTER_INSTANCE(timer) && (period_cycles > UINT16_MAX + 1)) {
		error("period_cycles too large: %d", (int)period_cycles);
	}

	uint32_t ll_channel = ch2ll(channel);
	uint32_t current_ll_channel = cmpl ? ch2ll_n(channel) : ll_channel;

	if (period_cycles == 0u) {
		LL_TIM_CC_DisableChannel(timer, current_ll_channel);
	}

	period_cycles -= 1u;

	if (!LL_TIM_CC_IsEnabledChannel(timer, current_ll_channel)) {
		LL_TIM_OC_InitTypeDef oc_init;

		LL_TIM_OC_StructInit(&oc_init);

		oc_init.OCMode = LL_TIM_OCMODE_PWM1;
		oc_init.CompareValue = pulse_cycles;

		if (cmpl) {
			oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
			oc_init.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
		} else {
			oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
			oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
		}

		if (LL_TIM_OC_Init(timer, ll_channel, &oc_init) != SUCCESS) {
			error("Could not initialize timer channel output");
		}

		LL_TIM_EnableARRPreload(timer);
		LL_TIM_OC_EnablePreload(timer, ll_channel);
		LL_TIM_SetAutoReload(timer, period_cycles);
		LL_TIM_GenerateEvent_UPDATE(timer);
	} else {
		set_timer_compare(timer, channel, pulse_cycles);
		LL_TIM_SetAutoReload(timer, period_cycles);
	}
}

}

inline void nspwmout_set_cycles(nspwmout_t *obj, uint32_t period_cycles, uint32_t pulse_cycles) {
	obj->period_cycles = period_cycles;
	obj->pulse_cycles = pulse_cycles;

	detail_nspwmout::set_cycles((TIM_TypeDef *)obj->pwm, obj->channel, (bool)obj->inverted, period_cycles, pulse_cycles);
}

inline void nspwmout_set_period_cycles(nspwmout_t *obj, uint32_t period_cycles) {
	nspwmout_set_cycles(obj, period_cycles, obj->pulse_cycles);
}

inline void nspwmout_set_pulse_cycles(nspwmout_t *obj, uint32_t pulse_cycles) {
	nspwmout_set_cycles(obj, obj->period_cycles, pulse_cycles);
}

inline uint32_t nspwmout_period_cycles(nspwmout_t *obj) {
	return obj->period_cycles;
}

inline uint32_t nspwmout_pulse_cycles(nspwmout_t *obj) {
	return obj->pulse_cycles;
}

}

#undef NSPWM_TIMER_MAX_CH 
#undef NSPWM_TIMER_MAX_NCH 

