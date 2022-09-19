#ifndef NSPWMOUT_H_INCLUDED
#define NSPWMOUT_H_INCLUDED

#include "nspwmout_api.h"

#define NSPWM_NSEC_PER_SEC (1000UL * 1000UL * 1000UL)

namespace mext {

class NsPwmOut {
public:
	NsPwmOut(PinName p) {
		mhal::nspwmout_init(&pwm, p);
		cycles_per_sec_ = mhal::nspwmout_get_tim_clk(&pwm);
	}

	void period(float seconds) {
		period_ns((int)(seconds * NSPWM_NSEC_PER_SEC));
	}

	void period_ms(int ms) {
		period_ns(ms * 1000 * 1000);
	}

	void period_us(int us) {
		period_ns(us * 1000);
	}

	void period_ns(int ns) {
		if (ns < 0) {
			ns = 0;
		}

		uint32_t period_cycles = ns_to_cycles(ns);

		CriticalSectionLock lock;
		float duty = read();
		mhal::nspwmout_set_cycles(&pwm, period_cycles, (uint32_t)(period_cycles * duty));
	}

	void pulsewidth(float seconds) {
		pulsewidth_ns((int)seconds * NSPWM_NSEC_PER_SEC);
	}

	void pulsewidth_ms(int ms) {
		pulsewidth_ns(ms * 1000 * 1000);
	}

	void pulsewidth_us(int us) {
		pulsewidth_ns(us * 1000);
	}

	void pulsewidth_ns(int ns) {
		if (ns < 0) {
			ns = 0;
		}

		uint32_t pulse_cycles = ns_to_cycles(ns);

		CriticalSectionLock lock;

		const uint32_t period_cycles = mhal::nspwmout_period_cycles(&pwm);
		if (pulse_cycles > period_cycles) {
			pulse_cycles = period_cycles;
		}

		mhal::nspwmout_set_pulse_cycles(&pwm, pulse_cycles);
	}

	int read_pulsewidth_us() {
		return read_pulsewidth_ns() / 1000;
	}

	int read_pulsewidth_ns() {
		CriticalSectionLock lock;
		return cycles_to_ns(mhal::nspwmout_pulse_cycles(&pwm));
	}

	int read_period_us() {
		return read_period_ns() / 1000;
	}

	int read_period_ns() {
		CriticalSectionLock lock;
		return cycles_to_ns(mhal::nspwmout_period_cycles(&pwm));
	}

	void write(float duty) {

		if (duty < 0.f) {
			duty = 0.f;
		} else if (duty > 1.f) {
			duty = 1.f;
		}

		CriticalSectionLock lock;

		const uint32_t pulse_cycles = (uint32_t)(mhal::nspwmout_period_cycles(&pwm) * duty);
		mhal::nspwmout_set_pulse_cycles(&pwm, pulse_cycles);
	}

	float read() {
		CriticalSectionLock lock;
		return (float)mhal::nspwmout_pulse_cycles(&pwm) / mhal::nspwmout_period_cycles(&pwm);
	}

	NsPwmOut &operator=(float value) {
		write(value);
		return *this;
	}

	NsPwmOut &operator=(NsPwmOut &rhs) {
		write(rhs.read());
		return *this;
	}

	operator float() {
		return read();
	}

	uint32_t resolution_ns() const {
		return NSPWM_NSEC_PER_SEC / cycles_per_sec_;
	}

	~NsPwmOut() {
		CriticalSectionLock lock; // lock as mbed API can possibly alter global environment
		mhal::nspwmout_free(&pwm);
	}

private:

	uint32_t ns_to_cycles(uint32_t ns) const {
		uint64_t cycles = ((uint64_t)ns * cycles_per_sec_) / NSPWM_NSEC_PER_SEC;
		if (cycles > UINT32_MAX) {
			error("too large");
		}
		return (uint32_t)cycles;
	}

	uint32_t cycles_to_ns(uint32_t cycles) const {
		uint64_t ns = ((uint64_t)cycles * NSPWM_NSEC_PER_SEC) / cycles_per_sec_;
		if (ns > UINT32_MAX) {
			error("too large");
		}
		return (uint32_t)ns;
	}

	DeepSleepLock lock;

	mhal::nspwmout_t pwm;
	uint32_t cycles_per_sec_;
};

}

#endif
