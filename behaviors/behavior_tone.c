/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT chiptuner_behavior_tone

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_tone_config {
  size_t pwms_len;
  struct pwm_dt_spec pwms[];
};

struct behavior_tone_inst_data {
  uint32_t tone;
  uint32_t timestamp;
};

struct behavior_tone_data {
  struct behavior_tone_inst_data *tones;
};

static int set_inst_freq(const struct device *dev, uint32_t idx,
                         uint32_t freq) {
  const struct behavior_tone_config *cfg = dev->config;
  struct behavior_tone_data *data = dev->data;

  int ret = pwm_set_dt(&cfg->pwms[idx], PWM_HZ(freq), PWM_HZ(freq) / 2);
  if (ret < 0) {
    LOG_WRN("pwm_set_dt rejected %d/%d with %d", freq, freq/2, ret);
    return ret;
  }

  data->tones[idx].tone = freq;
  data->tones[idx].timestamp = freq > 0 ? k_uptime_get() : 0;

  return 0;
}

static int
on_behavior_tone_binding_pressed(struct zmk_behavior_binding *binding,
                                 struct zmk_behavior_binding_event event) {
  const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
  const struct behavior_tone_config *cfg = dev->config;
  struct behavior_tone_data *data = dev->data;

  uint32_t oldest_idx = UINT32_MAX;
  for (int i = 0; i < cfg->pwms_len; i++) {
    if (data->tones[i].tone == 0) {
      int ret = set_inst_freq(dev, i, binding->param1);

      if (ret < 0) {
        LOG_WRN("Failed to set the tone at index %d", i);
      }

      LOG_DBG("Set the tone at index %d to %d", i, binding->param1);

      return ZMK_BEHAVIOR_OPAQUE;
    } else if (oldest_idx == UINT32_MAX ||
               data->tones[i].timestamp < data->tones[oldest_idx].timestamp) {
      oldest_idx = i;
    }
  }

  LOG_WRN("Failed to find a free PWM, overriding oldest tone");
  int ret = set_inst_freq(dev, oldest_idx, binding->param1);

  if (ret < 0) {
    LOG_WRN("Failed to set the tone at index %d", oldest_idx);
  }

  return ZMK_BEHAVIOR_OPAQUE;
}

static int
on_behavior_tone_binding_released(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event) {
  const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
  const struct behavior_tone_config *cfg = dev->config;
  struct behavior_tone_data *data = dev->data;

  for (int i = 0; i < cfg->pwms_len; i++) {
    if (data->tones[i].tone == binding->param1) {
      int ret = set_inst_freq(dev, i, 0);

      if (ret < 0) {
        LOG_WRN("Failed to clear the tone at index %d", i);
      }

      return ZMK_BEHAVIOR_OPAQUE;
    }
  }

  LOG_WRN("Failed to find a PWM to clear!");

  return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_tone_init(const struct device *dev) {
  const struct behavior_tone_config *cfg = dev->config;
  for (int i = 0; i < cfg->pwms_len; i++) {
	  if (!device_is_ready(cfg->pwms[i].dev)) {
		  LOG_WRN("PWM NOT READY!");
	}
  }

  return 0;
}

static const struct behavior_driver_api behavior_tone_driver_api = {
    .binding_pressed = on_behavior_tone_binding_pressed,
    .binding_released = on_behavior_tone_binding_released,
};

#define GET_PWM_SPEC(n, prop, i) PWM_DT_SPEC_GET_BY_IDX(n, i)

#define TONE_INST(n)                                                           \
  static struct behavior_tone_inst_data tones_##n[DT_INST_PROP_LEN(n, pwms)];  \
  static struct behavior_tone_data behavior_tone_data_##n = {                  \
      .tones = tones_##n,                                                      \
  };                                                                           \
  static const struct behavior_tone_config behavior_tone_config_##n = {        \
      .pwms_len = DT_INST_PROP_LEN(n, pwms),                                   \
      .pwms = {DT_INST_FOREACH_PROP_ELEM_SEP(n, pwms, GET_PWM_SPEC, (, ))},    \
  };                                                                           \
  BEHAVIOR_DT_INST_DEFINE(n, behavior_tone_init, NULL,                         \
                          &behavior_tone_data_##n, &behavior_tone_config_##n,  \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                          &behavior_tone_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TONE_INST)
