/*
 * Threshold-based temporary layer input processor for ZMK
 * Activates a layer only when accumulated movement exceeds a threshold
 */

#define DT_DRV_COMPAT zmk_input_processor_temp_layer_threshold

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct temp_layer_threshold_config {
    int32_t threshold;
    int32_t threshold_time_ms;
    int32_t require_prior_idle_ms;
    size_t num_positions;
    const uint32_t *excluded_positions;
};

struct temp_layer_threshold_state {
    const struct device *dev;
    int32_t accumulated_x;
    int32_t accumulated_y;
    int64_t accumulation_start_time;
    int8_t active_layer;
    int64_t last_tapped_time;
    bool layer_active;
    struct k_work_delayable layer_disable_work;
    struct k_work_delayable accumulation_reset_work;
};

static void layer_disable_callback(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct temp_layer_threshold_state *state = CONTAINER_OF(
        dwork, struct temp_layer_threshold_state, layer_disable_work);

    if (state->layer_active && state->active_layer >= 0) {
        LOG_DBG("Threshold temp layer: deactivating layer %d", state->active_layer);
        zmk_keymap_layer_deactivate(state->active_layer);
        state->layer_active = false;
        state->active_layer = -1;
    }
}

static void accumulation_reset_callback(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct temp_layer_threshold_state *state = CONTAINER_OF(
        dwork, struct temp_layer_threshold_state, accumulation_reset_work);

    state->accumulated_x = 0;
    state->accumulated_y = 0;
    LOG_DBG("Threshold temp layer: accumulation reset");
}

static bool is_position_excluded(const struct temp_layer_threshold_config *cfg,
                                  uint32_t position) {
    for (size_t i = 0; i < cfg->num_positions; i++) {
        if (cfg->excluded_positions[i] == position) {
            return true;
        }
    }
    return false;
}

static int position_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

#define CHECK_LAYER(n)                                                         \
    do {                                                                        \
        const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(n));              \
        struct temp_layer_threshold_state *state = (void *)dev->data;          \
        const struct temp_layer_threshold_config *cfg = dev->config;           \
        if (state->layer_active && !is_position_excluded(cfg, ev->position)) { \
            k_work_cancel_delayable(&state->layer_disable_work);               \
            LOG_DBG("Threshold temp layer: key press, deactivating layer");    \
            zmk_keymap_layer_deactivate(state->active_layer);                  \
            state->layer_active = false;                                       \
            state->active_layer = -1;                                          \
        }                                                                      \
        state->last_tapped_time = ev->timestamp;                               \
    } while (0);

    DT_INST_FOREACH_STATUS_OKAY(CHECK_LAYER)

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(temp_layer_threshold, position_state_changed_listener);
ZMK_SUBSCRIPTION(temp_layer_threshold, zmk_position_state_changed);

static int temp_layer_threshold_handle_event(const struct device *dev,
                                              struct input_event *event,
                                              uint32_t param1,
                                              uint32_t param2,
                                              struct zmk_input_processor_state *input_state) {
    const struct temp_layer_threshold_config *cfg = dev->config;
    struct temp_layer_threshold_state *state = (void *)dev->data;

    uint8_t layer = param1;
    uint32_t timeout_ms = param2;
    int64_t now = k_uptime_get();

    // Check prior idle requirement
    if (cfg->require_prior_idle_ms > 0) {
        if ((now - state->last_tapped_time) < cfg->require_prior_idle_ms) {
            return ZMK_INPUT_PROC_CONTINUE;
        }
    }

    // Accumulate movement
    if (event->type == INPUT_EV_REL) {
        if (event->code == INPUT_REL_X) {
            state->accumulated_x += (event->value > 0) ? event->value : -event->value;
        } else if (event->code == INPUT_REL_Y) {
            state->accumulated_y += (event->value > 0) ? event->value : -event->value;
        }
    }

    int32_t total_movement = state->accumulated_x + state->accumulated_y;

    LOG_DBG("Threshold temp layer: movement=%d, threshold=%d, layer_active=%d",
            total_movement, cfg->threshold, state->layer_active);

    // Reset accumulation timer
    k_work_reschedule(&state->accumulation_reset_work, K_MSEC(cfg->threshold_time_ms));

    // If layer is already active, just reset the timeout
    if (state->layer_active) {
        k_work_reschedule(&state->layer_disable_work, K_MSEC(timeout_ms));
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Check if threshold is exceeded
    if (total_movement >= cfg->threshold) {
        LOG_DBG("Threshold temp layer: threshold exceeded, activating layer %d", layer);
        zmk_keymap_layer_activate(layer);
        state->layer_active = true;
        state->active_layer = layer;

        // Reset accumulation
        state->accumulated_x = 0;
        state->accumulated_y = 0;

        // Schedule layer disable
        k_work_reschedule(&state->layer_disable_work, K_MSEC(timeout_ms));
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api temp_layer_threshold_driver_api = {
    .handle_event = temp_layer_threshold_handle_event,
};

static int temp_layer_threshold_init(const struct device *dev) {
    struct temp_layer_threshold_state *state = (void *)dev->data;
    state->dev = dev;
    state->accumulated_x = 0;
    state->accumulated_y = 0;
    state->active_layer = -1;
    state->layer_active = false;
    state->last_tapped_time = 0;

    k_work_init_delayable(&state->layer_disable_work, layer_disable_callback);
    k_work_init_delayable(&state->accumulation_reset_work, accumulation_reset_callback);

    return 0;
}

#define TEMP_LAYER_THRESHOLD_INST(n)                                                    \
    static const uint32_t excluded_positions_##n[] =                                    \
        DT_INST_PROP(n, excluded_positions);                                            \
    static struct temp_layer_threshold_state state_##n = {};                            \
    static const struct temp_layer_threshold_config config_##n = {                      \
        .threshold = DT_INST_PROP(n, threshold),                                        \
        .threshold_time_ms = DT_INST_PROP(n, threshold_time_ms),                        \
        .require_prior_idle_ms = DT_INST_PROP(n, require_prior_idle_ms),               \
        .num_positions = DT_INST_PROP_LEN(n, excluded_positions),                       \
        .excluded_positions = excluded_positions_##n,                                   \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, temp_layer_threshold_init, NULL, &state_##n, &config_##n, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,             \
                          &temp_layer_threshold_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TEMP_LAYER_THRESHOLD_INST)
