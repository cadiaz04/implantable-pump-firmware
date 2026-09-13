// =============================================================================
// main.cpp — Implantable Pump Firmware entry point
//
// Structure:
//   - setup() creates a FreeRTOS queue for flow-rate setpoints and spins up
//     controlTask() as its own task, pinned to a core, running on a fixed
//     period. This keeps pump timing deterministic regardless of what else
//     (serial I/O, future BLE, etc.) is happening.
//   - loop() is left free for lower-priority housekeeping (serial command
//     parsing here as a placeholder for bench testing).
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "drivers/stepper_driver.h"
#include "control/flow_controller.h"

static StepperDriver stepper(PIN_STEP, PIN_DIR, PIN_ENABLE);

static const PumpGeometry geometry = {
    STEPS_PER_REVOLUTION,
    MICROSTEPPING,
    LEAD_SCREW_MM_PER_REV,
    SYRINGE_AREA_MM2,
};
static FlowController flowController(geometry, MIN_FLOW_RATE_ML_HR, MAX_FLOW_RATE_ML_HR);

// Queue carries new flow-rate setpoints (mL/hr, as float) from loop()/serial
// into the control task. Keep it small — we only care about the latest value.
static QueueHandle_t setpointQueue;

static void controlTask(void* pvParameters) {
    (void)pvParameters;
    stepper.begin();
    stepper.setEnabled(true);

    float currentSetpointMlHr = 0.0f;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // Drain queue — take the most recent setpoint if one arrived.
        float newSetpoint;
        while (xQueueReceive(setpointQueue, &newSetpoint, 0) == pdTRUE) {
            currentSetpointMlHr = newSetpoint;
        }

        const float hz = flowController.computeStepFrequencyHz(currentSetpointMlHr);

        if (!flowController.isSafe()) {
            // Out-of-range or invalid setpoint: fail safe, do not move.
            stepper.stop();
            digitalWrite(PIN_STATUS_LED, HIGH);   // solid LED = fault state, placeholder
        } else {
            stepper.setStepFrequency(hz, /*forward=*/true);
            digitalWrite(PIN_STATUS_LED, (xTaskGetTickCount() / 10) % 2);   // slow blink = running
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);

    // Length-1 queue used with xQueueOverwrite: we only ever care about the
    // latest setpoint, not a history of them.
    setpointQueue = xQueueCreate(1, sizeof(float));

    xTaskCreatePinnedToCore(
        controlTask,
        "controlTask",
        4096,           // stack size (words) — increase if you add float-heavy math or FreeRTOS features
        nullptr,
        2,              // priority — higher than default loopTask (1)
        nullptr,
        1               // pin to core 1, leave core 0 for WiFi/BT stack if you add it later
    );

    Serial.println("Implantable pump firmware booted.");
    Serial.println("Send a number over serial to set flow rate in mL/hr (e.g. 5.0)");
}

void loop() {
    // Placeholder bench-test interface: type a number + newline in Serial Monitor
    // to set a flow rate. Replace with your real command/telemetry protocol
    // (BLE, UART framing, etc.) once you have one.
    if (Serial.available()) {
        float requested = Serial.parseFloat();
        while (Serial.available()) Serial.read();   // clear trailing newline/junk

        xQueueOverwrite(setpointQueue, &requested);
        Serial.print("Setpoint queued: ");
        Serial.print(requested);
        Serial.println(" mL/hr");
    }
    delay(10);
}
