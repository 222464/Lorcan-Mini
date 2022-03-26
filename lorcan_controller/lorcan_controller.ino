#include <iq_module_communication.hpp>

const float pi = 3.141592f;

// Callibration
const float offsets[] = { 0.32f, 0.32f, -0.61f, -0.61f, 0.68f, 0.93f, 0.0f, -0.36f };

const float maxAngleKp = 50.0f;
const float maxAngleKd = 0.1f;

const float initTorque = 0.05f;

const float maxAngle = pi / 3.0f;

IqSerial ser1(Serial1);
IqSerial ser2(Serial2);
IqSerial ser3(Serial3);
IqSerial ser4(Serial4);
IqSerial ser5(Serial5);
IqSerial ser6(Serial6);
IqSerial ser7(Serial7);
IqSerial ser8(Serial8);

IqSerial* sers[] = {
    &ser1,
    &ser2,
    &ser3,
    &ser4,
    &ser5,
    &ser6,
    &ser7,
    &ser8
};

MultiTurnAngleControlClient mot1(0);
MultiTurnAngleControlClient mot2(0);
MultiTurnAngleControlClient mot3(0);
MultiTurnAngleControlClient mot4(0);
MultiTurnAngleControlClient mot5(0);
MultiTurnAngleControlClient mot6(0);
MultiTurnAngleControlClient mot7(0);
MultiTurnAngleControlClient mot8(0);

MultiTurnAngleControlClient* mots[] = {
    &mot1,
    &mot2,
    &mot3,
    &mot4,
    &mot5,
    &mot6,
    &mot7,
    &mot8
};

void setup() {
    Serial.begin(9600); // Baud doesn't matter for Teensy 4.0-4.1

    for (int i = 0; i < 8; i++) {
        sers[i]->begin();

        sers[i]->set(mots[i]->angle_Kp_, maxAngleKp * initTorque);
        sers[i]->set(mots[i]->angle_Kd_, maxAngleKd);
        sers[i]->set(mots[i]->ctrl_angle_, offsets[i]);
    }
}

void loop() {
    // Wait for command
    while (Serial.available() < 16)
        delayMicroseconds(100);

    byte angles[8];

    for (int i = 0; i < 8; i++) {
        float obs_angle = 0.0f;
        sers[i]->get(mots[i]->obs_angular_displacement_, obs_angle);

        angles[i] = (byte)(min(1.0f, max(0.0f, obs_angle / pi * 0.5f + 0.5f)) * 255.0f + 0.5f);

        float angle = (Serial.read() / 255.0f * 2.0f - 1.0f) * pi;
        float torque = Serial.read() / 255.0f;

        sers[i]->set(mots[i]->ctrl_angle_, offsets[i] + min(maxAngle, max(-maxAngle, angle)));
        sers[i]->set(mots[i]->angle_Kp_, maxAngleKp * torque);
    }

    while (Serial.available() > 0)
        Serial.read();

    // Write angles
    if (Serial.availableForWrite() >= 8)
        Serial.write(angles, 8);
}
