// class written for 3DM-CV7 imu (sheteshreyash)
#include "SIM_MicroStrain.h"
#include <AP_Logger/AP_Logger.h>

using namespace SITL;

void MicroStrainCV7::send_ahrs_packet(void)
{
    const auto &fdm = _sitl->state;
    MicroStrain_Packet packet;

    // Packet headers for ahrs-like structure
    packet.header[0] = 0x75; // Sync One
    packet.header[1] = 0x65; // Sync Two
    packet.header[2] = 0x80; // ahrs data

    // Complementary Filter Quaternion
    float q1 = fdm.quaternion.q1;
    float q2 = fdm.quaternion.q2;
    float q3 = fdm.quaternion.q3;
    float q4 = fdm.quaternion.q4;

    float norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    if (norm > 0.0f) {
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        q4 /= norm;
    }

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x80/data/0x0a.htm
    packet.payload[packet.payload_size++] = 0x02; // Field Size for Quaternion
    packet.payload[packet.payload_size++] = 0x0A; // Quaternion Descriptor
    put_float(packet, q1); // w
    put_float(packet, q2); // x
    put_float(packet, q3); // y
    put_float(packet, q4); // z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x80/data/0x05.htm
    // Scaled Gyroscope (Angular rate in filtered data)
    packet.payload[packet.payload_size++] = 0x02; // Field Size for Scaled Gyroscope
    packet.payload[packet.payload_size++] = 0x05; // Scaled Gyroscope (Angular Rate) Descriptor
    put_float(packet, fdm.rollRate);  // Angular rate X (roll)
    put_float(packet, fdm.pitchRate); // Angular rate Y (pitch)
    put_float(packet, fdm.yawRate);   // Angular rate Z (yaw)

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x80/data/0x04.htm
    // Scaled Acceleration (Linear Acceleration in filtered data)
    packet.payload[packet.payload_size++] = 0x02; // Field Size for Scaled Acceleration
    packet.payload[packet.payload_size++] = 0x04; // Acceleration Descriptor
    put_float(packet, fdm.xAccel); // Acceleration X
    put_float(packet, fdm.yAccel); // Acceleration Y
    put_float(packet, fdm.zAccel); // Acceleration Z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x80/data/0x06.htm
    // Scaled Magnetometer
    packet.payload[packet.payload_size++] = 0x02; // Field Size for Magnetometer
    packet.payload[packet.payload_size++] = 0x06; // Magnetometer Descriptor
    put_float(packet, fdm.bodyMagField.x); // Scaled X magnetic field
    put_float(packet, fdm.bodyMagField.y); // Scaled Y magnetic field
    put_float(packet, fdm.bodyMagField.z); // Scaled Z magnetic field

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x80/data/0x0c.htm
    // Complementary Filter Euler Angles
    packet.payload[packet.payload_size++] = 0x0E; // Field Size for Euler Angles
    packet.payload[packet.payload_size++] = 0x0C; // Euler Angles Descriptor
    put_float(packet, fdm.rollDeg);  // Roll
    put_float(packet, fdm.pitchDeg); // Pitch
    put_float(packet, fdm.yawDeg);   // Yaw


    // Packet length update
    packet.header[3] = packet.payload_size;

    // Send the packet
    send_packet(packet);
}


void MicroStrainCV7::send_filter_packet(void)
{
    const auto &fdm = _sitl->state;
    MicroStrain_Packet packet;

    // Packet headers for CV7
    packet.header[0] = 0x75; // Sync One
    packet.header[1] = 0x65; // Sync Two
    packet.header[2] = 0x82; // Filter Descriptor

    // Quaternion orientation (Attitude Quaternion)
    float q1 = fdm.quaternion.q1;
    float q2 = fdm.quaternion.q2;
    float q3 = fdm.quaternion.q3;
    float q4 = fdm.quaternion.q4;

    float norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    if (norm > 0.0f) {
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        q4 /= norm;
    }

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x03.htm
    packet.payload[packet.payload_size++] = 0x04; // Field Size for Quaternion
    packet.payload[packet.payload_size++] = 0x03; // Quaternion Descriptor
    put_float(packet, q1); // w
    put_float(packet, q2); // x
    put_float(packet, q3); // y
    put_float(packet, q4); // z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x0e.htm
    // Angular Rates (Compensated Angular Rate)
    packet.payload[packet.payload_size++] = 0x04; // Field Size for Angular Rates
    packet.payload[packet.payload_size++] = 0x0E; // Angular Rate Descriptor
    put_float(packet, fdm.rollRate);  // Angular rate X (roll)
    put_float(packet, fdm.pitchRate); // Angular rate Y (pitch)
    put_float(packet, fdm.yawRate);   // Angular rate Z (yaw)

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x0d.htm
    // Linear Acceleration (Accel Bias)
    packet.payload[packet.payload_size++] = 0x04; // Field Size for Linear Acceleration
    packet.payload[packet.payload_size++] = 0x0D; // Acceleration Descriptor
    put_float(packet, fdm.xAccel); // Acceleration X
    put_float(packet, fdm.yAccel); // Acceleration Y
    put_float(packet, fdm.zAccel); // Acceleration Z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x05.htm
    // Euler Angles
    packet.payload[packet.payload_size++] = 0x10; // Field Size for Euler Angles
    packet.payload[packet.payload_size++] = 0x05; // Euler Angles Descriptor
    put_float(packet, fdm.rollDeg);  // Roll
    put_float(packet, fdm.pitchDeg); // Pitch
    put_float(packet, fdm.yawDeg);   // Yaw

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x1c.htm
    // Compensated Acceleration
    packet.payload[packet.payload_size++] = 0x04; // Field Size for Scaled Acceleration
    packet.payload[packet.payload_size++] = 0x1C; // Acceleration Descriptor
    put_float(packet, fdm.xAccel); // Acceleration X
    put_float(packet, fdm.yAccel); // Acceleration Y
    put_float(packet, fdm.zAccel); // Acceleration Z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x13.htm
    // Gravity Vector
    packet.payload[packet.payload_size++] = 0x04; // Field Size for Gravity Vector
    packet.payload[packet.payload_size++] = 0x13; // Gravity Vector Descriptor
    put_float(packet, fdm.gravity_x); // Gravity X
    put_float(packet, fdm.gravity_y); // Gravity Y
    put_float(packet, fdm.gravity_z); // Gravity Z

    // https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/0x82/data/0x21.htm
    // Pressure Altitude
    packet.payload[packet.payload_size++] = 0x08; // Field Size for Pressure Altitude
    packet.payload[packet.payload_size++] = 0x21; // Pressure Altitude Descriptor
    put_float(packet, fdm.altitude); // Altitude

    // Packet length update
    packet.header[3] = packet.payload_size;

    // Send the packet
    send_packet(packet);
}
