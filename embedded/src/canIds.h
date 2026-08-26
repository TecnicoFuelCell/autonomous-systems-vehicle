// CAN System

#ifndef CANIDS_H
#define CANIDS_H

#include <stdio.h>
#include <stdint.h>

// deadman_switch Module
typedef struct deadman_msg {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
} deadman_msg;


static deadman_msg deadman_msg_instance = {
    .CAN_ID = 32,
    .name = "mensagem do dead-man switch",
    .dlc = 1,
};

// pedal_mod Module
typedef struct pedal_msg {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        float scale;
        float offset;
        char* unit;
        char* comment;
        float min_value;
        float max_value;
        char* type;
        char* byte_order;
    } pedal_voltage;
} pedal_msg;


static pedal_msg pedal_msg_instance = {
    .CAN_ID = 33,
    .name = "mensagem do pedal",
    .dlc = 3,
    .pedal_voltage = {
        .name = "tensao do pedal",
        .start = 0,
        .length = 3,
        .scale = 0.00644,
        .offset = 0.0,
        .unit = "V",
        .comment = "Voltage from the pedal sensor (0.00 to 3.3 V)",
        .min_value = 0.0,
        .max_value = 3.3,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
};

// vesc_mod Module
typedef struct vesc_msg1 {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint16_t max_value;
        char* type;
        char* byte_order;
    } tempMosfet;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } avgMotorCurrent;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } avgInputCurrent;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } dutyCycleNow;
} vesc_msg1;


static vesc_msg1 vesc_msg1_instance = {
    .CAN_ID = 34,
    .name = "Mensagem 1 do VESC",
    .dlc = 8,
    .tempMosfet = {
        .name = "tempMosfet",
        .start = 0,
        .length = 2,
        .scale = 1,
        .offset = 0.0,
        .unit = "",
        .comment = "Temperature of MOSFET",
        .min_value = 0,
        .max_value = 1023,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .avgMotorCurrent = {
        .name = "avgMotorCurrent",
        .start = 2,
        .length = 1,
        .scale = 1,
        .offset = 0.0,
        .unit = "A",
        .comment = "Average Motor Current",
        .min_value = 0,
        .max_value = 255,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .avgInputCurrent = {
        .name = "avgInputCurrent",
        .start = 3,
        .length = 1,
        .scale = 1,
        .offset = 0.0,
        .unit = "A",
        .comment = "Average Input Current",
        .min_value = 0,
        .max_value = 255,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .dutyCycleNow = {
        .name = "dutyCycleNow",
        .start = 4,
        .length = 2,
        .scale = 1,
        .offset = 0.0,
        .unit = "%",
        .comment = "Duty Cycle (0-99)",
        .min_value = 0,
        .max_value = 99,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
};

// vesc_mod Module
typedef struct vesc_msg2 {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } RPM;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } inpVoltage;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint16_t max_value;
        char* type;
        char* byte_order;
    } wattHours;
} vesc_msg2;


static vesc_msg2 vesc_msg2_instance = {
    .CAN_ID = 66,
    .name = "Mensagem 2 do VESC",
    .dlc = 8,
    .RPM = {
        .name = "RPM",
        .start = 0,
        .length = 1,
        .scale = 1,
        .offset = 0.0,
        .unit = "rpm",
        .comment = "Engine RPM (0-60 km/h equivalent)",
        .min_value = 0,
        .max_value = 255,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .inpVoltage = {
        .name = "inpVoltage",
        .start = 1,
        .length = 1,
        .scale = 1,
        .offset = 0.0,
        .unit = "V",
        .comment = "Input Voltage (max 50V)",
        .min_value = 0,
        .max_value = 50,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .wattHours = {
        .name = "wattHours",
        .start = 2,
        .length = 2,
        .scale = 1,
        .offset = 0.0,
        .unit = "Wh",
        .comment = "Watt Hours (0-999)",
        .min_value = 0,
        .max_value = 999,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
};

// pc_mod Module
typedef struct cur_dir_msg {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint32_t max_value;
        char* type;
        char* byte_order;
    } time_stamp;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        int16_t min_value;
        uint16_t max_value;
        char* type;
        char* byte_order;
    } dir_Angle;
} cur_dir_msg;


static cur_dir_msg cur_dir_msg_instance = {
    .CAN_ID = 99,
    .name = "mensagem do PC para o motor de direcao",
    .dlc = 8,
    .time_stamp = {
        .name = "message timestamp",
        .start = 0,
        .length = 16,
        .scale = 1,
        .offset = 0.0,
        .unit = "ms",
        .comment = "Timestamp of the message",
        .min_value = 0,
        .max_value = 65536,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .dir_Angle = {
        .name = "angulo do motor",
        .start = 16,
        .length = 16,
        .scale = 1,
        .offset = 0.0,
        .unit = "°",
        .comment = "Direction motor angle",
        .min_value = -1350,
        .max_value = 1350,
        .type = "signed",
        .byte_order = "little_endian",
    },
};

// pc_mod Module
typedef struct pcsender_heartbeat_msg {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
} pcsender_heartbeat_msg;


static pcsender_heartbeat_msg pcsender_heartbeat_msg_instance = {
    .CAN_ID = 67,
};

// dir_mod Module
typedef struct dir_msg {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint32_t max_value;
        char* type;
        char* byte_order;
    } time_stamp;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        int16_t min_value;
        uint16_t max_value;
        char* type;
        char* byte_order;
    } dir_Angle;
} dir_msg;


static dir_msg dir_msg_instance = {
    .CAN_ID = 100,
    .name = "mensagem do motor de direcao para o PC",
    .dlc = 8,
    .time_stamp = {
        .name = "message timestamp",
        .start = 0,
        .length = 16,
        .scale = 1,
        .offset = 0.0,
        .unit = "ms",
        .comment = "Timestamp of the message",
        .min_value = 0,
        .max_value = 65536,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .dir_Angle = {
        .name = "angulo do motor",
        .start = 16,
        .length = 16,
        .scale = 1,
        .offset = 0.0,
        .unit = "°",
        .comment = "Direction motor angle",
        .min_value = -1350,
        .max_value = 1350,
        .type = "signed",
        .byte_order = "little_endian",
    },
};

// imu_mod Module
typedef struct imu_acc_message {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    char* name;
    uint8_t dlc;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        float scale;
        float offset;
        char* unit;
        char* comment;
        float min_value;
        float max_value;
        char* type;
        char* byte_order;
    } roll;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        float scale;
        float offset;
        char* unit;
        char* comment;
        float min_value;
        float max_value;
        char* type;
        char* byte_order;
    } pitch;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        float scale;
        float offset;
        char* unit;
        char* comment;
        float min_value;
        float max_value;
        char* type;
        char* byte_order;
    } yaw;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } status;
    struct {
        char* name;
        uint8_t start;
        uint8_t length;
        uint8_t scale;
        float offset;
        char* unit;
        char* comment;
        uint8_t min_value;
        uint8_t max_value;
        char* type;
        char* byte_order;
    } counter;
} imu_acc_message;


static imu_acc_message imu_acc_message_instance = {
    .CAN_ID = 5,
    .name = "imu_message",
    .dlc = 8,
    .roll = {
        .name = "roll",
        .start = 0,
        .length = 16,
        .scale = 0.01,
        .offset = 0.0,
        .unit = "deg",
        .comment = "IMU roll angle",
        .min_value = -180.0,
        .max_value = 180.0,
        .type = "signed",
        .byte_order = "little_endian",
    },
    .pitch = {
        .name = "pitch",
        .start = 16,
        .length = 16,
        .scale = 0.01,
        .offset = 0.0,
        .unit = "deg",
        .comment = "IMU pitch angle",
        .min_value = -180.0,
        .max_value = 180.0,
        .type = "signed",
        .byte_order = "little_endian",
    },
    .yaw = {
        .name = "yaw",
        .start = 32,
        .length = 16,
        .scale = 0.01,
        .offset = 0.0,
        .unit = "deg",
        .comment = "IMU yaw angle",
        .min_value = -180.0,
        .max_value = 180.0,
        .type = "signed",
        .byte_order = "little_endian",
    },
    .status = {
        .name = "status",
        .start = 48,
        .length = 8,
        .scale = 1,
        .offset = 0.0,
        .unit = "",
        .comment = "IMU status flags",
        .min_value = 0,
        .max_value = 255,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
    .counter = {
        .name = "counter",
        .start = 56,
        .length = 8,
        .scale = 1,
        .offset = 0.0,
        .unit = "",
        .comment = "Frame counter",
        .min_value = 0,
        .max_value = 255,
        .type = "unsigned",
        .byte_order = "little_endian",
    },
};

// imu_mod Module
typedef struct imu_mag_message {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    uint8_t dlc;
} imu_mag_message;


static imu_mag_message imu_mag_message_instance = {
    .CAN_ID = 37,
    .dlc = 8,
};

// imu_mod Module
typedef struct imu_gyro_message {
    uint8_t CAN_ID; // 11-bit CAN ID for this message
    uint8_t dlc;
} imu_gyro_message;


static imu_gyro_message imu_gyro_message_instance = {
    .CAN_ID = 69,
    .dlc = 8,
};


#endif // CANIDS_H
