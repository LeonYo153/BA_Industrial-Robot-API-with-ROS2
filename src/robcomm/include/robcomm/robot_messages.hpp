/** \file robot_messages.hpp
 * \brief UDP messages for robot communication.
 *        Robco UDP protocol v4.5
 *
 * Created on: 24.03.2023
 * Author: Bernhard Vorhofer
 * Contributor: -
 */

#ifndef ROBCOMM_ROBOT_MESSAGES_H
#define ROBCOMM_ROBOT_MESSAGES_H

#include <robcomm/types.hpp>
#include <stdint.h>
#include <sys/types.h>

namespace robcomm {

    enum MSG_TYPE {
        MSG_TYPE_GET_UDP_PROTOCOL_VERSION = 0x01,
        MSG_TYPE_GET_STATUS = 0x02,
        MSG_TYPE_GET_LAST_OCCURRED_ERRORS = 0x03,
        MSG_TYPE_GET_DETECTED_MODULES = 0x05,
        MSG_TYPE_GET_LAST_REACHED_WAYPOINT = 0x0F,
        MSG_TYPE_SET_JOINT_ABS = 0x10,
        MSG_TYPE_GET_JOINT_ABS = 0x11,
        MSG_TYPE_SET_JOINT_OFFS = 0x12,
        MSG_TYPE_SET_POSE_ABS = 0x14,
        MSG_TYPE_GET_POSE_ABS = 0x15,
        MSG_TYPE_SET_POSE_OFFS = 0x16,
        MSG_TYPE_SET_CONSTANT_VELOCITY_TRAJECTORY = 0x1A,
        MSG_TYPE_SET_GLOBAL_VELOCITY = 0x20,
        MSG_TYPE_GET_GLOBAL_VELOCITY = 0x21,
        MSG_TYPE_SET_OUTPUT = 0x30,
        MSG_TYPE_GET_OUTPUT = 0x31,
        MSG_TYPE_GET_INPUT = 0x33,
        MSG_TYPE_SET_JOINT_LEDS = 0x34,
        MSG_TYPE_SET_G_COMPENSATION = 0x41,
        MSG_TYPE_SET_PAYLOAD_MASS = 0x70,
        MSG_TYPE_GET_PAYLOAD_MASS = 0x71,
        MSG_TYPE_SET_TCP_SHIFT = 0x72,
        MSG_TYPE_GET_TCP_SHIFT = 0x73,
        MSG_TYPE_GET_JOINT_TEMPERATURE_PAIRS = 0xA0,
        MSG_TYPE_GET_JOINT_TORSIONS = 0xA1,
        MSG_TYPE_GET_JOINT_OVERLOADS_PERCENT = 0xA2,
        MSG_TYPE_GET_JOINT_TORQUES = 0xA3,
        MSG_TYPE_SET_ROBOT_STATE = 0xE0,
        MSG_TYPE_SET_STOP = 0xEE
    };

    struct __attribute__((packed)) SET_MSG {
        uint8_t type;
        uint8_t seq;
        uint16_t payload_len;
        char payload[];
    };

    struct __attribute__((packed)) MSG_SET_JOINT_OFFS {
        uint8_t num_joints;
        int32_t joint_angles[];
    };

    struct __attribute__((packed)) MSG_SET_ROBOT_STATE {
        uint8_t robot_state;
    };

    struct __attribute__((packed)) MSG_SET_OUTPUT {
        uint8_t bank;
        uint32_t address;
        uint32_t value;
    };

    struct __attribute__((packed)) GET_MSG {
        uint8_t type;
        uint8_t seq;
        uint8_t last_rx_seq;
        int8_t  last_rx_result;
        uint16_t payload_len;
        char payload[];
    };

    struct __attribute__((packed)) MSG_GET_UDP_PROTOCOL_VERSION {
        uint8_t major;
        uint8_t minor;
    };

    struct __attribute__((packed)) MSG_GET_STATUS {
        uint8_t error_flags;
        uint8_t robot_state;
        uint8_t safety_state;
        char data[];
        // Variable-length data:
        //  - 1x MSG_GET_STATUS_MODULES struct
        //  - 1x MSG_GET_STATUS_ERRORS struct
    };

    struct __attribute__((packed)) MSG_GET_DETECTED_MODULES {
        uint8_t n_modules;
        char data[];
        // Variable-length data:
        //  - n_modules x MSG_GET_DETECTED_MODULES_MODULE
    };

    struct __attribute__((packed)) MSG_GET_DETECTED_MODULES_MODULE {
        uint32_t id;
        uint32_t serial;
    };

    // Bitmask for robot error flag in GET_STATUS message robot_state field
    const uint8_t ROBOT_ERROR_MASK = 0x80;

    // Bitmask for robot state in GET_STATUS message robot_state field
    const uint8_t ROBOT_STATE_DETAIL_MASK = 0x7F;

    // Bitmask for safety error flag in GET_STATUS message safety_state field
    const uint8_t SAFETY_ERROR_MASK = 0x80;

    // Bitmask for safe stop state in GET_STATUS message safety_state field
    const uint8_t SAFE_STOP_STATE_MASK = 0x60;

    // Bitmask for safety mod GET_STATUS message safety_state field
    const uint8_t SAFETY_MODE_MASK = 0x1F;

    struct __attribute__((packed)) MSG_GET_STATUS_MODULES {
        uint8_t n_modules;
        uint8_t module_states[];
    };

    // Bitmask for module type in GET_STATUS message module_state field
    const uint8_t MODULE_TYPE_MASK = 0xE0;

    // Bistmask for module error in GET_STATUS message module_state field
    const uint8_t MODULE_ERROR_MASK = 0x10;

    // Bistmask for ethercat not op in GET_STATUS message module_state field
    const uint8_t ETHERCAT_NOT_OP_MASK = 0x08;

    // Bistmask for drive not op in GET_STATUS message module_state field
    const uint8_t DRIVE_NOT_OP_MASK = 0x04;

    struct __attribute__((packed)) MSG_GET_STATUS_ERRORS {
        uint8_t n_errors;
        uint16_t errors[];
    };

    struct __attribute__((packed)) MSG_GET_JOINT_ABS {
        uint8_t n_joints;
        int32_t joint_values[];
    };

    int32_t hton_angle(double rad);
    double ntoh_angle(int32_t nrad_div_pi);

    int32_t hton_linear(double meters);
    double ntoh_linear(int32_t micrometers);

    int len_SET_MSG(SET_MSG* m);
    SET_MSG* new_UDP_MSG(uint8_t msg_type, uint8_t seq, size_t payload_size);

    int len_MSG_SET_JOINT_OFFS(MSG_SET_JOINT_OFFS* m);
    SET_MSG* new_MSG_SET_JOINT_OFFS(uint8_t seq, int num_joints);

    int len_MSG_GET_STATUS_MODULES(MSG_GET_STATUS_MODULES* m);
    int len_MSG_GET_STATUS_ERRORS(MSG_GET_STATUS_ERRORS* m);

    RobotState msg_get_robot_state(uint8_t robot_state);
    SafeStopState msg_get_safe_stop_state (uint8_t safety_state);
    SafetyMode msg_get_safety_mode(uint8_t safety_state);
    RobotStatus msg_get_robot_status(MSG_GET_STATUS* msg);
    
    ModuleType msg_get_module_type(uint8_t module_state);
    ModuleState msg_get_module_state(MSG_GET_STATUS_MODULES* modules, int i);

    void print_UDP_MSG(SET_MSG* m);
}

#endif // ROBCOMM_ROBOT_MESSAGES_H
