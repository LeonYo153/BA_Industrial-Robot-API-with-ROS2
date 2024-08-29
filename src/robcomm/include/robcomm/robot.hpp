/** \file robot.hpp
 * \brief Robot class, handles communication with Robco robot.
 *
 * Created on: 24.03.2023
 * Author: Bernhard Vorhofer
 * Contributor: -
 * 
 * Copyright (C) 2023 RobCo GmbH - All Rights Reserved
 * 
 */

#ifndef ROBCOMM_ROBOT_H
#define ROBCOMM_ROBOT_H

#include <robcomm/types.hpp>
#include <robcomm/robot_messages.hpp>
#include <string>
#include <stdint.h>
#include <netinet/in.h>
#include <vector>
#include <list>

// Major protocol version this implementation is compatible with
#define COMPATIBLE_MAJOR_VERSION 4

namespace robcomm {
    class Robot {
        public:
            Robot();
            ~Robot();

            /**
             * @brief Initializes network sockets for robot connection.
             */
            void connect(std::string host, uint16_t rx_port_local, uint16_t tx_port_remote);

            /**
             * @brief Processes incoming messages from robot, to be called cyclically.
             */
            void receive();

            /**
             * @brief Request the robot to change into given state.
             * 
             * @param cmd State command
             */
            void set_state(RobotStateCommand cmd);

            /**
             * @brief Send jog command with given joint angles.
             * 
             * @param dqs Joint angle vector (values in rad/s), must have
             *        correct size (number of joints).
             */
            void jog_joints(std::vector<double> &dqs);

            void set_output(uint8_t bank, uint32_t address, uint32_t value);

            bool is_initialized();
            RobotStatus get_status();
            int get_module_count();
            uint32_t get_module_type_id(int i);
            ModuleState get_module_state(int i);
            int get_error_count();
            uint16_t get_error_code(int i);

            int get_joint_count();
            const std::vector<double>& getJointAngles() const;

        private:
            bool q_valid; // Set to true after first GET_JOINT_ABS message received
            bool status_valid; // Set to true after first GET_STATUS message received
            bool modules_valid; // Set to true after first GET_DETECTED_MODULES message received

            std::string host;
            uint16_t rx_port_local;
            uint16_t tx_port_remote;
            int sockfd_rx;
            int sockfd_tx;
            sockaddr_in robot_addr;
            sockaddr_in local_addr;
            char* recv_buffer;

            uint8_t robot_protocol_version_major;
            uint8_t robot_protocol_version_minor;
            uint8_t seq_counter;

            RobotStatus robot_status;
            std::vector<uint32_t> module_ids;
            std::vector<ModuleState> module_states;
            std::vector<uint16_t> error_codes;

            std::vector<double> q;

            SET_MSG* new_message(uint8_t msg_type, size_t payload_size);
            void send_message(SET_MSG* msg);

            void handle_get_message(GET_MSG* msg);
            void handle_get_udp_protocol_version(MSG_GET_UDP_PROTOCOL_VERSION* msg);
            void handle_get_status(MSG_GET_STATUS* msg);
            void handle_get_joint_abs(MSG_GET_JOINT_ABS* msg);
            void handle_get_detected_modules(MSG_GET_DETECTED_MODULES* msg);
    };
};

#endif // ROBCOMM_ROBOT_H
