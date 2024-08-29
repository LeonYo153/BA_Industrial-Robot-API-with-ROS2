#include <robcomm/robcomm.hpp>
#include <robcomm/robot_messages.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdexcept>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <cstdlib>

#define ROBCOMM_RECV_BUFFER_SIZE 1024 // receive buffer size in bytes

namespace robcomm
{

    Robot::Robot()
    {
        this->recv_buffer = new char[ROBCOMM_RECV_BUFFER_SIZE];

        this->robot_protocol_version_major = 0;
        this->robot_protocol_version_minor = 0;
    }

    void Robot::connect(std::string host, uint16_t rx_port_local, uint16_t tx_port_remote)
    {
        this->host = host;
        this->rx_port_local = rx_port_local;
        this->tx_port_remote = tx_port_remote;
        this->seq_counter = 0;

        if ((sockfd_rx = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
            throw std::runtime_error("RX Socket creation failed.");

        fcntl(sockfd_rx, F_SETFL, O_NONBLOCK); // set socket nonblocking

        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(rx_port_local);

        if (bind(sockfd_rx, (const sockaddr *)&local_addr, sizeof(local_addr)) < 0)
            throw std::runtime_error("RX Socket bind() failed.");

        if ((sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
            throw std::runtime_error("TX socket creation failed");

        memset(&robot_addr, 0, sizeof(robot_addr));

        robot_addr.sin_family = AF_INET;
        robot_addr.sin_port = htons(tx_port_remote);

        inet_aton(host.c_str(), &robot_addr.sin_addr);
    }

    bool Robot::is_initialized() {
        return q_valid && status_valid && modules_valid;
    }

    void Robot::receive()
    {
        sockaddr_in from_addr;

        socklen_t len = (socklen_t)sizeof(from_addr);
        int n;
        while ((n = recvfrom(sockfd_rx, recv_buffer, ROBCOMM_RECV_BUFFER_SIZE, 0, (sockaddr *)&from_addr, &len)) > 0) {
            if (n < sizeof(GET_MSG))
                throw std::runtime_error("Received packet is too short");

            printf("Received a message of size %d\n", n);

            // 打印接收的缓冲区内容（以十六进制显示）
            for (int i = 0; i < n; ++i) {
            printf("%02x ", (unsigned char)recv_buffer[i]);
            }
            printf("\n");
            GET_MSG *msg = (GET_MSG *)recv_buffer;
            handle_get_message(msg);
        }
    }

    void Robot::handle_get_message(GET_MSG *msg)
    {
        std::stringstream exception_ss;

        uint16_t payload_len = ntohs(msg->payload_len);

        switch (msg->type)
        {
        case MSG_TYPE_GET_UDP_PROTOCOL_VERSION:
            if (payload_len != 2) {
                exception_ss << "Invalid payload length " << payload_len << " for GET_UDP_PROTOCOL_VERSION messge";
                throw std::runtime_error(exception_ss.str());
            }
            handle_get_udp_protocol_version((MSG_GET_UDP_PROTOCOL_VERSION*)msg->payload);
            break;
        case MSG_TYPE_GET_STATUS:
            if (payload_len < 5) {
                exception_ss << "Invalid payload length " << payload_len << " for MSG_TYPE_GET_STATUS message";
                throw std::runtime_error(exception_ss.str());
            }
            handle_get_status((MSG_GET_STATUS*)msg->payload);
            status_valid = true;
            break;
        case MSG_TYPE_GET_JOINT_ABS:
            if (payload_len < 1) {
                exception_ss << "Invalid payload length " << payload_len << " for MSG_TYPE_GET_JOINT_ABS message";
                throw std::runtime_error(exception_ss.str());
            }
            handle_get_joint_abs((MSG_GET_JOINT_ABS*)msg->payload);
            q_valid = true;
            break;
        case MSG_TYPE_GET_DETECTED_MODULES:
            if (payload_len < 1) {
                exception_ss << "Invalid payload length " << payload_len << " for MSG_TYPE_GET_DETECTED_MODULES message";
                throw std::runtime_error(exception_ss.str());
            }
            handle_get_detected_modules((MSG_GET_DETECTED_MODULES*)msg->payload);
            modules_valid = true;
            break;
        case MSG_TYPE_GET_LAST_OCCURRED_ERRORS:
        case MSG_TYPE_GET_LAST_REACHED_WAYPOINT:
        case MSG_TYPE_SET_JOINT_ABS:
        case MSG_TYPE_SET_JOINT_OFFS:
        case MSG_TYPE_SET_POSE_ABS:
        case MSG_TYPE_GET_POSE_ABS:
        case MSG_TYPE_SET_POSE_OFFS:
        case MSG_TYPE_SET_CONSTANT_VELOCITY_TRAJECTORY:
        case MSG_TYPE_SET_GLOBAL_VELOCITY:
        case MSG_TYPE_GET_GLOBAL_VELOCITY:
        case MSG_TYPE_SET_OUTPUT:
        case MSG_TYPE_GET_OUTPUT:
        case MSG_TYPE_GET_INPUT:
        case MSG_TYPE_SET_JOINT_LEDS:
        case MSG_TYPE_SET_G_COMPENSATION:
        case MSG_TYPE_SET_PAYLOAD_MASS:
        case MSG_TYPE_GET_PAYLOAD_MASS:
        case MSG_TYPE_SET_TCP_SHIFT:
        case MSG_TYPE_GET_TCP_SHIFT:
        case MSG_TYPE_GET_JOINT_TEMPERATURE_PAIRS:
        case MSG_TYPE_GET_JOINT_TORSIONS:
        case MSG_TYPE_GET_JOINT_OVERLOADS_PERCENT:
        case MSG_TYPE_GET_JOINT_TORQUES:
        case MSG_TYPE_SET_ROBOT_STATE:
        case MSG_TYPE_SET_STOP:
            break;
        default:
            exception_ss << "Invalid message type '" << unsigned(msg->type) << "'";
            throw std::runtime_error(exception_ss.str());
            break;
        }
    }

    void Robot::handle_get_udp_protocol_version(MSG_GET_UDP_PROTOCOL_VERSION* msg) {
        std::stringstream exception_ss;

        if (robot_protocol_version_major == 0 && robot_protocol_version_minor == 0) {

            if (msg->major != COMPATIBLE_MAJOR_VERSION) {
                exception_ss << "Controller reports incompatible protocol version " <<
                    msg->major << "." << msg->minor;
                throw std::runtime_error(exception_ss.str());
            }

            robot_protocol_version_major = msg->major;
            robot_protocol_version_minor = msg->minor;
        } else if (robot_protocol_version_major != msg->major ||
                robot_protocol_version_minor != msg->minor) {
            exception_ss << "Controller changed protocol version from " <<
                robot_protocol_version_major << "." << robot_protocol_version_minor <<
                " to " << msg->major << "." << msg->minor << "during operation!";
            throw std::runtime_error(exception_ss.str());
        }
    }

    void Robot::handle_get_status(MSG_GET_STATUS* msg) {
        robot_status = msg_get_robot_status(msg);

        MSG_GET_STATUS_MODULES* modules = (MSG_GET_STATUS_MODULES*)msg->data;

        // Resize internal module state vector, if necessary
        module_states.resize(modules->n_modules);

        for (int i = 0; i < modules->n_modules; i ++) {
            ModuleState ms = msg_get_module_state(modules, i);
            module_states[i] = ms;
        }

        MSG_GET_STATUS_ERRORS* errors = (MSG_GET_STATUS_ERRORS*)(msg->data + len_MSG_GET_STATUS_MODULES(modules));

        // Resize internal error code vector, if necessary
        error_codes.resize(errors->n_errors);

        for (int i = 0; i < errors->n_errors; i ++) {
            error_codes[i] = errors->errors[i];
        }
    }

    void Robot::handle_get_joint_abs(MSG_GET_JOINT_ABS* msg) {
        std::stringstream exception_ss;

        // Resize joint angle vector if necessary
        q.resize(msg->n_joints);

        for (int i = 0; i < msg->n_joints; i ++) {
            q[i] = ntoh_angle(msg->joint_values[i]);
        }
    }

    void Robot::handle_get_detected_modules(MSG_GET_DETECTED_MODULES* msg) {
        module_ids.resize(msg->n_modules);

        int actual_n_modules = 0;
        for (int i = 0; i < msg->n_modules; i ++) {
            MSG_GET_DETECTED_MODULES_MODULE* module =
                (MSG_GET_DETECTED_MODULES_MODULE*)(&msg->data[i * sizeof(MSG_GET_DETECTED_MODULES_MODULE)]);

	    uint32_t module_id = ntohl(module->id);

            // Ignore clamps
            if (module_id >= 8000 && module_id <= 8999)
                continue;

            module_ids[actual_n_modules] = module_id;
            actual_n_modules ++;
        }

        // Resize again, as we ignored some modules.
        module_ids.resize(actual_n_modules);
    }

    SET_MSG* Robot::new_message(uint8_t msg_type, size_t payload_size) {
        return new_UDP_MSG(msg_type, seq_counter++, payload_size);
    }

    void Robot::send_message(SET_MSG* msg) {
        ssize_t n = sendto(sockfd_tx, msg, len_SET_MSG(msg), 0, (struct sockaddr*)&robot_addr,
            sizeof(robot_addr));

        if (n < 0) {
            throw std::runtime_error("Error while sending message");
        }
    }

    void Robot::set_state(RobotStateCommand cmd) {
        SET_MSG* msg = new_message(MSG_TYPE_SET_ROBOT_STATE, sizeof(MSG_SET_ROBOT_STATE));
        MSG_SET_ROBOT_STATE* payload = (MSG_SET_ROBOT_STATE*)msg->payload;

        payload->robot_state = (uint8_t)cmd;

        send_message(msg);

        free(msg);
    }

    void Robot::jog_joints(std::vector<double> &dqs) {
        if (dqs.size() != q.size())
            std::runtime_error("jog_joints command has size different from internal joint angle vector");

        // TODO: centralize handling of sequence number
        SET_MSG* msg = new_MSG_SET_JOINT_OFFS(seq_counter++, dqs.size());
        MSG_SET_JOINT_OFFS* payload = (MSG_SET_JOINT_OFFS*)msg->payload;

        for (int i = 0; i < dqs.size(); i++) {
            // / 100 because the controller will execute this dq within 10ms,
            // so we need to command 1/100th of the velocity value (rad/s) at
            // a time.
            payload->joint_angles[i] = hton_angle(dqs[i] / 100.0);
        }

        send_message(msg);

        free(msg);
    }

    void Robot::set_output(uint8_t bank, uint32_t address, uint32_t value) {
        SET_MSG* msg = new_message(MSG_TYPE_SET_OUTPUT, sizeof(MSG_SET_OUTPUT));
        MSG_SET_OUTPUT* payload = (MSG_SET_OUTPUT*)msg->payload;

        payload->bank = bank;
        payload->address = address;
        payload->value = value;

        send_message(msg);

        free(msg);
    }

    RobotStatus Robot::get_status() {
        return robot_status;
    }

    int Robot::get_module_count() {
        return module_ids.size();
    }

    uint32_t Robot::get_module_type_id(int i) {
        return module_ids[i];
    }

    ModuleState Robot::get_module_state(int i) {
        return module_states[i];
    }
    
    int Robot::get_error_count() {
        return error_codes.size();
    }

    uint16_t Robot::get_error_code(int i) {
        return error_codes[i];
    }

    int Robot::get_joint_count() {
        return q.size();
    }

    const std::vector<double>& Robot::getJointAngles() const {
        return q;
    }

    Robot::~Robot()
    {
        close(sockfd_rx);
        close(sockfd_tx);
        delete[] recv_buffer;
    }
}
