#include <iiwa_driver/iiwa.h>

// ROS Headers
#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>

// FRI Headers
#include <kuka/fri/ClientData.h>

#include <thread>

namespace iiwa_ros {
    Iiwa::Iiwa(ros::NodeHandle& nh)
    {
        init(nh);
    }

    Iiwa::~Iiwa()
    {
        // Disconnect from robot
        _disconnect_fri();

        // Delete FRI message data
        if (_fri_message_data)
            delete _fri_message_data;
    }

    void Iiwa::init(ros::NodeHandle& nh)
    {
        _nh = nh;
        _load_params(); // load parameters
        _init(); // initialize
        _controller_manager.reset(new controller_manager::ControllerManager(this, _nh));

        if (_init_fri())
            _initialized = true;
        else
            _initialized = false;
    }

    void Iiwa::run()
    {
        if (!_initialized) {
            ROS_ERROR_STREAM("Not connected to the robot. Cannot run!");
            return;
        }

        std::thread t1(&Iiwa::_ctrl_loop, this);
        t1.join();
    }

    bool Iiwa::initialized()
    {
        return _initialized;
    }

    void Iiwa::_init()
    {
        // Get joint names
        _num_joints = _joint_names.size();

        // Resize vectors
        _joint_position.resize(_num_joints);
        _joint_velocity.resize(_num_joints);
        _joint_effort.resize(_num_joints);
        _joint_position_command.resize(_num_joints);
        _joint_velocity_command.resize(_num_joints);
        _joint_effort_command.resize(_num_joints);

        // Initialize Controller
        for (int i = 0; i < _num_joints; ++i) {
            _joint_position[i] = _joint_velocity[i] = _joint_effort[i] = 0.;
            // Create joint state interface
            hardware_interface::JointStateHandle jointStateHandle(_joint_names[i], &_joint_position[i], &_joint_velocity[i], &_joint_effort[i]);
            _joint_state_interface.registerHandle(jointStateHandle);

            // Create position joint interface
            hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &_joint_position_command[i]);
            // joint_limits_interface::JointLimits limits;
            // joint_limits_interface::SoftJointLimits softLimits;
            // // getJointLimits(joint.name, nh_, limits);
            // joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            // _position_joint_limits_interface.registerHandle(jointLimitsHandle);
            _position_joint_interface.registerHandle(jointPositionHandle);

            // Create effort joint interface
            hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &_joint_effort_command[i]);
            _effort_joint_interface.registerHandle(jointEffortHandle);
        }

        registerInterface(&_joint_state_interface);
        registerInterface(&_position_joint_interface);
        registerInterface(&_effort_joint_interface);
        // registerInterface(&_position_joint_limits_interface);
    }

    void Iiwa::_ctrl_loop()
    {
        static ros::Rate rate(_control_freq);
        while (ros::ok()) {
            ros::Time time = ros::Time::now();

            // TO-DO: Get real elapsed time?
            auto elapsed_time = ros::Duration(1. / _control_freq);

            _read(elapsed_time);
            _controller_manager->update(ros::Time::now(), elapsed_time);
            _write(elapsed_time);

            rate.sleep();
        }
    }

    void Iiwa::_load_params()
    {
        ros::NodeHandle n_p("~");

        n_p.param("fri/port", _port, 30200); // Default port is 30200
        n_p.param<std::string>("fri/robot_ip", _remote_host, "192.170.10.2"); // Default robot ip is 192.170.10.2

        n_p.param("hardware_interface/control_freq", _control_freq, 50.);
        n_p.getParam("hardware_interface/joints", _joint_names);
    }

    void Iiwa::_read(ros::Duration elapsed_time)
    {
        // Read data from robot (via FRI)
        kuka::fri::ESessionState fri_state;
        _read_fri(fri_state);

        switch (fri_state) {
        case kuka::fri::MONITORING_WAIT:
        case kuka::fri::MONITORING_READY:
        case kuka::fri::COMMANDING_WAIT:
        case kuka::fri::COMMANDING_ACTIVE:
            _idle = false;
            break;
        case kuka::fri::IDLE: // if idle, do nothing
        default:
            _idle = true;
            return;
        }

        // Update ROS structures
        _joint_position_prev = _joint_position;

        for (int i = 0; i < _num_joints; i++) {
            _joint_position[i] = _robotState.getMeasuredJointPosition()[i];
            _joint_velocity[i] = filters::exponentialSmoothing((_joint_position[i] - _joint_position_prev[i]) / elapsed_time.toSec(), _joint_velocity[i], 0.2);
        }
    }

    void Iiwa::_write(ros::Duration elapsed_time)
    {
        if (_idle) // if idle, do nothing
            return;
        // reset commmand message
        _fri_message_data->resetCommandMessage();

        if (_robotState.getClientCommandMode() == kuka::fri::TORQUE) {
            _robotCommand.setTorque(_joint_effort_command.data());
            _robotCommand.setJointPosition(_joint_position.data());
        }
        else if (_robotState.getClientCommandMode() == kuka::fri::POSITION)
            _robotCommand.setJointPosition(_joint_position_command.data());
        // else ERROR

        _write_fri();
    }

    bool Iiwa::_init_fri()
    {
        _idle = true;

        // Create message/client data
        _fri_message_data = new kuka::fri::ClientData(_robotState.NUMBER_OF_JOINTS);

        // link monitoring and command message to wrappers
        _robotState.set_message(&_fri_message_data->monitoringMsg);
        _robotCommand.set_message(&_fri_message_data->commandMsg);

        // set specific message IDs
        _fri_message_data->expectedMonitorMsgID = _robotState.monitoring_message_id();
        _fri_message_data->commandMsg.header.messageIdentifier = _robotCommand.command_message_id();

        if (!_connect_fri())
            return false;

        return true;
    }

    bool Iiwa::_connect_fri()
    {
        if (_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Warning: client application already connected!\n");
            return true;
        }

        return _fri_connection.open(_port, _remote_host.c_str());
    }

    void Iiwa::_disconnect_fri()
    {
        if (_fri_connection.isOpen())
            _fri_connection.close();
    }

    bool Iiwa::_read_fri(kuka::fri::ESessionState& current_state)
    {
        if (!_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Error: client application is not connected!\n");
            return false;
        }

        // **************************************************************************
        // Receive and decode new monitoring message
        // **************************************************************************
        _message_size = _fri_connection.receive(_fri_message_data->receiveBuffer, kuka::fri::FRI_MONITOR_MSG_MAX_SIZE);

        if (_message_size <= 0) { // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
            // TO-DO: Use ROS output
            // printf("Error: failed while trying to receive monitoring message!\n");
            return false;
        }

        if (!_fri_message_data->decoder.decode(_fri_message_data->receiveBuffer, _message_size)) {
            return false;
        }

        // check message type (so that our wrappers match)
        if (_fri_message_data->expectedMonitorMsgID != _fri_message_data->monitoringMsg.header.messageIdentifier) {
            // TO-DO: Use ROS output
            // printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            //     (int)_fri_message_data->monitoringMsg.header.messageIdentifier,
            //     (int)_fri_message_data->expectedMonitorMsgID);
            return false;
        }

        current_state = (kuka::fri::ESessionState)_fri_message_data->monitoringMsg.connectionInfo.sessionState;

        if (_fri_message_data->lastState != current_state) {
            _on_fri_state_change(_fri_message_data->lastState, current_state);
            _fri_message_data->lastState = current_state;
        }

        return true;
    }

    bool Iiwa::_write_fri()
    {
        // **************************************************************************
        // Encode and send command message
        // **************************************************************************

        _fri_message_data->lastSendCounter++;
        // check if its time to send an answer
        if (_fri_message_data->lastSendCounter >= _fri_message_data->monitoringMsg.connectionInfo.receiveMultiplier) {
            _fri_message_data->lastSendCounter = 0;

            // set sequence counters
            _fri_message_data->commandMsg.header.sequenceCounter = _fri_message_data->sequenceCounter++;
            _fri_message_data->commandMsg.header.reflectedSequenceCounter = _fri_message_data->monitoringMsg.header.sequenceCounter;

            if (!_fri_message_data->encoder.encode(_fri_message_data->sendBuffer, _message_size)) {
                return false;
            }

            if (!_fri_connection.send(_fri_message_data->sendBuffer, _message_size)) {
                // TO-DO: Use ROS output
                // printf("Error: failed while trying to send command message!\n");
                return false;
            }
        }

        return true;
    }
} // namespace iiwa_ros