/**
 * @file hwt905_node.cpp
 * @author ponomarevda96@gmail.com
 */
void InclinometerDriverRos::process_mock_serial() {
    int time_sec = ros::Time::now().toSec();
    const uint32_t PERIOD_DURATION_SEC = 36;

    // horizontal
    if (time_sec % PERIOD_DURATION_SEC < 1) {
        _quaternion.q_0 = 0.815796;
        _quaternion.q_1 = -0.0127563;
        _quaternion.q_2 = -0.002563480;
        _quaternion.q_3 = -0.578125;
    } else if (time_sec % PERIOD_DURATION_SEC < 2) {
        _quaternion.q_0 = 0.816467;
        _quaternion.q_1 = -0.0139771;
        _quaternion.q_2 = -0.000762939;
        _quaternion.q_3 = -0.577148;
    } else if (time_sec % PERIOD_DURATION_SEC < 3) {
        _quaternion.q_0 = 0.817841;
        _quaternion.q_1 = -0.0142212;
        _quaternion.q_2 = 0.001831050;
        _quaternion.q_3 = -0.575165;
    }

    // get_quaternion_roll_rotation_head_down
    else if (time_sec % PERIOD_DURATION_SEC < 6) {
        _quaternion.q_0 = 0.817749;
        _quaternion.q_1 = -0.269257;
        _quaternion.q_2 = 0.158142;
        _quaternion.q_3 = -0.483398;
    }

    // horizontal
    else if (time_sec % PERIOD_DURATION_SEC < 7) {
        _quaternion.q_0 = 0.815796;
        _quaternion.q_1 = -0.0127563;
        _quaternion.q_2 = -0.002563480;
        _quaternion.q_3 = -0.578125;
    } else if (time_sec % PERIOD_DURATION_SEC < 8) {
        _quaternion.q_0 = 0.816467;
        _quaternion.q_1 = -0.0139771;
        _quaternion.q_2 = -0.000762939;
        _quaternion.q_3 = -0.577148;
    } else if (time_sec % PERIOD_DURATION_SEC < 9) {
        _quaternion.q_0 = 0.817841;
        _quaternion.q_1 = -0.0142212;
        _quaternion.q_2 = 0.001831050;
        _quaternion.q_3 = -0.575165;
    }

    // get_quaternion_roll_rotation_head_up
    else if (time_sec % PERIOD_DURATION_SEC < 12) {
        _quaternion.q_0 = 0.777313;
        _quaternion.q_1 = 0.286682;
        _quaternion.q_2 = -0.194305;
        _quaternion.q_3 = -0.525085;
    }

    // horizontal
    else if (time_sec % PERIOD_DURATION_SEC < 13) {
        _quaternion.q_0 = 0.815796;
        _quaternion.q_1 = -0.0127563;
        _quaternion.q_2 = -0.002563480;
        _quaternion.q_3 = -0.578125;
    } else if (time_sec % PERIOD_DURATION_SEC < 14) {
        _quaternion.q_0 = 0.816467;
        _quaternion.q_1 = -0.0139771;
        _quaternion.q_2 = -0.000762939;
        _quaternion.q_3 = -0.577148;
    } else if (time_sec % PERIOD_DURATION_SEC < 15) {
        _quaternion.q_0 = 0.817841;
        _quaternion.q_1 = -0.0142212;
        _quaternion.q_2 = 0.001831050;
        _quaternion.q_3 = -0.575165;
    }

    // get_quaternion_pitch_rotation_right
    else if (time_sec % PERIOD_DURATION_SEC < 16) {
        _quaternion.q_0 = 0.805145;
        _quaternion.q_1 = 0.21698;
        _quaternion.q_2 = 0.25412;
        _quaternion.q_3 = -0.489868;
    } else if (time_sec % PERIOD_DURATION_SEC < 17) {
        _quaternion.q_0 = 0.800323;
        _quaternion.q_1 = 0.231812;
        _quaternion.q_2 = 0.286133;
        _quaternion.q_3 = -0.473022;
    } else if (time_sec % PERIOD_DURATION_SEC < 18) {
        _quaternion.q_0 = 0.793793;
        _quaternion.q_1 = 0.233124;
        _quaternion.q_2 = 0.312195;
        _quaternion.q_3 = -0.466858;
    }

    // horizontal
    else if (time_sec % PERIOD_DURATION_SEC < 19) {
        _quaternion.q_0 = 0.815796;
        _quaternion.q_1 = -0.0127563;
        _quaternion.q_2 = -0.002563480;
        _quaternion.q_3 = -0.578125;
    } else if (time_sec % PERIOD_DURATION_SEC < 20) {
        _quaternion.q_0 = 0.816467;
        _quaternion.q_1 = -0.0139771;
        _quaternion.q_2 = -0.000762939;
        _quaternion.q_3 = -0.577148;
    } else if (time_sec % PERIOD_DURATION_SEC < 21) {
        _quaternion.q_0 = 0.817841;
        _quaternion.q_1 = -0.0142212;
        _quaternion.q_2 = 0.001831050;
        _quaternion.q_3 = -0.575165;
    }
}
