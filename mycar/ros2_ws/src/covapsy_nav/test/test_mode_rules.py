from covapsy_nav.mode_rules import DriveCommand
from covapsy_nav.mode_rules import select_mode_command


def test_idle_and_stopped_force_zero_output():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.2)
    pursuit = DriveCommand(linear_x=1.5, angular_z=-0.1)

    idle = select_mode_command("IDLE", reactive, pursuit, 0.5, min_front_dist=10.0)
    stopped = select_mode_command("STOPPED", reactive, pursuit, 0.5, min_front_dist=10.0)

    assert idle == DriveCommand()
    assert stopped == DriveCommand()


def test_mapping_caps_forward_speed():
    reactive = DriveCommand(linear_x=1.3, angular_z=0.15)
    cmd = select_mode_command(
        "MAPPING",
        reactive,
        DriveCommand(),
        mapping_speed_cap=0.5,
        min_front_dist=10.0,
    )
    assert cmd.linear_x == 0.5
    assert cmd.angular_z == reactive.angular_z


def test_racing_falls_back_to_reactive_when_pursuit_unavailable():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.25)
    pursuit = DriveCommand(linear_x=0.0, angular_z=-0.2)
    cmd = select_mode_command("RACING", reactive, pursuit, 0.5, min_front_dist=10.0)
    assert cmd == reactive


def test_front_obstacle_forces_stop_for_forward_motion():
    reactive = DriveCommand(linear_x=0.8, angular_z=0.1)
    cmd = select_mode_command(
        "REACTIVE",
        reactive,
        DriveCommand(),
        mapping_speed_cap=0.5,
        min_front_dist=0.1,
    )
    assert cmd == DriveCommand()

