from stingray_core_control.control.controllers import DepthController


def test_depth_controller_uses_sample_rate_instead_of_loop_derivative():
    controller = DepthController(
        K_p=1.0,
        K_1=1.0,
        K_2=0.0,
        K_i=0.0,
        I_min=-1.0,
        I_max=1.0,
        out_sat=1.0,
        ap_K=1.0,
        ap_T=0.0,
    )
    debug = {}
    controller.set_debug_hook(debug.update)

    controller.update(
        setpoint=0.30,
        measurement=0.20,
        measurement_rate=0.12,
        measurement_accel=0.0,
        dt=0.01,
        flag_setup_feedback_speed=False,
    )

    assert debug["measurement_rate"] == 0.12
