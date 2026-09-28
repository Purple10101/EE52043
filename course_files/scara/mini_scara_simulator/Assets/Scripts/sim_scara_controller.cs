using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class sim_scara_controller : MonoBehaviour
{
    public sim_servo_controller servo_controller;

    public Text axis0_cpos_text;
    public Text axis1_cpos_text;
    public Text axis2_cpos_text;
    public Text axis3_cpos_text;

    public Text x_text;
    public Text y_text;
    public Text z_text;
    public Text z_angle_text;

    public float link_1 = 0.125f;
    public float link_2 = 0.100f;
    public float z_extended = 0.015f;
    public float z_retracted = 0.065f;
    public float settling_time = 1.0f;

    float Axis0Max = 90f;
    float Axis0Min = -90f;
    float Axis1Max = 120f;
    float Axis1Min = 0f;
    float Axis2Max = 180f;
    float Axis2Min = -180f;
    float Axis3Max = 0.1f;
    float Axis3Min = 0.01f;

    public GameObject working_area;

    public struct scara_positions_t
    {
        public float theta1, theta2, theta3, s4;
    }

    public struct scara_coord_t
    {
        public float x, y, z, z_angle;
    }

    void Start()
    {

    }

    void Update()
    {
        update_axis_text();
    }

    void update_axis_text()
    {
        axis0_cpos_text.text = $"axis 0 (deg) : {servo_controller.axis0.cpos:0.00000}";
        axis1_cpos_text.text = $"axis 1 (deg) : {servo_controller.axis1.cpos:0.00000}";
        axis2_cpos_text.text = $"axis 2 (deg) : {servo_controller.axis2.cpos:0.00000}";
        axis3_cpos_text.text = $"axis 3 (m) : {servo_controller.axis3.cpos:0.00000}";



        scara_coord_t coords = calc_forward_kinematics(servo_controller.axis0.cpos,
                                                       servo_controller.axis1.cpos,
                                                       servo_controller.axis2.cpos,
                                                       servo_controller.axis3.cpos);
        x_text.text = $"x (m) : {coords.x:0.00000}";
        y_text.text = $"y (m) : {coords.y:0.00000}";
        z_text.text = $"z (m) : {coords.z:0.00000}";
        z_angle_text.text = $"z angle (deg) : {coords.z_angle:0.00000}";
    }

    public void scara_initialise(float _link_1, float _link_2, float z_min, float z_max, float _settling_time)
    {
        link_1 = _link_1;
        link_2 = _link_2;
        z_extended = z_min;
        z_retracted = z_max;
        settling_time = _settling_time;
    }

    public void scara_move_j(int axis, float angle, float T)
    {
        servo_controller.MoveJ(axis, angle, T);
    }

    public void scara_move_js(float angle0, float angle1, float angle2, float angle3, float T)
    {
        servo_controller.MoveJs(angle0, angle1, angle2, angle3, T);
    }

    public void scara_move_coord(float x, float y, float z, float angleZ, float T)
    {
        scara_positions_t new_pos = calc_inverse_kinematics(x, y, angleZ, z);
        if (new_pos.theta1 > Axis0Max || new_pos.theta1 < Axis0Min)
        {
            Debug.LogError("Axis 0 is out of bounds after SCARA_MOVE_COORD command");
        }
        if (new_pos.theta2 > Axis1Max || new_pos.theta2 < Axis1Min)
        {
            Debug.LogError("Axis 1 is out of bounds after SCARA_MOVE_COORD command");
        }
        if (new_pos.theta3 > Axis2Max || new_pos.theta3 < Axis2Min)
        {
            Debug.LogError("Axis 2 is out of bounds after SCARA_MOVE_COORD command");
        }
        if (new_pos.s4 > Axis3Max || new_pos.s4 < Axis3Min)
        {
            Debug.LogError("Axis 3 is out of bounds after SCARA_MOVE_COORD command");
        }
        servo_controller.MoveJs(new_pos.theta1, new_pos.theta2, new_pos.theta3, new_pos.s4, T);
    }

    public scara_positions_t calc_inverse_kinematics(float x, float y, float angleZ, float z)
    {
        scara_positions_t angles;

        float r = Mathf.Sqrt((x * x) + (y * y));

        float phi1 = Mathf.Acos(((link_2 * link_2) - (r * r) - (link_1 * link_1)) / (-2f * r * link_1));
        float phi2 = Mathf.Atan2(x, y);
        float phi3 = Mathf.Acos(((r * r) - (link_1 * link_1) - (link_2 * link_2)) / (-2f * link_1 * link_2));

        angles.theta1 = (phi2 - phi1) * Mathf.Rad2Deg;
        angles.theta2 = (Mathf.PI - phi3) * Mathf.Rad2Deg;
        angles.theta3 = angleZ - (angles.theta1 + angles.theta2);
        angles.s4 = Mathf.Clamp(z, z_extended, z_retracted);

        return angles;
    }

    public scara_coord_t calc_forward_kinematics(float theta1, float theta2, float theta3, float s4)
    {
        scara_coord_t coords;

        float l1 = link_1;
        float l2 = link_2;

        // Convert angles from degrees to radians
        float t1 = theta1 * (Mathf.PI / 180.0f);
        float t2 = theta2 * (Mathf.PI / 180.0f);

        // Compute X and Y position
        coords.x = l1 * Mathf.Sin(t1) + l2 * Mathf.Sin(t1 + t2);
        coords.y = l1 * Mathf.Cos(t1) + l2 * Mathf.Cos(t1 + t2);

        // Compute total rotation angle around Z
        coords.z_angle = theta1 + theta2 + theta3;

        coords.z = s4;

        return coords;
    }

    public void toggle_working_area()
    {
        if (working_area.activeSelf == true)
        {
            working_area.SetActive(false);
        }
        else
        {
            working_area.SetActive(true);
        }
    }
}
