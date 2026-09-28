using UnityEngine;

public class sim_servo : MonoBehaviour
{
    public float cpos = 0f;
    public bool linear = false;

    void Start()
    {

    }

    void Update()
    {
        servo_set_position();
    }

    void servo_set_position()
    {
        if (!linear)
        {
            transform.localRotation = Quaternion.Euler(0f, 0f, cpos);
        }
        else
        {
            Vector3 new_pos = transform.localPosition;
            new_pos.z = cpos;
            transform.localPosition = new_pos;
        }
    }
}
