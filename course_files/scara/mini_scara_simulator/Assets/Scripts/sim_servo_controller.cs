using System.Collections;
using UnityEngine;

public class sim_servo_controller : MonoBehaviour
{
    public sim_servo axis0;
    public sim_servo axis1;
    public sim_servo axis2;
    public sim_servo axis3;

    private const float SAMPLING_TIME = 10f; // in milliseconds

    public void MoveJ(int axisIndex, float targetAngle, float T)
    {
        sim_servo axis = GetAxis(axisIndex);
        float cpos = axis.cpos;
        float h = targetAngle - cpos;

        float alpha = 0.3f;
        float beta = 0.3f;

        CalcDoubleSParameters(T, h, alpha, beta, out float T_a, out float T_j, out float v_max, out float a_max, out float j_max);

        int samples = Mathf.RoundToInt(T * 1000f / SAMPLING_TIME);
        StartCoroutine(MoveSingleJointCoroutine(axis, cpos, T, h, T_a, T_j, v_max, a_max, j_max, samples));
    }

    public void MoveJs(float angle0, float angle1, float angle2, float angle3, float T)
    {
        sim_servo[] axes = { axis0, axis1, axis2, axis3 };
        float[] tpos = { angle0, angle1, angle2, angle3 };
        float[] cpos = new float[4];
        float[] h = new float[4];
        float[] T_a = new float[4];
        float[] T_j = new float[4];
        float[] v_max = new float[4];
        float[] a_max = new float[4];
        float[] j_max = new float[4];

        float alpha = 0.3f;
        float beta = 0.3f;

        for (int i = 0; i < 4; i++)
        {
            cpos[i] = axes[i].cpos;
            h[i] = tpos[i] - cpos[i];
            CalcDoubleSParameters(T, h[i], alpha, beta, out T_a[i], out T_j[i], out v_max[i], out a_max[i], out j_max[i]);
        }

        int samples = Mathf.RoundToInt(T * 1000f / SAMPLING_TIME);
        StartCoroutine(MoveMultipleJointsCoroutine(axes, cpos, T, h, T_a, T_j, v_max, a_max, j_max, samples));
    }

    private IEnumerator MoveSingleJointCoroutine(sim_servo servo, float cpos, float T, float h, float T_a, float T_j,
                                                 float v_max, float a_max, float j_max, int samples)
    {
        for (int i = 0; i < samples; i++)
        {
            float t = i * SAMPLING_TIME / 1000f;
            float dpos = cpos + CalcDoubleSValue(T, h, T_a, T_j, v_max, a_max, j_max, t);
            servo.cpos = dpos;
            yield return new WaitForSeconds(SAMPLING_TIME / 1000f);
        }
    }

    private IEnumerator MoveMultipleJointsCoroutine(sim_servo[] servos, float[] cpos, float T, float[] h, float[] T_a,
                                                    float[] T_j, float[] v_max, float[] a_max, float[] j_max, int samples)
    {
        for (int i = 0; i < samples; i++)
        {
            float t = i * SAMPLING_TIME / 1000f;
            for (int j = 0; j < 4; j++)
            {
                float dpos = cpos[j] + CalcDoubleSValue(T, h[j], T_a[j], T_j[j], v_max[j], a_max[j], j_max[j], t);
                servos[j].cpos = dpos;
            }
            yield return new WaitForSeconds(SAMPLING_TIME / 1000f);
        }
    }

    private sim_servo GetAxis(int index)
    {
        switch (index)
        {
            case 1: return axis1;
            case 2: return axis2;
            case 3: return axis3;
            default: return axis0;
        }
    }

    public void CalcDoubleSParameters(float T, float h, float alpha, float beta,
                                   out float T_a, out float T_j,
                                   out float v_max, out float a_max, out float j_max)
    {
        // accel / decel duration    0 < alpha < 0.5
        T_a = alpha * T;

        // jerk duration             0 < beta < 0.5
        T_j = beta * T_a;

        // calculate maximum velocity
        v_max = h / ((1f - alpha) * T);

        // calculate maximum acceleration
        a_max = h / (alpha * (1f - alpha) * (1f - beta) * T * T);

        // calculate maximum jerk
        j_max = h / (alpha * alpha * beta * (1f - alpha) * (1f - beta) * T * T * T);
    }

    public float CalcDoubleSValue(float T, float h, float T_a, float T_j,
                               float v_max, float a_max, float j_max, float t)
    {
        if (t >= 0f && t <= T_j)
        {
            return j_max * Mathf.Pow(t, 3) / 6f;
        }
        else if (t > T_j && t <= T_a - T_j)
        {
            return (a_max / 6f) * ((3f * Mathf.Pow(t, 2)) - (3f * T_j * t) + Mathf.Pow(T_j, 2));
        }
        else if (t > T_a - T_j && t <= T_a)
        {
            return (v_max * (T_a / 2f)) - (v_max * (T_a - t)) - (-j_max * Mathf.Pow(T_a - t, 3) / 6f);
        }
        else if (t > T_a && t <= T - T_a)
        {
            return v_max * (T_a / 2f) + (v_max * (t - T_a));
        }
        else if (t > T - T_a && t <= T - T_a + T_j)
        {
            return h - (v_max * (T_a / 2f)) + v_max * (t - T + T_a) - (j_max * Mathf.Pow(t - T + T_a, 3) / 6f);
        }
        else if (t > T - T_a + T_j && t <= T - T_j)
        {
            float dt = t - T + T_a;
            return h - (v_max * (T_a / 2f)) + v_max * dt
                 - ((a_max / 6f) * (3f * Mathf.Pow(dt, 2) - 3f * T_j * dt + Mathf.Pow(T_j, 2)));
        }
        else if (t > T - T_j && t <= T)
        {
            return h - (j_max * Mathf.Pow(T - t, 3) / 6f);
        }
        else
        {
            return 999f; // fallback
        }
    }

}
