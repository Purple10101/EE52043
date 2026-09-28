using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Concurrent;

public class sim_scara_controller_server : MonoBehaviour
{
    TcpListener server;
    TcpClient client;
    NetworkStream stream;
    Thread receiveThread;
    volatile bool isRunning = false;
    volatile bool isAppClosing = false;
    int successFlag = 1;

    ConcurrentQueue<Action> commandQueue = new ConcurrentQueue<Action>();

    public InputField ipInputField;
    public InputField portInputField;
    public Text statusText;
    public Button startServerButton;
    public sim_scara_controller scara_controller;

    float Axis0Max = 90f;
    float Axis0Min = -90f;
    float Axis1Max = 120f;
    float Axis1Min = 0f;
    float Axis2Max = 180f;
    float Axis2Min = -180f;
    float Axis3Max = 0.075f;
    float Axis3Min = 0.025f;


    void Start()
    {
        ipInputField.text = "127.0.0.1";
        portInputField.text = "9000";
        startServerButton.onClick.AddListener(OnStartServerClicked);


    }

    void Update()
    {
        while (commandQueue.TryDequeue(out var command))
        {
            command?.Invoke();
        }
    }

    void OnStartServerClicked()
    {
        if (IPAddress.TryParse(ipInputField.text, out IPAddress ip) &&
        int.TryParse(portInputField.text, out int port))
        {
            StartServer(ip, port);
        }
        else
        {
            SetStatus("invalid IP or port", Color.red);
        }
    }

    void StartServer(IPAddress ip, int port)
    {
        try
        {
            server = new TcpListener(ip, port);
            server.Start();
            SetStatus($"server listening on {ip}:{port}", Color.green);
            server.BeginAcceptTcpClient(OnClientConnected, null);
        }
        catch (Exception e)
        {
            SetStatus($"server error: {e.Message}", Color.red);
        }
    }

    void OnClientConnected(IAsyncResult ar)
    {
        try
        {
            client = server.EndAcceptTcpClient(ar);
            stream = client.GetStream();
            isRunning = true;

            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            SetStatus("client connected", Color.green);
            Debug.LogError("MATLAB CONNECTED");
        }
        catch (Exception e)
        {
            SetStatus($"client connection error: {e.Message}", Color.red);
        }
    }

    void ReceiveData()
    {
        byte[] buffer = new byte[48];

        try
        {
            while (isRunning && stream != null && stream.CanRead)
            {
                int length = stream.Read(buffer, 0, buffer.Length);
                if (length == 0) break; // Client disconnected

                int command = BitConverter.ToInt32(buffer, 0);

                switch (command)
                {
                    case 1:
                        float link_1 = BitConverter.ToSingle(buffer, 4);
                        float link_2 = BitConverter.ToSingle(buffer, 8);
                        float z_min = BitConverter.ToSingle(buffer, 12);
                        float z_max = BitConverter.ToSingle(buffer, 16);
                        float settling_time = BitConverter.ToSingle(buffer, 20);
                        commandQueue.Enqueue(() => HandleInit(link_1, link_2, z_min, z_max, settling_time));
                        break;

                    case 3:
                        int axis = BitConverter.ToInt32(buffer, 4);
                        float angle = BitConverter.ToSingle(buffer, 8);
                        float time3 = BitConverter.ToSingle(buffer, 12);
                        commandQueue.Enqueue(() => StartCoroutine(HandleMoveJoint(axis, angle, time3)));
                        break;

                    case 4:
                        float angle0 = BitConverter.ToSingle(buffer, 4);
                        float angle1 = BitConverter.ToSingle(buffer, 8);
                        float angle2 = BitConverter.ToSingle(buffer, 12);
                        float angle3 = BitConverter.ToSingle(buffer, 16);
                        float time4 = BitConverter.ToSingle(buffer, 20);
                        commandQueue.Enqueue(() => StartCoroutine(HandleMoveJoints(angle0, angle1, angle2, angle3, time4)));
                        break;

                    case 5:
                        float x = BitConverter.ToSingle(buffer, 4);
                        float y = BitConverter.ToSingle(buffer, 8);
                        float angleZ = BitConverter.ToSingle(buffer, 12);
                        float z = BitConverter.ToSingle(buffer, 16);
                        float time5 = BitConverter.ToSingle(buffer, 20);
                        commandQueue.Enqueue(() => StartCoroutine(HandleMoveCoord(x, y, z, angleZ, time5)));
                        break;

                    default:
                        Debug.Log($"unknown command: {command}");
                        break;
                }

                Thread.Sleep(10);
            }
        }
        catch (Exception e)
        {
            if (!isAppClosing)
                Debug.LogError($"receive error: {e.Message}");
        }
        finally
        {
            Debug.Log("client disconnected.");
            CleanupClient();
            if (!isAppClosing)
                server.BeginAcceptTcpClient(OnClientConnected, null);
        }
    }

    void CleanupClient()
    {
        isRunning = false;

        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }

        stream = null;
        client = null;

        // Don't call Join() from main thread
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread = null;
        }
    }

    // Command Handlers
    void HandleInit(float link_1, float link_2, float z_min, float z_max, float settling_time)
    {
        scara_controller.scara_initialise(link_1, link_2, z_min, z_max, settling_time);
        SendSuccess();
    }

    IEnumerator HandleMoveJoint(int axis, float angle, float time)
    {
        scara_controller.scara_move_j(axis, angle, time);
        yield return new WaitForSeconds(time + scara_controller.settling_time);
        switch (axis)
        {
            case 0:
                if (angle > Axis0Max || angle < Axis0Min)
                {
                    Debug.LogError("Axis 0 is out of bounds after SCARA_MOVE_JOINT command");
                }
                break;
            case 1:
                if (angle > Axis1Max || angle < Axis1Min)
                {
                    Debug.LogError("Axis 1 is out of bounds after SCARA_MOVE_JOINT command");
                }
                break;
            case 2:
                if (angle > Axis2Max || angle < Axis2Min)
                {
                    Debug.LogError("Axis 2 is out of bounds after SCARA_MOVE_JOINT command");
                }
                break;
            case 3:
                if (angle > Axis3Max || angle < Axis3Min)
                {
                    Debug.LogError("Axis 3 is out of bounds after SCARA_MOVE_JOINT command");
                }
                break;
        }
        SendSuccess();
    }

    IEnumerator HandleMoveJoints(float angle0, float angle1, float angle2, float angle3, float time)
    {
        scara_controller.scara_move_js(angle0, angle1, angle2, angle3, time);
        yield return new WaitForSeconds(time + scara_controller.settling_time);
        if (angle0 > Axis0Max || angle0 < Axis0Min)
        {
            Debug.LogError("Axis 0 is out of bounds after SCARA_MOVE_JOINTS command");
        }
        if (angle1 > Axis1Max || angle1 < Axis1Min)
        {
            Debug.LogError("Axis 1 is out of bounds after SCARA_MOVE_JOINTS command");
        }
        if (angle2 > Axis2Max || angle2 < Axis2Min)
        {
            Debug.LogError("Axis 2 is out of bounds after SCARA_MOVE_JOINTS command");
        }
        if (angle3 > Axis3Max || angle3 < Axis3Min)
        {
            Debug.LogError("Axis 3 is out of bounds after SCARA_MOVE_JOINTS command");
        }
        SendSuccess();
    }

    IEnumerator HandleMoveCoord(float x, float y, float z, float angleZ, float time)
    {
        scara_controller.scara_move_coord(x, y, z, angleZ, time);
        yield return new WaitForSeconds(time + scara_controller.settling_time);
        SendSuccess();
    }

    void SendSuccess()
    {
        try
        {
            if (stream != null && stream.CanWrite)
            {
                byte[] response = BitConverter.GetBytes(successFlag);
                Debug.Log(response);
                stream.Write(response, 0, response.Length);
                stream.Flush();
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"send error: {e.Message}");
        }
    }

    void SetStatus(string message, Color color)
    {
        if (statusText != null)
        {
            statusText.text = message;
            statusText.color = color;
        }
    }

    void OnApplicationQuit()
    {
        isAppClosing = true;
        isRunning = false;

        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }
        try { server?.Stop(); } catch { }

        stream = null;
        client = null;
        server = null;
    }
}
