using Unity.Netcode;
using Unity.Netcode.Transports.UTP;
using UnityEngine;

namespace MochiPound.Multiplayer
{
    [DisallowMultipleComponent]
    public sealed class MochiLanSession : MonoBehaviour
    {
        [SerializeField] private NetworkManager manager;
        [SerializeField] private UnityTransport transport;
        [SerializeField] private string hostAddress = "127.0.0.1";
        [SerializeField] private ushort port = 7777;
        [SerializeField] private bool showDesktopControls = true;

        private string status = "Idle";

        private void OnEnable()
        {
            if (manager != null)
                manager.OnClientDisconnectCallback += OnDisconnected;
        }

        private void OnDisable()
        {
            if (manager != null)
                manager.OnClientDisconnectCallback -= OnDisconnected;
        }

        public void SetHostAddress(string value)
        {
            hostAddress = value.Trim();
        }

        public void Host()
        {
            if (!CanStart()) return;

            transport.SetConnectionData("127.0.0.1", port, "0.0.0.0");
            status = manager.StartHost()
                ? "Hosting on port " + port
                : "Failed to start host";
        }

        public void Join()
        {
            Debug.Log($"[Mochi LAN] Join called. Target: {hostAddress}:{port}", this);

            if (!CanStart())
            {
                Debug.LogWarning($"[Mochi LAN] Join skipped. Session status: {status}", this);
                return;
            }

            if (string.IsNullOrWhiteSpace(hostAddress))
            {
                status = "Enter the host's LAN address";
                Debug.LogWarning($"[Mochi LAN] {status}", this);
                return;
            }

            transport.SetConnectionData(hostAddress, port);
            bool started = manager.StartClient();
            status = started
                ? "Connecting to " + hostAddress
                : "Failed to start client";

            Debug.Log($"[Mochi LAN] {status} (target {hostAddress}:{port})", this);
        }

        public void Leave()
        {
            if (manager != null && manager.IsListening)
                manager.Shutdown();

            status = "Idle";
        }

        private bool CanStart()
        {
            if (manager == null || transport == null)
            {
                status = "Assign NetworkManager and UnityTransport";
                Debug.LogError(status, this);
                return false;
            }

            return !manager.IsListening;
        }

        private void OnDisconnected(ulong client)
        {
            if (manager != null && !manager.IsServer && client == manager.LocalClientId)
                status = "Disconnected from host";
        }

        private void OnGUI()
        {
            if (!showDesktopControls) return;

            GUILayout.BeginArea(new Rect(12, 12, 330, 190), GUI.skin.box);
            GUILayout.Label("MochiPound LAN");

            if (manager != null && manager.IsListening)
            {
                GUILayout.Label(manager.IsConnectedClient && !manager.IsHost ? "Connected" : status);
                GUILayout.Label("Shared strikes: " + MochiNetworkPlayer.ConnectedStrikes);

                if (GUILayout.Button("Leave")) Leave();
            }
            else
            {
                GUILayout.Label(status);
                hostAddress = GUILayout.TextField(hostAddress);

                if (GUILayout.Button("Host")) Host();
                if (GUILayout.Button("Join")) Join();
            }

            GUILayout.EndArea();
        }
    }
}