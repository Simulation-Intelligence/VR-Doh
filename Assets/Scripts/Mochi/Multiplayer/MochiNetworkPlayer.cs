using System;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace MochiPound.Multiplayer
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(NetworkObject))]
    public sealed class MochiNetworkPlayer : NetworkBehaviour
    {
        [Serializable]
        public struct TrackingPose : INetworkSerializable, IEquatable<TrackingPose>
        {
            public byte Valid;
            public Vector3 Head, Left, Right;
            public Quaternion HeadRotation, LeftRotation, RightRotation;

            public void NetworkSerialize<T>(BufferSerializer<T> s) where T : IReaderWriter
            {
                s.SerializeValue(ref Valid);
                s.SerializeValue(ref Head);
                s.SerializeValue(ref Left);
                s.SerializeValue(ref Right);
                s.SerializeValue(ref HeadRotation);
                s.SerializeValue(ref LeftRotation);
                s.SerializeValue(ref RightRotation);
            }

            public bool Equals(TrackingPose other) =>
                Valid == other.Valid && Head.Equals(other.Head) && Left.Equals(other.Left) &&
                Right.Equals(other.Right) && HeadRotation.Equals(other.HeadRotation) &&
                LeftRotation.Equals(other.LeftRotation) && RightRotation.Equals(other.RightRotation);
        }

        [SerializeField] private Transform remoteHead;
        [SerializeField] private Transform remoteLeftHand;
        [SerializeField] private Transform remoteRightHand;
        [SerializeField] private GameObject remoteVisual;

        private readonly NetworkVariable<TrackingPose> pose = new(
            default, NetworkVariableReadPermission.Everyone, NetworkVariableWritePermission.Owner);
        private readonly NetworkVariable<int> strikes = new();
        private static readonly HashSet<MochiNetworkPlayer> Players = new();
        internal static MochiNetworkPlayer Local { get; private set; }
        private float nextPose;
        private double nextStrike;
        private bool hasRenderedPose;

        public static int ConnectedStrikes
        {
            get
            {
                int count = 0;
                foreach (var player in Players) count += player.strikes.Value;
                return count;
            }
        }

        public override void OnNetworkSpawn()
        {
            Players.Add(this);
            if (IsOwner) Local = this;
            if (remoteVisual != null) remoteVisual.SetActive(false);
        }

        public override void OnNetworkDespawn()
        {
            Players.Remove(this);
            if (Local == this) Local = null;
            hasRenderedPose = false;
            if (remoteVisual != null) remoteVisual.SetActive(false);
        }

        private void LateUpdate()
        {
            if (!IsSpawned) return;
            var rig = MochiLocalTracking.Active;
            if (rig == null || rig.StationFrame == null) return;
            Transform frame = rig.StationFrame;
            if (IsOwner)
            {
                if (Time.unscaledTime < nextPose) return;
                nextPose = Time.unscaledTime + .05f;
                TrackingPose sample = new();
                Read(rig.Head, frame, 1, ref sample.Valid, out sample.Head, out sample.HeadRotation);
                Read(rig.LeftHand, frame, 2, ref sample.Valid, out sample.Left, out sample.LeftRotation);
                Read(rig.RightHand, frame, 4, ref sample.Valid, out sample.Right, out sample.RightRotation);
                pose.Value = sample;
                return;
            }

            TrackingPose received = pose.Value;
            if ((received.Valid & 1) == 0) return;
            float blend = hasRenderedPose ? 1f - Mathf.Exp(-18f * Time.unscaledDeltaTime) : 1f;
            Apply(remoteHead, frame, received.Head, received.HeadRotation, blend);
            ApplyHand(remoteLeftHand, frame, received.Left, received.LeftRotation, received.Valid, 2, blend);
            ApplyHand(remoteRightHand, frame, received.Right, received.RightRotation, received.Valid, 4, blend);
            if (remoteVisual != null && !hasRenderedPose) remoteVisual.SetActive(true);
            hasRenderedPose = true;
        }

        private static void Read(Transform target, Transform frame, byte bit, ref byte valid,
            out Vector3 position, out Quaternion rotation)
        {
            position = Vector3.zero;
            rotation = Quaternion.identity;
            if (target == null || !target.gameObject.activeInHierarchy) return;
            valid |= bit;
            position = frame.InverseTransformPoint(target.position);
            rotation = Quaternion.Inverse(frame.rotation) * target.rotation;
        }

        private static void Apply(Transform target, Transform frame, Vector3 position,
            Quaternion rotation, float blend)
        {
            if (target == null) return;
            target.SetPositionAndRotation(
                Vector3.Lerp(target.position, frame.TransformPoint(position), blend),
                Quaternion.Slerp(target.rotation, frame.rotation * rotation, blend));
        }

        private static void ApplyHand(Transform target, Transform frame, Vector3 position,
            Quaternion rotation, byte valid, byte bit, float blend)
        {
            if (target == null) return;
            bool tracked = (valid & bit) != 0;
            if (tracked) Apply(target, frame, position, rotation, target.gameObject.activeSelf ? blend : 1f);
            if (target.gameObject.activeSelf != tracked) target.gameObject.SetActive(tracked);
        }

        internal void ReportStrike(float speed)
        {
            if (IsSpawned && IsOwner) SubmitStrikeRpc(speed);
        }

        [Rpc(SendTo.Server, RequireOwnership = true)]
        private void SubmitStrikeRpc(float speed)
        {
            if (float.IsNaN(speed) || float.IsInfinity(speed) || speed < .5f || speed > 25f)
                return;

            double now = Time.unscaledTimeAsDouble;
            if (now < nextStrike) return;
            nextStrike = now + .25;
            strikes.Value++;
        }
    }
}
