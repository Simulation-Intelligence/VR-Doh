using UnityEngine;

namespace MochiPound.Multiplayer
{
    [DisallowMultipleComponent]
    public sealed class MochiLocalTracking : MonoBehaviour
    {
        [SerializeField] private Transform head;
        [SerializeField] private Transform leftHand;
        [SerializeField] private Transform rightHand;
        [Tooltip("Set to the local bowl or a fixed transform at the station.")]
        [SerializeField] private Transform stationFrame;
        [SerializeField] private KineContact strikeSource;
        [Tooltip("Optional. Assign the KineTool object if KineContact is added at runtime.")]
        [SerializeField] private GameObject mallet;
        private KineContact subscribedSource;

        internal static MochiLocalTracking Active { get; private set; }
        internal Transform StationFrame => stationFrame;
        internal Transform Head => head;
        internal Transform LeftHand => leftHand;
        internal Transform RightHand => rightHand;

        private void OnEnable()
        {
            Active = this;
            TryBindMallet();
        }

        private void Update()
        {
            if (subscribedSource == null) TryBindMallet();
        }

        private void TryBindMallet()
        {
            var source = strikeSource;
            if (source == null && mallet != null) mallet.TryGetComponent(out source);
            if (source == null || source == subscribedSource) return;
            subscribedSource = source;
            subscribedSource.Contact += OnContact;
        }

        private void OnDisable()
        {
            if (subscribedSource != null) subscribedSource.Contact -= OnContact;
            subscribedSource = null;
            if (Active == this) Active = null;
        }

        private void OnContact(Vector3 point, float speed, KineContact.Kind kind)
        {
            if (kind == KineContact.Kind.Usu) SubmitStrike(speed);
        }

        // Also callable from an existing rhythm or dough-hit event.
        public void SubmitStrike(float speed)
        {
            if (MochiNetworkPlayer.Local != null)
                MochiNetworkPlayer.Local.ReportStrike(speed);
        }
    }
}
