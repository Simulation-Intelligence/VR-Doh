using System.Collections;
using MochiPound.Multiplayer;
using UnityEngine;

[RequireComponent(typeof(MochiLanSession))]
public sealed class QuestAutoJoin : MonoBehaviour
{
#if UNITY_ANDROID && !UNITY_EDITOR
    private IEnumerator Start()
    {
        yield return new WaitForSecondsRealtime(2f);
        Debug.Log("[Mochi LAN] Quest auto-join is calling Join()");
        GetComponent<MochiLanSession>().Join();
    }
#endif
}
