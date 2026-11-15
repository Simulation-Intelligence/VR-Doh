# MochiPound LAN prototype

Base: mergedWithSound at edbe4bf (Unity 6000.6.9f1). Uses Netcode for
GameObjects 2.7.0 with Unity Transport in direct-IP mode. Start with a copy
of your current Quest scene so your single-player scene is easy to compare.

## Wire the scene once

1. Open your existing Mochi scene from the Quest branch. Add a NetworkManager
   and UnityTransport to a new root object. Select UnityTransport in the
   NetworkManager config and keep its default direct transport protocol.
   Add MochiLanSession and assign the manager and transport references.
2. Create a new **network avatar prefab** with NetworkObject and
   MochiNetworkPlayer on the root. Add a visual child with separate Head,
   LeftHand and RightHand transforms. Basic colored spheres work for
   the smoke test. Assign these and the visual root in the Inspector.
   Set this prefab as the NetworkManager Player Prefab and register it
   with the NetworkManager's Network Prefabs list if needed.
   Keep the XR camera and input on the local rig in the scene.
3. Add MochiLocalTracking to the existing local XR rig. Assign the head and
   two tracked hand/controller transforms. Assign the bowl or a stable marker
   near it as Station Frame. Both clients need equivalent axes and meter scale
   (e.g. marker forward points toward the same side of the bowl). For the
   LAN smoke test, stand on opposite sides of your respective station markers.
4. Assign the mallet GameObject to Mallet. The tracking component subscribes
   to KineContact when it exists. The diagnostic count uses existing Usu
   contact events, so it is a bowl-contact count. If the active scene's real
   dough hit/rhythm event is already exposed through a UnityEvent, call
   SubmitStrike(float) there instead and leave Mallet/Strike Source empty to
   avoid counting twice.
5. Use the desktop overlay in Game View. One instance presses Host, the
   second enters the host IP and presses Join. Use 127.0.0.1 on the same
   computer, or the host's private IPv4 address for two devices on the same
   LAN. Both use UDP port 7777. Use an XR-interactable world-space UI linked
   to Host, SetHostAddress, Join and Leave for headset controls. Disable
   Show Desktop Controls for that build. A Mac host in Editor or a standalone
   build and a Quest client are suitable for a first device test.

## Verify

- When the client joins, both instances render the other player's head and
  hands, and only the local headset/camera drives the local view.
- An Usu impact by either player increments Shared Strikes on both. Repeated
  contacts are rate-limited by the host. The diagnostic total covers connected
  players, so it drops the departing player's strikes on disconnect.
- Disconnect, rejoin, and verify that the remote avatar is recreated.
- Remote targets are relative to the chosen station marker. For shared-space
  mixed reality, calibrate an anchor separately before co-located use.

## Scope and next integration

This is LAN presence plus a host-counted test event. Existing dough solvers,
rhythm score, music clocks, and grab ownership are still local. After the LAN
test, make one host-authoritative interaction per tool, publish compact dough
state and rhythm start time, and add a Relay or another external transport as
needed. For personalized rabbits, use prefab/visual variants driven by a small
outfit descriptor; track head and hands with IK and keep the local rig separate.
