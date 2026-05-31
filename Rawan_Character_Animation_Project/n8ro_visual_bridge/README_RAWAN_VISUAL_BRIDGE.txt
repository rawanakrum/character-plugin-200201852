Rawan N8RO visual bridge package
================================

What changed compared with the previous bridge:

1. The bridge still loads:
   userPlugins\sim\rawan\character_plugin_200201852.dll

2. It now also uses N8RO's sim-char-anim sample mechanism:
   modelFactoryRegistry -> animationModelNathanHuman -> registerAnimation(...)

3. It registers visible animation evaluators over existing supported clips:
   Idle Shake     -> Rawan PUSH-LIKE joint overrides
   Idle Breathing -> Rawan CLIMB-LIKE joint overrides

4. This avoids needing new Scenario Editor animation metadata. Those two codes already exist
   in NeutralCivilian_01's SupportedAnimationList.

Install:

Copy these into:
D:\N8RO_2\dev\samples\sim\sim-plugin

- include\SimPlugin.h
- include\arkheon\character\ICharacterController.h
- src\SimPlugin.cpp
- payload\character_plugin_200201852.dll
- sim-plugin.vcxproj

Then build Release | x64 in Visual Studio.

After build, run N8RO and check:
D:\N8RO_2\userPlugins\sim\rawan_bridge.log

Expected log:
- visual bridge loaded
- character DLL loaded: name='Rawan Character Plugin v0.1'
- visual registration ok: Idle Shake -> PUSH-LIKE, Idle Breathing -> CLIMB-LIKE
- tick ok ... visualRegistered=1

Lua:

Optionally copy lua\human_animation_loop.lua to:
D:\N8RO_2\data\resources\missions\human_animation_loop.lua

Expected visual sequence:
0-3 sec   Idle Walk Forward = built-in walk
3-6 sec   Idle Shake        = Rawan push-like joint overrides
6-9 sec   Idle Breathing    = Rawan climb-like joint overrides
9-12 sec  Idle Neutral

If the log says visualRegistered=0, N8RO did not expose animationModelNathanHuman to this plugin at startup.
