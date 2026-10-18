"""Drives the console like the UI would and checks what reached ROS."""
import asyncio, json, sys, time, urllib.request
import websockets
B = "http://127.0.0.1:8080"
fails = []
def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else "")); ok or fails.append(name)
def post(name, args=None):
    req = urllib.request.Request(f"{B}/api/actions/{name}", data=json.dumps({"request_id": f"{name}-{time.time()}", "client_id": "t", "operator": "ci", "args": args or {}}).encode(), headers={"content-type": "application/json"})
    return json.loads(urllib.request.urlopen(req).read())
def robot():
    return json.load(open("/tmp/fake_robot.json"))
def mega():
    return json.load(open("/tmp/fake_arduino.json"))
async def main():
    async with websockets.connect("ws://127.0.0.1:8080/ws", max_size=None) as ws:
        await ws.send(json.dumps({"type": "hello", "client_id": "t", "label": "ci"}))
        async def snap():
            await ws.send(json.dumps({"type": "beat"}))
            last = json.loads(await ws.recv())
            while True:
                try: last = json.loads(await asyncio.wait_for(ws.recv(), 0.05))
                except asyncio.TimeoutError: return last
        async def wait(pred, n=120):
            s = None
            for _ in range(n):
                s = await snap()
                if pred(s): return s
                await asyncio.sleep(0.25)
            return s
        s = await wait(lambda s: s["joints"]["fresh"] and s["perception"]["window"]["consecutive"] >= 3, 80)
        j = s["joints"]["values"] or {}
        check("real bridge: STATE4 → joint_states → console units", abs(j.get("joint_elbow", 0) - 60) < 0.1 and abs(j.get("base_x", 0) - 0.42) < 1e-6,
              f"elbow {j.get('joint_elbow')}, x {j.get('base_x')}")
        s = await wait(lambda s: s["joints"]["session_id"] is not None, 40)
        boot0 = mega()["boot"]
        check("bridge_info: boot_id from firmware BOOT line", s["joints"]["session_id"] == boot0, f"{s['joints']['session_id']} vs {boot0}")
        check("bridge asked the firmware for its boot (ID)", mega()["ids"] >= 1)
        # ---- hardware finding 2026-10-18: bridge live from outside, console unit down
        br = next(c for c in s["components"] if c["unit"] == "hp-bridge")
        check("live bridge with down unit shows 'running outside console'", br["health"] == "external", f"{br['health']}: {br['detail']}")
        check("…and it is not BRIDGE_DOWN", s["robot"]["state"] != "BRIDGE_DOWN", s["robot"]["state"])
        check("…and Start is blocked in the UI state", bool(br["start_block"]), br["start_block"] or "")
        post("take_control")
        r = post("unit_start", {"unit": "hp-bridge"})
        check("unit_start refused while an external reader holds the port", r["status"] == "rejected"
              and "already running outside the console" in r["detail"] and "arduino_reader" in r["detail"], r["detail"])
        r = post("unit_restart", {"unit": "hp-bridge"})
        check("unit_restart refused too", r["status"] == "rejected", r["detail"])
        import subprocess
        n = int(subprocess.run(["pgrep", "-fc", "^python3 .*arduino_reader"], capture_output=True, text=True).stdout.strip() or 0)
        check("still exactly one arduino_reader", n == 1, str(n))
        check("latched /arduino/ready seen", s["joints"]["ready"] is True)
        check("degraded executor_state from status text", s["executor"]["fresh"] and s["degraded"] is not None, s["degraded"] or "")
        check("detector diagnostics + poses → window", s["perception"]["window"]["consecutive"] >= 3, s["perception"]["state"])
        check("camera image decoded", next(f for f in s["feeds"] if f["id"] == "d405_color")["seq"] is not None)
        check("/molmopoint task_id via GetParameters", s["perception"]["task_id"] == 1, str(s["perception"]["task_id"]))
        post("take_control")
        for k in ("one_target", "area_clear", "at_start_spot"): post("declare", {"item": k})
        s = await snap()
        print("  robot state:", s["robot"]["state"], "| motor:", s["motor_power"]["value"], [r["text"] for r in s["motor_power"]["reasons"]],
              "| blockers:", [p["label"] for p in s["preflight"] if not p["ok"]])
        r = post("start_task", {"task": 1})
        check("start gated without MoveIt/TF/origin (correct)", r["status"] == "rejected", r["detail"])
        r = post("gripper_open")  # allowed: idle, joints fresh; the fake executor runs its sequence on any command
        check("gripper_open accepted", r["status"] == "sent", r["detail"])
        await asyncio.sleep(1.5)
        cmds = robot()["commands"]
        if cmds:
            check("GestureCommand on /gesture_control/command", cmds[-1][0] == "gripper_open" and cmds[-1][1] == "gripper"
                  and cmds[-1][3].startswith("console:"), str(cmds[-1]))
            s = await wait(lambda s: s["approval"] is not None, 60)
            check("approval derived from status text", s["approval"] is not None)
            check("plan preview from /display_planned_path", bool(s["approval"] and s["approval"]["preview_ready"]),
                  json.dumps(s["approval"]["preview"]["base_delta"]) if s["approval"] and s["approval"]["preview"] else "")
            check("timeline steps prefilled from execution map", len(s["executor"]["steps"]) >= 6, str(len(s["executor"]["steps"])))
            if not s["approval"]:
                print("FAILED:", fails); sys.exit(1)
            r = post("approve", {"plan_id": s["approval"]["plan_id"]})
            check("approve accepted", r["status"] == "sent", r["detail"])
            await asyncio.sleep(1.0)
            check("approval → Bool on approve_planned_path", robot()["approvals"][-1:] == [True], str(robot()["approvals"]))
            s = await wait(lambda s: s["robot"]["state"] in ("HARDWARE_FAULT", "ABORTED"), 40)
            check("abort parsed from status text", s["executor"]["last_result"] and s["executor"]["last_result"]["outcome"] == "aborted",
                  str(s["executor"]["last_result"]))
        stops0 = mega()["stops"]
        post("stop")
        await asyncio.sleep(1.0)
        check("STOP → /arduino/stop → bridge → firmware STOP", mega()["stops"] > stops0, f"{stops0} → {mega()['stops']}")
        # ---- Set start here via ZERO (no reset)
        post("ack_fault")
        r = post("flow_set_start", {"reason": "integration test"})
        check("set-start flow uses ZERO", "zero" in r["detail"], r["detail"])
        s = await wait(lambda s: s["flow"] and s["flow"]["step"] == "where", 40)
        r = post("flow_confirm", {"choice": "here"})
        s = await wait(lambda s: s["flow"] and s["flow"]["status"] != "active", 60)
        s = await wait(lambda s: s["origin"]["status"] == "confirmed", 8)  # confirmed in the same tick the flow completes
        check("ZERO reached the firmware", mega()["zeros"] == 1, str(mega()["zeros"]))
        check("odometry reads 0 after ZERO", abs(mega()["x"]) < 1e-9 and abs((s["joints"]["values"] or {}).get("base_x", 1)) < 1e-6)
        check("origin confirmed via ZERO", s["origin"]["status"] == "confirmed" and "ZERO" in (s["origin"]["note"] or ""),
              f"{s['origin']['status']} / {s['origin']['note']} / flow {s['flow']['status']} {s['flow']['error']}")
        check("same boot after ZERO (no reset)", mega()["boot"] == boot0)
        # ---- bridge restart (pty reopen, no DTR): same boot → origin kept
        import subprocess
        subprocess.run(["pkill", "-f", "arduino_reader.py"])
        await asyncio.sleep(1.0)
        subprocess.Popen(["bash", "-c", "python3 /repo/homeplus_moveit_config/scripts/arduino_reader.py --port /tmp/ttyFAKE >> /tmp/bridge.log 2>&1"])
        s = await wait(lambda s: s["joints"]["fresh"] and s["origin"]["status"] != "confirmed", 30)
        check("bridge restart without reset keeps the origin", s["origin"]["status"] == "confirmed", s["origin"]["status"])
        check("rosout captured (bridge log lines)", any(l["node"] == "arduino_bridge" and "Connected" in l["text"] for l in s["logs"]))
        # ---- firmware reboot: new boot → origin lost
        open("/tmp/fake_arduino.reset", "w").close()
        s = await wait(lambda s: s["origin"]["status"] == "lost", 40)
        check("firmware reboot (new BOOT id) loses the origin", s["origin"]["status"] == "lost", s["origin"]["lost_reason"] or "")
        # ---- reproduce the incident: a second reader on the same pty (started by hand, not by the console)
        open("/tmp/tf_drop", "w").close()
        subprocess.Popen(["bash", "-c", "python3 /repo/homeplus_moveit_config/scripts/arduino_reader.py --port /tmp/ttyFAKE >> /tmp/bridge2.log 2>&1"])
        s = await wait(lambda s: any(n["id"] == "dup-arduino_bridge" for n in s["next_actions"]), 40)
        dup = next((n for n in s["next_actions"] if n["id"] == "dup-arduino_bridge"), None)
        check("two arduino_bridge nodes flagged as a bad next action", dup is not None and dup["severity"] == "bad",
              dup["problem"] if dup else "")
        s = await wait(lambda s: any(l["source"].startswith("ambiguous") for l in s["logs"] if l["node"] == "arduino_bridge"), 20)
        check("bridge log lines tagged ambiguous while two share the name",
              any(l["source"].startswith("ambiguous") for l in s["logs"] if l["node"] == "arduino_bridge"),
              str(sorted({l["source"] for l in s["logs"] if l["node"] == "arduino_bridge"})))
        s = await wait(lambda s: any(n["id"] == "tf-drops" for n in s["next_actions"]), 20)
        tf = next((n for n in s["next_actions"] if n["id"] == "tf-drops"), None)
        check("TF-missing drops surface as a health warning", tf is not None, (tf or {}).get("fix", ""))
        r = post("unit_start", {"unit": "hp-bridge"})
        check("unit_start still refused with two readers live", r["status"] == "rejected", r["detail"])
        subprocess.run(["bash", "-c", "pkill -n -f arduino_reader.py; rm -f /tmp/tf_drop"])
        post("return_to_start", {"inspected": True, "release_cup": True})
        await asyncio.sleep(2.5)
        names = [c[0] for c in robot()["commands"]]
        print("  commands seen:", names)
    print("FAILED:", fails) if fails else print("ALL PASS")
    sys.exit(1 if fails else 0)
asyncio.run(main())
