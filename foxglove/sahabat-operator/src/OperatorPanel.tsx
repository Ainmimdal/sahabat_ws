import type {
  MessageEvent,
  PanelExtensionContext,
} from "@foxglove/extension";
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type Tab = "console" | "teleop" | "maps" | "waypoints";
type InputMode = "keyboard" | "gamepad";
type Status = {
  mode: number;
  active_map: string;
  emergency_stop: boolean;
  motor_enabled: boolean;
  control_owner: string;
  lease_expires_in: number;
  battery_percentage: number;
  navigation_state: string;
  diagnostic_message: string;
  pose: { x: number; y: number; theta: number };
  linear_velocity: number;
  angular_velocity: number;
  map_healthy: boolean;
  scan_healthy: boolean;
  tf_healthy: boolean;
  localization_healthy: boolean;
};
type MapInfo = { map_id: string; display_name: string; modified_at: { sec: number; nanosec: number } };
type Waypoint = {
  id: string;
  name: string;
  pose: { x: number; y: number; theta: number };
  dwell_seconds: number;
  enabled: boolean;
};
type PoseStamped = { pose: { position: { x: number; y: number }; orientation: { z: number; w: number } } };

const blankStatus: Status = {
  mode: 0, active_map: "", emergency_stop: true, motor_enabled: false,
  control_owner: "", lease_expires_in: 0, battery_percentage: Number.NaN,
  navigation_state: "idle", diagnostic_message: "Waiting for robot",
  pose: { x: 0, y: 0, theta: 0 }, linear_velocity: 0, angular_velocity: 0,
  map_healthy: false, scan_healthy: false, tf_healthy: false, localization_healthy: false,
};
const modeNames = ["Idle", "Mapping", "Localization", "Gallery"];

function OperatorPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [tab, setTab] = useState<Tab>("console");
  const [status, setStatus] = useState<Status>(blankStatus);
  const [connected, setConnected] = useState(false);
  const [clientId] = useState(() => `foxglove-${crypto.randomUUID().slice(0, 8)}`);
  const [leaseId, setLeaseId] = useState("");
  const [inputMode, setInputMode] = useState<InputMode>("keyboard");
  const [deadman, setDeadman] = useState(false);
  const [velocity, setVelocity] = useState({ linear: 0, angular: 0 });
  const [controllers, setControllers] = useState<Gamepad[]>([]);
  const [controllerIndex, setControllerIndex] = useState<number>(-1);
  const [deadzone, setDeadzone] = useState(0.12);
  const [invertY, setInvertY] = useState(true);
  const [deadmanButton, setDeadmanButton] = useState(4);
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [mapName, setMapName] = useState("");
  const [mapId, setMapId] = useState("");
  const [editable, setEditable] = useState(true);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [revision, setRevision] = useState(0);
  const [notice, setNotice] = useState("");
  const keys = useRef(new Set<string>());
  const sequence = useRef(0);
  const lastPublish = useRef(0);
  const leaseRef = useRef("");
  leaseRef.current = leaseId;

  const call = useCallback(async <T,>(service: string, request: unknown): Promise<T> => {
    if (!context.callService) throw new Error("This connection does not support services");
    return await context.callService(service, request) as T;
  }, [context]);

  const publish = useCallback((linear: number, angular: number, held: boolean) => {
    if (!context.publish) return;
    sequence.current += 1;
    lastPublish.current = performance.now();
    context.publish("/operator/teleop_command", {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: clientId },
      lease_id: leaseRef.current,
      sequence: sequence.current,
      deadman: held,
      twist: {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      },
    });
    setDeadman(held);
    setVelocity({ linear, angular });
  }, [clientId, context]);

  const estop = useCallback(async (reason: string) => {
    publish(0, 0, false);
    try {
      await call("/operator/set_emergency_stop", {
        active: true, lease_id: leaseRef.current, confirmation: reason,
      });
    } catch (error) { setNotice(String(error)); }
  }, [call, publish]);

  useEffect(() => {
    context.watch("currentFrame");
    context.watch("didSeek");
    context.subscribe([{ topic: "/operator/status" }, { topic: "/operator/waypoint_candidate" }]);
    context.advertise?.("/operator/teleop_command", "sahabat_interfaces/msg/TeleopCommand");
    context.onRender = (renderState, done) => {
      setConnected(Boolean(renderState.currentFrame));
      for (const event of renderState.currentFrame ?? []) {
        const item = event as MessageEvent<Status>;
        if (item.topic === "/operator/status") setStatus(item.message);
        if (item.topic === "/operator/waypoint_candidate") {
          const candidate = (event as MessageEvent<PoseStamped>).message.pose;
          const yaw = 2 * Math.atan2(candidate.orientation.z, candidate.orientation.w);
          setWaypoints((old) => [...old, {
            id: crypto.randomUUID(), name: `Waypoint ${old.length + 1}`,
            pose: { x: candidate.position.x, y: candidate.position.y, theta: yaw },
            dwell_seconds: 0, enabled: true,
          }]);
          setTab("waypoints");
        }
      }
      done();
    };
    return () => { context.onRender = undefined; };
  }, [context]);

  const acquire = useCallback(async () => {
    try {
      const result = await call<{ success: boolean; lease_id: string; message: string }>(
        "/operator/control_lease", { action: 0, client_id: clientId, lease_id: "" },
      );
      setNotice(result.message);
      if (result.success) setLeaseId(result.lease_id);
    } catch (error) { setNotice(String(error)); }
  }, [call, clientId]);

  useEffect(() => {
    if (!leaseId) return;
    const timer = window.setInterval(async () => {
      try {
        const result = await call<{ success: boolean; lease_id: string; message: string }>(
          "/operator/control_lease", { action: 1, client_id: clientId, lease_id: leaseRef.current },
        );
        if (!result.success) { setLeaseId(""); await estop("lease renewal failed"); }
      } catch { setLeaseId(""); await estop("bridge lost during lease renewal"); }
    }, 2000);
    return () => window.clearInterval(timer);
  }, [call, clientId, estop, leaseId]);

  const keyboardVelocity = useCallback(() => {
    const held = keys.current.has("Space") && leaseRef.current !== "";
    const linear = held ? (Number(keys.current.has("KeyW")) - Number(keys.current.has("KeyS"))) * 0.20 : 0;
    const angular = held ? (Number(keys.current.has("KeyA")) - Number(keys.current.has("KeyD"))) * 0.60 : 0;
    publish(linear, angular, held && (linear !== 0 || angular !== 0));
  }, [publish]);

  useEffect(() => {
    if (inputMode !== "keyboard") return;
    const down = (event: KeyboardEvent) => {
      if (["Space", "KeyW", "KeyA", "KeyS", "KeyD"].includes(event.code)) event.preventDefault();
      if (event.repeat) return;
      keys.current.add(event.code); keyboardVelocity();
    };
    const up = (event: KeyboardEvent) => { keys.current.delete(event.code); keyboardVelocity(); };
    window.addEventListener("keydown", down); window.addEventListener("keyup", up);
    return () => { window.removeEventListener("keydown", down); window.removeEventListener("keyup", up); };
  }, [inputMode, keyboardVelocity]);

  useEffect(() => {
    const focusLost = () => { keys.current.clear(); void estop("Foxglove focus lost"); };
    const visibility = () => { if (document.hidden) focusLost(); };
    window.addEventListener("blur", focusLost);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      window.removeEventListener("blur", focusLost);
      document.removeEventListener("visibilitychange", visibility);
      publish(0, 0, false);
    };
  }, [estop, publish]);

  useEffect(() => {
    if (inputMode !== "gamepad") return;
    const timer = window.setInterval(() => {
      const found = [...navigator.getGamepads()].filter((pad): pad is Gamepad => pad != null);
      setControllers(found);
      const pad = found.find((candidate) => candidate.index === controllerIndex);
      if (!pad) { if (deadman) void estop("gamepad disconnected"); return; }
      const held = Boolean(pad.buttons[deadmanButton]?.pressed) && Boolean(leaseRef.current);
      const axis = (value: number) => Math.abs(value) < deadzone ? 0 : value;
      const y = axis(pad.axes[1] ?? 0) * (invertY ? -1 : 1);
      const x = axis(pad.axes[0] ?? 0);
      publish(held ? y * 0.20 : 0, held ? -x * 0.60 : 0, held && (x !== 0 || y !== 0));
    }, 100);
    return () => window.clearInterval(timer);
  }, [controllerIndex, deadman, deadmanButton, deadzone, estop, inputMode, invertY, publish]);

  const changeInputMode = async (next: InputMode) => {
    await estop("input mode changed");
    keys.current.clear(); setInputMode(next);
  };

  const clearEstop = async () => {
    const result = await call<{ success: boolean; message: string }>("/operator/set_emergency_stop", {
      active: false, lease_id: leaseRef.current, confirmation: "CLEAR",
    }); setNotice(result.message);
  };
  const refreshMaps = async () => {
    const result = await call<{ maps: MapInfo[] }>("/operator/maps/list", {});
    setMaps(result.maps);
  };
  const saveMap = async () => {
    if (!mapId.match(/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/)) { setNotice("Map ID must use letters, numbers, _ or -"); return; }
    const overwrite = maps.some((map) => map.map_id === mapId) && window.confirm(`Archive and overwrite ${mapId}?`);
    if (maps.some((map) => map.map_id === mapId) && !overwrite) return;
    const result = await call<{ success: boolean; message: string }>("/operator/maps/save", {
      map_id: mapId, display_name: mapName || mapId, save_editable_session: editable,
      overwrite, lease_id: leaseRef.current,
    }); setNotice(result.message); await refreshMaps();
  };
  const switchMode = async (mode: string, selectedMap = "") => {
    const result = await call<{ success: boolean; message: string }>("/operator/set_mode", {
      mode, map_id: selectedMap, lease_id: leaseRef.current,
    }); setNotice(result.message);
  };
  const loadWaypoints = async () => {
    const result = await call<{ revision: number; waypoints: Waypoint[] }>("/operator/waypoints/get", { map_id: status.active_map });
    setRevision(result.revision); setWaypoints(result.waypoints);
  };
  const saveWaypoints = async () => {
    const result = await call<{ success: boolean; revision: number; message: string }>("/operator/waypoints/save", {
      map_id: status.active_map, expected_revision: revision, waypoints, lease_id: leaseRef.current,
    }); setRevision(result.revision); setNotice(result.message);
  };
  const addCurrentPose = () => setWaypoints((old) => [...old, {
    id: crypto.randomUUID(), name: `Waypoint ${old.length + 1}`,
    pose: { x: status.pose.x, y: status.pose.y, theta: status.pose.theta }, dwell_seconds: 0, enabled: true,
  }]);
  const updateWaypoint = (index: number, patch: Partial<Waypoint>) => {
    setWaypoints((all) => all.map((item, i) => i === index ? { ...item, ...patch } : item));
  };
  const moveWaypoint = (index: number, offset: number) => {
    setWaypoints((all) => {
      const target = index + offset;
      if (target < 0 || target >= all.length) return all;
      const copy = [...all];
      [copy[index], copy[target]] = [copy[target]!, copy[index]!];
      return copy;
    });
  };
  const patrol = async (command: number, waypointId = "") => {
    const result = await call<{ success: boolean; message: string }>("/operator/patrol", {
      command, waypoint_id: waypointId, loop: command === 1, lease_id: leaseRef.current,
    }); setNotice(result.message);
  };

  const statusRows = useMemo(() => [
    ["Connection", connected ? "Connected" : "Disconnected"], ["Mode", modeNames[status.mode] ?? "Unknown"],
    ["Map", status.active_map || "None"], ["Lease", leaseId ? `Mine (${status.lease_expires_in.toFixed(1)}s)` : status.control_owner || "None"],
    ["Motors", status.motor_enabled ? "Enabled" : "Disabled"], ["Localization", status.localization_healthy ? "Healthy" : "Not ready"],
    ["Navigation", status.navigation_state], ["Battery", Number.isFinite(status.battery_percentage) ? `${Math.round(status.battery_percentage)}%` : "Unknown"],
  ], [connected, leaseId, status]);

  return <div className="sahabat">
    <style>{css}</style>
    <header><div><b>SAHABAT</b><small>{status.diagnostic_message}</small></div>
      <button className="stop" onClick={() => void estop("operator E-stop")}>E-STOP</button></header>
    <nav>{(["console", "teleop", "maps", "waypoints"] as Tab[]).map((name) =>
      <button className={tab === name ? "active" : ""} onClick={() => setTab(name)} key={name}>{name}</button>)}</nav>
    {tab === "console" && <section><div className="grid">{statusRows.map(([label, value]) => <div className="card" key={label}><small>{label}</small><strong>{value}</strong></div>)}</div>
      <div className="actions"><button onClick={() => void acquire()}>Take control</button><button disabled={!leaseId || !status.emergency_stop} onClick={() => void clearEstop()}>Clear E-stop</button>
      <button onClick={() => void switchMode("idle")}>Safe idle</button><button onClick={() => void switchMode("mapping")}>Start mapping</button></div></section>}
    {tab === "teleop" && <section><div className="toggle"><button className={inputMode === "keyboard" ? "active" : ""} onClick={() => void changeInputMode("keyboard")}>Keyboard</button><button className={inputMode === "gamepad" ? "active" : ""} onClick={() => void changeInputMode("gamepad")}>Gamepad</button></div>
      {inputMode === "keyboard" ? <div className="help"><b>Hold Space + W/A/S/D</b><span>Release any key or leave this window to stop.</span></div> : <div className="form"><label>Controller<select value={controllerIndex} onChange={(e) => setControllerIndex(Number(e.target.value))}><option value={-1}>Select controller</option>{controllers.map((pad) => <option key={pad.index} value={pad.index}>{pad.id}</option>)}</select></label><label>Deadzone<input type="number" min="0" max="0.5" step="0.01" value={deadzone} onChange={(e) => setDeadzone(Number(e.target.value))}/></label><label>Deadman button<input type="number" min="0" value={deadmanButton} onChange={(e) => setDeadmanButton(Number(e.target.value))}/></label><label><input type="checkbox" checked={invertY} onChange={(e) => setInvertY(e.target.checked)}/> Invert Y axis</label></div>}
      <div className={`deadman ${deadman ? "held" : ""}`}>{deadman ? "DEADMAN HELD" : "STOPPED"}</div><code>linear {velocity.linear.toFixed(2)} m/s · angular {velocity.angular.toFixed(2)} rad/s</code></section>}
    {tab === "maps" && <section><div className="actions"><button onClick={() => void refreshMaps()}>Refresh</button></div><div className="form"><label>Map ID<input value={mapId} onChange={(e) => setMapId(e.target.value)}/></label><label>Display name<input value={mapName} onChange={(e) => setMapName(e.target.value)}/></label><label><input type="checkbox" checked={editable} onChange={(e) => setEditable(e.target.checked)}/> Keep editable SLAM session</label><button disabled={!leaseId} onClick={() => void saveMap()}>Save named map</button></div>{maps.map((map) => <div className="row" key={map.map_id}><div><b>{map.display_name}</b><small>{map.map_id}</small></div><button onClick={() => { setMapId(map.map_id); void switchMode("localization", map.map_id); }}>Load</button></div>)}</section>}
    {tab === "waypoints" && <section><div className="actions"><button onClick={() => void loadWaypoints()}>Load</button><button onClick={addCurrentPose}>Add current pose</button><button onClick={() => void saveWaypoints()}>Save revision {revision}</button><button onClick={() => void patrol(1)}>Patrol</button><button onClick={() => void patrol(2)}>Pause</button><button onClick={() => void patrol(3)}>Resume</button><button onClick={() => void patrol(4)}>Stop</button></div>{waypoints.map((point, index) => <div className="waypoint" key={point.id}><input aria-label="Name" value={point.name} onChange={(e) => updateWaypoint(index, {name: e.target.value})}/><input aria-label="X" type="number" step="0.05" value={point.pose.x} onChange={(e) => updateWaypoint(index, {pose: {...point.pose, x: Number(e.target.value)}})}/><input aria-label="Y" type="number" step="0.05" value={point.pose.y} onChange={(e) => updateWaypoint(index, {pose: {...point.pose, y: Number(e.target.value)}})}/><input aria-label="Yaw" type="number" step="0.05" value={point.pose.theta} onChange={(e) => updateWaypoint(index, {pose: {...point.pose, theta: Number(e.target.value)}})}/><input aria-label="Dwell" type="number" min="0" step="0.5" value={point.dwell_seconds} onChange={(e) => updateWaypoint(index, {dwell_seconds: Number(e.target.value)})}/><button disabled={index === 0} onClick={() => moveWaypoint(index, -1)}>↑</button><button disabled={index === waypoints.length - 1} onClick={() => moveWaypoint(index, 1)}>↓</button><button onClick={() => void patrol(0, point.id)}>Go</button><button onClick={() => setWaypoints((all) => all.filter((_, i) => i !== index))}>Delete</button></div>)}</section>}
    {notice && <footer>{notice}<button onClick={() => setNotice("")}>×</button></footer>}
  </div>;
}

const css = `
  *{box-sizing:border-box}.sahabat{font:13px Inter,system-ui;color:#e8edf2;background:#101820;min-height:100%;padding:12px}header{display:flex;justify-content:space-between;align-items:center;gap:12px}header b{font-size:20px;letter-spacing:.14em;color:#65d2c4}small{display:block;color:#98a9b8;margin-top:4px}.stop{background:#d62f3b!important;color:white!important;border:2px solid #ff7780!important;font-weight:900;font-size:17px;padding:14px 24px!important}button,input,select{background:#1d2b36;color:#e8edf2;border:1px solid #3c5262;border-radius:5px;padding:8px}button:disabled{opacity:.4}nav{display:flex;gap:4px;margin:14px 0}nav button{text-transform:capitalize;flex:1}.active{background:#176b63!important;border-color:#65d2c4!important}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.card{background:#17242e;border-left:3px solid #65d2c4;padding:10px}.card strong{display:block;margin-top:6px}.actions,.toggle{display:flex;flex-wrap:wrap;gap:7px;margin:12px 0}.form{display:grid;gap:9px}.form label{display:grid;gap:4px}.form label:has(input[type=checkbox]){display:flex}.help{display:grid;place-items:center;min-height:130px;background:#17242e}.deadman{text-align:center;margin:12px 0;padding:14px;background:#8a2830;font-weight:800}.deadman.held{background:#176b63}.row{display:flex;align-items:center;justify-content:space-between;gap:8px;border-top:1px solid #2c3b47;padding:9px 0}.waypoint{display:grid;grid-template-columns:2fr repeat(4,1fr) repeat(4,auto);gap:5px;border-top:1px solid #2c3b47;padding:8px 0}.waypoint input{min-width:0;width:100%}footer{position:sticky;bottom:4px;background:#dfb84a;color:#151515;padding:9px;display:flex;justify-content:space-between;margin-top:10px}footer button{color:#151515;background:transparent;border:0}`;

export function initOperatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OperatorPanel context={context} />);
  return () => root.unmount();
}
