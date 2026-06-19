import type { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ros2humble as ros2 } from "@foxglove/rosmsg-msgs-common";
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

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
  active_operation: string;
  pose: { x: number; y: number; theta: number };
  linear_velocity: number;
  angular_velocity: number;
  map_healthy: boolean;
  scan_healthy: boolean;
  tf_healthy: boolean;
  localization_healthy: boolean;
  localization_recovery_active?: boolean;
  localization_recovery_status?: string;
};
type MapInfo = {
  map_id: string;
  display_name: string;
  has_editable_session?: boolean;
  modified_at: { sec: number; nanosec: number };
};
type Waypoint = {
  id: string;
  name: string;
  pose: { x: number; y: number; theta: number };
  dwell_seconds: number;
  enabled: boolean;
};
type PoseStamped = {
  pose: {
    position: { x: number; y: number };
    orientation: { z: number; w: number };
  };
};

const blankStatus: Status = {
  mode: 0,
  active_map: "",
  emergency_stop: true,
  motor_enabled: false,
  control_owner: "",
  lease_expires_in: 0,
  battery_percentage: Number.NaN,
  navigation_state: "idle",
  diagnostic_message: "Waiting for robot",
  active_operation: "",
  pose: { x: 0, y: 0, theta: 0 },
  linear_velocity: 0,
  angular_velocity: 0,
  map_healthy: false,
  scan_healthy: false,
  tf_healthy: false,
  localization_healthy: false,
  localization_recovery_active: false,
  localization_recovery_status: "Not running",
};

const modeNames = ["Idle", "Mapping", "Localization", "Operate"];
const teleopDatatypes = new Map([
  ["std_msgs/Header", ros2["std_msgs/Header"]],
  ["sensor_msgs/Joy", ros2["sensor_msgs/Joy"]],
]);

function OperatorPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [status, setStatus] = useState<Status>(blankStatus);
  const [connected, setConnected] = useState(false);
  const [clientId] = useState(() => `foxglove-${crypto.randomUUID().slice(0, 8)}`);
  const [leaseId, setLeaseId] = useState("");
  const [inputMode, setInputMode] = useState<InputMode>("keyboard");
  const [commanding, setCommanding] = useState(false);
  const [velocity, setVelocity] = useState({ linear: 0, angular: 0 });
  const [controllers, setControllers] = useState<Gamepad[]>([]);
  const [controllerIndex, setControllerIndex] = useState(-1);
  const [deadzone, setDeadzone] = useState(0.12);
  const [invertY, setInvertY] = useState(true);
  const [linearSpeed, setLinearSpeed] = useState(0.20);
  const [angularSpeed, setAngularSpeed] = useState(0.60);
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [mapName, setMapName] = useState("");
  const [mapId, setMapId] = useState("");
  const [keepEditable, setKeepEditable] = useState(true);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [revision, setRevision] = useState(0);
  const [notice, setNotice] = useState("");
  const [busy, setBusy] = useState("");
  const keys = useRef(new Set<string>());
  const lastStatusAt = useRef(0);
  const leaseRef = useRef("");
  leaseRef.current = leaseId;

  const call = useCallback(async <T,>(service: string, request: unknown): Promise<T> => {
    if (!context.callService) {
      throw new Error("This Foxglove connection does not support ROS services");
    }
    return await context.callService(service, request) as T;
  }, [context]);

  const run = useCallback(async (name: string, task: () => Promise<void>) => {
    setBusy(name);
    try {
      await task();
    } catch (error) {
      setNotice(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy("");
    }
  }, []);

  const publish = useCallback((linear: number, angular: number, active: boolean) => {
    if (!context.publish) return;
    context.publish("/operator/foxglove_joy", {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: leaseRef.current },
      axes: [linear, angular],
      buttons: [active ? 1 : 0],
    });
    setCommanding(active);
    setVelocity({ linear, angular });
  }, [context]);

  const stopTeleop = useCallback(() => {
    keys.current.clear();
    publish(0, 0, false);
  }, [publish]);

  useEffect(() => {
    context.watch("currentFrame");
    context.subscribe([
      { topic: "/operator/status" },
      { topic: "/operator/waypoint_candidate" },
    ]);
    context.advertise?.("/operator/foxglove_joy", "sensor_msgs/Joy", {
      datatypes: teleopDatatypes,
    });
    context.onRender = (renderState, done) => {
      for (const event of renderState.currentFrame ?? []) {
        if (event.topic === "/operator/status") {
          lastStatusAt.current = performance.now();
          setConnected(true);
          setStatus((event as MessageEvent<Status>).message);
        } else if (event.topic === "/operator/waypoint_candidate") {
          const candidate = (event as MessageEvent<PoseStamped>).message.pose;
          const yaw = 2 * Math.atan2(candidate.orientation.z, candidate.orientation.w);
          setWaypoints((old) => [...old, {
            id: crypto.randomUUID(),
            name: `Waypoint ${old.length + 1}`,
            pose: { x: candidate.position.x, y: candidate.position.y, theta: yaw },
            dwell_seconds: 0,
            enabled: true,
          }]);
        }
      }
      done();
    };
    return () => {
      context.onRender = undefined;
      context.unadvertise?.("/operator/foxglove_joy");
    };
  }, [context]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (performance.now() - lastStatusAt.current > 2000) setConnected(false);
    }, 500);
    return () => window.clearInterval(timer);
  }, []);

  const acquire = useCallback(async () => {
    const result = await call<{
      success: boolean;
      lease_id: string;
      message: string;
    }>("/operator/control_lease", { action: 0, client_id: clientId, lease_id: "" });
    setNotice(result.message);
    if (result.success) setLeaseId(result.lease_id);
  }, [call, clientId]);

  const release = useCallback(async () => {
    stopTeleop();
    const result = await call<{ success: boolean; message: string }>(
      "/operator/control_lease",
      { action: 2, client_id: clientId, lease_id: leaseRef.current },
    );
    setNotice(result.message);
    if (result.success) setLeaseId("");
  }, [call, clientId, stopTeleop]);

  useEffect(() => {
    if (!leaseId) return;
    const timer = window.setInterval(async () => {
      try {
        const result = await call<{ success: boolean; message: string }>(
          "/operator/control_lease",
          { action: 1, client_id: clientId, lease_id: leaseRef.current },
        );
        if (!result.success) throw new Error(result.message);
      } catch {
        stopTeleop();
        setLeaseId("");
        setNotice("Control lease lost");
      }
    }, 2000);
    return () => window.clearInterval(timer);
  }, [call, clientId, leaseId, stopTeleop]);

  const keyboardVelocity = useCallback(() => {
    const moving = ["KeyW", "KeyA", "KeyS", "KeyD"].some((key) => keys.current.has(key));
    const active = moving && leaseRef.current !== "";
    const linear = active
      ? (Number(keys.current.has("KeyW")) - Number(keys.current.has("KeyS"))) * linearSpeed
      : 0;
    const angular = active
      ? (Number(keys.current.has("KeyA")) - Number(keys.current.has("KeyD"))) * angularSpeed
      : 0;
    publish(linear, angular, active);
  }, [angularSpeed, linearSpeed, publish]);

  useEffect(() => {
    if (inputMode !== "keyboard") return;
    const editable = (target: EventTarget | null) => {
      const element = target as { isContentEditable?: boolean; tagName?: string } | null;
      return element?.isContentEditable === true
        || ["INPUT", "TEXTAREA", "SELECT"].includes(element?.tagName ?? "");
    };
    const down = (event: KeyboardEvent) => {
      if (editable(event.target) || !["KeyW", "KeyA", "KeyS", "KeyD"].includes(event.code)) return;
      event.preventDefault();
      if (!event.repeat) keys.current.add(event.code);
      keyboardVelocity();
    };
    const up = (event: KeyboardEvent) => {
      if (!["KeyW", "KeyA", "KeyS", "KeyD"].includes(event.code)) return;
      event.preventDefault();
      keys.current.delete(event.code);
      keyboardVelocity();
    };
    const targets: Window[] = [window];
    try {
      if (window.top != undefined && window.top !== window) targets.push(window.top);
    } catch {
      // Foxglove may isolate extension frames; the local target still works.
    }
    const uniqueTargets = [...new Set(targets)];
    const timer = window.setInterval(keyboardVelocity, 100);
    for (const target of uniqueTargets) {
      target.addEventListener("keydown", down, true);
      target.addEventListener("keyup", up, true);
    }
    return () => {
      window.clearInterval(timer);
      for (const target of uniqueTargets) {
        target.removeEventListener("keydown", down, true);
        target.removeEventListener("keyup", up, true);
      }
      stopTeleop();
    };
  }, [inputMode, keyboardVelocity, stopTeleop]);

  useEffect(() => {
    const visibility = () => { if (document.hidden) stopTeleop(); };
    document.addEventListener("visibilitychange", visibility);
    return () => {
      document.removeEventListener("visibilitychange", visibility);
    };
  }, [stopTeleop]);

  useEffect(() => {
    if (inputMode !== "gamepad") return;
    const timer = window.setInterval(() => {
      const found = [...navigator.getGamepads()].filter((pad): pad is Gamepad => pad != null);
      setControllers(found);
      const pad = found.find((candidate) => candidate.index === controllerIndex);
      if (!pad) {
        publish(0, 0, false);
        return;
      }
      const axis = (value: number) => Math.abs(value) < deadzone ? 0 : value;
      const linearAxis = axis(pad.axes[1] ?? 0) * (invertY ? -1 : 1);
      const angularAxis = -axis(pad.axes[0] ?? 0);
      const active = Boolean(leaseRef.current)
        && (linearAxis !== 0 || angularAxis !== 0);
      publish(
        active ? linearAxis * linearSpeed : 0,
        active ? angularAxis * angularSpeed : 0,
        active,
      );
    }, 100);
    return () => {
      window.clearInterval(timer);
      stopTeleop();
    };
  }, [angularSpeed, controllerIndex, deadzone, inputMode, invertY, linearSpeed, publish, stopTeleop]);

  const switchMode = async (mode: string, selectedMap = "") => {
    stopTeleop();
    const result = await call<{ success: boolean; message: string }>("/operator/set_mode", {
      mode,
      map_id: selectedMap,
      lease_id: leaseRef.current,
    });
    setNotice(result.message);
  };

  const refreshMaps = useCallback(async () => {
    const result = await call<{ maps: MapInfo[] }>("/operator/maps/list", {});
    setMaps(result.maps);
  }, [call]);

  useEffect(() => {
    if (connected) void run("Refreshing maps", refreshMaps);
  }, [connected, refreshMaps, run]);

  const saveMap = async () => {
    if (!/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(mapId)) {
      setNotice("Map ID must use letters, numbers, _ or -");
      return;
    }
    const exists = maps.some((map) => map.map_id === mapId);
    const overwrite = exists && window.confirm(`Archive the old ${mapId} and replace it?`);
    if (exists && !overwrite) return;
    const result = await call<{ message: string }>("/operator/maps/save", {
      map_id: mapId,
      display_name: mapName.trim() || mapId,
      save_editable_session: keepEditable,
      overwrite,
      lease_id: leaseRef.current,
    });
    setNotice(result.message);
    await refreshMaps();
  };

  const loadWaypoints = useCallback(async () => {
    if (!status.active_map) return;
    const result = await call<{ revision: number; waypoints: Waypoint[] }>(
      "/operator/waypoints/get",
      { map_id: status.active_map },
    );
    setRevision(result.revision);
    setWaypoints(result.waypoints);
  }, [call, status.active_map]);

  useEffect(() => {
    if (status.active_map) void run("Loading routes", loadWaypoints);
  }, [loadWaypoints, run, status.active_map]);

  const saveWaypoints = async () => {
    const result = await call<{ revision: number; message: string }>(
      "/operator/waypoints/save",
      {
        map_id: status.active_map,
        expected_revision: revision,
        waypoints,
        lease_id: leaseRef.current,
      },
    );
    setRevision(result.revision);
    setNotice(result.message);
  };

  const addCurrentPose = () => setWaypoints((old) => [...old, {
    id: crypto.randomUUID(),
    name: `Waypoint ${old.length + 1}`,
    pose: { ...status.pose },
    dwell_seconds: 0,
    enabled: true,
  }]);

  const setDockHere = () => setWaypoints((old) => {
    const existing = old.findIndex((item) => item.name.trim().toLowerCase() === "dock");
    const dock: Waypoint = {
      id: existing >= 0 ? old[existing]!.id : crypto.randomUUID(),
      name: "dock",
      pose: { ...status.pose },
      dwell_seconds: 0,
      enabled: true,
    };
    if (existing < 0) return [dock, ...old];
    return old.map((item, index) => index === existing ? dock : item);
  });

  const updateWaypoint = (index: number, patch: Partial<Waypoint>) => {
    setWaypoints((all) => all.map((item, itemIndex) => (
      itemIndex === index ? { ...item, ...patch } : item
    )));
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
    const result = await call<{ message: string }>("/operator/patrol", {
      command,
      waypoint_id: waypointId,
      loop: command === 1,
      lease_id: leaseRef.current,
    });
    setNotice(result.message);
  };

  const localizationRecovery = async (action: 0 | 1) => {
    stopTeleop();
    const result = await call<{ message: string }>("/operator/localization/recovery", {
      action,
      lease_id: leaseRef.current,
    });
    setNotice(result.message);
  };

  const healthItems = useMemo(() => [
    ["Map", status.map_healthy],
    ["Lidar", status.scan_healthy],
    ["TF", status.tf_healthy],
    ["Localization", status.localization_healthy],
  ] as const, [status]);
  const hasLease = leaseId !== "";
  const canMove = connected && hasLease;
  const recoveryActive = status.localization_recovery_active === true;

  return <div className="sahabat">
    <style>{css}</style>
    <header className="topbar">
      <div className="identity">
        <b>SAHABAT</b>
        <span className={`connection ${connected ? "ok" : "bad"}`}>
          {connected ? "Robot online" : "Disconnected"}
        </span>
      </div>
    </header>

    <div className="summary">
      <div><small>Mode</small><strong>{modeNames[status.mode] ?? "Unknown"}</strong></div>
      <div><small>Map</small><strong>{status.active_map || "None"}</strong></div>
      <div><small>Control</small><strong>{hasLease ? "This laptop" : status.control_owner || "Unclaimed"}</strong></div>
      <div><small>Motion</small><strong>{status.navigation_state || "idle"}</strong></div>
    </div>

    <div className="healthStrip">
      {healthItems.map(([label, healthy]) => (
        <span className={healthy ? "ok" : "bad"} key={label}>{label}</span>
      ))}
    </div>

    <div className="controlBar">
      {!hasLease
        ? <button className="primary" disabled={!connected || busy !== ""} onClick={() => void run("Taking control", acquire)}>Take control</button>
        : <button onClick={() => void run("Releasing control", release)}>Release control</button>}
      <span>{status.active_operation || status.diagnostic_message}</span>
    </div>

    <main>
      <section className="stack driveSection">
        {!status.localization_healthy && status.mode >= 2 && <div className="warning">
          <div><b>Localization needs attention</b><span>Check the lidar overlay, then run recovery if it does not match the map.</span></div>
          <button disabled={!canMove || recoveryActive} onClick={() => void run("Starting recovery", () => localizationRecovery(0))}>Recover</button>
        </div>}

        <div className="sectionTitle"><div><h2>Manual drive</h2><p>WASD or one selected gamepad. No deadman button.</p></div><button onClick={stopTeleop}>Stop command</button></div>
        <div className="segmented">
          <button className={inputMode === "keyboard" ? "active" : ""} onClick={() => { stopTeleop(); setInputMode("keyboard"); }}>Keyboard</button>
          <button className={inputMode === "gamepad" ? "active" : ""} onClick={() => { stopTeleop(); setInputMode("gamepad"); }}>Gamepad</button>
        </div>

        {inputMode === "keyboard" ? <div className="drivePad">
          <div className="keys"><i>W</i><i>A</i><i>S</i><i>D</i></div>
          <div><b>Click this panel, then drive</b><span>Releasing the keys or leaving Foxglove stops the command.</span></div>
        </div> : <div className="form twoCol">
          <label>Controller<select value={controllerIndex} onChange={(event) => setControllerIndex(Number(event.target.value))}><option value={-1}>Select a connected controller</option>{controllers.map((pad) => <option key={pad.index} value={pad.index}>{pad.id}</option>)}</select></label>
          <label>Stick deadzone<input type="number" min="0" max="0.5" step="0.01" value={deadzone} onChange={(event) => setDeadzone(Number(event.target.value))}/></label>
          <label className="check"><input type="checkbox" checked={invertY} onChange={(event) => setInvertY(event.target.checked)}/> Invert forward axis</label>
        </div>}

        <div className="form twoCol speedControls">
          <label>Forward speed · {linearSpeed.toFixed(2)} m/s<input type="range" min="0.10" max="0.50" step="0.05" value={linearSpeed} onChange={(event) => setLinearSpeed(Number(event.target.value))}/></label>
          <label>Turn speed · {angularSpeed.toFixed(2)} rad/s<input type="range" min="0.30" max="1.20" step="0.10" value={angularSpeed} onChange={(event) => setAngularSpeed(Number(event.target.value))}/></label>
        </div>

        <div className={`command ${commanding ? "moving" : ""}`}>
          <strong>{commanding ? "COMMANDING" : hasLease ? "READY" : "NO CONTROL"}</strong>
          <span>{velocity.linear.toFixed(2)} m/s · {velocity.angular.toFixed(2)} rad/s</span>
        </div>
        <div className="pose">Pose <b>{status.pose.x.toFixed(2)}, {status.pose.y.toFixed(2)}</b> · yaw <b>{status.pose.theta.toFixed(2)}</b></div>
      </section>

      <section className="stack mapsSection">
        <div className="sectionTitle"><div><h2>Mapping session</h2><p>Create a map, give it a useful name, then save it.</p></div><button disabled={!hasLease || status.mode === 1} onClick={() => void run("Starting mapping", () => switchMode("mapping"))}>Start mapping</button></div>
        <div className="form">
          <label>Map ID <small>Short filename: gallery_ground_floor</small><input value={mapId} placeholder="gallery_ground_floor" onChange={(event) => setMapId(event.target.value)}/></label>
          <label>Display name<input value={mapName} placeholder="Gallery ground floor" onChange={(event) => setMapName(event.target.value)}/></label>
          <label className="check"><input type="checkbox" checked={keepEditable} onChange={(event) => setKeepEditable(event.target.checked)}/> Keep resumable mapping data</label>
          <button className="primary" disabled={!hasLease || status.mode !== 1 || busy !== ""} onClick={() => void run("Saving map", saveMap)}>Save current map</button>
        </div>

        <div className="sectionTitle"><div><h2>Saved maps</h2><p>Opening a map starts localization, routes, and recovery tools.</p></div><button onClick={() => void run("Refreshing maps", refreshMaps)}>Refresh</button></div>
        <div className="list">{maps.length === 0 && <div className="empty">No saved maps found.</div>}{maps.map((map) => (
          <article className={status.active_map === map.map_id ? "selected" : ""} key={map.map_id}>
            <div><b>{map.display_name || map.map_id}</b><span>{map.map_id}{map.has_editable_session ? " · resumable" : ""}</span></div>
            <button disabled={!hasLease || busy !== ""} onClick={() => void run("Opening map", () => switchMode("operations", map.map_id))}>Open</button>
          </article>
        ))}</div>
        <button className="quiet" disabled={!hasLease || status.mode === 0} onClick={() => void run("Stopping navigation", () => switchMode("idle"))}>Stop stack and return to idle</button>
      </section>

      <section className="stack routesSection">
        <div className="sectionTitle"><div><h2>Routes · {status.active_map || "no map"}</h2><p>Click a pose in the 3D panel or capture the robot’s current pose.</p></div><button disabled={!status.active_map} onClick={() => void run("Loading routes", loadWaypoints)}>Reload</button></div>
        <div className="actions">
          <button disabled={!status.active_map} onClick={addCurrentPose}>Add current pose</button>
          <button disabled={!status.active_map} onClick={setDockHere}>Set dock here</button>
          <button className="primary" disabled={!hasLease || !status.active_map} onClick={() => void run("Saving routes", saveWaypoints)}>Save changes</button>
        </div>
        <div className="patrol">
          <button disabled={!canMove || waypoints.length === 0} onClick={() => void run("Starting patrol", () => patrol(1))}>Start patrol</button>
          <button disabled={!hasLease} onClick={() => void run("Pausing patrol", () => patrol(2))}>Pause</button>
          <button disabled={!canMove} onClick={() => void run("Resuming patrol", () => patrol(3))}>Resume</button>
          <button disabled={!hasLease} onClick={() => void run("Stopping patrol", () => patrol(4))}>Stop</button>
        </div>
        <div className="waypoints">{waypoints.length === 0 && <div className="empty">No waypoints for this map.</div>}{waypoints.map((point, index) => (
          <article className={point.name.toLowerCase() === "dock" ? "dock" : ""} key={point.id}>
            <div className="waypointHead"><input aria-label="Waypoint name" value={point.name} onChange={(event) => updateWaypoint(index, { name: event.target.value })}/><label className="check"><input type="checkbox" checked={point.enabled} onChange={(event) => updateWaypoint(index, { enabled: event.target.checked })}/> Enabled</label></div>
            <div className="coordinates">
              <label>X<input type="number" step="0.05" value={point.pose.x} onChange={(event) => updateWaypoint(index, { pose: { ...point.pose, x: Number(event.target.value) } })}/></label>
              <label>Y<input type="number" step="0.05" value={point.pose.y} onChange={(event) => updateWaypoint(index, { pose: { ...point.pose, y: Number(event.target.value) } })}/></label>
              <label>Yaw<input type="number" step="0.05" value={point.pose.theta} onChange={(event) => updateWaypoint(index, { pose: { ...point.pose, theta: Number(event.target.value) } })}/></label>
              <label>Dwell<input type="number" min="0" step="0.5" value={point.dwell_seconds} onChange={(event) => updateWaypoint(index, { dwell_seconds: Number(event.target.value) })}/></label>
            </div>
            <div className="waypointActions"><button disabled={index === 0} onClick={() => moveWaypoint(index, -1)}>Up</button><button disabled={index === waypoints.length - 1} onClick={() => moveWaypoint(index, 1)}>Down</button><button disabled={!canMove || !point.enabled} onClick={() => void run("Navigating", () => patrol(0, point.id))}>Go</button><button className="dangerText" onClick={() => setWaypoints((all) => all.filter((_, itemIndex) => itemIndex !== index))}>Delete</button></div>
          </article>
        ))}</div>
      </section>

      <section className="stack healthSection">
        <div className="healthGrid">{healthItems.map(([label, healthy]) => <div className={healthy ? "healthy" : "unhealthy"} key={label}><small>{label}</small><strong>{healthy ? "Healthy" : "Not ready"}</strong></div>)}</div>
        <div className="recoveryCard">
          <div><h2>Localization recovery</h2><p>Globally reset AMCL, then rotate slowly until the lidar-to-map match is stable.</p></div>
          <div className="recoveryState"><b>{recoveryActive ? "ROTATING" : "IDLE"}</b><span>{status.localization_recovery_status || "Not running"}</span></div>
          <div className="actions"><button className="primary" disabled={!canMove || recoveryActive || status.mode < 2} onClick={() => void run("Starting recovery", () => localizationRecovery(0))}>Global relocalize + rotate</button><button disabled={!hasLease || !recoveryActive} onClick={() => void run("Stopping recovery", () => localizationRecovery(1))}>Stop recovery</button></div>
        </div>
        <div className="details">
          <div><small>Motors</small><b>{status.motor_enabled ? "Enabled" : "Disabled"}</b></div>
          <div><small>Battery</small><b>{Number.isFinite(status.battery_percentage) ? `${Math.round(status.battery_percentage)}%` : "Unknown"}</b></div>
          <div><small>Lease remaining</small><b>{hasLease ? `${status.lease_expires_in.toFixed(1)} s` : "None"}</b></div>
          <div><small>Linear / angular</small><b>{status.linear_velocity.toFixed(2)} / {status.angular_velocity.toFixed(2)}</b></div>
        </div>
      </section>
    </main>

    {busy && <div className="busy">{busy}…</div>}
    {notice && <footer><span>{notice}</span><button onClick={() => setNotice("")}>×</button></footer>}
  </div>;
}

const css = `
  :root{color-scheme:dark}*{box-sizing:border-box}.sahabat{--bg:#0b1117;--surface:#121c25;--surface2:#182630;--line:#2b3e4b;--text:#edf4f7;--muted:#91a6b2;--teal:#42c8b5;--teal2:#176b63;--red:#e33d49;--amber:#e4ad45;font:13px Inter,system-ui,sans-serif;color:var(--text);background:var(--bg);min-height:100%;padding:14px}button,input,select{font:inherit;color:var(--text);background:var(--surface2);border:1px solid var(--line);border-radius:7px;padding:9px 11px}button{cursor:pointer;font-weight:650}button:hover:not(:disabled){border-color:#5c7b8d}button:disabled{opacity:.38;cursor:not-allowed}h2,p{margin:0}h2{font-size:15px}p,span,small{color:var(--muted)}small{display:block;font-size:11px}.topbar{display:flex;justify-content:space-between;align-items:center;gap:12px}.identity{display:grid;gap:5px}.identity>b{font-size:20px;letter-spacing:.16em;color:var(--teal)}.connection:before,.healthStrip span:before{content:"";display:inline-block;width:7px;height:7px;border-radius:50%;background:currentColor;margin-right:6px}.ok{color:var(--teal)!important}.bad{color:#ff7c85!important}.summary{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:1px;background:var(--line);border:1px solid var(--line);border-radius:8px;overflow:hidden;margin:14px 0 8px}.summary>div{background:var(--surface);padding:9px;min-width:0}.summary strong{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;margin-top:3px}.healthStrip{display:flex;gap:12px;padding:5px 2px}.healthStrip span{font-size:11px}.controlBar{display:flex;align-items:center;gap:7px;margin:10px 0}.controlBar span{margin-left:auto;text-align:right}.primary{background:var(--teal2);border-color:var(--teal);color:white}.quiet{width:100%;background:transparent}button.active{background:var(--teal2);border-color:var(--teal)}main{display:grid;gap:14px;min-height:250px}.stack{display:grid;gap:14px;background:var(--surface);border:1px solid var(--line);border-radius:9px;padding:13px}.driveSection{order:1}.healthSection{order:2}.mapsSection{order:3}.routesSection{order:4}.sectionTitle{display:flex;justify-content:space-between;align-items:flex-start;gap:12px}.sectionTitle p{margin-top:4px}.warning{display:flex;align-items:center;justify-content:space-between;gap:10px;background:#362a16;border:1px solid #795b24;border-radius:8px;padding:10px}.warning div{display:grid;gap:4px}.segmented{display:grid;grid-template-columns:1fr 1fr;gap:4px}.drivePad{min-height:115px;display:grid;place-items:center;text-align:center;gap:12px;background:var(--surface2);border-radius:8px;padding:14px}.drivePad>div:last-child{display:grid;gap:5px}.keys{display:grid;grid-template-columns:repeat(3,32px);grid-template-rows:repeat(2,30px);gap:4px}.keys i{display:grid;place-items:center;background:#243744;border:1px solid #486273;border-radius:5px;font-style:normal;font-weight:800}.keys i:first-child{grid-column:2}.speedControls input{width:100%;accent-color:var(--teal)}.command{display:flex;justify-content:space-between;align-items:center;padding:12px;background:#172832;border-left:4px solid #526d7d;border-radius:6px}.command.moving{background:#15332f;border-color:var(--teal)}.pose{text-align:center;color:var(--muted)}.form{display:grid;gap:10px}.form label,.coordinates label{display:grid;gap:5px;color:var(--muted)}.twoCol{grid-template-columns:2fr 1fr}.check{display:flex!important;align-items:center;gap:7px!important}.check input{width:auto}.actions,.patrol,.waypointActions{display:flex;flex-wrap:wrap;gap:6px}.list,.waypoints{display:grid;gap:7px}.list article,.waypoints article{background:var(--surface2);border:1px solid var(--line);border-radius:8px;padding:10px}.list article{display:flex;justify-content:space-between;align-items:center}.list article>div{display:grid;gap:4px}.list article.selected{border-color:var(--teal)}.empty{text-align:center;color:var(--muted);padding:20px}.waypoints article.dock{border-left:4px solid var(--amber)}.waypointHead{display:flex;justify-content:space-between;gap:10px}.waypointHead>input{font-weight:750;flex:1}.coordinates{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:9px 0}.coordinates input{min-width:0;width:100%}.dangerText{color:#ff8b93}.healthGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:7px}.healthGrid>div{padding:12px;background:var(--surface2);border-left:4px solid}.healthGrid strong{display:block;margin-top:5px}.healthy{border-color:var(--teal)!important}.unhealthy{border-color:var(--red)!important}.recoveryCard{display:grid;gap:12px;padding:13px;background:var(--surface2);border-radius:8px}.recoveryCard p{margin-top:5px}.recoveryState{display:grid;gap:4px}.recoveryState b{color:var(--amber)}.details{display:grid;grid-template-columns:repeat(2,1fr);gap:7px}.details>div{background:var(--surface2);padding:10px;border-radius:7px}.details b{display:block;margin-top:4px}.busy{position:sticky;bottom:5px;margin-top:8px;background:#203844;border:1px solid #477184;padding:9px;border-radius:7px}footer{position:sticky;bottom:5px;margin-top:8px;display:flex;justify-content:space-between;align-items:center;gap:8px;background:var(--amber);color:#151515;padding:9px;border-radius:7px}footer span{color:#151515}footer button{padding:2px 7px;background:transparent;border:0;color:#151515;font-size:18px}@media(max-width:560px){.summary{grid-template-columns:repeat(2,1fr)}.coordinates{grid-template-columns:repeat(2,1fr)}.twoCol{grid-template-columns:1fr}.controlBar{flex-wrap:wrap}.controlBar span{width:100%;text-align:left}.sectionTitle{align-items:stretch;flex-direction:column}.sectionTitle>button{width:100%}}
`;

export function initOperatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OperatorPanel context={context} />);
  return () => root.unmount();
}
