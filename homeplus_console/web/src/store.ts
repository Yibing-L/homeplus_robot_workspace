import { create } from "zustand";
import type { Snapshot } from "./types";

// The browser holds no robot state of its own: it renders the last full snapshot
// and says so loudly when snapshots stop arriving (UI_PLAN §3.2 row 1).
export const LINK_LOST_MS = 1500;

function makeId(): string {
  return crypto.randomUUID?.() ?? Math.random().toString(36).slice(2) + Date.now().toString(36);
}

function readOperator(): string {
  try {
    return localStorage.getItem("hp.operator") || "operator";
  } catch {
    return "operator";
  }
}

const clientId = makeId();

interface ConsoleStore {
  snap: Snapshot | null;
  receivedAt: number; // performance.now() of the last snapshot
  connected: boolean;
  clientId: string;
  operator: string;
  pending: Record<string, string>; // action name -> request_id while the POST is in flight
  lastResult: Record<string, { status: string; detail: string; at: number }>;
  setOperator: (name: string) => void;
}

export const useConsole = create<ConsoleStore>((set) => ({
  snap: null,
  receivedAt: 0,
  connected: false,
  clientId,
  operator: readOperator(),
  pending: {},
  lastResult: {},
  setOperator: (name) => {
    try {
      localStorage.setItem("hp.operator", name);
    } catch {
      /* private window: name lasts for this tab */
    }
    set({ operator: name });
  },
}));

export function connect(): void {
  let ws: WebSocket | null = null;
  let beat: number | undefined;
  const open = () => {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => {
      useConsole.setState({ connected: true });
      const { operator } = useConsole.getState();
      ws?.send(JSON.stringify({ type: "hello", client_id: clientId, label: operator }));
      beat = window.setInterval(() => ws?.readyState === 1 && ws.send(JSON.stringify({ type: "beat" })), 1000);
    };
    ws.onmessage = (ev) => {
      const snap = JSON.parse(ev.data) as Snapshot;
      useConsole.setState({ snap, receivedAt: performance.now() });
    };
    ws.onclose = () => {
      window.clearInterval(beat);
      useConsole.setState({ connected: false });
      window.setTimeout(open, 1000);
    };
    ws.onerror = () => ws?.close();
  };
  open();
}

export async function act(name: string, args: Record<string, unknown> = {}): Promise<{ status: string; detail: string }> {
  const request_id = makeId();
  const { operator } = useConsole.getState();
  useConsole.setState((s) => ({ pending: { ...s.pending, [name]: request_id } }));
  try {
    const res = await fetch(`/api/actions/${name}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ request_id, client_id: clientId, operator, args }),
    });
    const body = res.ok ? await res.json() : { status: "failed", detail: `HTTP ${res.status}` };
    useConsole.setState((s) => ({ lastResult: { ...s.lastResult, [name]: { ...body, at: Date.now() } } }));
    return body;
  } catch {
    const body = { status: "failed", detail: "console unreachable: action not sent" };
    useConsole.setState((s) => ({ lastResult: { ...s.lastResult, [name]: { ...body, at: Date.now() } } }));
    return body;
  } finally {
    useConsole.setState((s) => {
      const pending = { ...s.pending };
      delete pending[name];
      return { pending };
    });
  }
}

export async function simFault(name: string, value: unknown): Promise<void> {
  await fetch("/api/sim/fault", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, value }),
  });
}
