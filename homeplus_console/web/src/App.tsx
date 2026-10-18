import { useState } from "react";
import { ActionBand } from "./components/ActionBand";
import { LogsDrawer, SimDrawer } from "./components/Drawers";
import { Feeds } from "./components/Feeds";
import { Footer } from "./components/Footer";
import { Header } from "./components/Header";
import { Health } from "./components/Health";
import { Timeline } from "./components/Timeline";
import { useConsole } from "./store";
import { fmtAge, useSinceSnapshot } from "./util";

export function App() {
  const snap = useConsole((s) => s.snap);
  const { since, lost } = useSinceSnapshot();
  const [drawer, setDrawer] = useState<"logs" | "sim" | null>(null);
  return (
    <div className={`app ${lost ? "link-lost" : ""}`}>
      <Header snap={snap} />
      {snap ? (
        <>
          <main>
            <Health snap={snap} />
            <Feeds snap={snap} />
            <Timeline snap={snap} />
          </main>
          <ActionBand snap={snap} />
          <Footer snap={snap} onLogs={() => setDrawer("logs")} onSim={() => setDrawer("sim")} />
        </>
      ) : (
        <main />
      )}
      {lost && (
        <div className="linklost" role="alert">
          <b>Console link lost</b>
          <p>
            {snap ? `No snapshot for ${fmtAge(since)}. Every value on screen is frozen and may be wrong.` : "Waiting for the console backend."}{" "}
            Buttons are disabled until the link returns.
          </p>
          <p className="estop">Use the physical E-stop on the robot.</p>
          <p>The STOP button still tries to reach the console.</p>
        </div>
      )}
      {snap && drawer === "logs" && <LogsDrawer snap={snap} onClose={() => setDrawer(null)} />}
      {snap && drawer === "sim" && <SimDrawer snap={snap} onClose={() => setDrawer(null)} />}
    </div>
  );
}
