import React from 'react';
import styles from './styles.module.css';

/**
 * ArchitectureDiagram
 *
 * Hand-built, code-defined system block diagram for the LiDAR evaluation
 * framework. Laid out as the actual lifecycle of a run, top to bottom:
 *
 *   1. Configuration & Setup   (gold band)   — config pair -> polysetup -> artifacts
 *   2. System Bringup          (grey band)   — the launch entry point
 *   3. Run Mode                (fork)        — record new bags, or replay existing ones
 *   4. Runtime Pipeline        (green band)  — ground truth -> filter -> evaluate -> report
 *   5. Outputs & Reporting     (orange band) — backend -> storage -> dashboard
 *   Legend
 *
 * Node names, packages and service names here are the real ones — keep them in
 * sync with the launch file and each package's console_scripts.
 */

function Box({variant, icon, title, sub, note, children}) {
  const cls = [styles.box, variant && styles[variant]].filter(Boolean).join(' ');
  return (
    <div className={cls}>
      <div className={styles.boxTitle}>
        {icon && <span className={styles.icon}>{icon}</span>}
        <span>{title}</span>
      </div>
      {sub && <div className={styles.boxSub}>{sub}</div>}
      {note && <div className={styles.boxNote}>{note}</div>}
      {children}
    </div>
  );
}

/** A labelled stage inside the runtime pipeline. */
function Stage({title, note, children}) {
  return (
    <div className={styles.group}>
      <p className={styles.groupTitle}>{title}</p>
      <div className={styles.row}>{children}</div>
      {note && <p className={styles.stageNote}>{note}</p>}
    </div>
  );
}

/** One cell of the runtime loop, placed into its named grid area. */
function LoopCell({area, children}) {
  return <div style={{gridArea: area}}>{children}</div>;
}

/** An arrow along the ring, labelled with what moves at that step. */
function LoopArrow({area, dir, label}) {
  const glyph = {right: '▶', down: '▼', left: '◀', up: '▲'}[dir];
  return (
    <LoopCell area={area}>
      <div className={styles.loopArrow}>
        <span className={styles.loopGlyph}>{glyph}</span>
        <span className={styles.loopLabel}>{label}</span>
      </div>
    </LoopCell>
  );
}

/**
 * A data-flow arrow between two stages. `label` is a short plain-English
 * description of what actually moves — deliberately not a topic or service
 * name, since those are listed once in the interfaces box instead.
 */
function Flow({label, neutral}) {
  const pill = [styles.flowPill, neutral && styles.flowPillNeutral]
    .filter(Boolean)
    .join(' ');
  return (
    <div className={styles.flow}>
      <span className={styles.flowStem} />
      <span className={pill}>{label}</span>
      <span className={styles.flowStem} />
      <span className={styles.flowHead}>▼</span>
    </div>
  );
}

export default function ArchitectureDiagram() {
  return (
    <div className={styles.diagram}>
      {/* ---------- 1. CONFIGURATION & SETUP ---------- */}
      <section className={`${styles.band} ${styles.setupBand}`}>
        <p className={styles.bandTitle}>1. Configuration &amp; Setup</p>
        <div className={styles.row}>
          <div className={styles.group}>
            <p className={styles.groupTitle}>Configuration Files (one pair per run)</p>
            <div className={styles.stack}>
              <Box
                variant="config"
                icon="📄"
                title="environment_configs/*.yaml"
                sub="(zones, obstacles, world offsets, bags)"
              />
              <Box
                variant="config"
                icon="📄"
                title="lidar_configs/*.yaml"
                sub="(TF frame, topic, mount pose, sweeps)"
              />
            </div>
          </div>

          <div className={styles.group}>
            <p className={styles.groupTitle}>Standalone Tool (not a node)</p>
            <Box
              variant="tools"
              icon="🖥️"
              title="polysetup"
              sub="(CLI, run via just recipes)"
              note="generates the URDF through lidar_zones · every path derived from --src-dir"
            />
          </div>

          <div className={styles.group}>
            <p className={styles.groupTitle}>Generated Artifacts</p>
            <div className={styles.stack}>
              <Box variant="internal" icon="📦" title="bench URDF" />
              <Box
                variant="internal"
                icon="⚙️"
                title="per-node parameter files + bringup launch"
              />
            </div>
          </div>
        </div>
        <div className={styles.svc}>
          just setup-ws &lt;environment&gt; &lt;lidar&gt; → resolve config pair → generate → colcon build
        </div>
      </section>

      <Flow label="generated configs" neutral />

      {/* ---------- 2. SYSTEM BRINGUP ---------- */}
      <section className={`${styles.band} ${styles.greyBand}`}>
        <p className={styles.bandTitle}>2. System Bringup</p>
        <Box
          variant="internal"
          icon="🚀"
          title="lidar_test_bench_bringup"
          sub="(ROS 2 launch entry point — just launch-bench)"
          note="launch file is regenerated per run; the recording chain is included or commented out to match the current mode"
        />
      </section>

      <Flow label="nodes running" neutral />

      {/* ---------- 3. RUN MODE (fork) ---------- */}
      <section className={`${styles.band} ${styles.greyBand}`}>
        <p className={styles.bandTitle}>3. Run Mode — just start-run</p>
        <div className={styles.row}>
          <div className={`${styles.trigger} ${styles.triggerGreen}`}>
            <p className={styles.triggerHead}>Mode A — Record new bags</p>
            <p className={styles.triggerSub}>just enable-bag-recording · hardware attached</p>
            <div className={styles.stack}>
              <Box
                variant="node"
                icon="🔁"
                title="lidar_automation_manager"
                note="entry: /lidar_test_bench_initiate (Trigger)"
              >
                <ol className={styles.ol}>
                  <li>Cycle the vendor driver, apply each parameter under test</li>
                  <li>Command the mount angle, let the scene settle</li>
                  <li>Record one bag per case, straight into its case folder</li>
                  <li>When every case is captured → /start_evaluation</li>
                </ol>
              </Box>
              <Box
                variant="node"
                icon="🔌"
                title="arduino_eth_bridge"
                sub="(servo / angle commands over TCP)"
              />
              <div className={styles.row}>
                <Box variant="hwBox" icon="🛰️" title="LiDAR + vendor driver" />
                <Box variant="hwBox" icon="⚙️" title="Servo mount" />
              </div>
            </div>
          </div>

          <div className={`${styles.trigger} ${styles.triggerGrey}`}>
            <p className={styles.triggerHead}>Mode B — Replay existing bags</p>
            <p className={styles.triggerSub}>just disable-bag-recording · no hardware needed</p>
            <Box
              variant="internal"
              icon="🛠️"
              title="Recording chain left out of the launch"
            >
              <ol className={styles.ol}>
                <li>No driver, no automation manager, no actuators</li>
                <li>start-run calls /start_evaluation directly</li>
                <li>Evaluates bags recorded on an earlier run</li>
              </ol>
            </Box>
            <div className={styles.svc}>
              the fast path for iterating on zones or metrics
            </div>
          </div>
        </div>
        <div className={styles.svc}>both modes converge on the same entry point</div>
      </section>

      <Flow label="evaluation started" neutral />

      {/* ---------- 4. RUNTIME PIPELINE ---------- */}
      <section className={`${styles.band} ${styles.runtimeBand}`}>
        <p className={styles.bandTitle}>4. Runtime Pipeline — ROS 2 Core Architecture</p>

        <Stage
          title="① Establish ground truth"
          note="the bench frames come from the generated URDF, so the expected answer is known before any data arrives"
        >
          <Box
            variant="node"
            icon="📡"
            title="lidar_bench_tf_broadcaster"
            sub="(lidar_transforms)"
            note="publishes the bench frames and the robot description"
          />
          <Box
            variant="node"
            icon="🔻"
            title="zones_orchestrator_node"
            sub="(lidar_zones)"
            note="resolves each zone against those frames into baseline profiles"
          />
        </Stage>

        <Flow label="expected geometry" />

        <p className={styles.groupTitle}>
          The evaluation loop — clockwise, once per scan, then once per case
        </p>

        <div className={styles.loop}>
          <LoopCell area="n2">
            <Box
              variant="node"
              icon="🧭"
              title="② eval_framework_manager_node"
              sub="(lidar_eval_orchestrator)"
              note="walks the case tree, replays each bag or takes live scans, and drives the loop"
            />
          </LoopCell>

          <LoopArrow area="aR" dir="right" label="one raw scan" />

          <LoopCell area="n3">
            <Box
              variant="node"
              icon="✂️"
              title="③ pointcloud_filter_node"
              sub="(lidar_pointcloud_filter)"
              note="cuts the scan two ways per zone — inside its volume, and along the rays aimed at it"
            />
          </LoopCell>

          <LoopArrow area="aD" dir="down" label="points per zone" />

          <LoopCell area="n4">
            <Box
              variant="node"
              icon="📐"
              title="④ LidarMetricsEngine"
              sub="(lidar_metrics_library — pure Python, not a node)"
              note="accumulates into each metric plugin, then reduces once the bag ends"
            />
          </LoopCell>

          <LoopArrow area="aL" dir="left" label="when the bag ends" />

          <LoopCell area="n5">
            <Box
              variant="node"
              icon="📈"
              title="⑤ metrics_reporting_node"
              sub="(lidar_reporting)"
              note="collects the case report, its visualization data and its recorded bag"
            />
          </LoopCell>

          <LoopArrow area="aU" dir="up" label="next case in the sweep" />

          <LoopCell area="hub">
            <div className={styles.loopHub}>
              <p className={styles.loopHubTitle}>↻ inner loop</p>
              <p className={styles.loopHubText}>
                ② → ③ → ④ repeats for <strong>every scan in the bag</strong>, each pass
                measured against the same baseline profiles
              </p>
            </div>
          </LoopCell>
        </div>

        <div className={styles.sideChannels}>
          <p className={styles.groupTitle}>Alongside the loop — live inspection, not part of the cycle</p>
          <div className={styles.row}>
            <Box
              variant="node"
              icon="🪟"
              title="visualizer_node"
              sub="(lidar_reporting)"
              note="draws fitted planes as they are computed"
            />
            <div className={styles.stack}>
              <span className={styles.flowLabel}>zone &amp; plane overlays → RViz</span>
              <span className={styles.flowLabel}>mount angle commands → servo</span>
              <span className={styles.flowLabel}>live scan inspection → Foxglove</span>
            </div>
          </div>
        </div>

        <div className={styles.interfaces}>
          <p className={styles.groupTitle}>Interfaces (lidar_test_bench_interfaces)</p>
          <ul>
            <li>FilterCloud (srv)</li>
            <li>GetProfiles (srv)</li>
            <li>Visualization (srv + msg)</li>
            <li>ExpectedZone (msg)</li>
            <li>Plane3D (msg)</li>
            <li>NumericalPointCloud (msg)</li>
            <li>Point4D (msg)</li>
          </ul>
        </div>
      </section>

      <Flow label="finished run" neutral />

      {/* ---------- 5. OUTPUTS & REPORTING ---------- */}
      <section className={`${styles.band} ${styles.outputBand}`}>
        <p className={styles.bandTitle}>5. Outputs &amp; Reporting</p>
        <div className={styles.row}>
          <Box
            variant="internal"
            icon="🗃️"
            title="lidar_eval_backends"
            sub="(swappable persistence layer)"
            note="one interface, read + write · backend chosen by registry · pluggable credentials"
          />
          <Box
            variant="storage"
            icon="📑"
            title="Google Sheets + Drive"
            sub="(the backend shipping today)"
            note="metrics tree · per-case visualization · recorded bags"
          />
          <Box
            variant="tools"
            icon="📊"
            title="polyview_app"
            sub="(Streamlit dashboard — standalone)"
            note="reads results back through the same interface"
          />
        </div>
        <div className={styles.svc}>
          adding another store = implement the interface + enable it in the registry; neither the
          reporting node nor the dashboard changes
        </div>
      </section>

      {/* ---------- LEGEND ---------- */}
      <div className={styles.legend}>
        <p className={styles.legendTitle}>Legend</p>
        <div className={styles.legendRow}>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#f7e3b8'}} /> Configuration input
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#ece7f6'}} /> Standalone tool (not a node)
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#e3f1da'}} /> ROS 2 runtime node
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#dcdcdc'}} /> Generated / internal
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#fbe6cb'}} /> Output / storage
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#ecd2d2'}} /> Physical hardware
          </span>
        </div>
        <div className={styles.legendRow} style={{marginTop: 10}}>
          <span className={styles.legendItem}>
            <span className={styles.lineSample} /> Data flow
          </span>
          <span className={styles.legendItem}>
            <span className={`${styles.lineSample} ${styles.lineDotted}`} /> Service interface
          </span>
          <span className={styles.legendItem}>
            <span className={`${styles.lineSample} ${styles.lineDashed}`} /> Optional / external
          </span>
        </div>
      </div>
    </div>
  );
}
