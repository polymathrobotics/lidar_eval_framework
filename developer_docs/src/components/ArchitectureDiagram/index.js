import React from 'react';
import styles from './styles.module.css';

/**
 * ArchitectureDiagram
 *
 * Hand-built, code-defined recreation of the LiDAR test bench system block
 * diagram (replaces the static architecture.png). Layout, grouping, colors and
 * flow annotations mirror the original figure:
 *
 *   1. Configuration & Setup   (gold band)
 *   Trigger 1 / Trigger 2      (left column: direct playback vs. automation)
 *   2. System Bringup          (grey band)
 *   3. Runtime Pipeline        (green band, ROS 2 core nodes + interfaces)
 *   Block 5: Target Hardware   (right column, dark)
 *   4. Outputs & Reporting     (orange band)
 *   Legend
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

export default function ArchitectureDiagram() {
  return (
    <div className={styles.diagram}>
      {/* ---------- 1. CONFIGURATION & SETUP ---------- */}
      <section className={`${styles.band} ${styles.setupBand}`}>
        <p className={styles.bandTitle}>1. Configuration &amp; Setup</p>
        <div className={styles.row}>
          <div className={styles.group}>
            <p className={styles.groupTitle}>Standalone Tools (not nodes)</p>
            <div className={styles.stack}>
              <Box variant="tools" icon="🖥️" title="polysetup" sub="(Tkinter GUI)" />
              <Box variant="tools" icon="📊" title="polyview_app" sub="(Streamlit)" />
            </div>
          </div>

          <div className={styles.group}>
            <p className={styles.groupTitle}>Configuration Files</p>
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
            <p className={styles.groupTitle}>Fixture Builder</p>
            <Box
              variant="config"
              icon="🛠️"
              title="lidar_eval_fixture"
              sub="(builds bench URDF)"
            />
          </div>

          <div className={styles.group}>
            <p className={styles.groupTitle}>Generated Artifacts</p>
            <Box
              variant="internal"
              icon="📦"
              title="bench URDF + downstream configs"
            />
          </div>
        </div>
      </section>

      {/* ---------- MID: triggers | pipeline | hardware ---------- */}
      <div className={styles.mid}>
        {/* LEFT COLUMN — triggers */}
        <div className={styles.col}>
          <div className={`${styles.trigger} ${styles.triggerGrey}`}>
            <p className={styles.triggerHead}>Trigger 1 — Direct Data Playback</p>
            <p className={styles.triggerSub}>(existing data)</p>
            <Box variant="internal" icon="🛠️" title="Scenario A">
              <ol className={styles.ol}>
                <li>Select config sweep</li>
                <li>Run direct evaluation of pre-existing bags</li>
              </ol>
            </Box>
          </div>

          <div className={`${styles.trigger} ${styles.triggerGreen}`}>
            <p className={styles.triggerHead}>Trigger 2 — Automation (Matrix Sweep)</p>
            <p className={styles.triggerSub}>(new data generation)</p>
            <div className={styles.stack}>
              <Box variant="node" icon="🔁" title="lidar_automation_manager">
                <ol className={styles.ol}>
                  <li>Configuration sweep, recording coordination</li>
                  <li>Move actuators, collect sensor data</li>
                  <li>Post-sweep trigger → lidar_controller</li>
                </ol>
              </Box>
              <Box
                variant="node"
                icon="🔌"
                title="arduino_eth_bridge"
                sub="(servo / angle commands over TCP)"
              />
              <Box variant="hwBox" icon="🛰️" title="LiDAR Sensor" />
              <Box variant="hwBox" icon="⚙️" title="Motors / Actuators" />
            </div>
          </div>
        </div>

        {/* CENTER COLUMN — bringup + runtime pipeline */}
        <div className={styles.col}>
          <section className={`${styles.band} ${styles.greyBand}`} style={{marginBottom: 0}}>
            <p className={styles.bandTitle}>2. System Bringup</p>
            <Box
              variant="internal"
              icon="🚀"
              title="lidar_test_bench_bringup"
              sub="(ROS 2 launch entry point)"
            />
            <div className={styles.flowRow}>
              <span className={styles.flowLabel}>/start_evaluation (SetBool) · direct playback entry</span>
              <span className={styles.flowLabel}>/start_evaluation (SetBool) · post-automation trigger</span>
            </div>
          </section>

          <div className={styles.arrowDown}>▼</div>

          <section className={`${styles.band} ${styles.runtimeBand}`} style={{marginBottom: 0}}>
            <p className={styles.bandTitle}>3. Runtime Pipeline — ROS 2 Core Nodes</p>
            <div className={styles.row}>
              <Box
                variant="node"
                icon="🔻"
                title="lidar_baseline_node"
                sub="(lidar_transforms)"
                note="profiles • /roi_filter • viz push"
              />
              <Box
                variant="node"
                icon="📡"
                title="lidar_bench_tf_broadcaster"
                sub="(lidar_transforms)"
                note="publishes TF tree"
              />
            </div>

            <div className={styles.svc}>
              /get_profiles (GetProfiles) · /roi_filter (FilterCloud) → spatial + projective clouds
            </div>

            <div className={styles.row}>
              <Box
                variant="node"
                icon="🧭"
                title="lidar_controller"
                sub="(lidar_test_bench)"
                note="orchestrates a run"
              />
              <Box
                variant="node"
                icon="📐"
                title="LidarMetricsEngine"
                sub="(lidar_metrics_library)"
                note="per-zone metric plugins"
              />
              <Box
                variant="node"
                icon="📈"
                title="grafana_reporter_node"
                sub="(lidar_reporting)"
                note="reporting & sinks"
              />
            </div>

            <div className={styles.flowRow}>
              <span className={styles.flowLabel}>per-zone clouds →</span>
              <span className={styles.flowLabel}>← results dict</span>
              <span className={styles.flowLabel}>/report_metrics (Trigger) →</span>
              <span className={styles.flowLabel}>/visualization (Visualization)</span>
            </div>

            <div className={styles.interfaces}>
              <p className={styles.groupTitle}>Interfaces (lidar_test_bench_interfaces)</p>
              <ul>
                <li>FilterCloud (srv)</li>
                <li>GetProfiles (srv)</li>
                <li>Visualization (srv/msg)</li>
                <li>ExpectedZone (msg)</li>
                <li>Plane3D (msg)</li>
                <li>NumericalPointCloud (msg)</li>
              </ul>
            </div>
          </section>
        </div>

        {/* RIGHT COLUMN — target hardware layer */}
        <div className={styles.col}>
          <div className={styles.hwLayer}>
            <p className={styles.bandTitle}>Block 5: Target Hardware Layer</p>
            <p className={styles.triggerSub} style={{color: '#d8d8d8'}}>(physical system)</p>
            <div className={styles.hwItem}>Target physical configuration matrix</div>
            <div className={styles.hwItem}>API definitions for generic actuators</div>
            <div className={styles.hwItem}>Example motor / sensor interface</div>
            <div className={styles.svc} style={{color: '#d0d0d0'}}>
              ↕ automation feedback (hardware state)
            </div>
          </div>
        </div>
      </div>

      <div className={styles.arrowDown}>▼</div>

      {/* ---------- 4. OUTPUTS & REPORTING ---------- */}
      <section className={`${styles.band} ${styles.outputBand}`}>
        <p className={styles.bandTitle}>4. Outputs &amp; Reporting</p>
        <div className={styles.row}>
          <Box variant="storage" icon="🗂️" title="Google Drive Database" sub="(optional)" />
          <Box variant="storage" icon="📟" title="Grafana" sub="(dashboards · optional sync)" />
          <Box
            variant="storage"
            icon="📑"
            title="Google Sheets / Drive"
            sub="(metrics → viz exports)"
          />
          <Box variant="tools" icon="📊" title="polyview_app" sub="(view results)" />
        </div>
      </section>

      {/* ---------- LEGEND ---------- */}
      <div className={styles.legend}>
        <p className={styles.legendTitle}>Legend</p>
        <div className={styles.legendRow}>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#ece7f6'}} /> Setup / Tools
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#f7e3b8'}} /> Configuration
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#e3f1da'}} /> ROS 2 Runtime Nodes
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#dcdcdc'}} /> Generated / Internal
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#fbe6cb'}} /> Outputs / Storage
          </span>
          <span className={styles.legendItem}>
            <span className={styles.swatch} style={{background: '#ecd2d2'}} /> Hardware Integration
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
