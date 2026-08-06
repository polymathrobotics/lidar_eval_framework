import math
from pathlib import Path

import streamlit as st
import yaml
from lidar_eval_backends.lidar_database_handler import LidarDatabaseHandler
import visualization_handler
import time

st.set_page_config(layout='wide', page_title='PolyView LiDAR')

class PolyViewApp:

    # Comparison-view only: Case 1 red, Case 2 teal. Shared by the case header chips
    # and the comparison chart series so the legends match the headings. Other views
    # keep the default palette.
    CASE_COLORS = ('#FF4B4B', '#2EC4B6')

    _LOGO_CSS = (Path(__file__).parent / 'css' / 'logo.css').read_text()
    _LOGO_PATH = Path(__file__).parent / 'css' / 'polymath_robotics_logo.png'

    def refresh_tokens(self):
        """(Re)authenticate the results-database handler and refresh its credentials.

        Prefer 1Password via the backend's authenticate() — the local flow is
        `eval $(op signin)` then `streamlit run`, so it reuses your terminal's 1Password session.
        If that fails (Streamlit Cloud has no `op` CLI), fall back to injected
        st.secrets['database_credentials'].
        """
        try:
            self.database_handler.authenticate()
        except Exception as op_err:
            try:
                creds = dict(st.secrets['database_credentials'])
            except Exception:
                raise RuntimeError(
                    f'Could not authenticate to the results database: 1Password failed '
                    f'({op_err}) and no [database_credentials] secret is configured.'
                ) from op_err
            self.database_handler.load_credentials(creds)

    def __init__(self):
        # Cached in session_state so a Streamlit rerun reuses the authenticated handler
        # instead of re-running the 1Password flow on every interaction.
        self.database_handler = st.session_state.get('database_handler')
        if self.database_handler is None:
            self.database_handler = LidarDatabaseHandler()
            self.refresh_tokens()
            st.session_state.database_handler = self.database_handler
        self.visualization_handler = visualization_handler.VisualizationHandler()

        settings_path = Path(__file__).parent / 'settings.yaml'
        with open(settings_path, 'r') as f:
            self._settings = yaml.safe_load(f)

        thresholds_path = Path(__file__).parent / 'thresholds.yaml'
        with open(thresholds_path, 'r') as f:
            self._thresholds_file = yaml.safe_load(f) or {}
        self._lidar_thresholds = self._thresholds_file.get('lidar_overrides', {})

        if 'env_names' not in st.session_state:
            st.session_state.env_names = []
        if 'metrics_data' not in st.session_state:
            st.session_state.metrics_data = {}
        if 'visualization_data' not in st.session_state:
            st.session_state.visualization_data = {}
        if 'thresholds' not in st.session_state:
            st.session_state.thresholds = self._load_thresholds()
        if 'lidar_thresholds' not in st.session_state:
            st.session_state.lidar_thresholds = self._lidar_thresholds.copy()
        if 'abstract_thresholds' not in st.session_state:
            st.session_state.abstract_thresholds = self._load_abstract_thresholds()
        if 'show_settings' not in st.session_state:
            st.session_state.show_settings = False
        if 'selected_environment' not in st.session_state:
            st.session_state.selected_environment = None

    def _render_sidebar_branding(self):
        st.sidebar.markdown(f'<style>{self._LOGO_CSS}</style>', unsafe_allow_html=True)
        st.sidebar.image(str(self._LOGO_PATH), use_container_width=True)
        st.sidebar.markdown(
            '<span class="polyview-logo">PolyView</span>'
            '<span class="polyview-tagline">LiDAR Evaluation Suite</span>'
            '<hr class="polyview-divider">',
            unsafe_allow_html=True,
        )

    def run(self):
        self._render_sidebar_branding()
        st.title("🔬 PolyView LiDAR Evaluation Suite")

        with st.expander("📖 Instructions & Workflow Guide", expanded=False):
            self.render_welcome_page()

        st.markdown("---")
        st.write("")

        self.render_lidar_refresh_button()
        self.render_environments_view()


    def render_welcome_page(self):
        """
        Render the PolyView introduction and navigation guide.
        This page is independent of loaded data so users understand
        the workflow before retrieving results.
        """

        # Header
        st.title("🔬 PolyView LiDAR Evaluation Suite")

        st.markdown(
            """
            **A unified dashboard for evaluating LiDAR performance, analyzing
            sensor behavior, and comparing hardware across controlled experiments.**
            """
        )

        st.divider()

        # Workflow overview
        st.subheader("🚀 Evaluation Workflow")


        workflow_steps = [
            ("📥", "Retrieve", "Load evaluation results"),
            ("🌎", "Select", "Choose environment & sensor"),
            ("📊", "Analyze", "Review LiDAR metrics"),
            ("🌐", "Visualize", "Inspect point cloud data"),
            ("📈", "Compare", "Benchmark sensors"),
        ]

        cols = st.columns(len(workflow_steps))

        for idx, (col, (icon, title, description)) in enumerate(zip(cols, workflow_steps)):

            with col:
                with st.container(border=True):

                    st.markdown(f"## {icon}")

                    st.markdown(
                        f"""
                        **{idx + 1}. {title}**

                        {description}
                        """
                    )

        st.divider()

        # Getting started
        st.subheader("📖 Getting Started")

        st.markdown(
            """
            PolyView organizes LiDAR evaluation results using the hierarchy:

            ```
            Environment
                └── LiDAR Sensor
                        └── Test Case
                                └── Metrics
            ```

            Follow the workflow below to explore results.
            """
        )

        workflow_details = [
            (
                "📥 1. Retrieve Results",
                """
                Click **Retrieve Results** to fetch the latest evaluation
                results from the database.
                """
            ),
            (
                "🌎 2. Select Environment",
                """
                Select an evaluation environment containing LiDAR sensors
                and associated test cases.
                """
            ),
            (
                "📡 3. Select LiDAR",
                """
                Choose the sensor you want to analyze. The overview page
                provides a summary of its performance.
                """
            ),
            (
                "📊 4. Analyze Metrics",
                """
                Review metrics grouped by evaluation category and spatial zone.
                Metrics are interpreted using configurable quality thresholds.
                """
            ),
            (
                "🌐 5. Explore Visualization",
                """
                Inspect point clouds, expected surfaces, fitted geometry,
                dropout regions, and problematic points.
                """
            ),
            (
                "⚖️ 6. Compare LiDARs",
                """
                Compare multiple sensors or test cases using multi-sensor
                comparison charts, normalized scores, and metric distributions.
                """
            ),
        ]

        for title, description in workflow_details:
            with st.expander(title):
                st.markdown(description)

        st.divider()

        # Dashboard views
        st.subheader("🖥 Dashboard Views")

        view_cards = [
            (
                "📊",
                "Metric Overview",
                "Understand LiDAR performance across evaluation zones.",
                [
                    "Performance by evaluation zone",
                    "Quality bands",
                    "Threshold interpretation",
                    "Percentile distributions",
                ],
            ),
            (
                "🌐",
                "3D Visualization",
                "Inspect spatial behavior and point cloud quality.",
                [
                    "Point clouds",
                    "Expected planes",
                    "PCA fitted surfaces",
                    "Spatial dropout",
                    "Worst points",
                ],
            ),
            (
                "📈",
                "Multi-Sensor Comparison",
                "Benchmark LiDAR sensors using normalized metrics.",
                [
                    "Compare LiDAR sensors",
                    "Category-level scores",
                    "Metric comparisons",
                    "Sensor benchmarking",
                ],
            ),
            (
                "📉",
                "Metric Distribution Analysis",
                "Analyze consistency, spread, and outliers.",
                [
                    "Understand variation",
                    "Identify outliers",
                    "Analyze consistency",
                ],
            ),
        ]

        cols = st.columns(2)

        for idx, (icon, title, subtitle, bullets) in enumerate(view_cards):

            with cols[idx % 2]:

                with st.container(border=True):

                    # Header
                    st.markdown(
                        f"""
                        ## {icon} {title}
                        """
                    )

                    st.caption(subtitle)

                    st.divider()

                    for item in bullets:
                        st.markdown(f"- {item}")

        # Metrics explanation
        st.subheader("📐 Understanding Metrics")

        metric_cols = st.columns(2)

        with metric_cols[0]:
            st.markdown(
                """
                #### Quality Bands

                🟢 **Great**

                Within expected performance.

                🟡 **OK**

                Acceptable performance.

                🔴 **Bad**

                Outside desired range.
                """
            )

        with metric_cols[1]:
            st.markdown(
                """
                #### Metric Direction

                **Lower is better**

                - Error
                - Noise
                - Dropout

                **Higher is better**

                - Point yield
                - Density
                - Coverage
                """
            )

        st.divider()

        # Tips
        st.subheader("💡 Recommended Workflow")

        st.info(
            """
            Start with **Metric Overview** → investigate using **3D Visualization**
            → validate using **Multi-Sensor Comparison Charts**.

            Use **Settings** to adjust metric thresholds when evaluating new
            sensors or different environments.
            """
        )

    @property
    def _env_data(self) -> dict:
        env = st.session_state.get('selected_environment')
        if not env:
            return {}
        return st.session_state.metrics_data.get(env, {})

    # ---- Shared abstract-metric helpers (overview, radar, comparison) ----------

    def _abstract_specs(self) -> list:
        """Flat, ordered list of abstract-metric specs from the schema +
        abstract_thresholds — the single source for iterating abstract metrics."""
        abstract_metrics = self.visualization_handler._schemas.get('abstract_metrics', {})
        abstract_thresholds = st.session_state.abstract_thresholds
        specs = []
        for group_name, group in abstract_metrics.items():
            for entry in group:
                category = entry['category']
                cat_thresh = abstract_thresholds.get(category, {})
                lower_is_better = cat_thresh.get('lower_is_better', True)
                keys_thresh = cat_thresh.get('keys', {})
                for key_entry in entry['keys']:
                    suffix = key_entry['key']
                    specs.append({
                        'group': group_name,
                        'category': category,
                        'suffix': suffix,
                        'label': key_entry.get('label', suffix.lstrip('_').replace('_', ' ').title()),
                        'description': key_entry.get('description', '').strip(),
                        'value_suffix': key_entry.get('value_suffix', '%'),
                        'lower_is_better': lower_is_better,
                        'raw_bands': keys_thresh.get(suffix),
                        'is_frac': suffix.endswith('_frac'),
                    })
        return specs

    @staticmethod
    def _scaled_bands(raw_bands, is_frac: bool):
        """Scale raw threshold bands for display (fractions shown as %)."""
        if not raw_bands:
            return None
        scale = 100.0 if is_frac else 1.0
        bands = {t: {'min': raw_bands[t]['min'] * scale, 'max': raw_bands[t]['max'] * scale}
                 for t in ('great', 'ok', 'bad') if t in raw_bands}
        return bands if all(t in bands for t in ('great', 'ok', 'bad')) else None

    def _zone_keys(self, case_datas: list, specs: list) -> list:
        """Ordered, de-duplicated raw zone keys (e.g. 'green_wall') found across
        the given case data dicts. Title-cased for display elsewhere via
        zone.replace('_', ' ').title(), which matches `_metric_rows` /
        `_metric_percentiles` zone labels and `_score_case_by_zone` keys."""
        zones = []
        for cd in case_datas:
            for spec in specs:
                suffix = spec['suffix']
                for key, val in cd.get(spec['category'], {}).items():
                    if key.endswith(suffix) and isinstance(val, (int, float)):
                        zone = key[: -len(suffix)].rstrip('_')
                        if zone and zone not in zones:
                            zones.append(zone)
        return zones

    @staticmethod
    def _metric_rows(cat_data: dict, spec: dict) -> list:
        """[(zone_label, display_value)] for one metric of one case."""
        suffix, is_frac = spec['suffix'], spec['is_frac']
        rows = []
        for key, val in cat_data.items():
            if key.endswith(suffix) and isinstance(val, (int, float)):
                display_val = val * 100.0 if is_frac else val
                zone_label = key[: -len(suffix)].replace('_', ' ').title() or key
                rows.append((zone_label, display_val))
        return rows

    @staticmethod
    def _metric_percentiles(cat_data: dict, spec: dict) -> dict:
        """{zone_label: {percentile_key: value}} for one metric of one case, or {}
        when the metric has no percentile family."""
        suffix = spec['suffix']
        if not suffix.endswith('_mean'):
            return {}
        stem = suffix[1:-len('_mean')]
        pct_keys = ['min', 'p10', 'p50', 'p90', 'p99', 'max']
        series = {}
        for key, val in cat_data.items():
            if not (key.endswith(suffix) and isinstance(val, (int, float))):
                continue
            zone = key[: -len(suffix)]
            pdict = {pk: float(cat_data[f'{zone}_{stem}_{pk}'])
                     for pk in pct_keys
                     if isinstance(cat_data.get(f'{zone}_{stem}_{pk}'), (int, float))}
            if 'p50' in pdict and len(pdict) >= 2:
                series[zone.replace('_', ' ').title() or zone] = pdict
        return series

    def _score_case_by_zone(self, case_data: dict, specs: list) -> dict:
        """Per-zone normalized scores for one case:
        {zone: {'detail': {metric_label: score}, 'category': {group: mean_score}}}."""
        by_suffix = {s['suffix']: s for s in specs if s['raw_bands']}
        detail: dict = {}
        cat_acc: dict = {}
        for cat_data in case_data.values():
            if not isinstance(cat_data, dict):
                continue
            for metric_key, val in cat_data.items():
                if not isinstance(val, (int, float)):
                    continue
                for suffix, s in sorted(by_suffix.items(), key=lambda kv: -len(kv[0])):
                    if metric_key.endswith(suffix):
                        zone = metric_key[: -len(suffix)].rstrip('_')
                        if not zone:
                            continue
                        score = self.visualization_handler.score_abstract(
                            float(val), s['raw_bands'], s['lower_is_better'])
                        detail.setdefault(zone, {})[s['label']] = score
                        cat_acc.setdefault(zone, {}).setdefault(s['group'].replace('_', ' '), []).append(score)
                        break
        group_order = [g.replace('_', ' ') for g in
                       self.visualization_handler._schemas.get('abstract_metrics', {}).keys()]
        result = {}
        for zone, dmap in detail.items():
            cats = cat_acc.get(zone, {})
            result[zone] = {
                'detail': dmap,
                'category': {g: round(sum(cats[g]) / len(cats[g]), 3) for g in group_order if cats.get(g)},
            }
        return result

    def render_environments_view(self):
        if not st.session_state.env_names:
            st.info('No data loaded. Click "Retrieve Results".')
            return
        st.selectbox(
            'Select Environment',
            options=st.session_state.env_names,
            key='selected_environment',
        )
        env = st.session_state.selected_environment
        if env:
            if env not in st.session_state.metrics_data:
                with st.spinner(f'Loading {env}...'):
                    st.session_state.metrics_data[env] = self.database_handler.retrieve_env_data(env)
            if st.sidebar.button('⚙️ Settings', use_container_width=True, key='settings_btn'):
                st.session_state.show_settings = not st.session_state.get('show_settings', False)
            if st.session_state.get('show_settings'):
                if st.button('← Back to Overview', key='settings_back'):
                    st.session_state.show_settings = False
                    st.rerun()
                self.render_settings_page()
            else:
                self.render_high_level_metrics_overview()

    @staticmethod
    def _cost_tier_label(cost, width: int = 500, cap: int = 4000) -> str | None:
        """Map a lidar cost to a dollar-sign price tier (hiding the actual range): one '$'
        per `width` band — 0–500 → '$', 500–1000 → '$$', … — capped at `cap // width`
        signs. Returns None when cost is missing or non-numeric (so callers can show a
        placeholder)."""
        if not isinstance(cost, (int, float)):
            return None
        max_tiers = max(1, cap // width)
        tiers = min(int(cost // width) + 1, max_tiers)
        return r'\$' * tiers

    def render_high_level_metrics_overview(self):
        if st.session_state.get('explore_lidar'):
            self.render_explore_further_button()
            return

        self.render_radar_view_button()
        if st.session_state.get('show_radar_view'):
            return

        lidars = list(self._env_data.keys())
        if not lidars:
            st.info('No LiDAR data available.')
            return

        if 'lidar_slide_idx' not in st.session_state:
            st.session_state.lidar_slide_idx = 0
        idx = max(0, min(st.session_state.lidar_slide_idx, len(lidars) - 1))

        lidar_name = st.selectbox(
            'LiDAR',
            options=lidars,
            index=idx,
            key='lidar_overview_select',
        )
        idx = lidars.index(lidar_name)
        st.session_state.lidar_slide_idx = idx
        st.caption(f'{idx + 1} / {len(lidars)}')

        col_explore, col_radar, col_info = st.columns(3)
        with col_explore:
            if st.button('🔍 Explore Visualization/Lidar Metrics Comparison', use_container_width=True, key='open_explore_further'):
                st.session_state.explore_lidar = lidar_name
                for i in range(4):
                    st.session_state.pop(f'explore_case_depth_{i}', None)
                st.rerun()
        with col_radar:
            if st.button('Multi-Sensor Comparison', use_container_width=True, key='open_radar_view'):
                st.session_state.show_radar_view = True
                st.rerun()
        with col_info:
            if st.button('ℹ️ Lidar Info', use_container_width=True, key='toggle_lidar_info'):
                st.session_state.show_lidar_info = not st.session_state.get('show_lidar_info', False)
        base_data = self._env_data.get(lidar_name, {}).get('base', {})

        if st.session_state.get('show_lidar_info'):
            meta = base_data.get('lidar_metadata', {})
            h = meta.get('lidar_horizontal_fov_deg')
            v = meta.get('lidar_vertical_fov_deg')
            tier = self._cost_tier_label(meta.get('lidar_cost'))
            st.markdown(
                f'<div style="font-size:20px; font-weight:800; color:#FFFFFF; '
                f'letter-spacing:0.05em; text-transform:uppercase; margin:14px 0 8px; '
                f'padding:9px 16px; border-left:5px solid #5138EE; border-radius:6px; '
                f'background:linear-gradient(90deg, rgba(81,56,238,0.32), rgba(81,56,238,0.0));">'
                f'{lidar_name} · Lidar Info</div>',
                unsafe_allow_html=True,
            )
            c1, c2, c3 = st.columns(3)
            c1.metric('Horizontal FOV', f'{h:.1f}°' if isinstance(h, (int, float)) else '—')
            c2.metric('Vertical FOV', f'{v:.1f}°' if isinstance(v, (int, float)) else '—')
            c3.metric('Cost tier', tier if tier else '—')

        specs = self._abstract_specs()
        zones = self._zone_keys([base_data], specs)
        if not zones:
            st.info('No metrics available for this LiDAR.')
            return

        # One tab per zone; each tab shows that zone's metrics across all groups.
        for zone, ztab in zip(zones, st.tabs([z.replace('_', ' ').title() for z in zones])):
            zone_title = zone.replace('_', ' ').title()
            with ztab:
                current_group = None
                for spec in specs:
                    cat_data = base_data.get(spec['category'], {})
                    rows = [(zl, v) for zl, v in self._metric_rows(cat_data, spec) if zl == zone_title]
                    if not rows:
                        continue
                    if spec['group'] != current_group:
                        if current_group is not None:
                            st.divider()
                        current_group = spec['group']
                        st.markdown(
                            f'<div style="font-size:24px; font-weight:800; color:#FFFFFF; '
                            f'letter-spacing:0.06em; text-transform:uppercase; '
                            f'margin:22px 0 10px; padding:10px 18px; '
                            f'border-left:5px solid #5138EE; border-radius:6px; '
                            f'background:linear-gradient(90deg, rgba(81,56,238,0.32), rgba(81,56,238,0.0));">'
                            f'{current_group.replace("_", " ")}</div>',
                            unsafe_allow_html=True,
                        )
                    bands = self._scaled_bands(spec['raw_bands'], spec['is_frac'])
                    if spec['description']:
                        st.markdown(
                            f'<div style="font-size:15px; color:rgba(255,255,255,0.82); '
                            f'line-height:1.55; margin:30px 0 16px;">{spec["description"]}</div>',
                            unsafe_allow_html=True,
                        )
                    fig = self.visualization_handler.make_bullet_figure(
                        title=spec['label'], rows=rows, bands=bands,
                        lower_is_better=spec['lower_is_better'], value_suffix=spec['value_suffix'],
                    )
                    st.plotly_chart(fig, width='stretch', key=f"bullet_{lidar_name}_{zone}_{spec['category']}_{spec['suffix']}")

                    # Explore-more: percentile distribution (metrics with a p50/p90/p99 family).
                    series = {zl: pd for zl, pd in self._metric_percentiles(cat_data, spec).items() if zl == zone_title}
                    if series:
                        tkey = f"dist_{lidar_name}_{zone}_{spec['category']}_{spec['suffix']}"
                        _, col_btn = st.columns([5, 2])
                        with col_btn:
                            if st.button('📊 Explore distribution', key=f'btn_{tkey}', use_container_width=True):
                                st.session_state[tkey] = not st.session_state.get(tkey, False)
                        if st.session_state.get(tkey, False):
                            dist_fig = self.visualization_handler.make_percentile_distribution(
                                spec['label'], series, spec['value_suffix'])
                            st.plotly_chart(dist_fig, width='stretch', key=f'chart_{tkey}')
                if current_group is not None:
                    st.divider()

    def render_explore_further_button(self, lidar_name: str = ''):
        if 'explore_lidar' not in st.session_state:
            st.session_state.explore_lidar = None

        if not st.session_state.explore_lidar:
            _, col_btn, _ = st.columns([3, 4, 3])
            with col_btn:
                if st.button('🔍 Explore Visualization/Lidar Metrics Comparison', use_container_width=True, key='open_explore_further'):
                    st.session_state.exploVre_lidar = lidar_name
                    for i in range(4):
                        st.session_state.pop(f'explore_case_depth_{i}', None)
                    st.rerun()
            return

        col_back, _ = st.columns([2, 8])
        with col_back:
            if st.button('← Back to Overview', key='explore_back'):
                st.session_state.explore_lidar = None
                st.session_state.pop('explore_display', None)
                st.session_state.pop('explore_view', None)
                st.rerun()

        self.selected_lidar = st.session_state.explore_lidar
        lidar_data = self._env_data.get(self.selected_lidar, {})

        st.markdown(
            f'<div style="font-size:30px; font-weight:800; color:#FFFFFF; '
            f'letter-spacing:0.04em; margin:18px 0 14px; padding:12px 20px; '
            f'border-left:6px solid #5138EE; border-radius:8px; '
            f'background:linear-gradient(90deg, rgba(81,56,238,0.35), rgba(81,56,238,0.0));">'
            f'{self.selected_lidar}</div>',
            unsafe_allow_html=True,
        )
        self._render_case_selector(lidar_data)

        _, col_display, _ = st.columns([3, 4, 3])
        with col_display:
            if st.button('Display', use_container_width=True, key='explore_display_btn'):
                st.session_state.explore_display = True
                st.session_state.pop('explore_view', None)
                env = st.session_state.get('selected_environment')
                case_path = st.session_state.get('explore_case_path', '')
                if env and case_path:
                    with st.spinner('Fetching visualization data...'):
                        viz = self.database_handler.retrieve_visualization_data(env, self.selected_lidar, case_path)
                        roi = viz.get('roi_cloud') if viz else None
                        print(
                            f'[PolyView] viz fetch env={env} lidar={self.selected_lidar} case={case_path} | '
                            f'keys={list(viz.keys()) if viz else None} | '
                            f"roi_cloud={'None' if roi is None else getattr(roi, 'shape', type(roi).__name__)} | "
                            f"fitted_planes={len(viz.get('fitted_planes', {})) if viz else 0} | "
                            f"profile_plane={bool(viz.get('profile_plane')) if viz else False} | "
                            f"orientation={bool(viz.get('orientation')) if viz else False}"
                        )
                        if viz:
                            st.session_state.visualization_data[self.selected_lidar] = viz

        if not st.session_state.get('explore_display'):
            return

        view_mode = st.session_state.get('explore_view')

        if view_mode is None:
            st.subheader('Choose a view')
            view_cards = [
                ('3D Visualization', '🌐', 'ROI cloud, expected planes, and PCA fit in 3D.'),
                ('Lidar Metrics Comparison', '📊', 'Side-by-side comparison against another LiDAR/case.'),
            ]
            cols = st.columns(len(view_cards))
            for col, (name, icon, desc) in zip(cols, view_cards):
                with col:
                    if st.button(f'{icon}  {name}', use_container_width=True, key=f'view_card_{name}'):
                        st.session_state.explore_view = name
                        st.rerun()
                    st.caption(desc)
            return

        if st.button('← Back to Views', key='view_back'):
            st.session_state.pop('explore_view', None)
            st.rerun()

        if view_mode == '3D Visualization':
            self.render_3d_view()
        elif view_mode == 'Lidar Metrics Comparison':
            self.render_comparison_graphs()

    def _render_case_selector(self, data: dict, depth: int = 0, path: list | None = None, state_prefix: str = 'explore') -> None:
        if path is None:
            path = []
        if self._is_metrics_leaf(data) or not data or depth > 3:
            st.session_state[f'{state_prefix}_case_path'] = '/'.join(str(p) for p in path)
            return
        options = list(data.keys())
        labels = [str(k).replace('_', ' ').title() if isinstance(k, str) else str(k) for k in options]
        label_text = ['Condition', 'Sub-case', 'Parameter', 'Value'][min(depth, 3)]
        default_index = labels.index('Base') if depth == 0 and 'Base' in labels else 0
        selected_label = st.selectbox(label_text, options=labels, index=default_index, key=f'{state_prefix}_case_depth_{depth}')
        selected_key = options[labels.index(selected_label)]
        self._render_case_selector(data[selected_key], depth + 1, path + [str(selected_key)], state_prefix)

    def _resolve_case_data(self, lidar_data: dict, state_prefix: str = 'explore') -> dict:
        data = lidar_data
        for depth in range(4):
            if self._is_metrics_leaf(data) or not data:
                return data
            options = list(data.keys())
            labels = [str(k).replace('_', ' ').title() if isinstance(k, str) else str(k) for k in options]
            selected_label = st.session_state.get(f'{state_prefix}_case_depth_{depth}')
            if selected_label not in labels:
                return {}
            selected_key = options[labels.index(selected_label)]
            data = data[selected_key]
        return data

    @staticmethod
    def _is_metrics_leaf(data) -> bool:
        if not isinstance(data, dict) or not data:
            return False
        first_val = next(iter(data.values()))
        if not isinstance(first_val, dict):
            return False
        first_inner = next(iter(first_val.values()), None)
        return isinstance(first_inner, (int, float))

    @property
    def _layer_adders(self):
        """Single source of truth for the 3D layers: (display name, draw fn), in draw
        order. Both render_3d_view and the visible-layers checkbox panel derive from
        this, so a layer can't draw without a toggle, nor a toggle control nothing."""
        vh = self.visualization_handler
        return [
            ('PointCloud', vh.add_point_cloud),
            ('Expected Planes', vh.add_expected_planes),
            ('Cropped Expected', vh.add_cropped_expected_planes),
            ('Fitted PCA Plane', vh.add_fitted_pca_plane),
            ('Spatial Dropout Analysis', vh.add_spatial_dropout_analysis),
            ('Worst Points', vh.add_worst_points),
        ]

    def render_3d_view(self):
        if not st.session_state.visualization_data:
            st.info('No visualization data loaded. Click "Retrieve Results".')
            return
        self.render_3d_button_panel()
        st.markdown(self.visualization_handler.glow_css, unsafe_allow_html=True)
        viz_data = st.session_state.visualization_data.get(self.selected_lidar, {})
        # Pull per-zone ROI padding from the case metrics so the cropped-expected
        # layer can inset the expected zone by the y/z padding used during filtering.
        case_metrics = self._resolve_case_data(self._env_data.get(self.selected_lidar, {}))
        pad_src = case_metrics.get('SpatialDropout') or case_metrics.get('AverageSpatialDropout') or {}
        padding_by_zone: dict = {}
        for key, val in pad_src.items():
            if not isinstance(val, (int, float)):
                continue
            if key.endswith('_y_padding_m'):
                padding_by_zone.setdefault(key[: -len('_y_padding_m')], {})['y'] = float(val)
            elif key.endswith('_z_padding_m'):
                padding_by_zone.setdefault(key[: -len('_z_padding_m')], {})['z'] = float(val)
        viz_data = {**viz_data, 'expected_padding': padding_by_zone}
        axes_viz_data = viz_data
        case_path = st.session_state.get('explore_case_path', '')
        if case_path and 'angle' in case_path.lower():
            try:
                angle_deg = float(case_path.split('/')[-1])
                orientation = {**viz_data.get('orientation', {}), 'yaw': math.radians(-angle_deg)}
                axes_viz_data = {**viz_data, 'orientation': orientation}
            except ValueError:
                pass
        layers = st.session_state.visible_layers
        fig = self.visualization_handler.make_3d_figure(viz_data)

        layer_adders = [
            ('PointCloud', self.visualization_handler.add_point_cloud, viz_data),
            ('Expected Planes', self.visualization_handler.add_expected_planes, viz_data),
            ('Cropped Expected', self.visualization_handler.add_cropped_expected_planes, viz_data),
            ('Fitted PCA Plane', self.visualization_handler.add_fitted_pca_plane, viz_data),
            ('Spatial Dropout Analysis', self.visualization_handler.add_spatial_dropout_analysis, viz_data),
            ('Worst Points', self.visualization_handler.add_worst_points, viz_data),
        ]
        layer_trace_ranges: list[tuple[str, int, int]] = []
        for name, adder, data in layer_adders:
            start = len(fig.data)
            adder(fig, data)
            layer_trace_ranges.append((name, start, len(fig.data)))

        self.visualization_handler.add_sensor_axes(fig, axes_viz_data)

        for name, start, end in layer_trace_ranges:
            visible = name in layers
            for i in range(start, end):
                fig.data[i].visible = visible

        self.visualization_handler.fit_scene_to_origin(fig)

        revision = f'3d_{self.selected_lidar}_{case_path}'
        fig.update_layout(
            uirevision=revision,
            scene_uirevision=revision,
            transition={'duration': 0},
        )
        st.plotly_chart(fig, width='stretch', key='3d_scene_chart')

        self._render_bag_download(case_path)


    def _render_bag_download(self, case_path: str) -> None:
        """Surfaces a direct, click-to-download link for the case's rosbag zip when one exists
        on Drive (shared 'anyone with link' at upload time). Silent when there's no bag."""
        env = st.session_state.get('selected_environment', '')
        if not (env and self.selected_lidar and case_path):
            return
        try:
            link = self.database_handler.retrieve_bag_download_link(env, self.selected_lidar, case_path)
        except Exception:
            link = None
        if not link:
            return
        st.markdown(f'📦 **Rosbag:** [Download recording (.zip)]({link})')
        st.caption('Large bags show a one-time Google scan warning — click "Download anyway".')


    def render_radar_view_button(self):
        if 'show_radar_view' not in st.session_state:
            st.session_state.show_radar_view = False

        if st.session_state.show_radar_view:
            if st.button('← Back to Overview', key='radar_back'):
                st.session_state.show_radar_view = False
                st.rerun()

            with st.expander('⚙️ Radar Thresholds', expanded=False):
                abstract_thresholds = st.session_state.abstract_thresholds
                updated = {cat: dict(cfg) for cat, cfg in abstract_thresholds.items()}
                for category, cat_config in abstract_thresholds.items():
                    st.markdown(f'**{category}**')
                    keys_config = cat_config.get('keys', {})
                    updated_keys = {}
                    for suffix, bands in keys_config.items():
                        label = suffix.lstrip('_').replace('_', ' ').title()
                        st.caption(label)
                        col_great, col_ok, col_bad = st.columns(3)
                        great_max = col_great.number_input('Great (max)', value=float(bands['great']['max']), step=0.001, format='%.4f', key=f'thresh_{category}_{suffix}_great')
                        ok_max = col_ok.number_input('Ok (max)', value=float(bands['ok']['max']), step=0.001, format='%.4f', key=f'thresh_{category}_{suffix}_ok')
                        bad_max = col_bad.number_input('Bad (max)', value=float(bands['bad']['max']), step=0.001, format='%.4f', key=f'thresh_{category}_{suffix}_bad')
                        updated_keys[suffix] = {
                            'great': {'min': bands['great']['min'], 'max': great_max},
                            'ok':    {'min': great_max,             'max': ok_max},
                            'bad':   {'min': ok_max,                'max': bad_max},
                        }
                    updated[category] = {**cat_config, 'keys': updated_keys}
                if st.button('Apply', key='apply_abstract_thresholds'):
                    st.session_state.abstract_thresholds = updated
                    st.rerun()

            specs = self._abstract_specs()
            zone_detail: dict = {}   # zone -> lidar -> {metric_label: score}
            zone_cat: dict = {}      # zone -> lidar -> {group: mean_score}
            for lidar, lidar_info in self._env_data.items():
                for zone, z in self._score_case_by_zone(lidar_info.get('base', {}), specs).items():
                    zone_detail.setdefault(zone, {})[lidar] = z['detail']
                    zone_cat.setdefault(zone, {})[lidar] = z['category']

            for zone, lidar_scores in zone_detail.items():
                cat_lidar_scores = zone_cat.get(zone, {})
                if cat_lidar_scores:
                    tri = self.visualization_handler.make_abstract_radar_figure(
                        f'{zone} · Category Scores', cat_lidar_scores
                    )
                    st.plotly_chart(tri, use_container_width=True, key=f'category_radar_{zone}')
                fig = self.visualization_handler.make_abstract_radar_figure(zone, lidar_scores)
                st.plotly_chart(fig, use_container_width=True, key=f'abstract_radar_{zone}')


    def render_comparison_graphs(self):
        st.markdown(self.visualization_handler.glow_css, unsafe_allow_html=True)
        all_lidars = list(self._env_data.keys())
        if not all_lidars:
            st.info('No LiDAR data available.')
            return

        case_a_lidar = self.selected_lidar or all_lidars[0]
        case_a_data = self._resolve_case_data(self._env_data.get(case_a_lidar, {}))
        case_a_path = st.session_state.get('explore_case_path', 'base') or 'base'
        case_a_label = f'{case_a_lidar} · {case_a_path.replace("/", " · ").title()}'

        st.markdown(f'**Case 1:** `{case_a_label}`')
        st.markdown('**Case 2:**')

        lidar_b = st.selectbox('LiDAR', options=all_lidars, key='cmp_b_lidar')
        self._render_case_selector(self._env_data.get(lidar_b, {}), state_prefix='cmp_b')
        case_b_data = self._resolve_case_data(self._env_data.get(lidar_b, {}), state_prefix='cmp_b')
        case_b_path = st.session_state.get('cmp_b_case_path', 'base') or 'base'
        case_b_label = f'{lidar_b} · {case_b_path.replace("/", " · ").title()}'

        if not case_a_data or not case_b_data:
            st.info('Select a valid case to compare.')
            return

        tag_a, tag_b = 'Case 1', 'Case 2'

        def _case_chip(tag, label, accent):
            return (
                f'<div style="flex:1; min-width:240px; padding:14px 20px; border-radius:10px; '
                f'border-left:6px solid {accent}; '
                f'background:linear-gradient(90deg, {accent}33, {accent}0d);">'
                f'<div style="font-size:13px; font-weight:700; letter-spacing:0.12em; '
                f'text-transform:uppercase; color:{accent}; margin-bottom:4px;">{tag}</div>'
                f'<div style="font-size:22px; font-weight:800; color:#FFFFFF;">{label}</div>'
                f'</div>'
            )

        st.markdown(
            f'<div style="display:flex; gap:16px; flex-wrap:wrap; margin:16px 0 8px;">'
            f'{_case_chip(tag_a, case_a_label, self.CASE_COLORS[0])}'
            f'{_case_chip(tag_b, case_b_label, self.CASE_COLORS[1])}'
            f'</div>',
            unsafe_allow_html=True,
        )

        specs = self._abstract_specs()

        def _group_heading(text):
            st.markdown(
                f'<div style="font-size:22px; font-weight:800; color:#FFFFFF; '
                f'letter-spacing:0.05em; text-transform:uppercase; margin:20px 0 8px; '
                f'padding:9px 16px; border-left:5px solid #5138EE; border-radius:6px; '
                f'background:linear-gradient(90deg, rgba(81,56,238,0.32), rgba(81,56,238,0.0));">'
                f'{text}</div>',
                unsafe_allow_html=True,
            )

        score_a = self._score_case_by_zone(case_a_data, specs)
        score_b = self._score_case_by_zone(case_b_data, specs)
        zones = self._zone_keys([case_a_data, case_b_data], specs)
        if not zones:
            st.info('No metrics available to compare.')
            return

        # Zone is the tab, so rows are labeled by LiDAR (bold + accent color so the
        # two rows stay distinguishable) followed by the dimmer case name, one line.
        # make_bullet_figure sizes its left margin to the longest label.
        def _row_label(lidar, case_part, accent):
            return (f'<span style="font-size:15px;font-weight:800;color:{accent};">{lidar}</span>'
                    f'<span style="font-size:13px;color:rgba(255,255,255,0.7);"> · {case_part}</span>')

        case_a_part = case_a_path.replace('/', ' · ').title()
        case_b_part = case_b_path.replace('/', ' · ').title()

        # One tab per zone; each tab shows that zone's radars, bullets, distributions.
        for zone, ztab in zip(zones, st.tabs([z.replace('_', ' ').title() for z in zones])):
            zone_title = zone.replace('_', ' ').title()
            with ztab:
                # ---- Radars: category triangle + detailed radar, both cases ------
                _group_heading('Multi-Sensor Comparison Charts')
                detail, cat, radar_colors = {}, {}, []
                if zone in score_a:
                    detail[case_a_label] = score_a[zone]['detail']
                    cat[case_a_label] = score_a[zone]['category']
                    radar_colors.append(self.CASE_COLORS[0])
                if zone in score_b:
                    detail[case_b_label] = score_b[zone]['detail']
                    cat[case_b_label] = score_b[zone]['category']
                    radar_colors.append(self.CASE_COLORS[1])
                if cat:
                    tri = self.visualization_handler.make_abstract_radar_figure(f'{zone_title} · Category Scores', cat, radar_colors)
                    st.plotly_chart(tri, use_container_width=True, key=f'cmp_category_radar_{zone}')
                if detail:
                    rad = self.visualization_handler.make_abstract_radar_figure(zone_title, detail, radar_colors)
                    st.plotly_chart(rad, use_container_width=True, key=f'cmp_abstract_radar_{zone}')

                # ---- Abstract metric bars, both cases for this zone -------------
                st.divider()
                current_group = None
                for spec in specs:
                    ra = dict(self._metric_rows(case_a_data.get(spec['category'], {}), spec))
                    rb = dict(self._metric_rows(case_b_data.get(spec['category'], {}), spec))
                    va, vb = ra.get(zone_title), rb.get(zone_title)
                    if va is None and vb is None:
                        continue
                    if spec['group'] != current_group:
                        current_group = spec['group']
                        _group_heading(current_group.replace('_', ' '))
                    rows = []
                    if va is not None:
                        rows.append((_row_label(case_a_lidar, case_a_part, self.CASE_COLORS[0]), va))
                    if vb is not None:
                        rows.append((_row_label(lidar_b, case_b_part, self.CASE_COLORS[1]), vb))
                    fig = self.visualization_handler.make_bullet_figure(
                        title=spec['label'], rows=rows,
                        bands=self._scaled_bands(spec['raw_bands'], spec['is_frac']),
                        lower_is_better=spec['lower_is_better'], value_suffix=spec['value_suffix'],
                    )
                    st.plotly_chart(fig, width='stretch', key=f"cmp_bullet_{zone}_{spec['category']}_{spec['suffix']}")

                # ---- Percentile box plots (both cases overlaid) ----------------
                st.divider()
                _group_heading('Distributions')
                for spec in specs:
                    sa = self._metric_percentiles(case_a_data.get(spec['category'], {}), spec)
                    sb = self._metric_percentiles(case_b_data.get(spec['category'], {}), spec)
                    pa, pb = sa.get(zone_title), sb.get(zone_title)
                    if pa is None and pb is None:
                        continue
                    series = {}
                    if pa is not None:
                        series[f'{tag_a} · {case_a_label}'] = pa
                    if pb is not None:
                        series[f'{tag_b} · {case_b_label}'] = pb
                    dist_fig = self.visualization_handler.make_percentile_distribution(
                        spec['label'], series, spec['value_suffix'])
                    st.plotly_chart(dist_fig, width='stretch', key=f"cmp_dist_{zone}_{spec['category']}_{spec['suffix']}")

    def _resolve_thresholds(self, lidar_name: str) -> dict:
        global_thresholds = st.session_state.get('thresholds', {})
        per_lidar = st.session_state.get('lidar_thresholds', {}).get(lidar_name, {})
        if not per_lidar:
            return global_thresholds
        return {**global_thresholds, **per_lidar}

    def render_lidar_metrics(self):
        metrics = self._env_data.get(self.selected_lidar, {})
        if not metrics:
            return
        st.subheader('LiDAR Metrics')
        main_figs, bottom_figs = self.visualization_handler.render_single_lidar_metrics(
            self.selected_lidar, metrics, self._resolve_thresholds(self.selected_lidar),
            self._settings.get('secondary_axis_keys', []),
            self._settings.get('plot_y_padding', 0.3),
            self._settings.get('split_by_suffix_categories', []),
            self._settings.get('split_exclude_suffixes', {}),
            self._settings.get('category_skip_key_suffixes', {}),
        )
        for category, fig in main_figs + bottom_figs:
            st.plotly_chart(fig, use_container_width=True, key=f'metrics_{self.selected_lidar}_{category}')

    def render_lidar_refresh_button(self):
        if st.button("Retrieve Results"):
            with st.spinner("Fetching latest test results..."):
                self.retrieve_results_data()
                time.sleep(1)  # Simulate loading time
            st.success("Data refreshed!")

    def _load_thresholds(self) -> dict:
        defaults = self._thresholds_file.get('thresholds', {}) or {}
        user_overrides = self._settings.get('thresholds', {}) or {}
        return {**defaults, **user_overrides}

    def _load_abstract_thresholds(self) -> dict:
        """Abstract thresholds from thresholds.yaml, with any per-category overrides
        saved in settings.yaml taking precedence (the Settings page writes there,
        so thresholds.yaml stays the documented baseline)."""
        base = self._thresholds_file.get('abstract_thresholds', {}) or {}
        override = self._settings.get('abstract_thresholds', {}) or {}
        return {**base, **override}

    def _save_abstract_thresholds(self, updated: dict) -> None:
        self._settings['abstract_thresholds'] = updated
        settings_path = Path(__file__).parent / 'settings.yaml'
        with open(settings_path, 'w') as f:
            yaml.dump(self._settings, f, allow_unicode=True)

    def render_settings_page(self):
        st.title('⚙️ Metric Thresholds')
        st.markdown(
            'Configure the **great / ok / bad** bands used for the metric bullets and radar scoring. '
            'Each metric uses a single great, ok, and bad band; values are in the metric\'s native '
            'units (% , meters, or fraction). Saved to `settings.yaml`.'
        )
        abstract_thresholds = st.session_state.abstract_thresholds
        if not abstract_thresholds:
            st.info('No metric thresholds defined in thresholds.yaml.')
            return

        band_names = ('great', 'ok', 'bad')
        # Friendly per-metric labels from the schema (e.g. "Mean Depth Error (%) · pooled").
        spec_labels = {(s['category'], s['suffix']): s['label'] for s in self._abstract_specs()}

        def _spaced(name: str) -> str:
            # Split CamelCase category names into words: ZoneSurfaceDepthError -> Zone Surface Depth Error
            return ''.join(f' {c}' if c.isupper() and i else c for i, c in enumerate(name)).strip()

        with st.form('abstract_thresholds_form'):
            updated: dict = {}
            for category, cat_config in abstract_thresholds.items():
                # Plain sans-serif heading — avoids the global h3 style (uppercase,
                # letter-spaced monospace) that makes the names hard to read.
                st.markdown(
                    f'<div style="font-family:sans-serif !important; font-size:21px; '
                    f'font-weight:700; color:#FFFFFF; letter-spacing:normal; '
                    f'text-transform:none; margin:20px 0 6px;">{_spaced(category)}</div>',
                    unsafe_allow_html=True,
                )
                lower_is_better = st.checkbox(
                    'Lower is better',
                    value=bool(cat_config.get('lower_is_better', True)),
                    key=f'ab_{category}_lib',
                    help='When on, smaller values score as "great". Turn off for metrics where higher is better.',
                )
                header = st.columns([2, 1, 1, 1, 1, 1, 1])
                for col, txt in zip(header, ['Metric', 'Great min', 'Great max', 'OK min', 'OK max', 'Bad min', 'Bad max']):
                    col.markdown(f'**{txt}**')

                updated_keys: dict = {}
                for suffix, bands in cat_config.get('keys', {}).items():
                    label = spec_labels.get((category, suffix)) or suffix.lstrip('_').replace('_', ' ').title()
                    cols = st.columns([2, 1, 1, 1, 1, 1, 1])
                    cols[0].markdown(
                        f'<div style="font-family:sans-serif !important; font-size:14px; '
                        f'color:rgba(255,255,255,0.9); text-transform:none; letter-spacing:normal; '
                        f'padding-top:6px;">{label}</div>',
                        unsafe_allow_html=True,
                    )
                    new_bands: dict = {}
                    for i, band in enumerate(band_names):
                        b = bands.get(band, {})
                        bmin = cols[1 + i * 2].number_input(
                            f'{band} min', value=float(b.get('min', 0.0)), step=0.001, format='%g',
                            label_visibility='collapsed', key=f'ab_{category}_{suffix}_{band}_min')
                        bmax = cols[2 + i * 2].number_input(
                            f'{band} max', value=float(b.get('max', 0.0)), step=0.001, format='%g',
                            label_visibility='collapsed', key=f'ab_{category}_{suffix}_{band}_max')
                        new_bands[band] = {'min': bmin, 'max': bmax}
                    updated_keys[suffix] = new_bands
                updated[category] = {**cat_config, 'lower_is_better': lower_is_better, 'keys': updated_keys}
                st.divider()

            if st.form_submit_button('💾 Save Thresholds', use_container_width=True):
                st.session_state.abstract_thresholds = updated
                self._save_abstract_thresholds(updated)
                st.success('Thresholds saved!')

    def render_3d_button_panel(self):
        all_layers = ['PointCloud', 'Expected Planes', 'Cropped Expected', 'Fitted PCA Plane', 'Spatial Dropout Analysis', 'Worst Points']
        if 'visible_layers' not in st.session_state:
            st.session_state.visible_layers = list(all_layers)
        st.markdown('**Visible Layers**')
        cols = st.columns(len(all_layers))
        selected = []
        for col, layer in zip(cols, all_layers):
            checked = col.checkbox(
                layer,
                value=layer in st.session_state.visible_layers,
                key=f'layer_toggle_{layer}',
            )
            if checked:
                selected.append(layer)
        st.session_state.visible_layers = selected

    def retrieve_results_data(self):
        self.database_handler.clear_cache()
        st.session_state.env_names = self.database_handler.retrieve_environments()
        st.session_state.metrics_data = {}
        st.session_state.visualization_data = {}




def main():
    app = PolyViewApp()
    app.run()

if __name__ == "__main__":
    main()
