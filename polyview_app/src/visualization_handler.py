import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import yaml
from plotly.subplots import make_subplots

_VIZ_PATH = Path(__file__).parent / 'visualization.yaml'
_CSS_PATH = Path(__file__).parent / 'css' / 'charts.css'


class VisualizationHandler:

    # Shared plotly modebar styling. Was repeated inline at every call site.
    _MODEBAR = dict(bgcolor='rgba(0,0,0,0)', color='rgba(255,255,255,0.2)',
                    activecolor='#5138EE')

    def __init__(self):
        viz = yaml.safe_load(_VIZ_PATH.read_text())
        self._config = viz['config']
        self._schemas = viz['schemas']
        self.glow_css = f'<style>{_CSS_PATH.read_text()}</style>'

    def make_3d_figure(self, viz_data: dict) -> go.Figure:
        layout = self._config['layout']
        fig = go.Figure()
        fig.update_layout(
            scene=dict(
                xaxis=dict(title='X (m)', backgroundcolor='rgba(0,0,0,0)', gridcolor='rgba(255,255,255,0.08)', color='rgba(255,255,255,0.8)'),
                yaxis=dict(title='Y (m)', backgroundcolor='rgba(0,0,0,0)', gridcolor='rgba(255,255,255,0.08)', color='rgba(255,255,255,0.8)'),
                zaxis=dict(title='Z (m)', backgroundcolor='rgba(0,0,0,0)', gridcolor='rgba(255,255,255,0.08)', color='rgba(255,255,255,0.8)'),
                bgcolor='rgba(8,10,18,1.0)',
                aspectmode='data',
            ),
            paper_bgcolor=layout['paper_color'],
            height=720,
            legend=dict(
                font=dict(color='white', size=11),
                bgcolor='rgba(10,12,20,0.9)',
                bordercolor='rgba(81,56,238,0.35)',
                borderwidth=1,
                x=0.01, y=0.99,
                xanchor='left', yanchor='top',
            ),
            margin=dict(t=50, b=20, l=20, r=120),
            title=dict(text='<b>3D LiDAR Scene</b>', font=dict(color='white', size=16), x=0.5),
            modebar=self._MODEBAR,
        )
        return fig

    def add_point_cloud(self, fig: go.Figure, viz_data: dict) -> None:
        R, _ = self._parse_orientation(viz_data)
        cloud = viz_data.get('roi_cloud')
        if cloud is None:
            cloud = viz_data.get('filtered_roi_cloud')
        if cloud is None or len(cloud) == 0:
            return
        cloud_xyz = (R @ cloud[:, :3].T).T
        fig.add_trace(go.Scatter3d(
            x=cloud_xyz[:, 0], y=cloud_xyz[:, 1], z=cloud_xyz[:, 2],
            mode='markers',
            marker=dict(
                size=2,
                color=cloud[:, 3],
                colorscale=[[0, '#00BFFF'], [0.5, '#39FF14'], [1, '#FFD700']],
                opacity=0.85,
                colorbar=dict(title='Intensity', thickness=10, len=0.45, x=1.02),
            ),
            name='ROI Cloud',
            hovertemplate='x: %{x:.3f}  y: %{y:.3f}  z: %{z:.3f}<extra>ROI Cloud</extra>',
        ))

    def add_expected_planes(self, fig: go.Figure, viz_data: dict) -> None:
        fp = viz_data.get('fitted_planes', {})
        if not fp:
            return
        zones = [k.replace('_expected_x', '') for k in fp if k.endswith('_expected_x')]
        if not zones:
            return
        colors = ['#00BFFF', '#39FF14']
        for idx, zone in enumerate(zones):
            x_ref = float(fp.get(f'{zone}_expected_x', 0.0))
            y_min = float(fp.get(f'{zone}_expected_y_min', -1.0))
            y_max = float(fp.get(f'{zone}_expected_y_max', 1.0))
            z_min = float(fp.get(f'{zone}_expected_z_min', 0.0))
            z_max = float(fp.get(f'{zone}_expected_z_max', 2.0))
            color = colors[idx % len(colors)]
            # close the loop: bottom-left → bottom-right → top-right → top-left → bottom-left
            fig.add_trace(go.Scatter3d(
                x=[x_ref, x_ref, x_ref, x_ref, x_ref],
                y=[y_min, y_max, y_max, y_min, y_min],
                z=[z_min, z_min, z_max, z_max, z_min],
                mode='lines',
                line=dict(color=color, width=4),
                name=f'Expected Plane ({zone})',
                hovertemplate=f'Expected plane · {zone}<extra></extra>',
            ))

    def add_cropped_expected_planes(self, fig: go.Figure, viz_data: dict) -> None:
        """Draws the expected zone after insetting it by the per-zone y/z ROI
        padding — i.e. the region actually scored once the padding ring is
        cropped out. Overlays the solid expected plane as a dashed inset."""
        fp = viz_data.get('fitted_planes', {})
        padding = viz_data.get('expected_padding', {})
        if not fp:
            return
        zones = [k.replace('_expected_x', '') for k in fp if k.endswith('_expected_x')]
        for zone in zones:
            pad = padding.get(zone, {})
            y_pad = float(pad.get('y', 0.0))
            z_pad = float(pad.get('z', 0.0))
            x_ref = float(fp.get(f'{zone}_expected_x', 0.0))
            y_min = float(fp.get(f'{zone}_expected_y_min', -1.0)) + y_pad
            y_max = float(fp.get(f'{zone}_expected_y_max', 1.0)) - y_pad
            z_min = float(fp.get(f'{zone}_expected_z_min', 0.0)) + z_pad
            z_max = float(fp.get(f'{zone}_expected_z_max', 2.0)) - z_pad
            if y_max <= y_min or z_max <= z_min:
                continue
            fig.add_trace(go.Scatter3d(
                x=[x_ref, x_ref, x_ref, x_ref, x_ref],
                y=[y_min, y_max, y_max, y_min, y_min],
                z=[z_min, z_min, z_max, z_max, z_min],
                mode='lines',
                line=dict(color='#FF8C00', width=4, dash='dash'),
                name=f'Cropped Expected ({zone})',
                hovertemplate=f'Cropped expected · {zone}<extra></extra>',
            ))

    def add_fitted_pca_plane(self, fig: go.Figure, viz_data: dict) -> None:
        fp = viz_data.get('fitted_planes', {})
        if not fp:
            return
        zones = [k.replace('_plane_center_x', '') for k in fp if k.endswith('_plane_center_x')]
        colors = ['#FF6B6B', '#FFD700']
        for idx, zone in enumerate(zones):
            cx = float(fp.get(f'{zone}_plane_center_x', 0.0))
            cy = float(fp.get(f'{zone}_plane_center_y', 0.0))
            cz = float(fp.get(f'{zone}_plane_center_z', 0.0))
            nx = float(fp.get(f'{zone}_plane_normal_x', 1.0))
            ny = float(fp.get(f'{zone}_plane_normal_y', 0.0))
            nz = float(fp.get(f'{zone}_plane_normal_z', 0.0))
            y_min = float(fp.get(f'{zone}_plane_bounds_y_min', cy - 0.5))
            y_max = float(fp.get(f'{zone}_plane_bounds_y_max', cy + 0.5))
            z_min = float(fp.get(f'{zone}_plane_bounds_z_min', cz - 0.5))
            z_max = float(fp.get(f'{zone}_plane_bounds_z_max', cz + 0.5))
            corners = np.array([
                [cx - (ny * (y - cy) + nz * (z - cz)) / nx if abs(nx) > 1e-6 else cx, y, z]
                for y, z in [(y_min, z_min), (y_max, z_min), (y_max, z_max), (y_min, z_max)]
            ])
            fig.add_trace(go.Mesh3d(
                x=corners[:, 0].tolist(), y=corners[:, 1].tolist(), z=corners[:, 2].tolist(),
                i=[0, 0], j=[1, 2], k=[2, 3],
                opacity=0.25,
                color=colors[idx % len(colors)],
                name=f'PCA Plane ({zone})',
                hovertemplate=f'Fitted PCA plane · {zone}<extra></extra>',
            ))

    def add_sensor_axes(self, fig: go.Figure, viz_data: dict) -> None:
        R, origin = self._parse_orientation(viz_data)
        for i, (color, label) in enumerate(zip(['#FF6B6B', '#39FF14', '#00BFFF'], ['X fwd', 'Y left', 'Z up'])):
            tip = origin + R[:, i] * 0.4
            fig.add_trace(go.Scatter3d(
                x=[float(origin[0]), float(tip[0])],
                y=[float(origin[1]), float(tip[1])],
                z=[float(origin[2]), float(tip[2])],
                mode='lines',
                line=dict(color=color, width=7),
                name=f'LiDAR {label}',
            ))
        fig.add_trace(go.Scatter3d(
            x=[float(origin[0])], y=[float(origin[1])], z=[float(origin[2])],
            mode='markers',
            marker=dict(size=7, color='white', symbol='circle'),
            name='LiDAR Origin',
        ))

    def add_spatial_dropout_analysis(self, fig: go.Figure, viz_data: dict) -> None:
        dead_cells = viz_data.get('dead_cells', {})
        if not dead_cells:
            return
        fitted_planes = viz_data.get('fitted_planes', {})
        cell_size = float(dead_cells.get('dead_cell_size_m', 0.05))

        # Derive zones from the available zone geometry, not from the dead-cell
        # keys: a zone with zero dropout has no dead-cell entries, but we still
        # want to render its (all-live) coverage grid instead of drawing nothing.
        # The grid spans the full expected bounds to match how the metric indexes
        # cells (dead-cell coords are reported relative to the full y_min/z_min).
        zones = [k.replace('_expected_x', '') for k in fitted_planes if k.endswith('_expected_x')]
        palette = self._config['palette']
        padding = viz_data.get('expected_padding', {})
        for zone_idx, zone in enumerate(sorted(zones)):
            x_ref = float(fitted_planes.get(f'{zone}_expected_x', 0.0))
            y_min = float(fitted_planes.get(f'{zone}_expected_y_min', 0.0))
            y_max = float(fitted_planes.get(f'{zone}_expected_y_max', 1.0))
            z_min = float(fitted_planes.get(f'{zone}_expected_z_min', 0.0))
            z_max = float(fitted_planes.get(f'{zone}_expected_z_max', 1.0))
            n_y = max(1, int(np.ceil((y_max - y_min) / cell_size)))
            n_z = max(1, int(np.ceil((z_max - z_min) / cell_size)))

            # Only cells whose centers fall inside the zone shrunk by the y/z
            # padding are scored by the metric, so the padding ring gets no grid
            # and no color. Validity is separable per row/column (matching the
            # metric's valid-cell mask), so the scored region is a cell-aligned
            # rectangle. Indexing stays keyed to the full y_min/z_min so reported
            # dead-cell coordinates still map to the right cell.
            pad = padding.get(zone, {})
            y_pad = float(pad.get('y', 0.0))
            z_pad = float(pad.get('z', 0.0))
            valid_iy = [iy for iy in range(n_y)
                        if y_min + y_pad <= y_min + (iy + 0.5) * cell_size <= y_max - y_pad]
            valid_iz = [iz for iz in range(n_z)
                        if z_min + z_pad <= z_min + (iz + 0.5) * cell_size <= z_max - z_pad]
            if not valid_iy or not valid_iz:
                continue
            sy0 = y_min + valid_iy[0] * cell_size
            sy1 = min(y_min + (valid_iy[-1] + 1) * cell_size, y_max)
            sz0 = z_min + valid_iz[0] * cell_size
            sz1 = min(z_min + (valid_iz[-1] + 1) * cell_size, z_max)

            dead_set = set()
            for key, val in dead_cells.items():
                if key.startswith(f'{zone}_dead_cell_') and key.endswith('_y_m'):
                    idx_str = key[len(f'{zone}_dead_cell_'):-4]
                    z_key = f'{zone}_dead_cell_{idx_str}_z_m'
                    if z_key in dead_cells:
                        iy = int((float(val) - y_min) / cell_size)
                        iz = int((float(dead_cells[z_key]) - z_min) / cell_size)
                        dead_set.add((iy, iz))

            live_x, live_y, live_z, live_i, live_j, live_k = [], [], [], [], [], []
            dead_x, dead_y, dead_z, dead_i, dead_j, dead_k = [], [], [], [], [], []
            for iy in valid_iy:
                for iz in valid_iz:
                    y0, y1 = y_min + iy * cell_size, min(y_min + (iy + 1) * cell_size, y_max)
                    z0, z1 = z_min + iz * cell_size, min(z_min + (iz + 1) * cell_size, z_max)
                    corners = [(x_ref, y0, z0), (x_ref, y1, z0), (x_ref, y1, z1), (x_ref, y0, z1)]
                    vx, vy, vz, vi, vj, vk = (dead_x, dead_y, dead_z, dead_i, dead_j, dead_k) if (iy, iz) in dead_set else (live_x, live_y, live_z, live_i, live_j, live_k)
                    base = len(vx)
                    for cx, cy, cz in corners:
                        vx.append(cx); vy.append(cy); vz.append(cz)
                    vi += [base, base]; vj += [base + 1, base + 2]; vk += [base + 2, base + 3]

            if live_x:
                fig.add_trace(go.Mesh3d(x=live_x, y=live_y, z=live_z, i=live_i, j=live_j, k=live_k, color='#FF4444', opacity=0.8, name=f'Live Cells ({zone})', showlegend=True))
            if dead_x:
                fig.add_trace(go.Mesh3d(x=dead_x, y=dead_y, z=dead_z, i=dead_i, j=dead_j, k=dead_k, color='#111111', opacity=0.9, name=f'Dead Cells ({zone})', showlegend=True))

            gx, gy, gz = [], [], []
            for iy in range(valid_iy[0], valid_iy[-1] + 2):
                y = min(y_min + iy * cell_size, y_max)
                gx += [x_ref, x_ref, None]; gy += [y, y, None]; gz += [sz0, sz1, None]
            for iz in range(valid_iz[0], valid_iz[-1] + 2):
                z = min(z_min + iz * cell_size, z_max)
                gx += [x_ref, x_ref, None]; gy += [sy0, sy1, None]; gz += [z, z, None]
            grid_color = palette[zone_idx % len(palette)]
            fig.add_trace(go.Scatter3d(x=gx, y=gy, z=gz, mode='lines', line=dict(color=grid_color, width=3), name=f'Grid ({zone})', showlegend=False, hoverinfo='skip'))

            bx = [x_ref, x_ref, x_ref, x_ref, x_ref]
            by = [sy0, sy1, sy1, sy0, sy0]
            bz = [sz0, sz0, sz1, sz1, sz0]
            fig.add_trace(go.Scatter3d(x=bx, y=by, z=bz, mode='lines', line=dict(color='white', width=5), name=f'Zone Border ({zone})', showlegend=False, hoverinfo='skip'))

            if dead_set:
                dx, dy, dz = [], [], []
                for iy, iz in dead_set:
                    y0 = y_min + iy * cell_size
                    y1 = min(y_min + (iy + 1) * cell_size, y_max)
                    z0 = z_min + iz * cell_size
                    z1 = min(z_min + (iz + 1) * cell_size, z_max)
                    dx += [x_ref, x_ref, x_ref, x_ref, x_ref, None]
                    dy += [y0, y1, y1, y0, y0, None]
                    dz += [z0, z0, z1, z1, z0, None]
                fig.add_trace(go.Scatter3d(x=dx, y=dy, z=dz, mode='lines', line=dict(color='white', width=2), name=f'Dead Cell Borders ({zone})', showlegend=False, hoverinfo='skip'))

    def add_worst_points(self, fig: go.Figure, viz_data: dict) -> None:
        """Plot the per-metric worst outlier points as distinct markers so they
        stand out against the clean cloud. Coordinates are already lidar-relative
        (the reporter translated them), so they overlay the rendered cloud."""
        wp = viz_data.get('worst_points', {})
        if not wp:
            return
        xs, ys, zs, labels = [], [], [], []
        for key in wp:
            if not key.endswith('_x'):
                continue
            base = key[:-2]
            yk, zk = f'{base}_y', f'{base}_z'
            if yk in wp and zk in wp:
                xs.append(float(wp[key]))
                ys.append(float(wp[yk]))
                zs.append(float(wp[zk]))
                labels.append(base)
        if not xs:
            return
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode='markers',
            marker=dict(size=2, color='#FF00FF', opacity=0.95),
            name='Worst Points',
            text=labels,
            hovertemplate='%{text}<br>x: %{x:.3f}  y: %{y:.3f}  z: %{z:.3f}<extra>Worst Point</extra>',
        ))

    def fit_scene_to_origin(self, fig: go.Figure, pad_frac: float = 0.05) -> None:
        """Frame the 3D scene so it always spans from the lidar origin (0,0,0)
        out to the farthest rendered point on each axis. With projective
        geometry the returns can land far forward or very close, so we anchor
        every axis at the origin and stretch it to the data extent. aspectmode
        stays 'data', so the framed window keeps true proportions."""
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for tr in fig.data:
            if getattr(tr, 'visible', True) is False:
                continue
            for axis, bucket in (('x', xs), ('y', ys), ('z', zs)):
                vals = getattr(tr, axis, None)
                if vals is None:
                    continue
                for v in vals:
                    if v is None:
                        continue
                    try:
                        f = float(v)
                    except (TypeError, ValueError):
                        continue
                    if np.isfinite(f):
                        bucket.append(f)
        if not (xs and ys and zs):
            return

        def _rng(vals: list[float]) -> list[float]:
            lo = min(vals + [0.0])
            hi = max(vals + [0.0])
            pad = (hi - lo) * pad_frac or 0.1
            return [lo - pad, hi + pad]

        fig.update_layout(scene=dict(
            xaxis=dict(range=_rng(xs)),
            yaxis=dict(range=_rng(ys)),
            zaxis=dict(range=_rng(zs)),
        ))

    def _parse_orientation(self, viz_data: dict) -> tuple[np.ndarray, np.ndarray]:
        orientation = viz_data.get('orientation', {})
        pitch = float(orientation.get('pitch', 0.0))
        roll = float(orientation.get('roll', 0.0))
        yaw = float(orientation.get('yaw', 0.0))
        return self._euler_to_rotation(pitch, roll, yaw), np.array([0.0, 0.0, 0.0])

    def _euler_to_rotation(self, pitch: float, roll: float, yaw: float) -> np.ndarray:
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr               ],
        ])

    @staticmethod
    def _to_sig_figs(val: float, sig: int = 3) -> float:
        if val == 0:
            return 0.0
        return float(f'{val:.{sig}g}')

    def render_single_lidar_metrics(self, lidar_name: str, metrics: dict, thresholds: dict | None = None, secondary_axis_keys: list | None = None, y_padding: float = 0.3, split_by_suffix_categories: list | None = None, split_exclude_suffixes: dict | None = None, category_skip_key_suffixes: dict | None = None) -> tuple[list, list]:
        layout = self._config['layout']
        mc = self._config['metric_chart']
        main_figs: list = []
        bottom_figs: list = []
        secondary_suffixes = set(secondary_axis_keys or [])

        def _is_secondary(key: str) -> bool:
            return key in secondary_suffixes or any(key.endswith(f'_{s}') for s in secondary_suffixes)
        skip_suffixes_map = category_skip_key_suffixes or {}
        split_set = set(split_by_suffix_categories or [])
        sorted_categories = sorted(metrics.items(), key=lambda x: ('Intensity' in x[0], 'NoiseRegion' in x[0]))
        for category, values in sorted_categories:
            if not isinstance(values, dict):
                continue
            skip_suffixes = skip_suffixes_map.get(category, [])
            items = {k: self._to_sig_figs(v) for k, v in values.items() if isinstance(v, (int, float)) and k != 'visualization' and not any(k.endswith(s) for s in skip_suffixes)}
            if not items:
                continue
            if category in split_set:
                suffix_groups: dict = defaultdict(dict)
                for k, v in items.items():
                    suffix_groups[k.rsplit('_', 1)[-1]][k] = v
                excluded = set((split_exclude_suffixes or {}).get(category, []))
                subcategory_items = [(f'{category} · {suffix}', grp, False) for suffix, grp in sorted(suffix_groups.items()) if suffix not in excluded]
            else:
                primary_items = {k: v for k, v in items.items() if not _is_secondary(k)}
                secondary_items = {k: v for k, v in items.items() if _is_secondary(k)}
                subcategory_items = [
                    (f'{category}', primary_items if primary_items else items, False),
                    (f'{category} (counts)', secondary_items, True),
                ]
            for title, fig_items, is_counts in subcategory_items:
                if not fig_items:
                    continue
                color = self._config['count_bar_color'] if is_counts else self._config['palette'][0]
                display_keys = [self._label_key(k) for k in fig_items.keys()]
                vals = list(fig_items.values())
                data_min = min(vals)
                data_max = max(vals)
                data_range = data_max - data_min
                if data_range == 0:
                    data_range = abs(data_max) * 0.1 or 0.001
                pad = data_range * y_padding
                y_range = [data_min - pad, data_max + pad]
                fig = go.Figure()
                fig.add_trace(go.Bar(
                    x=display_keys,
                    y=vals,
                    marker=dict(color=color, opacity=0.85, line=dict(color=mc['bar_border_color'], width=mc['bar_border_width'])),
                    hovertemplate='<b>%{x}</b><br>%{y:.5g}<extra></extra>',
                    name=title,
                    showlegend=False,
                ))
                for key, val in zip(display_keys, vals):
                    fig.add_annotation(
                        x=key, y=mc['label_y_paper'],
                        xref='x', yref='paper',
                        text=f'{val:.4g}',
                        showarrow=False,
                        yanchor='bottom', xanchor='center',
                        font=dict(size=mc['label_font_size'], color='rgba(255,255,255,0.7)'),
                    )
                if not is_counts:
                    self._apply_threshold_bands(fig, title, category, fig_items, thresholds, y_range)
                unit = '#' if is_counts else self._schemas['category_units'].get(category, '')
                if unit:
                    fig.add_annotation(
                        text=f'<b>({unit})</b>',
                        xref='paper', yref='paper',
                        x=0, y=0,
                        showarrow=False,
                        xanchor='right', yanchor='top',
                        font=dict(size=11, color='rgba(255,255,255,0.8)'),
                    )
                fig.update_layout(
                    title=dict(text=f'<b>{title}</b>', font=dict(color='white', size=13), x=0.0),
                    paper_bgcolor=layout['paper_color'],
                    plot_bgcolor='rgba(8,10,18,0.0)',
                    height=mc['height'],
                    margin=dict(t=mc['margin_top'], b=mc['margin_bottom'], l=mc['margin_left'], r=mc['margin_right']),
                    xaxis=dict(
                        showticklabels=True,
                        tickangle=mc['tick_angle'],
                        tickfont=dict(size=mc['tick_font_size'], color='rgba(255,255,255,0.8)'),
                        showgrid=False,
                        zeroline=False,
                    ),
                    yaxis=dict(
                        tickfont=dict(size=mc['tick_font_size'], color='rgba(255,255,255,0.7)'),
                        gridcolor='rgba(255,255,255,0.04)',
                        zerolinecolor='rgba(255,255,255,0.15)',
                        tickformat='.3g',
                        range=y_range,
                    ),
                    showlegend=True,
                    legend=dict(
                        x=mc['legend_x'], y=mc['legend_y'],
                        xanchor=mc['legend_xanchor'], yanchor=mc['legend_yanchor'],
                        font=dict(color='white', size=mc['legend_font_size']),
                        bgcolor='rgba(10,12,20,0.7)',
                        bordercolor='rgba(255,255,255,0.1)',
                        borderwidth=1,
                    ),
                    modebar=self._MODEBAR,
                )
                fig.update_yaxes(range=y_range)
                keys = list(fig_items.keys())
                has_zones = len(keys) > 1 and len({k.split('_')[0] for k in keys}) > 1
                if is_counts:
                    bottom_figs.append((title, fig))
                elif category in split_set or has_zones:
                    main_figs.append((title, fig))
                else:
                    bottom_figs.append((title, fig))
        return main_figs, bottom_figs

    def _label_key(self, key: str) -> str:
        key_labels = self._schemas.get('key_labels', {})
        zones = key_labels.get('zones', {})
        suffixes = key_labels.get('suffixes', {})
        for zone_key, zone_label in sorted(zones.items(), key=lambda x: -len(x[0])):
            if key.startswith(zone_key + '_'):
                suffix = key[len(zone_key) + 1:]
                suffix_label = suffixes.get(suffix, suffix.replace('_', ' ').title())
                return f'{zone_label} · {suffix_label}'
        return suffixes.get(key, key.replace('_', ' ').title())

    def _zones_for_key(self, threshold_list: list, key_name: str) -> dict:
        for entry in threshold_list:
            if key_name in entry.get('keys', []):
                return {k: v for k, v in entry.items() if k != 'keys'}
        return {}

    def _apply_threshold_bands(self, fig: go.Figure, title: str, category: str, items: dict, thresholds: dict | None, y_range: list | None = None) -> None:
        if not thresholds:
            return
        threshold_config = thresholds.get(title) or thresholds.get(category)
        if threshold_config is None:
            return
        zone_fill_colors = self._config['zone_fill_colors']
        zone_legend_colors = self._config['zone_legend_colors']
        zone_border_width = self._config['metric_chart']['zone_border_width']
        legend_marker_size = self._config['metric_chart']['legend_marker_size']
        legend_entries: dict[str, str] = {}  # label -> legend color, first occurrence wins
        if isinstance(threshold_config, list):
            for key_idx, key in enumerate(items.keys()):
                zones = self._zones_for_key(threshold_config, key)
                for zone, fill_color in zone_fill_colors.items():
                    zone_data = zones.get(zone, {})
                    if not zone_data.get('enabled'):
                        continue
                    label = zone_data.get('label', '')
                    if label and label not in legend_entries:
                        legend_entries[label] = zone_legend_colors[zone]
                    y0, y1 = zone_data['min'], zone_data['max']
                    if y_range is not None:
                        if y1 <= y_range[0] or y0 >= y_range[1]:
                            continue
                        y0 = max(y0, y_range[0])
                        y1 = min(y1, y_range[1])
                    fig.add_shape(
                        type='rect',
                        x0=key_idx - 0.5, x1=key_idx + 0.5,
                        y0=y0, y1=y1,
                        fillcolor=fill_color, opacity=1.0,
                        line=dict(color=zone_legend_colors[zone], width=zone_border_width),
                        xref='x', yref='y', layer='below',
                    )
        else:
            for zone, fill_color in zone_fill_colors.items():
                zone_data = threshold_config.get(zone, {})
                if not zone_data.get('enabled'):
                    continue
                label = zone_data.get('label', '')
                if label and label not in legend_entries:
                    legend_entries[label] = zone_legend_colors[zone]
                y0, y1 = zone_data['min'], zone_data['max']
                if y_range is not None:
                    if y1 <= y_range[0] or y0 >= y_range[1]:
                        continue
                    y0 = max(y0, y_range[0])
                    y1 = min(y1, y_range[1])
                fig.add_hrect(y0=y0, y1=y1, fillcolor=fill_color, opacity=1.0, line_color=zone_legend_colors[zone], line_width=zone_border_width, layer='below')
        for label, leg_color in legend_entries.items():
            fig.add_trace(go.Scatter(
                x=[None], y=[None],
                mode='markers',
                marker=dict(color=leg_color, size=legend_marker_size, symbol='square'),
                name=label,
                showlegend=True,
            ))

    def make_abstract_radar_figure(self, zone_name: str, lidar_scores: dict, colors: list | None = None) -> go.Figure:
        if not lidar_scores:
            return go.Figure()
        layout = self._config['layout']
        lidar_names = list(lidar_scores.keys())
        if colors is None:
            colors = self._colors(len(lidar_names))
        all_labels = list(dict.fromkeys(label for scores in lidar_scores.values() for label in scores))
        if not all_labels:
            return go.Figure()
        fig = go.Figure()
        theta_ring = all_labels + [all_labels[0]]
        for r_val, fill_color, border_color in [
            (1.0,  'rgba(57,255,20,0.13)',  'rgba(57,255,20,0.30)'),
            (0.75, 'rgba(255,200,0,0.15)',  'rgba(255,200,0,0.35)'),
            (0.40, 'rgba(255,50,50,0.18)',  'rgba(255,50,50,0.40)'),
        ]:
            fig.add_trace(go.Scatterpolar(
                r=[r_val] * len(theta_ring), theta=theta_ring,
                fill='toself', fillcolor=fill_color,
                line=dict(width=0.8, color=border_color),
                showlegend=False, hoverinfo='skip',
            ))
        for lidar_idx, lidar_name in enumerate(lidar_names):
            scores = [lidar_scores[lidar_name].get(label, 0.0) for label in all_labels]
            color = colors[lidar_idx]
            fig.add_trace(go.Scatterpolar(
                r=scores + [scores[0]],
                theta=all_labels + [all_labels[0]],
                fill='toself',
                fillcolor=self._hex_to_rgba(color, 0.22),
                line=dict(color=color, width=3),
                marker=dict(size=7, color=color, symbol='circle', line=dict(color='white', width=1.5)),
                name=lidar_name,
                hovertemplate='<b>%{theta}</b><br>Score: <b>%{r:.3f}</b><extra>' + lidar_name + '</extra>',
            ))
        fig.update_layout(
            polar=dict(
                bgcolor='rgba(10,12,20,0.85)',
                radialaxis=dict(
                    visible=True, range=[0, 1],
                    tickfont=dict(size=9, color='rgba(255,255,255,0.55)'),
                    gridcolor='rgba(255,255,255,0.08)',
                    linecolor='rgba(255,255,255,0.06)',
                    tickvals=[0.20, 0.575, 0.875],
                    ticktext=['bad', 'ok', 'great'],
                ),
                angularaxis=dict(
                    tickfont=dict(size=12, color='rgba(255,255,255,0.9)', family='sans-serif'),
                    gridcolor='rgba(255,255,255,0.06)',
                    linecolor='rgba(255,255,255,0.1)',
                    direction='clockwise',
                ),
            ),
            title=dict(
                text=f'<b>{zone_name.replace("_", " ").title()}</b>  ·  green = great  ·  yellow = ok  ·  red = bad',
                font=dict(size=14, color='rgba(255,255,255,0.65)'), x=0.5, xanchor='center',
                y=0.98, yanchor='top',
            ),
            showlegend=True,
            legend=dict(font=dict(size=12, color='white'), bgcolor='rgba(10,12,20,0.9)',
                        bordercolor='rgba(81,56,238,0.35)', borderwidth=1,
                        x=1.08, y=1.0, itemsizing='constant'),
            paper_bgcolor=layout['paper_color'],
            height=560,
            margin=dict(t=115, b=50, l=100, r=220),
            modebar=self._MODEBAR,
        )
        return fig

    def _hex_to_rgba(self, hex_color: str, alpha: float) -> str:
        h = hex_color.lstrip('#')
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return f'rgba({r},{g},{b},{alpha})'

    def _colors(self, n: int) -> list[str]:
        palette = self._config['palette']
        return [palette[i % len(palette)] for i in range(n)]

    def make_bullet_figure(
        self,
        title: str,
        rows: list[tuple[str, float]],
        bands: dict | None,
        lower_is_better: bool,
        value_suffix: str = '%',
    ) -> go.Figure:
        good_color = '#22C55E'
        warn_color = '#EAB308'
        crit_color = '#EF4444'

        def _status(val: float) -> tuple[str, str]:
            if not bands:
                return '#E5E7EB', 'No band'
            if lower_is_better:
                if val <= bands['great']['max']:
                    return good_color, 'Good'
                if val <= bands['ok']['max']:
                    return warn_color, 'Warning'
                return crit_color, 'Critical'
            if val >= bands['great']['min']:
                return good_color, 'Good'
            if val >= bands['ok']['min']:
                return warn_color, 'Warning'
            return crit_color, 'Critical'

        if bands:
            great = bands['great']
            ok = bands['ok']
            bad = bands['bad']
            if lower_is_better:
                x_lo, x_hi = float(great['min']), float(bad['max'])
                band_segs = [
                    ('Good', float(great['min']), float(great['max']), good_color),
                    ('Warning', float(ok['min']), float(ok['max']), warn_color),
                    ('Critical', float(bad['min']), float(bad['max']), crit_color),
                ]
            else:
                x_lo, x_hi = float(bad['min']), float(great['max'])
                band_segs = [
                    ('Critical', float(bad['min']), float(bad['max']), crit_color),
                    ('Warning', float(ok['min']), float(ok['max']), warn_color),
                    ('Good', float(great['min']), float(great['max']), good_color),
                ]
        else:
            vals = [v for _, v in rows] or [1.0]
            x_lo, x_hi = 0.0, max(vals) * 1.2
            band_segs = []

        zones = [name for name, _ in rows]
        values = [val for _, val in rows]

        # Size the left margin to the longest row label (visible text, HTML stripped)
        # so labels stay on one line however long the LiDAR/case name is.
        max_label_chars = max((len(re.sub(r'<[^>]+>', '', name)) for name in zones), default=10)
        left_margin = int(min(440, max(130, max_label_chars * 7.5)))

        fig = go.Figure()

        # Threshold bands — the colored "track". Lower opacity so the value needle
        # reads clearly on top; dark separators between segments.
        for band_name, b_min, b_max, color in band_segs:
            fig.add_trace(go.Bar(
                x=[max(0.0, b_max - b_min)] * len(zones),
                y=zones,
                base=[b_min] * len(zones),
                orientation='h',
                marker=dict(color=color, line=dict(color='#0e1117', width=2)),
                opacity=0.5,
                name=band_name,
                width=0.5,
                hoverinfo='skip',
            ))

        # Value needle — a tall, high-contrast vertical line at the value. Its
        # size is fixed in pixels, so even sub-1% values stay clearly visible
        # against the band instead of collapsing into a sliver of a bar.
        fig.add_trace(go.Scatter(
            x=values,
            y=zones,
            mode='markers',
            marker=dict(symbol='line-ns', size=58, line=dict(color='white', width=5)),
            showlegend=False,
            hovertemplate='%{y}: %{x:.3g}' + value_suffix + '<extra></extra>',
        ))
        # Bead cap on the needle for a bit more polish / visibility.
        fig.add_trace(go.Scatter(
            x=values,
            y=zones,
            mode='markers',
            marker=dict(symbol='diamond', size=11, color='white', line=dict(color='#0e1117', width=1)),
            showlegend=False,
            hoverinfo='skip',
        ))

        # Big, status-colored value at the right edge — the focal number.
        annotations = []
        for zone, val in rows:
            status_color, status_label = _status(val)
            annotations.append(dict(
                x=1.0, xref='paper',
                y=zone,
                text=f'<b>{val:.3g}{value_suffix}</b>',
                xanchor='left', yanchor='middle',
                font=dict(color=status_color, size=26),
                showarrow=False,
                xshift=14,
                hovertext=status_label,
            ))

        fig.update_layout(
            title=dict(text=title, font=dict(color='white', size=15), x=0.0, xanchor='left'),
            barmode='overlay',
            bargap=0.45,
            paper_bgcolor='#0e1117',
            plot_bgcolor='#0e1117',
            height=80 + 88 * len(zones),
            margin=dict(l=left_margin, r=140, t=46, b=34),
            xaxis=dict(
                range=[x_lo, x_hi],
                color='rgba(255,255,255,0.6)',
                gridcolor='rgba(255,255,255,0.05)',
                zeroline=False,
                ticksuffix=value_suffix,
                tickfont=dict(size=11),
            ),
            yaxis=dict(
                color='white',
                autorange='reversed',
                tickfont=dict(size=14),
                showgrid=False,
            ),
            legend=dict(
                orientation='h',
                yanchor='bottom', y=1.0,
                xanchor='right', x=1.0,
                font=dict(color='rgba(255,255,255,0.75)', size=10),
                bgcolor='rgba(0,0,0,0)',
                itemsizing='constant',
            ),
            annotations=annotations,
        )
        return fig

    def make_percentile_distribution(self, title: str, series: dict, value_suffix: str = '%') -> go.Figure:
        """Horizontal box plot of a metric's percentile family, one box per zone,
        built from precomputed percentiles (box = p10..p90, median = p50, whiskers
        to min/p99 or max). Every percentile value is labeled so it's readable.
        `series` is {zone_label: {percentile_key: value}} where keys are a subset
        of {min, p10, p50, p90, p99, max}."""
        palette = self._config['palette']
        fig = go.Figure()
        zones = sorted(series.keys())
        for i, zone in enumerate(zones):
            p = series[zone]
            median = p.get('p50')
            q1 = p.get('p10', median)
            q3 = p.get('p90', p.get('p99', median))
            lo = p.get('min', q1)
            hi = p.get('max', p.get('p99', q3))
            color = palette[i % len(palette)]
            fig.add_trace(go.Box(
                name=zone, y=[zone], orientation='h',
                q1=[q1], median=[median], q3=[q3], lowerfence=[lo], upperfence=[hi],
                marker=dict(color=color),
                line=dict(color=color, width=2),
                fillcolor=self._hex_to_rgba(color, 0.25),
                width=0.3,
                hoverinfo='skip',
                showlegend=False,
            ))
            # Percentile markers + labels. Labels are annotations with a fixed
            # pixel offset above/below the box so they clear it — textposition
            # alone leaves them sitting on the box. Alternate so neighbours don't
            # collide.
            order = [pk for pk in ('min', 'p10', 'p50', 'p90', 'p99', 'max') if pk in p]
            xs = [p[pk] for pk in order]
            fig.add_trace(go.Scatter(
                x=xs, y=[zone] * len(xs),
                mode='markers',
                marker=dict(size=7, color='white', line=dict(color=color, width=1.5)),
                name=zone, showlegend=False,
                hovertemplate='%{x:.3g}' + value_suffix + '<extra>' + zone + '</extra>',
            ))
            for pk in order:
                fig.add_annotation(
                    x=p[pk], y=zone,
                    text=f'<b>{pk}</b><br>{p[pk]:.3g}{value_suffix}',
                    showarrow=False,
                    yshift=48,  # all labels on one level, above the box
                    font=dict(size=12, color='white'),
                    align='center',
                )
        unit = value_suffix.strip() or '%'
        fig.update_layout(
            title=dict(text=f'{title} — distribution', font=dict(color='white', size=13), x=0.0),
            paper_bgcolor='#0e1117',
            plot_bgcolor='rgba(8,10,18,0.0)',
            height=150 + 95 * len(zones),
            margin=dict(t=88, b=45, l=120, r=40),
            xaxis=dict(title=f'Value ({unit})', color='rgba(255,255,255,0.7)',
                       gridcolor='rgba(255,255,255,0.06)', tickfont=dict(size=10),
                       tickformat='.3g', ticksuffix=value_suffix, zeroline=False),
            yaxis=dict(color='white', tickfont=dict(size=12), showgrid=False, automargin=True),
            showlegend=False,
            modebar=self._MODEBAR,
        )
        return fig

    @staticmethod
    def score_abstract(val: float, bands: dict, lower_is_better: bool) -> float:
        great = bands['great']
        ok = bands['ok']
        bad = bands['bad']

        def _interp(val: float, lo: float, hi: float, score_hi: float, score_lo: float) -> float:
            t = (val - lo) / (hi - lo) if hi != lo else 0.0
            return score_hi - t * (score_hi - score_lo)

        if lower_is_better:
            if val <= great['max']:
                return round(max(0.0, min(1.0, _interp(val, great['min'], great['max'], 1.0, 0.75))), 4)
            elif val <= ok['max']:
                return round(_interp(val, ok['min'], ok['max'], 0.75, 0.40), 4)
            else:
                return round(max(0.0, _interp(val, bad['min'], bad['max'], 0.40, 0.0)), 4)
        else:
            if val >= great['min']:
                return round(max(0.0, min(1.0, _interp(val, great['max'], great['min'], 1.0, 0.75))), 4)
            elif val >= ok['min']:
                return round(_interp(val, ok['max'], ok['min'], 0.75, 0.40), 4)
            else:
                return round(max(0.0, _interp(val, bad['max'], bad['min'], 0.40, 0.0)), 4)
