#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""
Run the leo-link --payload=regenerative ns-3 example, collect traces into a timestamped results
directory, and generate summary figures for geometry/channel and DL MAC.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SUPTITLE_FONT_SIZE = 18
TITLE_FONT_SIZE = 14
LABEL_FONT_SIZE = 13
TICK_FONT_SIZE = 11
LEGEND_FONT_SIZE = 10
ANNOTATION_FONT_SIZE = 10


GEOMETRY_LOG_NAME = "log.txt"
CHANNEL_CONDITION_TRACE_NAME = "ChannelConditionTrace.txt"
RUN_LOG_NAME = "run.log"
FIG1_NAME = "fig1_geometry_channel.png"
FIG2_NAME = "fig2_dl_mac_mcs_sinr.png"
FIG3_NAME = "fig3_rank_los.png"
ELEVATION_STATS_CSV_NAME = "elevation_stats.csv"
ELEVATION_STATS_REPORT_NAME = "elevation_stats.txt"
ELEVATION_STATS_FIG_NAME = "fig_elevation_stats.png"
DEFAULT_PARAM_FILE_NAME = "ntn/params/leo_link_analysis_params.txt"
USED_PARAM_FILE_NAME = "used_parameters.txt"

ELEVATION_BIN_DEG = 10
PERCENTILE_LO = 5
PERCENTILE_HI = 95
MAX_RANK2_MARKERS = 2000
RANK2_COLOR = "tab:blue"

PARAMETER_KEYS = {
    "duration",
    "precision",
    "scenario",
    "frequency_hz",
    "bandwidth_hz",
    "sat_eirp_dbw_per_mhz",
    "vsat_antenna_gain_db",
    "sat_antenna_gain_db",
    "transport",
    "udp_rate_mbps",
    "udp_packet_size",
    "s1u_link_delay_ms",
    "remote_host_delay_ms",
    "use_fixed_mcs",
    "fixed_mcs",
    "observer_mode",
    "observer_elevation_boundary_deg",
    "enable_mimo_feedback",
    "mimo_pm_search_method",
    "mimo_full_search_codebook",
    "mimo_rank_limit",
    "mimo_rank_threshold",
    "mimo_rank_technique",
    "mimo_subband_size",
    "mimo_downsampling_technique",
    "ue_antenna_rows",
    "ue_antenna_columns",
    "ue_num_vertical_ports",
    "ue_num_horizontal_ports",
    "gnb_antenna_rows",
    "gnb_antenna_columns",
    "gnb_num_vertical_ports",
    "gnb_num_horizontal_ports",
    "ue_is_dual_polarized",
    "gnb_is_dual_polarized",
    "ue_pol_slant_angle_deg",
    "gnb_pol_slant_angle_deg",
    "results_root",
    "throughput_window_s",
    "enable_harq_retx",
    "enable_dynamic_rtt",
    "num_harq_process",
    "n1_delay",
    "n2_delay",
    "rlc_reordering_timer_ms",
    "num_ues",
    "ue_spacing_deg",
    "ue_radii_km",
    "beam_hpbw_deg",
    "ue_noise_figure_db",
}


@dataclass(frozen=True)
class RunConfig:
    duration: int
    precision: int
    scenario: str
    frequency_hz: float
    bandwidth_hz: float
    sat_eirp_dbw_per_mhz: float
    vsat_antenna_gain_db: float
    sat_antenna_gain_db: float
    transport: str
    udp_rate_mbps: float
    udp_packet_size: int
    s1u_link_delay_ms: float
    remote_host_delay_ms: float
    use_fixed_mcs: bool
    fixed_mcs: int
    observer_mode: str
    observer_elevation_boundary_deg: float
    enable_mimo_feedback: bool
    mimo_pm_search_method: str
    mimo_full_search_codebook: str
    mimo_rank_limit: int
    mimo_rank_threshold: float
    mimo_rank_technique: str
    mimo_subband_size: int
    mimo_downsampling_technique: str
    ue_antenna_rows: int
    ue_antenna_columns: int
    ue_num_vertical_ports: int
    ue_num_horizontal_ports: int
    gnb_antenna_rows: int
    gnb_antenna_columns: int
    gnb_num_vertical_ports: int
    gnb_num_horizontal_ports: int
    ue_is_dual_polarized: bool
    gnb_is_dual_polarized: bool
    ue_pol_slant_angle_deg: float
    gnb_pol_slant_angle_deg: float
    throughput_window_s: float
    enable_harq_retx: bool = False
    enable_dynamic_rtt: bool = False
    num_harq_process: int = 16
    n1_delay: int = 2
    n2_delay: int = 2
    rlc_reordering_timer_ms: int = 100
    num_ues: int = 1
    ue_spacing_deg: float = 0.05
    ue_radii_km: str = ""
    beam_hpbw_deg: str = ""
    ue_noise_figure_db: str = ""


def parse_bool(raw_value: str) -> bool:
    value = raw_value.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: {raw_value}")


def resolve_config_path(raw_value: str, ns3_root: Path) -> str:
    path = Path(raw_value).expanduser()
    if not path.is_absolute():
        path = ns3_root / path
    return str(path.resolve())


def default_parameter_values(ns3_root: Path) -> dict[str, object]:
    return {
        "duration": 200,
        "precision": 1000,
        "scenario": "NTN-Rural",
        "frequency_hz": 3.5e9,
        "bandwidth_hz": 100e6,
        "sat_eirp_dbw_per_mhz": 10.0,
        "vsat_antenna_gain_db": 0.0,
        "sat_antenna_gain_db": 0.0,
        "transport": "udp",
        "udp_rate_mbps": 50.0,
        "udp_packet_size": 1200,
        "s1u_link_delay_ms": 0.0,
        "remote_host_delay_ms": 10.0,
        "use_fixed_mcs": True,
        "fixed_mcs": 20,
        "observer_mode": "overhead",
        "observer_elevation_boundary_deg": 10.0,
        "enable_mimo_feedback": True,
        "mimo_pm_search_method": "ns3::NrPmSearchFull",
        "mimo_full_search_codebook": "ns3::NrCbTwoPort",
        "mimo_rank_limit": 2,
        "mimo_rank_threshold": 0.0,
        "mimo_rank_technique": "Sasaoka",
        "mimo_subband_size": 32,
        "mimo_downsampling_technique": "FirstPRB",
        "ue_antenna_rows": 1,
        "ue_antenna_columns": 2,
        "ue_num_vertical_ports": 1,
        "ue_num_horizontal_ports": 2,
        "gnb_antenna_rows": 2,
        "gnb_antenna_columns": 2,
        "gnb_num_vertical_ports": 1,
        "gnb_num_horizontal_ports": 1,
        "ue_is_dual_polarized": False,
        "gnb_is_dual_polarized": True,
        "ue_pol_slant_angle_deg": 90.0,
        "gnb_pol_slant_angle_deg": 0.0,
        "results_root": str((ns3_root / "results").resolve()),
        "throughput_window_s": 1.0,
        "enable_harq_retx": False,
        "enable_dynamic_rtt": False,
        "num_harq_process": 16,
        "n1_delay": 2,
        "n2_delay": 2,
        "rlc_reordering_timer_ms": 100,
        "num_ues": 1,
        "ue_spacing_deg": 0.05,
        "ue_radii_km": "",
        "beam_hpbw_deg": "",
        "ue_noise_figure_db": "",
    }


def coerce_parameter_value(key: str, raw_value: str, ns3_root: Path) -> object:
    if key in {
        "duration",
        "precision",
        "udp_packet_size",
        "fixed_mcs",
        "mimo_rank_limit",
        "mimo_subband_size",
        "ue_antenna_rows",
        "ue_antenna_columns",
        "ue_num_vertical_ports",
        "ue_num_horizontal_ports",
        "gnb_antenna_rows",
        "gnb_antenna_columns",
        "gnb_num_vertical_ports",
        "gnb_num_horizontal_ports",
        "num_harq_process",
        "n1_delay",
        "n2_delay",
        "rlc_reordering_timer_ms",
        "num_ues",
    }:
        return int(raw_value)
    if key in {
        "udp_rate_mbps",
        "frequency_hz",
        "bandwidth_hz",
        "sat_eirp_dbw_per_mhz",
        "vsat_antenna_gain_db",
        "sat_antenna_gain_db",
        "observer_elevation_boundary_deg",
        "mimo_rank_threshold",
        "ue_pol_slant_angle_deg",
        "gnb_pol_slant_angle_deg",
        "throughput_window_s",
        "s1u_link_delay_ms",
        "remote_host_delay_ms",
        "ue_spacing_deg",
    }:
        return float(raw_value)
    if key in {
        "use_fixed_mcs",
        "enable_mimo_feedback",
        "ue_is_dual_polarized",
        "gnb_is_dual_polarized",
        "enable_harq_retx",
        "enable_dynamic_rtt",
    }:
        return parse_bool(raw_value)
    if key == "results_root":
        return resolve_config_path(raw_value, ns3_root)
    return raw_value.strip()


def load_parameter_file(path: Path, ns3_root: Path) -> dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(f"Parameter file does not exist: {path}")

    parameters: dict[str, object] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError(f"Invalid parameter line {line_number} in {path}: {raw_line.rstrip()}")
            key, value = [part.strip() for part in line.split("=", 1)]
            if key not in PARAMETER_KEYS:
                raise ValueError(f"Unsupported parameter '{key}' in {path}")
            parameters[key] = coerce_parameter_value(key, value, ns3_root)
    return parameters


def write_used_parameters(result_dir: Path, param_file: Path, args: argparse.Namespace) -> None:
    lines = [
        f"param_file={param_file}",
        f"duration={args.duration}",
        f"precision={args.precision}",
        f"scenario={args.scenario}",
        f"frequency_hz={args.frequency_hz}",
        f"bandwidth_hz={args.bandwidth_hz}",
        f"sat_eirp_dbw_per_mhz={args.sat_eirp_dbw_per_mhz}",
        f"vsat_antenna_gain_db={args.vsat_antenna_gain_db}",
        f"sat_antenna_gain_db={args.sat_antenna_gain_db}",
        f"transport={args.transport}",
        f"udp_rate_mbps={args.udp_rate_mbps}",
        f"udp_packet_size={args.udp_packet_size}",
        f"s1u_link_delay_ms={args.s1u_link_delay_ms}",
        f"remote_host_delay_ms={args.remote_host_delay_ms}",
        f"use_fixed_mcs={1 if args.use_fixed_mcs else 0}",
        f"fixed_mcs={args.fixed_mcs}",
        f"observer_mode={args.observer_mode}",
        f"observer_elevation_boundary_deg={args.observer_elevation_boundary_deg}",
        f"enable_mimo_feedback={1 if args.enable_mimo_feedback else 0}",
        f"mimo_pm_search_method={args.mimo_pm_search_method}",
        f"mimo_full_search_codebook={args.mimo_full_search_codebook}",
        f"mimo_rank_limit={args.mimo_rank_limit}",
        f"mimo_rank_threshold={args.mimo_rank_threshold}",
        f"mimo_rank_technique={args.mimo_rank_technique}",
        f"mimo_subband_size={args.mimo_subband_size}",
        f"mimo_downsampling_technique={args.mimo_downsampling_technique}",
        f"ue_antenna_rows={args.ue_antenna_rows}",
        f"ue_antenna_columns={args.ue_antenna_columns}",
        f"ue_num_vertical_ports={args.ue_num_vertical_ports}",
        f"ue_num_horizontal_ports={args.ue_num_horizontal_ports}",
        f"gnb_antenna_rows={args.gnb_antenna_rows}",
        f"gnb_antenna_columns={args.gnb_antenna_columns}",
        f"gnb_num_vertical_ports={args.gnb_num_vertical_ports}",
        f"gnb_num_horizontal_ports={args.gnb_num_horizontal_ports}",
        f"ue_is_dual_polarized={1 if args.ue_is_dual_polarized else 0}",
        f"gnb_is_dual_polarized={1 if args.gnb_is_dual_polarized else 0}",
        f"ue_pol_slant_angle_deg={args.ue_pol_slant_angle_deg}",
        f"gnb_pol_slant_angle_deg={args.gnb_pol_slant_angle_deg}",
        f"results_root={args.results_root}",
        f"throughput_window_s={args.throughput_window_s}",
        f"num_ues={args.num_ues}",
        f"ue_radii_km={args.ue_radii_km}",
        f"beam_hpbw_deg={args.beam_hpbw_deg}",
        f"ue_noise_figure_db={args.ue_noise_figure_db}",
    ]
    (result_dir / USED_PARAM_FILE_NAME).write_text("\n".join(lines) + "\n", encoding="utf-8")

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ns3_root = resolve_ns3_root()
    default_param_file = ns3_root / DEFAULT_PARAM_FILE_NAME

    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument(
        "--param-file",
        default=str(default_param_file),
        help="Path to the key=value parameter file used for default run settings.",
    )
    pre_args, _ = pre_parser.parse_known_args(argv)
    param_file = Path(pre_args.param_file).expanduser()
    if not param_file.is_absolute():
        param_file = (ns3_root / param_file).resolve()

    parameter_defaults = default_parameter_values(ns3_root)
    parameter_file_values = load_parameter_file(param_file, ns3_root)
    parameter_defaults.update(parameter_file_values)

    parser = argparse.ArgumentParser(
        description="Run leo-link --payload=regenerative, collect traces into a result folder, and generate two plots.",
        parents=[pre_parser],
    )
    parser.add_argument("--duration", type=int, default=parameter_defaults["duration"])
    parser.add_argument("--precision", type=int, default=parameter_defaults["precision"])
    parser.add_argument(
        "--scenario",
        default=parameter_defaults["scenario"],
        choices=["NTN-Rural", "NTN-DenseUrban"],
    )
    parser.add_argument(
        "--frequency-hz",
        type=float,
        default=parameter_defaults["frequency_hz"],
        help="NR carrier frequency in Hz.",
    )
    parser.add_argument(
        "--bandwidth-hz",
        type=float,
        default=parameter_defaults["bandwidth_hz"],
        help="NR channel bandwidth in Hz.",
    )
    parser.add_argument(
        "--sat-eirp-dbw-per-mhz",
        type=float,
        default=parameter_defaults["sat_eirp_dbw_per_mhz"],
        help="Satellite downlink EIRP density in dBW/MHz.",
    )
    parser.add_argument(
        "--vsat-antenna-gain-db",
        type=float,
        default=parameter_defaults["vsat_antenna_gain_db"],
        help="VSAT (UE) antenna gain in dBi.",
    )
    parser.add_argument(
        "--sat-antenna-gain-db",
        type=float,
        default=parameter_defaults["sat_antenna_gain_db"],
        help="Satellite (gNB) antenna element gain in dBi.",
    )
    parser.add_argument("--enable-harq-retx", dest="enable_harq_retx", type=parse_bool,
                        default=parameter_defaults["enable_harq_retx"])
    parser.add_argument("--enable-dynamic-rtt", dest="enable_dynamic_rtt", type=parse_bool,
                        default=parameter_defaults["enable_dynamic_rtt"])
    parser.add_argument("--num-harq-process", dest="num_harq_process", type=int,
                        default=parameter_defaults["num_harq_process"])
    parser.add_argument("--n1-delay", dest="n1_delay", type=int,
                        default=parameter_defaults["n1_delay"])
    parser.add_argument("--n2-delay", dest="n2_delay", type=int,
                        default=parameter_defaults["n2_delay"])
    parser.add_argument("--rlc-reordering-timer-ms", dest="rlc_reordering_timer_ms", type=int,
                        default=parameter_defaults["rlc_reordering_timer_ms"])
    parser.add_argument("--num-ues", dest="num_ues", type=int,
                        default=parameter_defaults["num_ues"])
    parser.add_argument("--ue-spacing-deg", dest="ue_spacing_deg", type=float,
                        default=parameter_defaults["ue_spacing_deg"])
    parser.add_argument("--ue-radii-km", dest="ue_radii_km",
                        default=parameter_defaults["ue_radii_km"],
                        help="comma list of UE distances from the beam centre; overrides the spacing")
    parser.add_argument("--beam-hpbw-deg", dest="beam_hpbw_deg",
                        default=parameter_defaults["beam_hpbw_deg"],
                        help="spot-beam half-power beamwidth; empty keeps the preset's, 0 disables the taper")
    parser.add_argument("--ue-noise-figure-db", dest="ue_noise_figure_db",
                        default=parameter_defaults["ue_noise_figure_db"],
                        help="UE noise figure; empty keeps the --ntnPreset terminal's")
    parser.add_argument("--transport", default=parameter_defaults["transport"], choices=["udp", "tcp"])
    parser.add_argument("--udp-rate-mbps", type=float, default=parameter_defaults["udp_rate_mbps"])
    parser.add_argument("--udp-packet-size", type=int, default=parameter_defaults["udp_packet_size"])
    parser.add_argument("--s1u-link-delay-ms", type=float, default=parameter_defaults["s1u_link_delay_ms"])
    parser.add_argument(
        "--remote-host-delay-ms",
        type=float,
        default=parameter_defaults["remote_host_delay_ms"],
    )
    parser.add_argument(
        "--observer-mode",
        default=parameter_defaults["observer_mode"],
        choices=["overhead", "full-pass"],
        help="Observer placement mode for leo-link --payload=regenerative.",
    )
    parser.add_argument(
        "--observer-elevation-boundary-deg",
        type=float,
        default=parameter_defaults["observer_elevation_boundary_deg"],
        help="Elevation boundary for full-pass mode; the observer starts at this elevation and the pass stops when it descends below it.",
    )
    parser.add_argument(
        "--enable-mimo-feedback",
        dest="enable_mimo_feedback",
        action="store_true",
        default=parameter_defaults["enable_mimo_feedback"],
        help="Enable MIMO CQI/RI/PMI feedback in leo-link --payload=regenerative.",
    )
    parser.add_argument(
        "--no-enable-mimo-feedback",
        dest="enable_mimo_feedback",
        action="store_false",
        help="Disable MIMO CQI/RI/PMI feedback in leo-link --payload=regenerative.",
    )
    parser.add_argument("--mimo-pm-search-method", default=parameter_defaults["mimo_pm_search_method"])
    parser.add_argument("--mimo-full-search-codebook", default=parameter_defaults["mimo_full_search_codebook"])
    parser.add_argument("--mimo-rank-limit", type=int, default=parameter_defaults["mimo_rank_limit"])
    parser.add_argument("--mimo-rank-threshold", type=float, default=parameter_defaults["mimo_rank_threshold"])
    parser.add_argument("--mimo-rank-technique", default=parameter_defaults["mimo_rank_technique"])
    parser.add_argument("--mimo-subband-size", type=int, default=parameter_defaults["mimo_subband_size"])
    parser.add_argument(
        "--mimo-downsampling-technique",
        default=parameter_defaults["mimo_downsampling_technique"],
    )
    parser.add_argument("--ue-antenna-rows", type=int, default=parameter_defaults["ue_antenna_rows"])
    parser.add_argument("--ue-antenna-columns", type=int, default=parameter_defaults["ue_antenna_columns"])
    parser.add_argument(
        "--ue-num-vertical-ports",
        type=int,
        default=parameter_defaults["ue_num_vertical_ports"],
    )
    parser.add_argument(
        "--ue-num-horizontal-ports",
        type=int,
        default=parameter_defaults["ue_num_horizontal_ports"],
    )
    parser.add_argument("--gnb-antenna-rows", type=int, default=parameter_defaults["gnb_antenna_rows"])
    parser.add_argument(
        "--gnb-antenna-columns",
        type=int,
        default=parameter_defaults["gnb_antenna_columns"],
    )
    parser.add_argument(
        "--gnb-num-vertical-ports",
        type=int,
        default=parameter_defaults["gnb_num_vertical_ports"],
    )
    parser.add_argument(
        "--gnb-num-horizontal-ports",
        type=int,
        default=parameter_defaults["gnb_num_horizontal_ports"],
    )
    parser.add_argument(
        "--ue-is-dual-polarized",
        dest="ue_is_dual_polarized",
        action="store_true",
        default=parameter_defaults["ue_is_dual_polarized"],
        help="Use dual-polarized UE antenna elements.",
    )
    parser.add_argument(
        "--no-ue-is-dual-polarized",
        dest="ue_is_dual_polarized",
        action="store_false",
        help="Use single-polarized UE antenna elements.",
    )
    parser.add_argument(
        "--gnb-is-dual-polarized",
        dest="gnb_is_dual_polarized",
        action="store_true",
        default=parameter_defaults["gnb_is_dual_polarized"],
        help="Use dual-polarized gNB antenna elements.",
    )
    parser.add_argument(
        "--no-gnb-is-dual-polarized",
        dest="gnb_is_dual_polarized",
        action="store_false",
        help="Use single-polarized gNB antenna elements.",
    )
    parser.add_argument(
        "--ue-pol-slant-angle-deg",
        type=float,
        default=parameter_defaults["ue_pol_slant_angle_deg"],
    )
    parser.add_argument(
        "--gnb-pol-slant-angle-deg",
        type=float,
        default=parameter_defaults["gnb_pol_slant_angle_deg"],
    )
    parser.add_argument(
        "--use-fixed-mcs",
        dest="use_fixed_mcs",
        action="store_true",
        default=parameter_defaults["use_fixed_mcs"],
        help="Enable fixed MCS scheduling in leo-link --payload=regenerative (default: enabled).",
    )
    parser.add_argument(
        "--no-use-fixed-mcs",
        dest="use_fixed_mcs",
        action="store_false",
        help="Disable fixed MCS scheduling in leo-link --payload=regenerative.",
    )
    parser.add_argument("--fixed-mcs", type=int, default=parameter_defaults["fixed_mcs"])
    parser.add_argument(
        "--results-root",
        default=parameter_defaults["results_root"],
        help="Parent directory for timestamped result folders.",
    )
    parser.add_argument(
        "--throughput-window-s",
        type=float,
        default=parameter_defaults["throughput_window_s"],
        help="Time bin size in seconds for DL MAC throughput.",
    )
    parser.add_argument(
        "--plot-only",
        action="store_true",
        help="Skip running ns-3 and regenerate plots from an existing result folder.",
    )
    parser.add_argument(
        "--input-dir",
        default="",
        help="Existing result directory to use with --plot-only.",
    )
    args = parser.parse_args(argv)
    args.param_file = str(param_file)
    return args


def resolve_ns3_root() -> Path:
    # This script lives in ntn/scripts/; the ns-3 root is two levels up.
    return Path(__file__).resolve().parents[2]


def create_result_dir(results_root: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    result_dir = results_root / f"leo-link-{timestamp}"
    result_dir.mkdir(parents=True, exist_ok=False)
    return result_dir








def build_leo_link_command(config: RunConfig, trace_file: Path) -> list[str]:
    channel_condition_trace_file = trace_file.parent / CHANNEL_CONDITION_TRACE_NAME
    example_args = [
        "leo-link",
        "--payload=regenerative",
        f"--traceFile={shlex.quote(str(trace_file))}",
        f"--channelConditionTraceFile={shlex.quote(str(channel_condition_trace_file))}",
        f"--precision={config.precision}",
        f"--duration={config.duration}",
        f"--scenario={config.scenario}",
        f"--dlFrequencyHz={config.frequency_hz}",
        f"--bandwidthHz={config.bandwidth_hz}",
        f"--satEIRP={config.sat_eirp_dbw_per_mhz}",
        f"--ueAntennaElementGainDbi={config.vsat_antenna_gain_db}",
        f"--satAntennaGainDb={config.sat_antenna_gain_db}",
        f"--transport={config.transport}",
        f"--udpRateMbps={config.udp_rate_mbps}",
        f"--udpPacketSize={config.udp_packet_size}",
        f"--s1uLinkDelayMs={config.s1u_link_delay_ms}",
        f"--remoteHostDelayMs={config.remote_host_delay_ms}",
        f"--useFixedMcs={1 if config.use_fixed_mcs else 0}",
        f"--fixedMcs={config.fixed_mcs}",
        f"--observerMode={config.observer_mode}",
        f"--observerElevationBoundaryDeg={config.observer_elevation_boundary_deg}",
        f"--enableMimoFeedback={1 if config.enable_mimo_feedback else 0}",
        f"--pmSearchMethod={config.mimo_pm_search_method}",
        f"--fullSearchCb={config.mimo_full_search_codebook}",
        f"--rankLimit={config.mimo_rank_limit}",
        f"--rankThreshold={config.mimo_rank_threshold}",
        f"--rankTechnique={config.mimo_rank_technique}",
        f"--subbandSize={config.mimo_subband_size}",
        f"--downsamplingTechnique={config.mimo_downsampling_technique}",
        f"--ueAntennaRows={config.ue_antenna_rows}",
        f"--ueAntennaColumns={config.ue_antenna_columns}",
        f"--ueNumVerticalPorts={config.ue_num_vertical_ports}",
        f"--ueNumHorizontalPorts={config.ue_num_horizontal_ports}",
        f"--gnbAntennaRows={config.gnb_antenna_rows}",
        f"--gnbAntennaColumns={config.gnb_antenna_columns}",
        f"--gnbNumVerticalPorts={config.gnb_num_vertical_ports}",
        f"--gnbNumHorizontalPorts={config.gnb_num_horizontal_ports}",
        f"--ueIsDualPolarized={1 if config.ue_is_dual_polarized else 0}",
        f"--gnbIsDualPolarized={1 if config.gnb_is_dual_polarized else 0}",
        f"--uePolSlantAngleDeg={config.ue_pol_slant_angle_deg}",
        f"--gnbPolSlantAngleDeg={config.gnb_pol_slant_angle_deg}",
        f"--enableHarqReTx={1 if config.enable_harq_retx else 0}",
        f"--numHarqProcess={config.num_harq_process}",
        f"--n1Delay={config.n1_delay}",
        f"--n2Delay={config.n2_delay}",
        f"--rlcReorderingTimerMs={config.rlc_reordering_timer_ms}",
        f"--numUes={config.num_ues}",
        f"--ueSpacingDeg={config.ue_spacing_deg}",
    ]
    if config.ue_radii_km:
        example_args.append(f"--ueRadiiKm={config.ue_radii_km}")
    if config.beam_hpbw_deg:
        example_args.append(f"--beamHpbwDeg={config.beam_hpbw_deg}")
    if config.ue_noise_figure_db:
        example_args.append(f"--ueNoiseFigureDb={config.ue_noise_figure_db}")
    # Each run executes in its own result directory, so every trace lands there.
    return ["./ns3", "run", f"--cwd={trace_file.parent}", " ".join(example_args)]


def ns3_run_environment() -> dict[str, str]:
    env = os.environ.copy()
    # Optional allocator override. It is intentionally opt-in because library paths vary
    # by distribution and architecture: JEMALLOC_LIB=/path/to/libjemalloc.so.
    jemalloc_lib = env.get("JEMALLOC_LIB")
    if jemalloc_lib:
        if not os.path.isfile(jemalloc_lib):
            raise FileNotFoundError(f"JEMALLOC_LIB does not exist: {jemalloc_lib}")
        existing_preload = env.get("LD_PRELOAD", "")
        env["LD_PRELOAD"] = (
            f"{jemalloc_lib}:{existing_preload}" if existing_preload else jemalloc_lib
        )
    return env


def run_leo_link(ns3_root: Path, result_dir: Path, config: RunConfig) -> None:
    result_dir.mkdir(parents=True, exist_ok=True)
    trace_file = result_dir / GEOMETRY_LOG_NAME
    run_log = result_dir / RUN_LOG_NAME
    command = build_leo_link_command(config, trace_file)
    run_env = ns3_run_environment()

    import resource
    import time as time_mod

    rss_before_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    wall_start = time_mod.monotonic()
    with run_log.open("w", encoding="utf-8") as log_handle:
        log_handle.write(f"cwd={result_dir}\n")
        log_handle.write(f"command={' '.join(command)}\n\n")
        log_handle.write(f"PATH={run_env.get('PATH', '')}\n\n")
        log_handle.flush()

        completed = subprocess.run(
            command,
            cwd=ns3_root,
            env=run_env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    wall_elapsed = time_mod.monotonic() - wall_start
    # ru_maxrss is the max over all waited-for children; valid for the run just
    # finished as long as no larger child preceded it in this process.
    child_max_rss_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    (result_dir / "runtime.txt").write_text(
        f"wall_seconds={wall_elapsed:.1f}\n"
        f"child_max_rss_mib={child_max_rss_kib / 1024.0:.1f}\n"
        f"child_max_rss_before_mib={rss_before_kib / 1024.0:.1f}\n"
    )
    print(f"Run wall time: {wall_elapsed:.1f} s; child max RSS: "
          f"{child_max_rss_kib / 1024.0:.1f} MiB")

    if completed.returncode != 0:
        raise RuntimeError(
            f"leo-link --payload=regenerative failed with exit code {completed.returncode}. See {run_log}"
        )


def ns3_time_token_to_seconds(token: str) -> float | None:
    value = token.strip()
    if not value:
        return None
    if value.startswith("+"):
        value = value[1:]
    for suffix, scale in (("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)):
        if value.endswith(suffix):
            try:
                return float(value[: -len(suffix)]) * scale
            except ValueError:
                return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_geometry_trace(path: Path) -> pd.DataFrame:
    rows: list[dict[str, float]] = []
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "elevation_deg", "slant_distance_km"])

    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        next(handle, None)
        for line in handle:
            parts = line.strip().split(":")
            if len(parts) < 10:
                continue
            time_s = ns3_time_token_to_seconds(parts[0])
            if time_s is None:
                continue
            try:
                elevation_deg = float(parts[8])
                slant_distance_km = float(parts[9]) / 1000.0
            except ValueError:
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "elevation_deg": elevation_deg,
                    "slant_distance_km": slant_distance_km,
                }
            )
    return pd.DataFrame(rows)


def load_e2e_delay_stats(path: Path) -> pd.DataFrame:
    columns = ["time_s", "delay_ms", "delay_min_ms", "delay_max_ms"]
    if not path.exists():
        return pd.DataFrame(columns=columns)
    raw = pd.read_csv(path, sep=r"\s+", engine="python", comment="%", on_bad_lines="skip")
    if raw.empty or len(raw.columns) < 14:
        return pd.DataFrame(columns=columns)
    raw = raw.iloc[:, :14].copy()
    raw.columns = [
        "start_s",
        "end_s",
        "cell_id",
        "imsi",
        "rnti",
        "lcid",
        "n_tx_pdus",
        "tx_bytes",
        "n_rx_pdus",
        "rx_bytes",
        "delay_s",
        "stddev_s",
        "min_s",
        "max_s",
    ]
    for column in ("start_s", "end_s", "delay_s", "min_s", "max_s"):
        raw[column] = pd.to_numeric(raw[column], errors="coerce")
    raw = raw.dropna(subset=["start_s", "end_s", "delay_s"]).reset_index(drop=True)
    raw["time_s"] = 0.5 * (raw["start_s"] + raw["end_s"])
    raw["delay_ms"] = raw["delay_s"] * 1e3
    raw["delay_min_ms"] = raw["min_s"] * 1e3
    raw["delay_max_ms"] = raw["max_s"] * 1e3
    return raw[columns]


def load_pdcp_rx_stats(path: Path) -> pd.DataFrame:
    columns = ["time_s", "packet_size", "delay_ms"]
    if not path.exists():
        return pd.DataFrame(columns=columns)
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or len(raw.columns) < 6:
        return pd.DataFrame(columns=columns)
    raw = raw.iloc[:, :6].copy()
    raw.columns = ["time_s", "cell_id", "rnti", "lcid", "packet_size", "delay_s"]
    raw["time_s"] = pd.to_numeric(raw["time_s"], errors="coerce")
    raw["packet_size"] = pd.to_numeric(raw["packet_size"], errors="coerce")
    raw["delay_s"] = pd.to_numeric(raw["delay_s"], errors="coerce")
    raw = raw.dropna(subset=["time_s", "delay_s"]).reset_index(drop=True)
    raw["delay_ms"] = raw["delay_s"] * 1e3
    return raw[columns]


def compute_modeled_latency(
    geometry: pd.DataFrame, s1u_link_delay_ms: float, remote_host_delay_ms: float
) -> pd.DataFrame:
    columns = [
        "time_s",
        "radio_one_way_ms",
        "one_way_total_ms",
        "rtt_total_ms",
    ]
    if geometry.empty or "time_s" not in geometry.columns or "slant_distance_km" not in geometry.columns:
        return pd.DataFrame(columns=columns)
    data = geometry[["time_s", "slant_distance_km"]].copy()
    data["radio_one_way_ms"] = (data["slant_distance_km"] * 1000.0 / 299_792_458.0) * 1e3
    fixed_one_way_ms = s1u_link_delay_ms + remote_host_delay_ms
    data["one_way_total_ms"] = data["radio_one_way_ms"] + fixed_one_way_ms
    data["rtt_total_ms"] = 2.0 * data["one_way_total_ms"]
    return data[columns]


def read_used_parameters(result_dir: Path) -> dict[str, str]:
    path = result_dir / USED_PARAM_FILE_NAME
    if not path.exists():
        return {}
    parameters: dict[str, str] = {}
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or "=" not in line:
                continue
            key, value = [part.strip() for part in line.split("=", 1)]
            parameters[key] = value
    return parameters


def load_ctrl_sinr(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or len(raw.columns) < 5:
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    raw = raw.iloc[:, :5].copy()
    raw.columns = ["time_s", "cell_id", "rnti", "bwp_id", "sinr_linear"]
    raw["time_s"] = pd.to_numeric(raw["time_s"], errors="coerce")
    raw["sinr_linear"] = pd.to_numeric(raw["sinr_linear"], errors="coerce")
    raw = raw.dropna(subset=["time_s", "sinr_linear"]).reset_index(drop=True)
    raw["sinr_db"] = np.nan
    positive = raw["sinr_linear"] > 0.0
    raw.loc[positive, "sinr_db"] = 10.0 * np.log10(raw.loc[positive, "sinr_linear"])
    return raw[["time_s", "sinr_db"]]


def load_pathloss(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "pathloss_db"])
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or len(raw.columns) < 5:
        return pd.DataFrame(columns=["time_s", "pathloss_db"])
    raw = raw.iloc[:, :5].copy()
    raw.columns = ["time_s", "cell_id", "bwp_id", "imsi", "pathloss_db"]
    raw["time_s"] = pd.to_numeric(raw["time_s"], errors="coerce")
    raw["pathloss_db"] = pd.to_numeric(raw["pathloss_db"], errors="coerce")
    raw = raw.dropna(subset=["time_s", "pathloss_db"]).reset_index(drop=True)
    raw["pathloss_db"] = np.abs(raw["pathloss_db"])
    return raw[["time_s", "pathloss_db"]]


def load_data_sinr(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or len(raw.columns) < 5:
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    raw = raw.iloc[:, :5].copy()
    raw.columns = ["time_s", "cell_id", "rnti", "bwp_id", "sinr_db"]
    raw["time_s"] = pd.to_numeric(raw["time_s"], errors="coerce")
    raw["sinr_db"] = pd.to_numeric(raw["sinr_db"], errors="coerce")
    raw = raw.dropna(subset=["time_s", "sinr_db"]).reset_index(drop=True)
    return raw[["time_s", "sinr_db"]]


def load_rx_packet_trace_sinr(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or "Time" not in raw.columns or "SINR(dB)" not in raw.columns:
        return pd.DataFrame(columns=["time_s", "sinr_db"])
    if "direction" in raw.columns:
        raw = raw[raw["direction"] == "DL"].copy()
    raw["time_s"] = pd.to_numeric(raw["Time"], errors="coerce")
    raw["sinr_db"] = pd.to_numeric(raw["SINR(dB)"], errors="coerce")
    raw = raw.dropna(subset=["time_s", "sinr_db"]).reset_index(drop=True)
    return raw[["time_s", "sinr_db"]]


def load_rx_packet_trace(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame()
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or "Time" not in raw.columns:
        return pd.DataFrame()
    column_map = {
        "Time": "time_s",
        "tbSize": "tb_size",
        "SINR(dB)": "sinr_db",
        "CQI": "cqi",
    }
    raw = raw.rename(columns=column_map)
    if "direction" in raw.columns:
        raw = raw[raw["direction"] == "DL"].copy()
    for column in ("time_s", "tb_size", "mcs", "rank", "rv", "sinr_db", "cqi", "corrupt", "TBler"):
        if column in raw.columns:
            raw[column] = pd.to_numeric(raw[column], errors="coerce")
    if "TBler" in raw.columns:
        raw = raw.rename(columns={"TBler": "tbler"})
    return raw.dropna(subset=["time_s"]).reset_index(drop=True)


def load_dl_sinr(result_dir: Path) -> pd.DataFrame:
    data_sinr = load_data_sinr(result_dir / "DlDataSinr.txt")
    if not data_sinr.empty:
        return data_sinr
    return load_rx_packet_trace_sinr(result_dir / "RxPacketTrace.txt")


def load_rank_trace(path: Path) -> pd.DataFrame:
    columns = ["time_s", "rank"]
    if not path.exists():
        return pd.DataFrame(columns=columns)
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or "Time" not in raw.columns or "rank" not in raw.columns:
        return pd.DataFrame(columns=columns)
    if "direction" in raw.columns:
        raw = raw[raw["direction"] == "DL"].copy()
    raw["time_s"] = pd.to_numeric(raw["Time"], errors="coerce")
    raw["rank"] = pd.to_numeric(raw["rank"], errors="coerce")
    raw = raw.dropna(subset=columns).reset_index(drop=True)
    return raw[columns]


def extract_rank2_times(rank: pd.DataFrame) -> pd.DataFrame:
    columns = ["time_s"]
    if rank.empty or "rank" not in rank.columns or "time_s" not in rank.columns:
        return pd.DataFrame(columns=columns)

    rank2 = rank.loc[rank["rank"] == 2, columns].dropna().sort_values("time_s")
    if len(rank2) <= MAX_RANK2_MARKERS:
        return rank2.reset_index(drop=True)

    stride = int(np.ceil(len(rank2) / MAX_RANK2_MARKERS))
    return rank2.iloc[::stride].reset_index(drop=True)


def select_rank2_series_points(series: pd.DataFrame, rank2: pd.DataFrame, y_col: str) -> pd.DataFrame:
    columns = ["time_s", y_col]
    if (
        series.empty
        or rank2.empty
        or "time_s" not in series.columns
        or "time_s" not in rank2.columns
        or y_col not in series.columns
    ):
        return pd.DataFrame(columns=columns)

    x = pd.to_numeric(series["time_s"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(series[y_col], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    x = x[valid]
    y = y[valid]
    if x.size == 0:
        return pd.DataFrame(columns=columns)

    order = np.argsort(x)
    x = x[order]
    y = y[order]
    times = pd.to_numeric(rank2["time_s"], errors="coerce").dropna().to_numpy(dtype=float)
    times = times[(times >= x.min()) & (times <= x.max())]
    if times.size == 0:
        return pd.DataFrame(columns=columns)

    indices = np.searchsorted(x, times)
    indices = np.clip(indices, 0, x.size - 1)
    previous = np.clip(indices - 1, 0, x.size - 1)
    use_previous = np.abs(times - x[previous]) < np.abs(times - x[indices])
    nearest = np.where(use_previous, previous, indices)
    nearest = np.unique(nearest)
    return pd.DataFrame({"time_s": x[nearest], y_col: y[nearest]})


def select_rank2_los_points(condition: pd.DataFrame, rank2: pd.DataFrame) -> pd.DataFrame:
    columns = ["time_s", "is_los"]
    if (
        condition.empty
        or rank2.empty
        or "time_s" not in condition.columns
        or "time_s" not in rank2.columns
        or "is_los" not in condition.columns
    ):
        return pd.DataFrame(columns=columns)

    condition_time = pd.to_numeric(condition["time_s"], errors="coerce").to_numpy(dtype=float)
    condition_los = pd.to_numeric(condition["is_los"], errors="coerce").to_numpy(dtype=float)
    valid_condition = np.isfinite(condition_time) & np.isfinite(condition_los)
    condition_time = condition_time[valid_condition]
    condition_los = condition_los[valid_condition]
    if condition_time.size == 0:
        return pd.DataFrame(columns=columns)

    order = np.argsort(condition_time)
    condition_time = condition_time[order]
    condition_los = condition_los[order]

    rank2_time = pd.to_numeric(rank2["time_s"], errors="coerce").to_numpy(dtype=float)
    rank2_time = rank2_time[np.isfinite(rank2_time)]
    rank2_time = rank2_time[
        (rank2_time >= condition_time.min()) & (rank2_time <= condition_time.max())
    ]
    if rank2_time.size == 0:
        return pd.DataFrame(columns=columns)

    condition_indices = np.searchsorted(condition_time, rank2_time, side="right") - 1
    condition_indices = np.clip(condition_indices, 0, condition_time.size - 1)
    los = np.round(condition_los[condition_indices]).clip(0, 1)
    return pd.DataFrame({"time_s": rank2_time, "is_los": los})


def add_direct_label(
    ax: plt.Axes,
    text: str,
    color: str,
    *,
    x_axes: float = 0.02,
    y_axes: float = 0.86,
) -> None:
    ax.text(
        x_axes,
        y_axes,
        text,
        transform=ax.transAxes,
        color=color,
        fontsize=LEGEND_FONT_SIZE,
        fontweight="bold",
        bbox={"facecolor": "white", "edgecolor": color, "alpha": 0.85, "pad": 3},
    )


def overlay_rank2_los_markers(ax: plt.Axes, rank2_los: pd.DataFrame) -> None:
    if rank2_los.empty or not {"time_s", "is_los"}.issubset(rank2_los.columns):
        return

    times = pd.to_numeric(rank2_los["time_s"], errors="coerce").to_numpy(dtype=float)
    los = pd.to_numeric(rank2_los["is_los"], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(times) & np.isfinite(los)
    if not np.any(valid):
        return
    times = times[valid]
    los = np.round(los[valid]).clip(0, 1)

    ax.scatter(
        times,
        los,
        marker="o",
        s=34,
        edgecolors="black",
        linewidths=1.3,
        color=RANK2_COLOR,
        alpha=0.9,
        zorder=5,
    )
    los_count = int(np.sum(los == 1))
    nlos_count = int(np.sum(los == 0))
    if los_count:
        add_direct_label(ax, f"Rank 2 LoS ({los_count})", RANK2_COLOR, x_axes=0.76, y_axes=0.74)
    if nlos_count:
        add_direct_label(ax, f"Rank 2 NLoS ({nlos_count})", RANK2_COLOR, x_axes=0.76, y_axes=0.14)


def load_channel_condition_trace(path: Path) -> pd.DataFrame:
    columns = ["time_s", "is_los", "condition"]
    if not path.exists():
        return pd.DataFrame(columns=columns)
    raw = pd.read_csv(path, sep=r"\s+", engine="python", on_bad_lines="skip")
    if raw.empty or "Time" not in raw.columns or "is_los" not in raw.columns:
        return pd.DataFrame(columns=columns)
    raw["time_s"] = pd.to_numeric(raw["Time"], errors="coerce")
    raw["is_los"] = pd.to_numeric(raw["is_los"], errors="coerce")
    if "condition" not in raw.columns:
        raw["condition"] = ""
    raw = raw.dropna(subset=["time_s", "is_los"]).reset_index(drop=True)
    return raw[columns]


def load_dl_mac_stats(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["time_s", "rv", "mcs", "tb_size"])
    raw = pd.read_csv(path, sep=r"\s+", engine="python", comment="%", on_bad_lines="skip")
    if raw.empty:
        return pd.DataFrame(columns=["time_s", "rv", "mcs", "tb_size"])
    expected_columns = [
        "time_s",
        "cell_id",
        "bwp_id",
        "imsi",
        "rnti",
        "frame",
        "sframe",
        "slot",
        "sym_start",
        "num_sym",
        "harq_id",
        "ndi",
        "rv",
        "mcs",
        "tb_size",
    ]
    column_count = min(len(raw.columns), len(expected_columns))
    raw = raw.iloc[:, :column_count].copy()
    raw.columns = expected_columns[:column_count]
    for column in ("time_s", "rv", "mcs", "tb_size"):
        if column in raw.columns:
            raw[column] = pd.to_numeric(raw[column], errors="coerce")
    needed = [column for column in ("time_s", "rv", "mcs", "tb_size") if column in raw.columns]
    if len(needed) < 4:
        return pd.DataFrame(columns=["time_s", "rv", "mcs", "tb_size"])
    return raw.dropna(subset=needed).reset_index(drop=True)[["time_s", "rv", "mcs", "tb_size"]]


def bin_edges(times: np.ndarray, window_s: float) -> np.ndarray:
    start = np.floor(np.min(times) / window_s) * window_s
    end = np.ceil(np.max(times) / window_s) * window_s
    edges = np.arange(start, end + window_s, window_s)
    if edges.size < 2:
        edges = np.array([start, start + window_s], dtype=float)
    return edges


def compute_throughput(mac: pd.DataFrame, window_s: float) -> pd.DataFrame:
    if mac.empty:
        return pd.DataFrame(columns=["time_s", "throughput_mbps"])
    mac = mac[mac["rv"] == 0].copy()
    if mac.empty:
        return pd.DataFrame(columns=["time_s", "throughput_mbps"])

    times = mac["time_s"].to_numpy(dtype=float)
    bits = mac["tb_size"].to_numpy(dtype=float) * 8.0
    edges = bin_edges(times, window_s)
    indices = np.digitize(times, edges) - 1
    valid = (indices >= 0) & (indices < edges.size - 1)
    bits_sum = np.bincount(indices[valid], weights=bits[valid], minlength=edges.size - 1)
    centers = edges[:-1] + (window_s / 2.0)
    throughput_mbps = bits_sum / window_s / 1e6
    return pd.DataFrame({"time_s": centers, "throughput_mbps": throughput_mbps})


def warn_missing(result_dir: Path, file_names: list[str]) -> None:
    missing = [name for name in file_names if not (result_dir / name).exists()]
    for name in missing:
        print(f"Warning: missing expected input file {name}")


def annotate_missing(ax: plt.Axes, message: str) -> None:
    ax.text(
        0.5,
        0.5,
        message,
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=ANNOTATION_FONT_SIZE,
        fontweight="bold",
    )
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)


def style_axis_text(ax: plt.Axes) -> None:
    ax.title.set_fontsize(TITLE_FONT_SIZE)
    ax.title.set_fontweight("bold")
    ax.xaxis.label.set_fontsize(LABEL_FONT_SIZE)
    ax.xaxis.label.set_fontweight("bold")
    ax.yaxis.label.set_fontsize(LABEL_FONT_SIZE)
    ax.yaxis.label.set_fontweight("bold")
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontsize(TICK_FONT_SIZE)
        label.set_fontweight("bold")


def plot_or_annotate_line(
    ax: plt.Axes,
    data: pd.DataFrame,
    x_col: str,
    y_col: str,
    *,
    title: str,
    ylabel: str,
    color: str,
    kind: str = "line",
    linewidth: float = 1.5,
    marker_size: float = 10.0,
) -> None:
    ax.set_ylabel(ylabel, fontsize=LABEL_FONT_SIZE, fontweight="bold")
    if data.empty or x_col not in data.columns or y_col not in data.columns:
        annotate_missing(ax, "No valid data")
        style_axis_text(ax)
        return
    x = pd.to_numeric(data[x_col], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(data[y_col], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    if not np.any(valid):
        annotate_missing(ax, "No valid data")
        style_axis_text(ax)
        return
    if kind == "scatter":
        ax.scatter(x[valid], y[valid], s=marker_size, color=color, alpha=0.7)
    elif kind == "step":
        ax.step(x[valid], y[valid], where="mid", color=color, linewidth=linewidth)
    else:
        ax.plot(x[valid], y[valid], color=color, linewidth=linewidth)
    ax.grid(alpha=0.3)
    style_axis_text(ax)


def plot_latency_panel(
    ax: plt.Axes,
    modeled: pd.DataFrame,
    pdcp_delay: pd.DataFrame,
    rlc_delay: pd.DataFrame,
    *,
    s1u_link_delay_ms: float,
    remote_host_delay_ms: float,
) -> None:
    ax.set_ylabel("Latency (ms)", fontsize=LABEL_FONT_SIZE, fontweight="bold")

    has_data = False
    if not modeled.empty:
        ax.plot(
            modeled["time_s"],
            modeled["radio_one_way_ms"],
            color="tab:gray",
            linestyle="--",
            linewidth=1.3,
            label="Radio one-way",
        )
        ax.plot(
            modeled["time_s"],
            modeled["one_way_total_ms"],
            color="tab:brown",
            linewidth=1.6,
            label="Modeled one-way total",
        )
        has_data = True

    if not pdcp_delay.empty:
        ax.step(
            pdcp_delay["time_s"],
            pdcp_delay["delay_ms"],
            where="mid",
            color="tab:blue",
            linewidth=1.4,
            label="PDCP E2E mean",
        )
        has_data = True

    if not rlc_delay.empty:
        ax.step(
            rlc_delay["time_s"],
            rlc_delay["delay_ms"],
            where="mid",
            color="tab:green",
            linewidth=1.4,
            label="RLC E2E mean",
        )
        has_data = True

    if not has_data:
        annotate_missing(ax, "No valid data")
        style_axis_text(ax)
        return

    ax.grid(alpha=0.3)
    legend = ax.legend(loc="best", fontsize=LEGEND_FONT_SIZE)
    if legend is not None:
        for text in legend.get_texts():
            text.set_fontweight("bold")
            text.set_fontsize(LEGEND_FONT_SIZE)
    ax.text(
        0.01,
        0.98,
        f"S1-U={s1u_link_delay_ms:.2f} ms, RemoteHost={remote_host_delay_ms:.2f} ms",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=ANNOTATION_FONT_SIZE,
        fontweight="bold",
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.7},
    )
    style_axis_text(ax)


def generate_figure1(result_dir: Path, config: RunConfig) -> Path:
    warn_missing(result_dir, [GEOMETRY_LOG_NAME, "DlPathlossTrace.txt"])
    geometry = parse_geometry_trace(result_dir / GEOMETRY_LOG_NAME)
    pathloss = load_pathloss(result_dir / "DlPathlossTrace.txt")

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)

    plot_or_annotate_line(
        axes[0],
        geometry,
        "time_s",
        "slant_distance_km",
        title="Slant Distance vs Time",
        ylabel="Distance (km)",
        color="tab:red",
    )
    plot_or_annotate_line(
        axes[1],
        geometry,
        "time_s",
        "elevation_deg",
        title="Elevation vs Time",
        ylabel="Elevation (deg)",
        color="tab:green",
    )
    plot_or_annotate_line(
        axes[2],
        pathloss,
        "time_s",
        "pathloss_db",
        title="Path Loss vs Time",
        ylabel="Path Loss (dB)",
        color="tab:purple",
    )
    axes[2].set_xlabel("Time (s)", fontsize=LABEL_FONT_SIZE, fontweight="bold")
    style_axis_text(axes[2])
    fig.tight_layout()

    output = result_dir / FIG1_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return output


def generate_figure2(result_dir: Path, config: RunConfig) -> Path:
    warn_missing(result_dir, ["NrDlMacStats.txt", GEOMETRY_LOG_NAME, "RxPacketTrace.txt"])
    if not (result_dir / "DlDataSinr.txt").exists() and not (result_dir / "RxPacketTrace.txt").exists():
        print("Warning: missing expected input file DlDataSinr.txt or RxPacketTrace.txt")
    mac = load_dl_mac_stats(result_dir / "NrDlMacStats.txt")
    throughput = compute_throughput(mac, config.throughput_window_s)
    data_sinr = load_dl_sinr(result_dir)
    rank = load_rank_trace(result_dir / "RxPacketTrace.txt")
    rank2 = extract_rank2_times(rank)
    rank2_throughput = select_rank2_series_points(throughput, rank2, "throughput_mbps")
    rank2_sinr = select_rank2_series_points(data_sinr, rank2, "sinr_db")

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)

    plot_or_annotate_line(
        axes[0],
        throughput,
        "time_s",
        "throughput_mbps",
        title=f"DL MAC Throughput vs Time ({config.throughput_window_s:.1f}s bins, rv=0)",
        ylabel="Throughput (Mbps)",
        color="tab:purple",
        kind="step",
    )
    if not rank2_throughput.empty:
        axes[0].scatter(
            rank2_throughput["time_s"],
            rank2_throughput["throughput_mbps"],
            s=36,
            color=RANK2_COLOR,
            edgecolors="black",
            linewidths=0.4,
            zorder=5,
        )
        add_direct_label(axes[0], "Rank 2 throughput", RANK2_COLOR)
    plot_or_annotate_line(
        axes[1],
        mac,
        "time_s",
        "mcs",
        title="MCS vs Time",
        ylabel="MCS",
        color="tab:orange",
        kind="scatter",
        marker_size=12.0,
    )
    plot_or_annotate_line(
        axes[2],
        data_sinr,
        "time_s",
        "sinr_db",
        title="DL Data SINR vs Time",
        ylabel="Effective SINR (dB)",
        color="tab:red",
        linewidth=1.0,
    )
    if not rank2_sinr.empty:
        axes[2].scatter(
            rank2_sinr["time_s"],
            rank2_sinr["sinr_db"],
            s=36,
            color=RANK2_COLOR,
            edgecolors="black",
            linewidths=0.4,
            zorder=5,
        )
        add_direct_label(axes[2], "Rank 2 SINR", RANK2_COLOR)
    axes[2].set_xlabel("Time (s)", fontsize=LABEL_FONT_SIZE, fontweight="bold")
    style_axis_text(axes[2])
    fig.tight_layout()

    output = result_dir / FIG2_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return output


def generate_figure3(result_dir: Path, config: RunConfig) -> Path:
    warn_missing(result_dir, [CHANNEL_CONDITION_TRACE_NAME, "RxPacketTrace.txt"])
    rank = load_rank_trace(result_dir / "RxPacketTrace.txt")
    rank2 = extract_rank2_times(rank)
    condition = load_channel_condition_trace(result_dir / CHANNEL_CONDITION_TRACE_NAME)

    fig, los_ax = plt.subplots(1, 1, figsize=(12, 4), sharex=True)
    los_ax.set_xlabel("Time (s)", fontsize=LABEL_FONT_SIZE, fontweight="bold")
    los_ax.set_ylabel("LoS", fontsize=LABEL_FONT_SIZE, fontweight="bold")
    if condition.empty:
        annotate_missing(los_ax, "No valid LoS data")
    else:
        rank2_los = select_rank2_los_points(condition, rank2)
        los_ax.step(
            condition["time_s"],
            condition["is_los"],
            where="post",
            color="tab:green",
            linewidth=1.5,
        )
        los_ax.fill_between(
            condition["time_s"],
            0,
            condition["is_los"],
            step="post",
            color="tab:green",
            alpha=0.18,
        )
        overlay_rank2_los_markers(los_ax, rank2_los)
        los_ax.set_yticks([0, 1])
        los_ax.set_yticklabels(["NLoS", "LoS"])
        los_ax.set_ylim(-0.1, 1.1)
    los_ax.grid(alpha=0.3)
    style_axis_text(los_ax)

    fig.tight_layout()
    output = result_dir / FIG3_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return output


def interpolate_elevation(geometry: pd.DataFrame, times: np.ndarray) -> np.ndarray:
    if geometry.empty:
        return np.full_like(times, np.nan, dtype=float)
    geometry = geometry.sort_values("time_s")
    return np.interp(
        times,
        geometry["time_s"].to_numpy(dtype=float),
        geometry["elevation_deg"].to_numpy(dtype=float),
    )


def stats_for(values: np.ndarray, label: str) -> dict[str, float | int]:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {
            f"{label}_n": 0,
            f"{label}_mean": np.nan,
            f"{label}_sigma": np.nan,
            f"{label}_p{PERCENTILE_LO}": np.nan,
            f"{label}_p{PERCENTILE_HI}": np.nan,
        }
    return {
        f"{label}_n": int(values.size),
        f"{label}_mean": float(np.mean(values)),
        f"{label}_sigma": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        f"{label}_p{PERCENTILE_LO}": float(np.percentile(values, PERCENTILE_LO)),
        f"{label}_p{PERCENTILE_HI}": float(np.percentile(values, PERCENTILE_HI)),
    }


def compute_elevation_tagged_throughput(
    mac: pd.DataFrame, geometry: pd.DataFrame, window_s: float
) -> pd.DataFrame:
    throughput = compute_throughput(mac, window_s)
    if throughput.empty:
        return pd.DataFrame(columns=["time_s", "throughput_mbps", "elevation_deg"])
    throughput["elevation_deg"] = interpolate_elevation(
        geometry, throughput["time_s"].to_numpy(dtype=float)
    )
    return throughput


def fold_pass_angle_to_elevation(values: np.ndarray) -> np.ndarray:
    """Map 0-180 degree pass angle samples to 0-90 degree elevation bins."""
    values = np.asarray(values, dtype=float)
    folded = 90.0 - np.abs(values - 90.0)
    return np.clip(folded, 0.0, 90.0)


def elevation_bin_label(lo: float) -> str:
    return f"{int(lo)}-{int(lo + ELEVATION_BIN_DEG)}deg"


def centered_elevation_bin_label(center: float) -> str:
    half_bin = ELEVATION_BIN_DEG / 2.0
    lo = max(0.0, center - half_bin)
    hi = min(90.0, center + half_bin)
    return f"{int(lo)}-{int(hi)}deg"


def build_elevation_stats(result_dir: Path, config: RunConfig) -> pd.DataFrame:
    geometry = parse_geometry_trace(result_dir / GEOMETRY_LOG_NAME)
    if geometry.empty:
        raise RuntimeError(f"{GEOMETRY_LOG_NAME} is empty or missing")
    geometry = geometry.sort_values("time_s").reset_index(drop=True)
    stats_geometry = geometry.copy()
    stats_geometry["elevation_deg"] = fold_pass_angle_to_elevation(
        stats_geometry["elevation_deg"].to_numpy(dtype=float)
    )

    rx = load_rx_packet_trace(result_dir / "RxPacketTrace.txt")
    if not rx.empty:
        rx["elevation_deg"] = interpolate_elevation(
            stats_geometry, rx["time_s"].to_numpy(dtype=float)
        )

    mac = load_dl_mac_stats(result_dir / "NrDlMacStats.txt")
    if not mac.empty:
        mac["elevation_deg"] = interpolate_elevation(
            stats_geometry, mac["time_s"].to_numpy(dtype=float)
        )

    sinr = load_dl_sinr(result_dir)
    if not sinr.empty:
        sinr["elevation_deg"] = interpolate_elevation(
            stats_geometry, sinr["time_s"].to_numpy(dtype=float)
        )

    pathloss = load_pathloss(result_dir / "DlPathlossTrace.txt")
    if not pathloss.empty:
        pathloss["elevation_deg"] = interpolate_elevation(
            stats_geometry, pathloss["time_s"].to_numpy(dtype=float)
        )

    throughput = compute_elevation_tagged_throughput(
        mac, stats_geometry, config.throughput_window_s
    )

    half_bin = ELEVATION_BIN_DEG / 2.0
    centers = np.arange(0.0, 90.0 + ELEVATION_BIN_DEG, ELEVATION_BIN_DEG)

    rows: list[dict[str, object]] = []
    for center in centers:
        lo = center - half_bin
        hi = center + half_bin
        display_lo = max(0.0, lo)
        display_hi = min(90.0, hi)
        row: dict[str, object] = {
            "elev_bin": centered_elevation_bin_label(center),
            "elev_lo": float(display_lo),
            "elev_hi": float(display_hi),
            "elev_center": float(center),
        }
        include_hi = center == centers[-1]

        if not sinr.empty:
            mask = (sinr["elevation_deg"] >= lo) & (
                (sinr["elevation_deg"] <= hi) if include_hi else (sinr["elevation_deg"] < hi)
            )
            row.update(stats_for(sinr.loc[mask, "sinr_db"].to_numpy(dtype=float), "sinr_db"))
        else:
            row.update(stats_for(np.array([], dtype=float), "sinr_db"))

        if not mac.empty:
            mac0 = mac[mac["rv"] == 0]
            mask = (mac0["elevation_deg"] >= lo) & (
                (mac0["elevation_deg"] <= hi) if include_hi else (mac0["elevation_deg"] < hi)
            )
            row.update(stats_for(mac0.loc[mask, "mcs"].to_numpy(dtype=float), "mcs"))
        else:
            row.update(stats_for(np.array([], dtype=float), "mcs"))

        if not throughput.empty:
            mask = (throughput["elevation_deg"] >= lo) & (
                (throughput["elevation_deg"] <= hi)
                if include_hi
                else (throughput["elevation_deg"] < hi)
            )
            row.update(
                stats_for(throughput.loc[mask, "throughput_mbps"].to_numpy(dtype=float),
                          "tput_mbps")
            )
        else:
            row.update(stats_for(np.array([], dtype=float), "tput_mbps"))

        if not rx.empty:
            mask = (rx["elevation_deg"] >= lo) & (
                (rx["elevation_deg"] <= hi) if include_hi else (rx["elevation_deg"] < hi)
            )
            sub = rx[mask]
            if "corrupt" in sub.columns:
                corrupt = pd.to_numeric(sub["corrupt"], errors="coerce").dropna()
                row["bler_n"] = int(corrupt.size)
                row["bler_n_corrupt"] = int(corrupt.sum()) if corrupt.size else 0
                row["bler"] = float(corrupt.mean()) if corrupt.size else np.nan
            if "cqi" in sub.columns:
                row.update(stats_for(sub["cqi"].dropna().to_numpy(dtype=float), "cqi"))

        if not pathloss.empty:
            mask = (pathloss["elevation_deg"] >= lo) & (
                (pathloss["elevation_deg"] <= hi)
                if include_hi
                else (pathloss["elevation_deg"] < hi)
            )
            values = pathloss.loc[mask, "pathloss_db"].to_numpy(dtype=float)
            values = values[np.isfinite(values)]
            row["pl_mean"] = float(np.mean(values)) if values.size else np.nan
            row["pl_sigma"] = float(np.std(values, ddof=1)) if values.size > 1 else np.nan

        rows.append(row)

    stats = pd.DataFrame(rows)
    numeric_columns = stats.select_dtypes(include=[np.number]).columns
    return stats[stats[numeric_columns].notna().any(axis=1)].reset_index(drop=True)


def print_elevation_table(
    stats: pd.DataFrame,
    title: str,
    columns: list[tuple[str, int]],
    headers: list[str],
    formats: list[str],
) -> None:
    separator = "-" * 100
    print()
    print(f"  {title}")
    print(separator)
    print("  " + "  ".join(f"{header:>{width}}" for header, (_, width) in zip(headers, columns)))
    print(separator)
    for _, row in stats.iterrows():
        parts = []
        for (column, width), fmt in zip(columns, formats):
            value = row.get(column, np.nan)
            if fmt == "s":
                parts.append(f"{str(value):>{width}}")
            elif pd.isna(value):
                parts.append(f"{'N/A':>{width}}")
            elif fmt == "d":
                parts.append(f"{int(value):>{width}}")
            else:
                parts.append(f"{float(value):>{width}.{fmt}}")
        print("  " + "  ".join(parts))


def print_elevation_report(stats: pd.DataFrame, throughput_window_s: float) -> None:
    print()
    print("=" * 100)
    print(f"  LEO LINK STATISTICS PER {ELEVATION_BIN_DEG}-DEGREE ELEVATION BIN")
    print("=" * 100)
    width = 12

    if "sinr_db_mean" in stats.columns:
        print_elevation_table(
            stats,
            "DL DATA SINR (dB)",
            [
                ("elev_bin", width),
                ("sinr_db_n", 8),
                ("sinr_db_mean", width),
                ("sinr_db_sigma", width),
                (f"sinr_db_p{PERCENTILE_LO}", width),
                (f"sinr_db_p{PERCENTILE_HI}", width),
            ],
            ["Elev Bin", "N", "Mean", "Sigma", f"P{PERCENTILE_LO}", f"P{PERCENTILE_HI}"],
            ["s", "d", "2f", "2f", "2f", "2f"],
        )

    if "mcs_mean" in stats.columns:
        print_elevation_table(
            stats,
            "MCS (new transmissions, rv=0)",
            [
                ("elev_bin", width),
                ("mcs_n", 8),
                ("mcs_mean", width),
                ("mcs_sigma", width),
                (f"mcs_p{PERCENTILE_LO}", width),
                (f"mcs_p{PERCENTILE_HI}", width),
            ],
            ["Elev Bin", "N", "Mean", "Sigma", f"P{PERCENTILE_LO}", f"P{PERCENTILE_HI}"],
            ["s", "d", "2f", "2f", "2f", "2f"],
        )

    if "tput_mbps_mean" in stats.columns:
        print_elevation_table(
            stats,
            f"DL THROUGHPUT (Mbps, {throughput_window_s:.1f}s windows, rv=0)",
            [
                ("elev_bin", width),
                ("tput_mbps_n", 8),
                ("tput_mbps_mean", width),
                ("tput_mbps_sigma", width),
                (f"tput_mbps_p{PERCENTILE_LO}", width),
                (f"tput_mbps_p{PERCENTILE_HI}", width),
            ],
            ["Elev Bin", "Windows", "Mean", "Sigma", f"P{PERCENTILE_LO}", f"P{PERCENTILE_HI}"],
            ["s", "d", "1f", "1f", "1f", "1f"],
        )

    if "bler" in stats.columns:
        print_elevation_table(
            stats,
            "BLER (all transmissions)",
            [("elev_bin", width), ("bler_n", 10), ("bler_n_corrupt", 10), ("bler", 10)],
            ["Elev Bin", "N TBs", "Corrupted", "BLER"],
            ["s", "d", "d", "4f"],
        )

    if "cqi_mean" in stats.columns:
        print_elevation_table(
            stats,
            "CQI",
            [
                ("elev_bin", width),
                ("cqi_n", 8),
                ("cqi_mean", width),
                ("cqi_sigma", width),
                (f"cqi_p{PERCENTILE_LO}", width),
                (f"cqi_p{PERCENTILE_HI}", width),
            ],
            ["Elev Bin", "N", "Mean", "Sigma", f"P{PERCENTILE_LO}", f"P{PERCENTILE_HI}"],
            ["s", "d", "2f", "2f", "2f", "2f"],
        )

    if "pl_mean" in stats.columns:
        print_elevation_table(
            stats,
            "DL PATHLOSS (dB)",
            [("elev_bin", width), ("pl_mean", width), ("pl_sigma", width)],
            ["Elev Bin", "Mean", "Sigma"],
            ["s", "2f", "4f"],
        )

    print()
    print("=" * 100)


def plot_elevation_metric_with_bands(
    ax: plt.Axes,
    stats: pd.DataFrame,
    centers: np.ndarray,
    mean_col: str,
    low_col: str,
    high_col: str,
    color: str,
    ylabel: str,
) -> None:
    y = stats[mean_col].to_numpy(dtype=float)
    low = stats[low_col].to_numpy(dtype=float)
    high = stats[high_col].to_numpy(dtype=float)
    yerr = np.vstack([np.clip(y - low, 0, None), np.clip(high - y, 0, None)])
    ax.errorbar(centers, y, yerr=yerr, fmt="o-", capsize=5, color=color, label="Simulation")
    ax.fill_between(centers, low, high, alpha=0.15, color=color)
    ax.set_ylabel(ylabel, fontsize=LABEL_FONT_SIZE)


def annotate_point_values(
    ax: plt.Axes, x_values: np.ndarray, y_values: np.ndarray, fmt: str
) -> None:
    for x_value, y_value in zip(x_values, y_values):
        if not np.isfinite(x_value) or not np.isfinite(y_value):
            continue
        ax.annotate(
            format(y_value, fmt),
            (x_value, y_value),
            xytext=(0, 6),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
            fontweight="bold",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.65, "pad": 1.0},
        )
    ax.margins(y=0.15)


def plot_elevation_summary(stats: pd.DataFrame, result_dir: Path) -> Path | None:
    panels: list[tuple[str, str]] = []
    if "sinr_db_mean" in stats.columns:
        panels.append(("sinr", "DL Data SINR (dB)"))
    if "mcs_mean" in stats.columns:
        panels.append(("mcs", "MCS (rv=0)"))
    if "tput_mbps_mean" in stats.columns:
        panels.append(("tput", "DL Throughput (Mbps)"))
    if "bler" in stats.columns:
        panels.append(("bler", "BLER"))
    if not panels:
        return None

    if "elev_center" in stats.columns:
        centers = stats["elev_center"].to_numpy(dtype=float)
        labels = [f"{int(center)}" for center in stats["elev_center"]]
    else:
        centers = ((stats["elev_lo"] + stats["elev_hi"]) / 2.0).to_numpy(dtype=float)
        labels = [f"{int(lo)}" for lo in stats["elev_lo"]]
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    ncols = 2
    nrows = int(np.ceil(len(panels) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4 * nrows), squeeze=False)
    axes_flat = axes.flatten()

    for index, (key, title) in enumerate(panels):
        ax = axes_flat[index]
        color = colors[index % len(colors)]
        if key == "sinr":
            plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "sinr_db_mean",
                f"sinr_db_p{PERCENTILE_LO}",
                f"sinr_db_p{PERCENTILE_HI}",
                color,
                "SINR (dB)",
            )
            annotate_point_values(ax, centers, stats["sinr_db_mean"].to_numpy(dtype=float), ".1f")
        elif key == "mcs":
            plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "mcs_mean",
                f"mcs_p{PERCENTILE_LO}",
                f"mcs_p{PERCENTILE_HI}",
                color,
                "MCS index",
            )
            annotate_point_values(ax, centers, stats["mcs_mean"].to_numpy(dtype=float), ".1f")
        elif key == "tput":
            plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "tput_mbps_mean",
                f"tput_mbps_p{PERCENTILE_LO}",
                f"tput_mbps_p{PERCENTILE_HI}",
                color,
                "Throughput (Mbps)",
            )
            annotate_point_values(
                ax, centers, stats["tput_mbps_mean"].to_numpy(dtype=float), ".1f"
            )
        elif key == "bler":
            bler = stats["bler"].to_numpy(dtype=float)
            ax.plot(centers, bler, "s-", color=color)
            ax.set_ylabel("BLER", fontsize=LABEL_FONT_SIZE)
            annotate_point_values(ax, centers, bler, ".3f")

        ax.set_xlabel("Elevation (deg)", fontsize=LABEL_FONT_SIZE, fontweight="bold")
        ax.set_xticks(centers)
        ax.set_xticklabels(labels, rotation=45)
        ax.grid(alpha=0.3)
        style_axis_text(ax)

    for index in range(len(panels), len(axes_flat)):
        axes_flat[index].set_visible(False)

    fig.tight_layout()

    output = result_dir / ELEVATION_STATS_FIG_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return output


def generate_elevation_analysis(result_dir: Path, config: RunConfig) -> list[Path]:
    stats = build_elevation_stats(result_dir, config)
    if stats.empty:
        print("Warning: no valid elevation statistics generated")
        return []

    report_buffer = io.StringIO()
    with contextlib.redirect_stdout(report_buffer):
        print_elevation_report(stats, config.throughput_window_s)
    report_text = report_buffer.getvalue()
    print(report_text, end="")

    report_output = result_dir / ELEVATION_STATS_REPORT_NAME
    report_output.write_text(report_text, encoding="utf-8")
    print(f"Saved elevation report: {report_output}")

    csv_output = result_dir / ELEVATION_STATS_CSV_NAME
    stats.to_csv(csv_output, index=False, float_format="%.4f")
    print(f"Saved elevation CSV: {csv_output}")

    outputs = [report_output, csv_output]
    fig_output = plot_elevation_summary(stats, result_dir)
    if fig_output is not None:
        outputs.append(fig_output)
        print(f"Saved elevation plot: {fig_output}")
    return outputs


def resolve_plot_config(result_dir: Path, fallback: RunConfig) -> RunConfig:
    used = read_used_parameters(result_dir)

    def float_or(key: str, default: float) -> float:
        try:
            return float(used.get(key, default))
        except ValueError:
            return default

    return RunConfig(
        duration=fallback.duration,
        precision=fallback.precision,
        scenario=fallback.scenario,
        frequency_hz=float_or("frequency_hz", fallback.frequency_hz),
        bandwidth_hz=float_or("bandwidth_hz", fallback.bandwidth_hz),
        sat_eirp_dbw_per_mhz=float_or("sat_eirp_dbw_per_mhz", fallback.sat_eirp_dbw_per_mhz),
        vsat_antenna_gain_db=float_or("vsat_antenna_gain_db", fallback.vsat_antenna_gain_db),
        sat_antenna_gain_db=float_or("sat_antenna_gain_db", fallback.sat_antenna_gain_db),
        transport=fallback.transport,
        udp_rate_mbps=fallback.udp_rate_mbps,
        udp_packet_size=fallback.udp_packet_size,
        s1u_link_delay_ms=float_or("s1u_link_delay_ms", fallback.s1u_link_delay_ms),
        remote_host_delay_ms=float_or(
            "remote_host_delay_ms", fallback.remote_host_delay_ms
        ),
        use_fixed_mcs=fallback.use_fixed_mcs,
        fixed_mcs=fallback.fixed_mcs,
        observer_mode=fallback.observer_mode,
        observer_elevation_boundary_deg=float_or(
            "observer_elevation_boundary_deg",
            fallback.observer_elevation_boundary_deg,
        ),
        enable_mimo_feedback=fallback.enable_mimo_feedback,
        mimo_pm_search_method=fallback.mimo_pm_search_method,
        mimo_full_search_codebook=fallback.mimo_full_search_codebook,
        mimo_rank_limit=fallback.mimo_rank_limit,
        mimo_rank_threshold=float_or("mimo_rank_threshold", fallback.mimo_rank_threshold),
        mimo_rank_technique=fallback.mimo_rank_technique,
        mimo_subband_size=fallback.mimo_subband_size,
        mimo_downsampling_technique=fallback.mimo_downsampling_technique,
        ue_antenna_rows=fallback.ue_antenna_rows,
        ue_antenna_columns=fallback.ue_antenna_columns,
        ue_num_vertical_ports=fallback.ue_num_vertical_ports,
        ue_num_horizontal_ports=fallback.ue_num_horizontal_ports,
        gnb_antenna_rows=fallback.gnb_antenna_rows,
        gnb_antenna_columns=fallback.gnb_antenna_columns,
        gnb_num_vertical_ports=fallback.gnb_num_vertical_ports,
        gnb_num_horizontal_ports=fallback.gnb_num_horizontal_ports,
        ue_is_dual_polarized=fallback.ue_is_dual_polarized,
        gnb_is_dual_polarized=fallback.gnb_is_dual_polarized,
        ue_pol_slant_angle_deg=float_or("ue_pol_slant_angle_deg", fallback.ue_pol_slant_angle_deg),
        gnb_pol_slant_angle_deg=float_or(
            "gnb_pol_slant_angle_deg",
            fallback.gnb_pol_slant_angle_deg,
        ),
        throughput_window_s=float_or("throughput_window_s", fallback.throughput_window_s),
    )


def generate_plots(result_dir: Path, config: RunConfig) -> list[Path]:
    plot_config = resolve_plot_config(result_dir, config)
    outputs = [
        generate_figure1(result_dir, plot_config),
        generate_figure2(result_dir, plot_config),
        generate_figure3(result_dir, plot_config),
    ]
    for path in outputs:
        print(f"Saved plot: {path}")
    outputs.extend(generate_elevation_analysis(result_dir, plot_config))
    return outputs


def validate_plot_input_dir(result_dir: Path) -> None:
    if not result_dir.exists():
        raise FileNotFoundError(f"Input directory does not exist: {result_dir}")
    if not result_dir.is_dir():
        raise NotADirectoryError(f"Input path is not a directory: {result_dir}")


def main() -> int:
    args = parse_args()
    ns3_root = resolve_ns3_root()
    config = RunConfig(
        duration=args.duration,
        precision=args.precision,
        scenario=args.scenario,
        frequency_hz=args.frequency_hz,
        bandwidth_hz=args.bandwidth_hz,
        sat_eirp_dbw_per_mhz=args.sat_eirp_dbw_per_mhz,
        vsat_antenna_gain_db=args.vsat_antenna_gain_db,
        sat_antenna_gain_db=args.sat_antenna_gain_db,
        transport=args.transport,
        udp_rate_mbps=args.udp_rate_mbps,
        udp_packet_size=args.udp_packet_size,
        s1u_link_delay_ms=args.s1u_link_delay_ms,
        remote_host_delay_ms=args.remote_host_delay_ms,
        use_fixed_mcs=args.use_fixed_mcs,
        fixed_mcs=args.fixed_mcs,
        observer_mode=args.observer_mode,
        observer_elevation_boundary_deg=args.observer_elevation_boundary_deg,
        enable_mimo_feedback=args.enable_mimo_feedback,
        mimo_pm_search_method=args.mimo_pm_search_method,
        mimo_full_search_codebook=args.mimo_full_search_codebook,
        mimo_rank_limit=args.mimo_rank_limit,
        mimo_rank_threshold=args.mimo_rank_threshold,
        mimo_rank_technique=args.mimo_rank_technique,
        mimo_subband_size=args.mimo_subband_size,
        mimo_downsampling_technique=args.mimo_downsampling_technique,
        ue_antenna_rows=args.ue_antenna_rows,
        ue_antenna_columns=args.ue_antenna_columns,
        ue_num_vertical_ports=args.ue_num_vertical_ports,
        ue_num_horizontal_ports=args.ue_num_horizontal_ports,
        gnb_antenna_rows=args.gnb_antenna_rows,
        gnb_antenna_columns=args.gnb_antenna_columns,
        gnb_num_vertical_ports=args.gnb_num_vertical_ports,
        gnb_num_horizontal_ports=args.gnb_num_horizontal_ports,
        ue_is_dual_polarized=args.ue_is_dual_polarized,
        gnb_is_dual_polarized=args.gnb_is_dual_polarized,
        ue_pol_slant_angle_deg=args.ue_pol_slant_angle_deg,
        gnb_pol_slant_angle_deg=args.gnb_pol_slant_angle_deg,
        throughput_window_s=args.throughput_window_s,
        enable_harq_retx=args.enable_harq_retx,
        enable_dynamic_rtt=args.enable_dynamic_rtt,
        num_harq_process=args.num_harq_process,
        n1_delay=args.n1_delay,
        n2_delay=args.n2_delay,
        rlc_reordering_timer_ms=args.rlc_reordering_timer_ms,
        num_ues=args.num_ues,
        ue_spacing_deg=args.ue_spacing_deg,
        ue_radii_km=args.ue_radii_km,
        beam_hpbw_deg=args.beam_hpbw_deg,
        ue_noise_figure_db=args.ue_noise_figure_db,
    )

    try:
        if args.plot_only:
            if not args.input_dir:
                raise ValueError("--plot-only requires --input-dir")
            result_dir = Path(args.input_dir).resolve()
            validate_plot_input_dir(result_dir)
            generate_plots(result_dir, config)
        else:
            results_root = Path(args.results_root).resolve()
            result_dir = create_result_dir(results_root)
            print(f"Created result directory: {result_dir}")
            write_used_parameters(result_dir, Path(args.param_file), args)
            run_leo_link(ns3_root, result_dir, config)
            generate_plots(result_dir, config)
            print(f"Results saved in: {result_dir}")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
