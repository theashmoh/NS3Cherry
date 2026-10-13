#!/usr/bin/env python3
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
"""
Run the leo-link ns-3 example with parameters loaded from
leo_link_analysis_params_D2C.txt, collect traces into a timestamped results
directory, and save the exact run configuration.
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

import run_leo_link_analysis as leo_plots


GEOMETRY_LOG_NAME = "log.txt"
RUN_LOG_NAME = "run.log"
DEFAULT_PARAM_FILE_NAME = "ntn/params/leo_link_analysis_params_D2C.txt"
USED_PARAM_FILE_NAME = "used_parameters.txt"
MAX_RANK_POINTS = 50000
SPEED_OF_LIGHT_MPS = 299_792_458.0


def parse_bool(raw_value: str) -> bool:
    value = raw_value.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: {raw_value}")




@dataclass(frozen=True)
class BentPipeRunConfig:
    parameters: dict[str, str]



def load_parameter_file(path: Path) -> dict[str, str]:
    if not path.exists():
        raise FileNotFoundError(f"Parameter file does not exist: {path}")

    parameters: dict[str, str] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError(f"Invalid parameter line {line_number} in {path}: {raw_line.rstrip()}")
            key, value = [part.strip() for part in line.split("=", 1)]
            parameters[key] = value
    return parameters



def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ns3_root = Path(__file__).resolve().parents[2]  # ntn/scripts/ -> ns-3 root
    default_param_file = ns3_root / DEFAULT_PARAM_FILE_NAME

    parser = argparse.ArgumentParser(
        description="Run leo-link with D2C parameters.",
    )
    parser.add_argument(
        "--param-file",
        default=str(default_param_file),
        help="Path to the key=value parameter file used for run settings.",
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
    param_file = Path(args.param_file).expanduser()
    if not param_file.is_absolute():
        param_file = (ns3_root / param_file).resolve()


    parameters = load_parameter_file(param_file)
    args.param_file = str(param_file)
    args.parameters = parameters
    return args


def validate_plot_input_dir(result_dir: Path) -> None:
    if not result_dir.exists():
        raise FileNotFoundError(f"Input directory does not exist: {result_dir}")
    if not result_dir.is_dir():
        raise NotADirectoryError(f"Input path is not a directory: {result_dir}")


def create_result_dir(results_root: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    result_dir = results_root / f"leo-link-{timestamp}"
    result_dir.mkdir(parents=True, exist_ok=False)
    return result_dir








def choose_rank_plot_points(rank: leo_plots.pd.DataFrame) -> leo_plots.pd.DataFrame:
    if len(rank) <= MAX_RANK_POINTS:
        return rank

    counts = rank["rank"].value_counts(dropna=True)
    if counts.empty:
        return rank.iloc[:: int(leo_plots.np.ceil(len(rank) / MAX_RANK_POINTS))].reset_index(
            drop=True
        )

    dominant_rank = counts.idxmax()
    rare_rank = rank[rank["rank"] != dominant_rank]
    dominant = rank[rank["rank"] == dominant_rank]
    dominant_budget = MAX_RANK_POINTS - len(rare_rank)
    if dominant_budget <= 0:
        stride = int(leo_plots.np.ceil(len(rare_rank) / MAX_RANK_POINTS))
        return rare_rank.iloc[::stride].reset_index(drop=True)

    stride = int(leo_plots.np.ceil(len(dominant) / dominant_budget))
    selected = leo_plots.pd.concat([dominant.iloc[::stride], rare_rank]).sort_index()
    return selected.reset_index(drop=True)


def build_bent_pipe_command(
    config: BentPipeRunConfig, trace_file: Path
) -> list[str]:
    """Build the ns3 run command, forwarding all parameters from the config file."""

    channel_condition_trace_file = trace_file.parent / leo_plots.CHANNEL_CONDITION_TRACE_NAME
    example_args = [
        "leo-link",
        "--payload=transparent",
        f"--linkBudgetTraceFile={shlex.quote(str(trace_file))}",
        f"--channelConditionTraceFile={shlex.quote(str(channel_condition_trace_file))}",
    ]
    
    # Map parameter names from file to CLI argument names
    param_to_arg = {
        "precision": "precision",
        "duration": "duration",
        "scenario": "scenario",
        "frequency_hz": "dlFrequencyHz",
        "bandwidth_hz": "bandwidthHz",
        "gateway_tx_power_dbm": "gatewayTxPowerDbm",
        "ue_tx_power_dbm": "ueTxPowerDbm",
        "sat_feeder_rx_gain_db": "satFeederRxGainDb",
        "sat_eirp_dbw_per_mhz": "satEIRP",
        "sat_antenna_gain_db": "satAntennaGainDb",
        "gateway_lat_deg": "gatewayLatDeg",
        "gateway_lon_deg": "gatewayLonDeg",
        "relay_loss_db": "relayLossDb",
        "receiver_nf_db": "receiverNfDb",
        "relay_processing_delay_us": "relayProcessingDelayUs",
        "remote_host_delay_ms": "remoteHostDelayMs",
        "s1u_link_delay_ms": "s1uLinkDelayMs",
        "transport": "transport",
        "udp_rate_mbps": "udpRateMbps",
        "udp_packet_size": "udpPacketSize",
        "use_fixed_mcs": "useFixedMcs",
        "fixed_mcs": "fixedMcs",
        "observer_mode": "observerMode",
        "observer_elevation_boundary_deg": "observerElevationBoundaryDeg",
        "observer_lat_deg": "observerLatDeg",
        "observer_lon_deg": "observerLonDeg",
        "enable_mimo_feedback": "enableMimoFeedback",
        "mimo_pm_search_method": "pmSearchMethod",
        "mimo_full_search_codebook": "fullSearchCb",
        "mimo_rank_limit": "rankLimit",
        "mimo_rank_threshold": "rankThreshold",
        "mimo_rank_technique": "rankTechnique",
        "mimo_subband_size": "subbandSize",
        "mimo_downsampling_technique": "downsamplingTechnique",
        "ue_antenna_rows": "ueAntennaRows",
        "ue_antenna_columns": "ueAntennaColumns",
        "ue_num_vertical_ports": "ueNumVerticalPorts",
        "ue_num_horizontal_ports": "ueNumHorizontalPorts",
        "gnb_antenna_rows": "gnbAntennaRows",
        "gnb_antenna_columns": "gnbAntennaColumns",
        "gnb_num_vertical_ports": "gnbNumVerticalPorts",
        "gnb_num_horizontal_ports": "gnbNumHorizontalPorts",
        "ue_is_dual_polarized": "ueIsDualPolarized",
        "gnb_is_dual_polarized": "gnbIsDualPolarized",
        "ue_pol_slant_angle_deg": "uePolSlantAngleDeg",
        "gnb_pol_slant_angle_deg": "gnbPolSlantAngleDeg",
        "enable_harq_retx": "enableHarqReTx",
        "num_harq_process": "numHarqProcess",
        "n1_delay": "n1Delay",
        "n2_delay": "n2Delay",
        "rlc_reordering_timer_ms": "rlcReorderingTimerMs",
        "num_ues": "numUes",
        "ue_spacing_deg": "ueSpacingDeg",
    }
    
    for param_key, arg_name in param_to_arg.items():
        if param_key in config.parameters:
            value = config.parameters[param_key]
            # Convert boolean string values
            if param_key in {
                "use_fixed_mcs",
                "enable_mimo_feedback",
                "ue_is_dual_polarized",
                "gnb_is_dual_polarized",
            }:
                value = "1" if parse_bool(value) else "0"
            example_args.append(f"--{arg_name}={value}")
    
    # Each run executes in its own result directory, so every trace lands there.
    return ["./ns3", "run", f"--cwd={trace_file.parent}", " ".join(example_args)]


def write_used_parameters(result_dir: Path, param_file: Path, parameters: dict[str, str]) -> None:
    lines = [f"param_file={param_file}", "example=leo-link"]
    lines.extend(f"{key}={value}" for key, value in sorted(parameters.items()))
    (result_dir / USED_PARAM_FILE_NAME).write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_bent_pipe(
    ns3_root: Path,
    result_dir: Path,
    config: BentPipeRunConfig,
) -> None:
    trace_file = result_dir / GEOMETRY_LOG_NAME
    run_log = result_dir / RUN_LOG_NAME
    command = build_bent_pipe_command(config, trace_file)
    run_env = leo_plots.ns3_run_environment()

    with run_log.open("w", encoding="utf-8") as log_handle:
        log_handle.write(f"cwd={result_dir}\n")
        log_handle.write(f"command={' '.join(command)}\n\n")
        log_handle.write(f"PATH={run_env.get('PATH', '')}\n\n")
        log_handle.flush()
        import resource
        import time as time_mod

        wall_start = time_mod.monotonic()
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
        child_max_rss_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
        (result_dir / "runtime.txt").write_text(
            f"wall_seconds={wall_elapsed:.1f}\n"
            f"child_max_rss_mib={child_max_rss_kib / 1024.0:.1f}\n"
        )
        print(f"Run wall time: {wall_elapsed:.1f} s; child max RSS: "
              f"{child_max_rss_kib / 1024.0:.1f} MiB")

    if completed.returncode != 0:
        raise RuntimeError(
            f"leo-link failed with exit code {completed.returncode}. See {run_log}"
        )


def free_space_path_loss_db(distance_m: leo_plots.np.ndarray, frequency_hz: float) -> leo_plots.np.ndarray:
    distance_m = leo_plots.np.maximum(distance_m, 1.0)
    return 20.0 * leo_plots.np.log10(
        (4.0 * leo_plots.np.pi * distance_m * frequency_hz) / SPEED_OF_LIGHT_MPS
    )


def load_bent_pipe_geometry(path: Path, frequency_hz: float) -> leo_plots.pd.DataFrame:
    columns = [
        "time_s",
        "gt_sat1_distance_km",
        "isl_distance_km",
        "sat2_ue_distance_km",
        "total_distance_km",
        "gt_sat1_elevation_deg",
        "sat2_ue_elevation_deg",
        "elevation_deg",
        "pathloss_db",
        "ul_pathloss_db",
        "gt_sat1_loss_db",
        "gt_sat1_loss_delta_db",
        "sat2_ue_loss_db",
        "sat2_ue_loss_delta_db",
        "access_loss_delta_sum_db",
        "radio_one_way_delay_ms",
        "snr_sat1_db",
        "snr_sat2_db",
        "gamma2_db",
        "gamma_ue_cascade_db",
        "gamma_ue_formula_db",
        "snr_ue_db",
    ]
    if not path.exists():
        return leo_plots.pd.DataFrame(columns=columns)

    raw = leo_plots.pd.read_csv(path, sep=r"\s+", engine="python")
    required = {
        "Time",
        "GT_sat1_distance_m",
        "ISL_distance_m",
        "sat2_UE_distance_m",
        "GT_sat1_elevation_deg",
        "sat2_UE_elevation_deg",
    }
    if raw.empty or not required.issubset(raw.columns):
        return leo_plots.pd.DataFrame(columns=columns)

    data = leo_plots.pd.DataFrame()
    data["time_s"] = leo_plots.pd.to_numeric(raw["Time"], errors="coerce")
    data["gt_sat1_distance_m"] = leo_plots.pd.to_numeric(
        raw["GT_sat1_distance_m"], errors="coerce"
    )
    data["isl_distance_m"] = leo_plots.pd.to_numeric(raw["ISL_distance_m"], errors="coerce")
    data["sat2_ue_distance_m"] = leo_plots.pd.to_numeric(
        raw["sat2_UE_distance_m"], errors="coerce"
    )
    data["gt_sat1_elevation_deg"] = leo_plots.pd.to_numeric(
        raw["GT_sat1_elevation_deg"], errors="coerce"
    )
    data["sat2_ue_elevation_deg"] = leo_plots.pd.to_numeric(
        raw["sat2_UE_elevation_deg"], errors="coerce"
    )
    data = data.dropna()
    if data.empty:
        return leo_plots.pd.DataFrame(columns=columns)
    raw_valid = raw.loc[data.index].reset_index(drop=True)
    data = data.reset_index(drop=True)

    data["gt_sat1_distance_km"] = data["gt_sat1_distance_m"] / 1000.0
    data["isl_distance_km"] = data["isl_distance_m"] / 1000.0
    data["sat2_ue_distance_km"] = data["sat2_ue_distance_m"] / 1000.0
    data["total_distance_km"] = (
        data["gt_sat1_distance_km"] + data["isl_distance_km"] + data["sat2_ue_distance_km"]
    )
    data["elevation_deg"] = data[
        ["gt_sat1_elevation_deg", "sat2_ue_elevation_deg"]
    ].min(axis=1)

    if "DL_pathloss_db" in raw.columns:
        data["pathloss_db"] = leo_plots.pd.to_numeric(raw_valid["DL_pathloss_db"], errors="coerce")
    else:
        data["pathloss_db"] = (
            free_space_path_loss_db(data["gt_sat1_distance_m"].to_numpy(dtype=float),
                                    frequency_hz)
            + free_space_path_loss_db(data["isl_distance_m"].to_numpy(dtype=float),
                                      frequency_hz)
            + free_space_path_loss_db(data["sat2_ue_distance_m"].to_numpy(dtype=float),
                                      frequency_hz)
        )
    if "UL_pathloss_db" in raw.columns:
        data["ul_pathloss_db"] = leo_plots.pd.to_numeric(raw_valid["UL_pathloss_db"], errors="coerce")
    else:
        data["ul_pathloss_db"] = data["pathloss_db"]

    optional_columns = {
        "GT_sat1_loss_db": "gt_sat1_loss_db",
        "GT_sat1_loss_delta_db": "gt_sat1_loss_delta_db",
        "sat2_UE_loss_db": "sat2_ue_loss_db",
        "sat2_UE_loss_delta_db": "sat2_ue_loss_delta_db",
        "access_loss_delta_sum_db": "access_loss_delta_sum_db",
        "radio_one_way_delay_ms": "radio_one_way_delay_ms",
        "SNR_sat1_db": "snr_sat1_db",
        "SNR_sat2_db": "snr_sat2_db",
        "gamma_2_db": "gamma2_db",
        "gamma_UE_cascade_db": "gamma_ue_cascade_db",
        "SNR_UE_db": "snr_ue_db",
    }
    for raw_name, output_name in optional_columns.items():
        if raw_name in raw_valid.columns:
            data[output_name] = leo_plots.pd.to_numeric(raw_valid[raw_name], errors="coerce")
        else:
            data[output_name] = leo_plots.np.nan
    gamma1_linear = 10.0 ** (data["snr_sat2_db"] / 10.0)
    gamma2_linear = 10.0 ** (data["gamma2_db"] / 10.0)
    gamma_ue_linear = (gamma1_linear * gamma2_linear) / (
        gamma1_linear + gamma2_linear + 1.0
    )
    data["gamma_ue_formula_db"] = 10.0 * leo_plots.np.log10(gamma_ue_linear)
    return data[columns]


def load_bent_pipe_geometry_for_elevation_stats(
    result_dir: Path, frequency_hz: float
) -> leo_plots.pd.DataFrame:
    geometry = load_bent_pipe_geometry(result_dir / GEOMETRY_LOG_NAME, frequency_hz)
    if geometry.empty:
        return leo_plots.pd.DataFrame(columns=["time_s", "elevation_deg", "slant_distance_km"])
    data = geometry[["time_s", "elevation_deg", "total_distance_km"]].copy()
    data = data.rename(columns={"total_distance_km": "slant_distance_km"})
    return data.sort_values("time_s").reset_index(drop=True)


def load_bent_pipe_dl_sinr_1s(result_dir: Path) -> leo_plots.pd.DataFrame:
    sinr = load_bent_pipe_cascade_sinr(result_dir)
    if sinr.empty:
        return leo_plots.pd.DataFrame(columns=["time_s", "sinr_db"])
    data = sinr[["time_s", "sinr_db"]].copy()
    data["time_bin_s"] = leo_plots.np.floor(data["time_s"].to_numpy(dtype=float))
    return (
        data.groupby("time_bin_s", as_index=False)["sinr_db"]
        .mean()
        .rename(columns={"time_bin_s": "time_s"})
        .sort_values("time_s")
        .reset_index(drop=True)
    )


def load_bent_pipe_cascade_sinr(result_dir: Path) -> leo_plots.pd.DataFrame:
    geometry = load_bent_pipe_geometry(result_dir / GEOMETRY_LOG_NAME, 0.0)
    columns = ["time_s", "sinr_db"]
    if geometry.empty:
        return leo_plots.pd.DataFrame(columns=columns)

    if "gamma_ue_cascade_db" in geometry.columns and geometry["gamma_ue_cascade_db"].notna().any():
        data = leo_plots.pd.DataFrame(
            {
                "time_s": geometry["time_s"],
                "sinr_db": geometry["gamma_ue_cascade_db"],
            }
        )
    elif {"snr_sat2_db", "gamma2_db"}.issubset(geometry.columns):
        gamma1 = 10.0 ** (geometry["snr_sat2_db"].to_numpy(dtype=float) / 10.0)
        gamma2 = 10.0 ** (geometry["gamma2_db"].to_numpy(dtype=float) / 10.0)
        gamma_ue = (gamma1 * gamma2) / (gamma1 + gamma2 + 1.0)
        data = leo_plots.pd.DataFrame(
            {
                "time_s": geometry["time_s"],
                "sinr_db": 10.0 * leo_plots.np.log10(gamma_ue),
            }
        )
    else:
        return leo_plots.pd.DataFrame(columns=columns)

    return data.dropna(subset=columns).sort_values("time_s").reset_index(drop=True)


def load_bent_pipe_gamma_sinr(result_dir: Path) -> leo_plots.pd.DataFrame:
    columns = ["time_s", "gamma1_db", "gamma2_db"]
    raw_path = result_dir / GEOMETRY_LOG_NAME
    if not raw_path.exists():
        return leo_plots.pd.DataFrame(columns=columns)
    raw = leo_plots.pd.read_csv(raw_path, sep=r"\s+", engine="python")
    if raw.empty:
        return leo_plots.pd.DataFrame(columns=columns)

    if "gamma_1_db" in raw.columns:
        gamma1 = raw["gamma_1_db"]
    elif "SNR_sat2_db" in raw.columns:
        gamma1 = raw["SNR_sat2_db"]
    else:
        return leo_plots.pd.DataFrame(columns=columns)

    if "gamma_2_db" not in raw.columns:
        return leo_plots.pd.DataFrame(columns=columns)

    data = leo_plots.pd.DataFrame(
        {
            "time_s": leo_plots.pd.to_numeric(raw["Time"], errors="coerce"),
            "gamma1_db": leo_plots.pd.to_numeric(gamma1, errors="coerce"),
            "gamma2_db": leo_plots.pd.to_numeric(raw["gamma_2_db"], errors="coerce"),
        }
    )
    return data.dropna(subset=columns).sort_values("time_s").reset_index(drop=True)


def load_bent_pipe_channel_condition(path: Path) -> leo_plots.pd.DataFrame:
    columns = ["time_s", "is_los", "gt_sat1_is_los", "sat2_ue_is_los"]
    if not path.exists():
        return leo_plots.pd.DataFrame(columns=columns)
    raw = leo_plots.pd.read_csv(path, sep=r"\s+", engine="python")
    if raw.empty or "Time" not in raw.columns or "is_los" not in raw.columns:
        return leo_plots.pd.DataFrame(columns=columns)
    raw = raw.rename(columns={"Time": "time_s"})
    for column in columns:
        if column in raw.columns:
            raw[column] = leo_plots.pd.to_numeric(raw[column], errors="coerce")
    if "gt_sat1_is_los" not in raw.columns:
        raw["gt_sat1_is_los"] = leo_plots.np.nan
    if "sat2_ue_is_los" not in raw.columns:
        raw["sat2_ue_is_los"] = leo_plots.np.nan
    return raw.dropna(subset=["time_s", "is_los"]).reset_index(drop=True)[columns]


def load_bent_pipe_phy_sinr_1s(result_dir: Path) -> leo_plots.pd.DataFrame:
    sinr = leo_plots.load_dl_sinr(result_dir)
    if sinr.empty:
        return leo_plots.pd.DataFrame(columns=["time_s", "sinr_db"])
    data = sinr[["time_s", "sinr_db"]].copy()
    data["time_bin_s"] = leo_plots.np.floor(data["time_s"].to_numpy(dtype=float))
    return (
        data.groupby("time_bin_s", as_index=False)["sinr_db"]
        .mean()
        .rename(columns={"time_bin_s": "time_s"})
        .sort_values("time_s")
        .reset_index(drop=True)
    )


def generate_bent_pipe_figure1(
    result_dir: Path, frequency_hz: float, min_service_elev_deg: float = 0.0
) -> Path:
    geometry = load_bent_pipe_geometry(result_dir / GEOMETRY_LOG_NAME, frequency_hz)
    gamma_sinr = load_bent_pipe_gamma_sinr(result_dir)

    # Optionally clip to the useful service window (service elevation >= threshold).
    # Below the horizon the 3GPP NTN path-loss elevation terms diverge (1/sin(elev)),
    # producing an unphysical loss/SINR spike at the ~0 deg crossing.
    if (
        min_service_elev_deg > 0.0
        and not geometry.empty
        and "sat2_ue_elevation_deg" in geometry.columns
    ):
        useful = geometry[geometry["sat2_ue_elevation_deg"] >= min_service_elev_deg]
        if not useful.empty:
            t_lo, t_hi = float(useful["time_s"].min()), float(useful["time_s"].max())
            geometry = geometry[
                (geometry["time_s"] >= t_lo) & (geometry["time_s"] <= t_hi)
            ]
            if not gamma_sinr.empty:
                gamma_sinr = gamma_sinr[
                    (gamma_sinr["time_s"] >= t_lo) & (gamma_sinr["time_s"] <= t_hi)
                ]

    fig, axes = leo_plots.plt.subplots(4, 1, figsize=(12, 8.4), sharex=False)

    # Subplot 0: Slant distances for two satellites
    axes[0].set_ylabel("Slant Distance (km)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if geometry.empty:
        leo_plots.annotate_missing(axes[0], "No valid geometry data")
    else:
        axes[0].plot(geometry["time_s"], geometry["gt_sat1_distance_km"], label="Satellite 1 (GT-sat1)",
                     color="tab:blue", linewidth=1.5)
        axes[0].plot(geometry["time_s"], geometry["sat2_ue_distance_km"], label="Satellite 2 (sat2-UE)",
                     color="tab:green", linewidth=1.5)
        axes[0].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    axes[0].grid(alpha=0.3)
    leo_plots.style_axis_text(axes[0])

    # Subplot 1: Elevation angles for two satellites
    axes[1].set_ylabel("Elevation Angle (deg)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if geometry.empty:
        leo_plots.annotate_missing(axes[1], "No valid geometry data")
    else:
        axes[1].plot(geometry["time_s"], geometry["gt_sat1_elevation_deg"], label="Satellite 1 (GT-sat1)",
                     color="tab:blue", linewidth=1.5)
        axes[1].plot(geometry["time_s"], geometry["sat2_ue_elevation_deg"], label="Satellite 2 (sat2-UE)",
                     color="tab:green", linewidth=1.5)
        axes[1].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    axes[1].grid(alpha=0.3)
    leo_plots.style_axis_text(axes[1])

    # Subplot 2: feeder/access hop SINR diagnostics
    axes[2].set_ylabel(
        r"$\gamma_1$, $\gamma_2$ SINR (dB)",
        fontsize=leo_plots.LABEL_FONT_SIZE,
        fontweight="bold",
    )
    if gamma_sinr.empty:
        leo_plots.annotate_missing(axes[2], "No gamma1/gamma2 SINR data")
    else:
        axes[2].plot(gamma_sinr["time_s"], gamma_sinr["gamma1_db"],
                     label=r"$\gamma_1$ SINR",
                     color="tab:blue",
                     linewidth=1.2)
        axes[2].plot(gamma_sinr["time_s"], gamma_sinr["gamma2_db"],
                     label=r"$\gamma_2$ SINR",
                     color="tab:red",
                     linewidth=1.2)
        axes[2].axhline(0.0, color="0.4", linewidth=0.8)
        axes[2].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    axes[2].grid(alpha=0.3)
    leo_plots.style_axis_text(axes[2])

    # Subplot 3: Feeder and service link path losses
    axes[3].set_ylabel("Path Loss (dB)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    axes[3].set_xlabel("Time (s)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if geometry.empty:
        leo_plots.annotate_missing(axes[3], "No valid geometry data")
    elif geometry["gt_sat1_loss_db"].notna().any() or geometry["sat2_ue_loss_db"].notna().any():
        axes[3].plot(geometry["time_s"], geometry["gt_sat1_loss_db"],
                     label="Feeder link (GT-sat1)", color="tab:blue", linewidth=1.5)
        axes[3].plot(geometry["time_s"], geometry["sat2_ue_loss_db"],
                     label="Service link (sat2-UE)", color="tab:green", linewidth=1.5)
        axes[3].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    else:
        leo_plots.plot_or_annotate_line(
            axes[3],
            geometry,
            "time_s",
            "pathloss_db",
            title="",
            ylabel="Path Loss (dB)",
            color="tab:purple",
        )
    axes[3].grid(alpha=0.3)
    leo_plots.style_axis_text(axes[3])
    fig.tight_layout()

    output = result_dir / leo_plots.FIG1_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    leo_plots.plt.close(fig)
    return output


def generate_bent_pipe_figure2(
    result_dir: Path, config: object, frequency_hz: float, min_service_elev_deg: float = 0.0
) -> Path:
    leo_plots.warn_missing(result_dir, ["NrDlMacStats.txt", "RxPacketTrace.txt", GEOMETRY_LOG_NAME])
    mac = leo_plots.load_dl_mac_stats(result_dir / "NrDlMacStats.txt")
    throughput = leo_plots.compute_throughput(mac, config.throughput_window_s)
    phy_sinr = load_bent_pipe_phy_sinr_1s(result_dir)

    # Optionally clip to the useful service window (elevation >= threshold); below the
    # horizon the geometry-derived SINR diverges (1/sin(elev) path-loss terms).
    if min_service_elev_deg > 0.0:
        geom = load_bent_pipe_geometry(result_dir / GEOMETRY_LOG_NAME, frequency_hz)
        if not geom.empty and "sat2_ue_elevation_deg" in geom.columns:
            useful = geom[geom["sat2_ue_elevation_deg"] >= min_service_elev_deg]
            if not useful.empty:
                t_lo, t_hi = float(useful["time_s"].min()), float(useful["time_s"].max())
                for df in (mac, throughput, phy_sinr):
                    if not df.empty and "time_s" in df.columns:
                        df.drop(df[(df["time_s"] < t_lo) | (df["time_s"] > t_hi)].index,
                                inplace=True)

    fig, axes = leo_plots.plt.subplots(3, 1, figsize=(12, 7.2), sharex=False)

    leo_plots.plot_or_annotate_line(
        axes[0],
        throughput,
        "time_s",
        "throughput_mbps",
        title=f"DL MAC Throughput vs Time ({config.throughput_window_s:.1f}s bins, rv=0)",
        ylabel="Throughput (Mbps)",
        color="tab:purple",
        kind="step",
    )
    leo_plots.plot_or_annotate_line(
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
    axes[2].set_ylabel("SINR (dB)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if phy_sinr.empty:
        leo_plots.annotate_missing(axes[2], "No valid SINR data")
    else:
        axes[2].plot(
            phy_sinr["time_s"],
            phy_sinr["sinr_db"],
            color="tab:red",
            linewidth=1.0,
            label=r"$\gamma_{UE}$ SINR",
        )
        axes[2].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
        axes[2].grid(alpha=0.3)
    axes[2].set_xlabel("Time (s)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    leo_plots.style_axis_text(axes[2])
    fig.tight_layout()

    output = result_dir / leo_plots.FIG2_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    leo_plots.plt.close(fig)
    return output


def generate_bent_pipe_figure3(result_dir: Path) -> Path:
    rank = leo_plots.load_rank_trace(result_dir / "RxPacketTrace.txt")
    condition = load_bent_pipe_channel_condition(
        result_dir / leo_plots.CHANNEL_CONDITION_TRACE_NAME
    )

    fig, axes = leo_plots.plt.subplots(2, 1, figsize=(12, 5.0), sharex=True)

    axes[0].set_ylabel("Rank", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if rank.empty:
        leo_plots.annotate_missing(axes[0], "No valid rank data")
    else:
        plot_rank = choose_rank_plot_points(rank)
        axes[0].scatter(
            plot_rank["time_s"],
            plot_rank["rank"],
            s=7.0,
            color="tab:blue",
            alpha=0.55,
            label="DL rank",
        )
        finite_rank = rank["rank"].dropna()
        if not finite_rank.empty:
            rank_min = int(finite_rank.min())
            rank_max = int(finite_rank.max())
            axes[0].set_yticks(range(rank_min, rank_max + 1))
            axes[0].set_ylim(rank_min - 0.4, rank_max + 0.4)
        axes[0].grid(alpha=0.3)
        axes[0].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    leo_plots.style_axis_text(axes[0])

    axes[1].set_xlabel("Time (s)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    axes[1].set_ylabel("LoS", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
    if not condition.empty:
        los_time = condition["time_s"]
        los_column = "sat2_ue_is_los"
        if los_column not in condition.columns or condition[los_column].isna().all():
            los_column = "is_los"
        is_los = condition[los_column]
    else:
        los_time = None
        is_los = None

    if los_time is None or is_los is None:
        leo_plots.annotate_missing(axes[1], "No valid LoS data")
    else:
        axes[1].step(
            los_time,
            is_los,
            where="post",
            color="tab:red",
            linewidth=1.6,
            label="sat2-UE",
        )
        axes[1].set_yticks([0, 1])
        axes[1].set_yticklabels(["NLoS", "LoS"])
        axes[1].set_ylim(-0.1, 1.1)
        axes[1].grid(alpha=0.3)
        axes[1].legend(loc="best", fontsize=leo_plots.LEGEND_FONT_SIZE)
    leo_plots.style_axis_text(axes[1])

    fig.tight_layout()
    output = result_dir / leo_plots.FIG3_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    leo_plots.plt.close(fig)
    return output


def build_bent_pipe_elevation_stats(result_dir: Path, config: object) -> leo_plots.pd.DataFrame:
    frequency_hz = float(getattr(config, "frequency_hz", 2e9))
    throughput_window_s = float(getattr(config, "throughput_window_s", 1.0))
    geometry = load_bent_pipe_geometry_for_elevation_stats(result_dir, frequency_hz)
    if geometry.empty:
        raise RuntimeError(f"{GEOMETRY_LOG_NAME} is empty or missing")

    rx = leo_plots.load_rx_packet_trace(result_dir / "RxPacketTrace.txt")
    if not rx.empty:
        rx["elevation_deg"] = leo_plots.interpolate_elevation(
            geometry, rx["time_s"].to_numpy(dtype=float)
        )

    mac = leo_plots.load_dl_mac_stats(result_dir / "NrDlMacStats.txt")
    if not mac.empty:
        mac["elevation_deg"] = leo_plots.interpolate_elevation(
            geometry, mac["time_s"].to_numpy(dtype=float)
        )

    sinr = load_bent_pipe_cascade_sinr(result_dir)
    if not sinr.empty:
        sinr["elevation_deg"] = leo_plots.interpolate_elevation(
            geometry, sinr["time_s"].to_numpy(dtype=float)
        )

    bent_geometry = load_bent_pipe_geometry(result_dir / GEOMETRY_LOG_NAME, frequency_hz)
    pathloss = leo_plots.pd.DataFrame(columns=["time_s", "pathloss_db", "elevation_deg"])
    if not bent_geometry.empty:
        pathloss = bent_geometry[["time_s", "pathloss_db", "elevation_deg"]].copy()

    throughput = leo_plots.compute_elevation_tagged_throughput(
        mac, geometry, throughput_window_s
    )

    elev_max = float(geometry["elevation_deg"].max())
    edges = leo_plots.np.arange(
        0.0,
        elev_max + leo_plots.ELEVATION_BIN_DEG,
        leo_plots.ELEVATION_BIN_DEG,
    )
    if edges.size < 2:
        edges = leo_plots.np.array([0.0, leo_plots.ELEVATION_BIN_DEG], dtype=float)

    rows: list[dict[str, object]] = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        row: dict[str, object] = {
            "elev_bin": leo_plots.elevation_bin_label(lo),
            "elev_lo": float(lo),
            "elev_hi": float(hi),
        }

        if not sinr.empty:
            mask = (sinr["elevation_deg"] >= lo) & (sinr["elevation_deg"] < hi)
            row.update(
                leo_plots.stats_for(
                    sinr.loc[mask, "sinr_db"].to_numpy(dtype=float),
                    "sinr_db",
                )
            )
        else:
            row.update(leo_plots.stats_for(leo_plots.np.array([], dtype=float), "sinr_db"))

        if not mac.empty:
            mac0 = mac[mac["rv"] == 0]
            mask = (mac0["elevation_deg"] >= lo) & (mac0["elevation_deg"] < hi)
            row.update(
                leo_plots.stats_for(mac0.loc[mask, "mcs"].to_numpy(dtype=float), "mcs")
            )
        else:
            row.update(leo_plots.stats_for(leo_plots.np.array([], dtype=float), "mcs"))

        if not throughput.empty:
            mask = (throughput["elevation_deg"] >= lo) & (throughput["elevation_deg"] < hi)
            row.update(
                leo_plots.stats_for(
                    throughput.loc[mask, "throughput_mbps"].to_numpy(dtype=float),
                    "tput_mbps",
                )
            )
        else:
            row.update(leo_plots.stats_for(leo_plots.np.array([], dtype=float), "tput_mbps"))

        if not rx.empty:
            mask = (rx["elevation_deg"] >= lo) & (rx["elevation_deg"] < hi)
            sub = rx[mask]
            if "corrupt" in sub.columns:
                corrupt = leo_plots.pd.to_numeric(sub["corrupt"], errors="coerce").dropna()
                row["bler_n"] = int(corrupt.size)
                row["bler_n_corrupt"] = int(corrupt.sum()) if corrupt.size else 0
                row["bler"] = float(corrupt.mean()) if corrupt.size else leo_plots.np.nan
            if "cqi" in sub.columns:
                row.update(leo_plots.stats_for(sub["cqi"].dropna().to_numpy(dtype=float), "cqi"))

        if not pathloss.empty:
            mask = (pathloss["elevation_deg"] >= lo) & (pathloss["elevation_deg"] < hi)
            values = pathloss.loc[mask, "pathloss_db"].to_numpy(dtype=float)
            values = values[leo_plots.np.isfinite(values)]
            row["pl_mean"] = float(leo_plots.np.mean(values)) if values.size else leo_plots.np.nan
            row["pl_sigma"] = (
                float(leo_plots.np.std(values, ddof=1)) if values.size > 1 else leo_plots.np.nan
            )

        rows.append(row)

    stats = leo_plots.pd.DataFrame(rows)
    numeric_columns = stats.select_dtypes(include=[leo_plots.np.number]).columns
    return stats[stats[numeric_columns].notna().any(axis=1)].reset_index(drop=True)


def plot_bent_pipe_elevation_summary(
    stats: leo_plots.pd.DataFrame, result_dir: Path
) -> Path | None:
    panels: list[tuple[str, str]] = []
    if "sinr_db_mean" in stats.columns:
        panels.append(("sinr", "Cascade UE SINR (dB)"))
    if "mcs_mean" in stats.columns:
        panels.append(("mcs", "MCS (rv=0)"))
    if "tput_mbps_mean" in stats.columns:
        panels.append(("tput", "DL Throughput (Mbps)"))
    if "bler" in stats.columns:
        panels.append(("bler", "BLER"))
    if not panels:
        return None

    centers = ((stats["elev_lo"] + stats["elev_hi"]) / 2.0).to_numpy(dtype=float)
    labels = [f"{int(lo)}" for lo in stats["elev_lo"]]
    colors = leo_plots.plt.rcParams["axes.prop_cycle"].by_key()["color"]
    ncols = 2
    nrows = int(leo_plots.np.ceil(len(panels) / ncols))
    fig, axes = leo_plots.plt.subplots(nrows, ncols, figsize=(14, 4 * nrows), squeeze=False)
    axes_flat = axes.flatten()

    for index, (key, _title) in enumerate(panels):
        ax = axes_flat[index]
        color = colors[index % len(colors)]
        if key == "sinr":
            leo_plots.plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "sinr_db_mean",
                f"sinr_db_p{leo_plots.PERCENTILE_LO}",
                f"sinr_db_p{leo_plots.PERCENTILE_HI}",
                color,
                "Cascade UE SINR (dB)",
            )
            leo_plots.annotate_point_values(
                ax, centers, stats["sinr_db_mean"].to_numpy(dtype=float), ".1f"
            )
        elif key == "mcs":
            leo_plots.plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "mcs_mean",
                f"mcs_p{leo_plots.PERCENTILE_LO}",
                f"mcs_p{leo_plots.PERCENTILE_HI}",
                color,
                "MCS index",
            )
            leo_plots.annotate_point_values(
                ax, centers, stats["mcs_mean"].to_numpy(dtype=float), ".1f"
            )
        elif key == "tput":
            leo_plots.plot_elevation_metric_with_bands(
                ax,
                stats,
                centers,
                "tput_mbps_mean",
                f"tput_mbps_p{leo_plots.PERCENTILE_LO}",
                f"tput_mbps_p{leo_plots.PERCENTILE_HI}",
                color,
                "Throughput (Mbps)",
            )
            leo_plots.annotate_point_values(
                ax, centers, stats["tput_mbps_mean"].to_numpy(dtype=float), ".1f"
            )
        elif key == "bler":
            bler = stats["bler"].to_numpy(dtype=float)
            ax.plot(centers, bler, "s-", color=color)
            ax.set_ylabel("BLER", fontsize=leo_plots.LABEL_FONT_SIZE)
            leo_plots.annotate_point_values(ax, centers, bler, ".3f")

        ax.set_xlabel("Elevation (deg)", fontsize=leo_plots.LABEL_FONT_SIZE, fontweight="bold")
        ax.set_xticks(centers)
        ax.set_xticklabels(labels, rotation=45)
        ax.grid(alpha=0.3)
        leo_plots.style_axis_text(ax)

    for index in range(len(panels), len(axes_flat)):
        axes_flat[index].set_visible(False)

    fig.tight_layout()

    output = result_dir / leo_plots.ELEVATION_STATS_FIG_NAME
    fig.savefig(output, dpi=200, bbox_inches="tight")
    leo_plots.plt.close(fig)
    return output


def generate_bent_pipe_elevation_analysis(result_dir: Path, config: object) -> list[Path]:
    stats = build_bent_pipe_elevation_stats(result_dir, config)
    if stats.empty:
        print("Warning: no valid elevation statistics generated")
        return []

    throughput_window_s = float(getattr(config, "throughput_window_s", 1.0))
    report_buffer = io.StringIO()
    with contextlib.redirect_stdout(report_buffer):
        leo_plots.print_elevation_report(stats, throughput_window_s)
    report_text = report_buffer.getvalue().replace(
        "DL DATA SINR (dB)",
        "CASCADE UE SINR (dB)",
    )
    print(report_text, end="")

    report_output = result_dir / leo_plots.ELEVATION_STATS_REPORT_NAME
    report_output.write_text(report_text, encoding="utf-8")
    print(f"Saved elevation report: {report_output}")

    csv_output = result_dir / leo_plots.ELEVATION_STATS_CSV_NAME
    stats.to_csv(csv_output, index=False, float_format="%.4f")
    print(f"Saved elevation CSV: {csv_output}")

    outputs = [report_output, csv_output]
    fig_output = plot_bent_pipe_elevation_summary(stats, result_dir)
    if fig_output is not None:
        outputs.append(fig_output)
        print(f"Saved elevation plot: {fig_output}")
    return outputs


def resolved_plot_parameters(result_dir: Path, config: BentPipeRunConfig) -> dict[str, str]:
    parameters = dict(config.parameters)
    parameters.update(leo_plots.read_used_parameters(result_dir))
    return parameters


def generate_bent_pipe_plots(result_dir: Path, config: BentPipeRunConfig) -> list[Path]:
    """Generate the same plot artifact set as run_leo_link_analysis.py."""
    parameters = resolved_plot_parameters(result_dir, config)
    frequency_hz = float(parameters.get("frequency_hz", 2e9))
    throughput_window_s = float(parameters.get("throughput_window_s", 1.0))
    
    # Create a config object compatible with leo_plots functions
    class AnalysisConfig:
        def __init__(self) -> None:
            self.frequency_hz = frequency_hz
            self.throughput_window_s = throughput_window_s
    
    analysis_config = AnalysisConfig()
    
    # Generate the standard set of plots
    outputs = []
    
    # Figure 1: Geometry and Path Loss
    try:
        fig1 = generate_bent_pipe_figure1(result_dir, frequency_hz)
        outputs.append(fig1)
    except Exception as e:
        print(f"Warning: Failed to generate Figure 1: {e}")
    
    # Figure 2: DL MAC Throughput, MCS, and UE PHY SINR
    try:
        fig2 = generate_bent_pipe_figure2(result_dir, analysis_config, frequency_hz)
        outputs.append(fig2)
    except Exception as e:
        print(f"Warning: Failed to generate Figure 2: {e}")
    
    # Figure 3: Visibility/LOS
    try:
        fig3 = generate_bent_pipe_figure3(result_dir)
        outputs.append(fig3)
    except Exception as e:
        print(f"Warning: Failed to generate Figure 3: {e}")
    
    # Elevation analysis (Figure 4 and statistics)
    try:
        elevation_outputs = generate_bent_pipe_elevation_analysis(result_dir, analysis_config)
        outputs.extend(elevation_outputs)
    except Exception as e:
        print(f"Warning: Failed to generate elevation analysis: {e}")
    
    for path in outputs:
        print(f"Saved plot: {path}")
    
    return outputs


def main() -> int:
    args = parse_args()
    ns3_root = Path(__file__).resolve().parents[2]  # ntn/scripts/ -> ns-3 root
    
    # Get results_root from parameters
    results_root_str = args.parameters["results_root"]
    results_root = Path(results_root_str).expanduser()
    if not results_root.is_absolute():
        results_root = (ns3_root / results_root).resolve()
    
    config = BentPipeRunConfig(parameters=args.parameters)

    try:
        if args.plot_only:
            if not args.input_dir:
                raise ValueError("--plot-only requires --input-dir")
            result_dir = Path(args.input_dir).expanduser().resolve()
            validate_plot_input_dir(result_dir)
            generate_bent_pipe_plots(result_dir, config)
        else:
            result_dir = create_result_dir(results_root)
            print(f"Created result directory: {result_dir}")
            write_used_parameters(result_dir, Path(args.param_file), args.parameters)
            run_bent_pipe(ns3_root, result_dir, config)
            generate_bent_pipe_plots(result_dir, config)
            print(f"Results saved in: {result_dir}")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
