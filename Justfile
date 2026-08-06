# Expose recipe arguments to shell recipes as $1, $2, ... so values reach bash as real argv
# entries rather than being pasted into the script text.
set positional-arguments := true

# Load .env and export it into every recipe, so BAG_RECORDING written by the enable/disable
# recipes is visible to the next setup-ws.
set dotenv-load := true

src_dir := justfile_directory()
env_config_dir := src_dir / "environment_configs"
lidar_config_dir := src_dir / "lidar_configs"

# Recipe to configure the framework workspace

setup-ws environment lidar:
    #!/usr/bin/env bash
    set -euo pipefail

    # Resolve a bare config name to exactly one file under the given config tree. Refuses to guess:
    # no match lists what is available, several matches lists the candidates.
    resolve_config() {
        local label="$1" root="$2" name="$3"
        name="${name%.yaml}"
        name="${name%.yml}"

        local matches=()
        mapfile -t matches < <(
            find "${root}" -type f \( -iname "${name}.yaml" -o -iname "${name}.yml" \) | sort
        )

        if [ "${#matches[@]}" -eq 0 ]; then
            echo "[ERROR] no ${label} config named '${name}' under ${root}" >&2
            echo "        available:" >&2
            find "${root}" -type f \( -name '*.yaml' -o -name '*.yml' \) -printf '          %P\n' \
                | sort >&2
            return 1
        fi

        if [ "${#matches[@]}" -gt 1 ]; then
            echo "[ERROR] ambiguous ${label} config '${name}' — matches:" >&2
            printf '          %s\n' "${matches[@]}" >&2
            return 1
        fi

        printf '%s\n' "${matches[0]}"
    }

    env_file="$(resolve_config environment '{{ env_config_dir }}' "$1")"
    lidar_file="$(resolve_config lidar '{{ lidar_config_dir }}' "$2")"

    echo "environment: ${env_file}"
    echo "lidar:       ${lidar_file}"

    polysetup-ws-sync \
        --src-dir '{{ src_dir }}' \
        --lidar-file "${lidar_file}" \
        --env-file "${env_file}"

    colcon build --packages-up-to lidar_test_bench_bringup

    # Sourcing here would only affect this recipe's subshell, so just remind the caller.
    echo
    echo "Build complete. In your shell, run:"
    echo "    source install/setup.bash"



launch-bench:
    ros2 launch lidar_test_bench_bringup lidar_test_bench_launch.yaml


bench_initiate_service := "/lidar_automation_manager/lidar_test_bench_initiate"
start_evaluation_service := "/start_evaluation"

# Kick off a run. With BAG_RECORDING=true the automation manager owns the run (driver lifecycle,
# angle sweeps, bag recording) and calls /start_evaluation itself per case; otherwise we drive the
# orchestrator directly against already-recorded bags.
start-run:
    #!/usr/bin/env bash
    set -euo pipefail

    recording="$(echo "${BAG_RECORDING:-true}" | tr '[:upper:]' '[:lower:]')"

    case "${recording}" in
        true|1|yes|on)
            echo "BAG_RECORDING=${recording} -> initiating automated bench run"
            ros2 service call \
                '{{ bench_initiate_service }}' \
                std_srvs/srv/Trigger '{}'
            ;;
        false|0|no|off)
            echo "BAG_RECORDING=${recording} -> starting evaluation on recorded bags"
            ros2 service call \
                '{{ start_evaluation_service }}' \
                std_srvs/srv/SetBool '{data: true}'
            ;;
        *)
            echo "[ERROR] BAG_RECORDING='${BAG_RECORDING:-}' is not a boolean" >&2
            echo "        run 'just enable-bag-recording' or 'just disable-bag-recording'" >&2
            exit 1
            ;;
    esac

# Stop an in-progress evaluation.
stop-run:
    ros2 service call '{{ start_evaluation_service }}' std_srvs/srv/SetBool '{data: false}'



# Turn bag recording off; the next setup-ws comments the recording chain out of the launch file.
disable-bag-recording:
    polysetup-bag-recording --src-dir '{{ src_dir }}' --bag-recording-status false

# Turn bag recording back on: the next setup-ws leaves the recording chain uncommented.
enable-bag-recording:
    polysetup-bag-recording --src-dir '{{ src_dir }}' --bag-recording-status true










