# Repository Guidelines

## Project Structure & Module Organization
This repository is a MATLAB/Simulink-based drone control framework. Core modules live in top-level folders such as `controller/`, `estimator/`, `reference/`, `sensor/`, `model/`, `environment/`, and `connector/`. Simulation entry points include `simulator.slx`, `mainGUI.m`, and setup helpers like `set_for_simulink.m`. Experiments and test scripts are in `experiment/`. Data and configuration artifacts are stored in `Data/` and `.mat` files like `setting.mat` and `plant_setting.mat`. PX4-related assets and generated code are under `PX4/` and `*_ert_rtw/`.

## Build, Test, and Development Commands
There is no standalone build step; use MATLAB/Simulink.
- Open the simulation model: `open("simulator.slx")`
- Launch the GUI: run `mainGUI` in the MATLAB command window
- Initialize paths/settings: run `set_for_simulink`
- Run a test script: `run("experiment/test_drone_throttle.m")`
Adjust paths in MATLAB if the repo root is not on the MATLAB path.

## Coding Style & Naming Conventions
Follow the project’s naming rules from `README.md`:
- Classes: `UpperCamelCase` with underscores (e.g., `C_MPC`)
- Properties/instances: `lowerCamelCase` (e.g., `weightQ`)
- Methods/functions: `snake_case` (e.g., `initialize_states`)
- Base workspace vars: concise `lowerCamelCase` (e.g., `agent`)
- Flags: `f` + `UpperCamelCase` (e.g., `fInitialPosition`)
Avoid meaningless temporary names; favor descriptive indices over `i/j` when clarity improves.

## Testing Guidelines
Tests are lightweight MATLAB scripts in `experiment/` named `test_*.m`. Run them directly with `run(...)`. There is no explicit test framework or coverage enforcement, so include plots/logs or saved outputs when validating behavior.

## Commit & Pull Request Guidelines
Recent history shows short, descriptive commit messages (often in Japanese) without a strict template. Keep commits focused and describe the functional change. For pull requests, include:
- Purpose and scope of changes
- How to reproduce (scripts/models, key parameters)
- Any plots/screenshots from simulations
- Linked issues or related experiments

## Data & Configuration Tips
Many parameters live in `.mat` files. When changing them, document the intent and expected impact in the PR to avoid hidden behavior changes.
