# Accelerate Refactor Design

## Context

`hanmole_navigation` currently implements the velocity acceleration chain with two public headers and four source files:

- `include/hanmole_navigation/cmd_vel_boost_source.hpp`
- `include/hanmole_navigation/cmd_vel_mux_stamped.hpp`
- `src/cmd_vel_boost_source_node.cpp`
- `src/cmd_vel_boost_source_main.cpp`
- `src/cmd_vel_mux_stamped_node.cpp`
- `src/cmd_vel_mux_stamped_main.cpp`

The code builds two component libraries and two standalone executables. `navigation_launch.py` supports both standalone `Node` actions and `ComposableNode` plugin loading. Tests include the two old headers and link both component targets.

## Goal

Consolidate the acceleration-chain source layout into:

- `include/hanmole_navigation/accelerate.hpp`
- `src/accelerate.cpp`

The refactor should preserve the current runtime behavior:

- ROS node names remain `cmd_vel_boost_source` and `cmd_vel_mux_stamped`.
- Standalone executable names remain `cmd_vel_boost_source_node` and `cmd_vel_mux_stamped_node`.
- Component plugin class names remain `hanmole_navigation::CmdVelBoostSourceNode` and `hanmole_navigation::CmdVelMuxStampedNode`.
- Existing topic defaults, parameter names, launch behavior, and tests remain behaviorally equivalent.

## Considered Approaches

### Recommended: Source Consolidation Only

Create one public header and one implementation file while preserving two ROS nodes, two standalone executables, and two component registrations.

`accelerate.cpp` contains both node implementations and uses compile definitions to select which standalone `main()` is built for each executable target. The component library builds from the same source with component registration enabled.

This keeps the public ROS behavior stable and limits the refactor to file boundaries and CMake target wiring.

### Alternative: Single Runtime Node

Create one `accelerate_node` process that instantiates both internal nodes or merges the two functions into one node.

This reduces runtime process count but changes launch semantics, respawn behavior, parameter namespaces, and composition behavior. It is not recommended for this refactor because it combines a file cleanup with behavioral architecture changes.

### Alternative: Compatibility Wrapper Headers

Add `accelerate.hpp`, but keep the old headers as thin wrappers that include the new header.

This helps external code that still includes the old paths, but it does not fully remove the old public files. It is useful if downstream packages depend on these headers. If no external dependency exists, the old headers can be removed in the same change.

## Detailed Design

### Header

`accelerate.hpp` declares all types and helper functions currently split across the two headers:

- `BoostSourceConfig`
- `boostLinearX`
- `forwardCorridorIsClear`
- `twistQualifiesForBoost`
- `makeBoostedCommandIfAllowed`
- `CmdVelBoostSourceNode`
- `StampedMuxInputState`
- `stampedMuxInputIsFresh`
- `selectActiveStampedInput`
- `makeMuxOutput`
- `CmdVelMuxStampedNode`

The namespace remains `hanmole_navigation`.

### Implementation

`accelerate.cpp` includes `hanmole_navigation/accelerate.hpp` and contains the implementation blocks from both existing node source files.

It also contains optional standalone entry points guarded by compile definitions:

- `HANMOLE_NAVIGATION_ACCELERATE_BOOST_MAIN` builds `main()` for `CmdVelBoostSourceNode`.
- `HANMOLE_NAVIGATION_ACCELERATE_MUX_MAIN` builds `main()` for `CmdVelMuxStampedNode`.
- `HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS` enables both `RCLCPP_COMPONENTS_REGISTER_NODE` registrations.

This lets CMake build one shared component target and two executable targets from the same physical source file without keeping separate `*_main.cpp` files.

### CMake

Replace the two old component libraries with one shared target built from `src/accelerate.cpp`.

The standalone executables also build from `src/accelerate.cpp`, each with its own compile definition selecting the desired `main()`.

Tests should include `accelerate.hpp` and link the consolidated acceleration component target.

### Launch And Config

No launch or config behavior should change in the recommended approach.

The existing standalone executable names are preserved, so the `Node(executable=...)` launch paths continue to work. The existing plugin class names are preserved, so `ComposableNode(plugin=...)` launch paths continue to work.

### Tests

Update `test_cmd_vel_boost_chain.cpp` to include `hanmole_navigation/accelerate.hpp`.

Run the package tests after implementation:

- `colcon test --packages-select hanmole_navigation`
- `colcon test-result --verbose`

If the workspace build cache has stale targets from removed files, run a clean package build before testing.

## Scope Boundaries

This refactor does not change acceleration math, obstacle checks, mux selection logic, topic defaults, parameter names, node names, or launch conditions.

It also does not introduce a new runtime `accelerate_node` executable unless requested separately.
